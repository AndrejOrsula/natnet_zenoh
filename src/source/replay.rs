//! Replays a recorded capture (`.natnet` or `.aucap`) as a source.

use std::path::PathBuf;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant, SystemTime};

use natnet::{ConnectionInfo, ModelDef, ServerInfo, Version};
use serde::{Deserialize, Serialize};
use tracing::{info, warn};

use super::capture::{CaptureReader, MAX_PAUSE_NS};
use super::dispatch::{self, Dispatcher, Outcome, Policy};
use super::synthetic::parse_version;
use super::{ConnectionState, SourceCommand, SourceContext, SourceEvent};

/// Replay settings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ReplayConfig {
    /// The capture file.
    pub path: PathBuf,
    /// Playback speed; 0 replays as fast as the consumer keeps up.
    pub rate: f64,
    /// Restart at the end.
    pub looping: bool,
    /// Skip this many seconds of the recording.
    pub start_offset_secs: f64,
    /// Protocol version; forces the decoder when set. Otherwise a recorded
    /// SERVERINFO decides, falling back to 4.1.
    pub protocol_version: Option<String>,
    /// Camera clock frequency when the capture has no SERVERINFO, Hz.
    pub clock_frequency: u64,
    /// A raw MODELDEF datagram, for captures without one.
    pub model_file: Option<PathBuf>,
    /// Validation policy.
    pub parse_policy: Policy,
}

impl Default for ReplayConfig {
    fn default() -> Self {
        Self {
            path: PathBuf::new(),
            rate: 1.0,
            looping: false,
            start_offset_secs: 0.0,
            protocol_version: None,
            clock_frequency: 10_000_000,
            model_file: None,
            parse_policy: Policy::Compatible,
        }
    }
}

impl ReplayConfig {
    /// Checks the settings.
    ///
    /// # Errors
    /// A message naming the problem.
    pub fn validate(&self) -> Result<(), String> {
        if self.path.as_os_str().is_empty() {
            return Err("replay.path is required".into());
        }
        if !(self.rate == 0.0 || (self.rate.is_finite() && (1e-3..=1e6).contains(&self.rate))) {
            return Err("replay.rate must be 0 (unpaced) or in [0.001, 1e6]".into());
        }
        if !(self.start_offset_secs >= 0.0 && self.start_offset_secs.is_finite()) {
            return Err("replay.start_offset_secs must be non-negative".into());
        }
        if let Some(v) = &self.protocol_version {
            parse_version(v)?;
        }
        Ok(())
    }
}

/// Scans up to 20,000 records for SERVERINFO and MODELDEF datagrams.
fn scan(cfg: &ReplayConfig) -> std::io::Result<(Option<ServerInfo>, Option<Vec<u8>>)> {
    let mut info = None;
    let mut model = None;
    for record in CaptureReader::open(&cfg.path)?.take(20_000) {
        // A read error ends the scan; replay reports it when it reaches that record.
        let Ok(record) = record else { break };
        match record
            .bytes
            .get(..2)
            .map(|b| u16::from_le_bytes([b[0], b[1]]))
        {
            Some(natnet::NAT_SERVERINFO) if info.is_none() => {
                info = ServerInfo::parse(&record.bytes).ok();
            }
            Some(natnet::NAT_MODELDEF) if model.is_none() => model = Some(record.bytes),
            _ => {}
        }
        if info.is_some() && model.is_some() {
            break;
        }
    }
    Ok((info, model))
}

/// Runs the replay until the end (or forever when looping).
pub fn run(cfg: &ReplayConfig, ctx: &SourceContext, commands: &Receiver<SourceCommand>) {
    let forced = cfg
        .protocol_version
        .as_deref()
        .and_then(|v| parse_version(v).ok());
    let mut dispatcher = Dispatcher::new(cfg.parse_policy, forced);
    let (recorded_info, recorded_model) = match scan(cfg) {
        Ok(found) => found,
        Err(e) => {
            ctx.send(SourceEvent::DecodeError(format!(
                "{}: {e}",
                cfg.path.display()
            )));
            ctx.send(SourceEvent::Connection(ConnectionState::Finished));
            return;
        }
    };
    let version = forced
        .or(recorded_info.as_ref().map(|i| i.protocol_version))
        .unwrap_or(Version::V4_1);
    if let Err(e) = dispatcher.adopt(version) {
        ctx.send(SourceEvent::DecodeError(e));
        ctx.send(SourceEvent::Connection(ConnectionState::Finished));
        return;
    }
    let mut info = recorded_info.unwrap_or_else(|| ServerInfo {
        application_name: "capture replay".into(),
        server_version: Version::new(0, 0, 0, 0),
        protocol_version: version,
        high_res_clock_frequency: cfg.clock_frequency,
        connection: ConnectionInfo {
            data_port: 0,
            multicast: false,
            multicast_group: [0; 4],
        },
    });
    if forced.is_some() {
        info.protocol_version = version;
    }
    if !ctx.send(SourceEvent::ServerInfo(info)) {
        return;
    }
    // The model: a recorded MODELDEF wins over the file.
    let mut model: Option<(ModelDef, Version)> =
        recorded_model
            .as_deref()
            .and_then(|d| match dispatcher.decode_modeldef(d) {
                Ok(m) => Some(m),
                Err(e) => {
                    warn!("recorded MODELDEF: {e}");
                    None
                }
            });
    if model.is_none()
        && let Some(path) = &cfg.model_file
    {
        match dispatch::load_modeldef(path, version, cfg.parse_policy) {
            Ok(md) => model = Some((md, version)),
            Err(e) => warn!("model file: {e}"),
        }
    }
    if model.is_none() {
        warn!(
            "the capture has no model definition and none was configured; bodies will be named by id"
        );
    }
    if let Some((md, v)) = &model
        && !ctx.send(SourceEvent::ModelDef(md.clone(), *v))
    {
        return;
    }
    ctx.send(SourceEvent::Connection(ConnectionState::Streaming));
    info!("replaying {} (protocol {version})", cfg.path.display());

    // Unpaced replays wait for the consumer rather than dropping frames.
    ctx.set_lossless(cfg.rate == 0.0);
    let skip_ns = (cfg.start_offset_secs * 1e9) as u64;
    loop {
        let reader = match CaptureReader::open(&cfg.path) {
            Ok(r) => r,
            Err(e) => {
                ctx.send(SourceEvent::DecodeError(format!(
                    "{}: {e}",
                    cfg.path.display()
                )));
                break;
            }
        };
        let mut first: Option<u64> = None;
        let mut origin: Option<(u64, Instant)> = None;
        let mut last: Option<u64> = None;
        let mut frames_this_pass = 0_u64;
        let mut read_failed = false;
        let pass_start = Instant::now();
        for record in reader {
            if ctx.stopped() {
                return;
            }
            for cmd in commands.try_iter() {
                match cmd {
                    SourceCommand::RefreshModel => {
                        if let Some((md, v)) = &model {
                            ctx.send(SourceEvent::ModelDef(md.clone(), *v));
                        }
                    }
                }
            }
            let record = match record {
                Ok(r) => r,
                Err(e) => {
                    warn!("capture read stopped: {e}");
                    ctx.send(SourceEvent::DecodeError(e.to_string()));
                    read_failed = true;
                    break;
                }
            };
            let t0 = *first.get_or_insert(record.time_ns);
            if record.time_ns.saturating_sub(t0) < skip_ns {
                continue;
            }
            if cfg.rate > 0.0 {
                // Long pauses in the recording are skipped, not waited out;
                // after a step back, pacing restarts rather than racing
                // through the records until time catches up.
                if let Some(l) = last {
                    if record.time_ns.saturating_sub(l) > MAX_PAUSE_NS {
                        warn!(
                            "skipping a {:.0} s pause in the capture",
                            record.time_ns.saturating_sub(l) as f64 * 1e-9
                        );
                        origin = None;
                    } else if record.time_ns < l {
                        warn!("the capture's time steps back; pacing restarts");
                        origin = None;
                    }
                }
                last = Some(record.time_ns);
                let (o_t, o_wall) = *origin.get_or_insert((record.time_ns, Instant::now()));
                let offset = Duration::try_from_secs_f64(
                    record.time_ns.saturating_sub(o_t) as f64 * 1e-9 / cfg.rate,
                )
                .unwrap_or(Duration::MAX);
                // An offset beyond the clock's range (a corrupt timestamp)
                // waits until stopped.
                let due = o_wall
                    .checked_add(offset)
                    .unwrap_or(o_wall + Duration::from_secs(u64::from(u32::MAX)));
                // Sleep in slices so stop requests stay responsive.
                loop {
                    let now = Instant::now();
                    if now >= due || ctx.stopped() {
                        break;
                    }
                    std::thread::sleep((due - now).min(Duration::from_millis(100)));
                }
            }
            match dispatcher.dispatch(ctx, &record.bytes, SystemTime::now(), Instant::now()) {
                Outcome::Closed => return,
                Outcome::Frame { .. } => frames_this_pass += 1,
                Outcome::ModelDef => {
                    if let Ok(m) = dispatcher.decode_modeldef(&record.bytes) {
                        model = Some(m);
                    }
                }
                _ => {}
            }
        }
        if !cfg.looping || ctx.stopped() || read_failed {
            break;
        }
        if frames_this_pass == 0 {
            warn!("the capture has no frames to loop over");
            break;
        }
        // A paced pass spanning no recorded time (a single frame) would spin.
        if cfg.rate > 0.0 && pass_start.elapsed() < Duration::from_millis(10) {
            std::thread::sleep(Duration::from_millis(10));
        }
        info!("replay looping");
    }
    ctx.send(SourceEvent::Connection(ConnectionState::Finished));
}

#[cfg(test)]
mod tests {
    use std::fs::{self, OpenOptions};
    use std::io::Write;
    use std::sync::mpsc;
    use std::time::{SystemTime, UNIX_EPOCH};

    use super::*;
    use crate::source::capture::{CaptureWriter, Format, Record};
    use crate::source::synthetic::{Scene, SyntheticConfig};

    #[test]
    fn corrupt_tail_ends_loop_after_one_valid_frame() {
        let path = std::env::temp_dir().join(format!(
            "natnet-zenoh-corrupt-replay-{}-{}.natnet",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let scene = Scene::new(SyntheticConfig::default()).unwrap();
        let mut writer = CaptureWriter::create(&path, Format::NatNet).unwrap();
        writer
            .write(&Record {
                time_ns: 0,
                source: None,
                bytes: scene.frame_datagram(1).unwrap(),
            })
            .unwrap();
        drop(writer);
        OpenOptions::new()
            .append(true)
            .open(&path)
            .unwrap()
            .write_all(&[1, 2, 3])
            .unwrap();

        let cfg = ReplayConfig {
            path: path.clone(),
            rate: 0.0,
            looping: true,
            ..ReplayConfig::default()
        };
        let mut source =
            crate::source::SourceHandle::spawn("test-corrupt-replay", 16, move |ctx, commands| {
                run(&cfg, &ctx, &commands);
            })
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut seen = Vec::new();
        let mut repeated_frame = false;
        let mut finished = false;
        while Instant::now() < deadline && !repeated_frame && !finished {
            match source.next_event(Duration::from_millis(25)) {
                Ok(event) => {
                    if matches!(event, SourceEvent::Frame { .. })
                        && seen
                            .iter()
                            .any(|old| matches!(old, SourceEvent::Frame { .. }))
                    {
                        repeated_frame = true;
                    }
                    if matches!(event, SourceEvent::Connection(ConnectionState::Finished)) {
                        finished = true;
                    }
                    seen.push(event);
                }
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
            }
        }
        source.stop();
        drop(source);
        let cleanup = fs::remove_file(path);
        let cleanup_succeeded = cleanup.is_ok();

        let frames = seen
            .iter()
            .filter(|e| matches!(e, SourceEvent::Frame { .. }))
            .count();
        let errors = seen
            .iter()
            .filter(|e| matches!(e, SourceEvent::DecodeError(_)))
            .count();
        assert!(
            cleanup_succeeded,
            "capture fixture cleanup failed: {cleanup:?}"
        );
        assert!(
            !repeated_frame,
            "loop replayed a valid frame after corruption (frames={frames}, errors={errors}, finished={finished})"
        );
        assert!(finished, "replay did not finish before deadline");
        assert_eq!(frames, 1, "replay emitted repeated frames");
        assert_eq!(errors, 1, "replay did not report corrupt tail");
        assert!(matches!(
            seen.last(),
            Some(SourceEvent::Connection(ConnectionState::Finished))
        ));
    }
}
