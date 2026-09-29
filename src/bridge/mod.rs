//! The bridge: source events in, zenoh and ROS 2 outputs out.

use std::sync::Mutex;
use std::sync::atomic::AtomicBool;

pub mod native;
pub mod ros_out;
pub mod snapshot;

/// State shared with service handlers running on zenoh threads.
#[derive(Debug, Default)]
pub struct Shared {
    server_info: Mutex<serde_json::Value>,
    model: Mutex<serde_json::Value>,
    stats: Mutex<serde_json::Value>,
    /// A service asked for the clock synchronization to restart.
    pub reset_requested: AtomicBool,
}

impl Shared {
    fn slot(&self, name: &str) -> Option<&Mutex<serde_json::Value>> {
        match name {
            "server_info" => Some(&self.server_info),
            "model" => Some(&self.model),
            "stats" => Some(&self.stats),
            _ => None,
        }
    }

    /// Stores a JSON document.
    pub fn set(&self, name: &str, value: serde_json::Value) {
        if let Some(slot) = self.slot(name) {
            *slot
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner) = value;
        }
    }

    /// A stored JSON document, serialized (`null` when absent).
    #[must_use]
    pub fn snapshot_json(&self, name: &str) -> String {
        self.slot(name).map_or_else(
            || "null".into(),
            |s| {
                s.lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner)
                    .to_string()
            },
        )
    }
}

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use natnet::{ServerInfo, Version};
use tracing::{debug, info, warn};

use crate::config::{Config, SourceKind};
use crate::mocap::timing::StampPolicy;
use crate::ros::RosError;
use crate::ros::context::{Context, ContextOptions, SessionConfig};
use crate::ros::msgs::diagnostic_msgs::{DiagnosticStatus, KeyValue};
use crate::ros::rosout::{RosoutHandle, RosoutWorker};
use crate::source::{self, ConnectionState, SourceEvent, SourceHandle};
use snapshot::{Processor, Snapshot};

/// A bridge failure.
#[derive(Debug, thiserror::Error)]
pub enum BridgeError {
    /// Zenoh or ROS setup failed.
    #[error(transparent)]
    Ros(#[from] RosError),
    /// The source could not start.
    #[error("source: {0}")]
    Source(String),
}

/// Opens the zenoh session described by the configuration (environment
/// variables fill what the file leaves unset, like rmw_zenoh).
///
/// # Errors
/// Malformed environment variables or zenoh errors.
pub fn open_context(cfg: &Config) -> Result<Context, RosError> {
    Context::new(&context_options(cfg)?)
}

/// The session options [`open_context`] uses.
///
/// # Errors
/// Malformed environment variables or overrides.
pub fn context_options(cfg: &Config) -> Result<ContextOptions, RosError> {
    let mut opts = ContextOptions::from_env()?;
    if let Some(d) = cfg.ros.domain_id {
        opts.domain_id = d;
    }
    opts.enclave.clone_from(&cfg.ros.enclave);
    opts.compat = cfg.ros.rmw_zenoh;
    if let Some(f) = &cfg.zenoh.config_file {
        opts.config = SessionConfig::File(f.clone());
    } else if let Some(j) = &cfg.zenoh.config_json5 {
        opts.config = SessionConfig::Json5(j.clone());
    }
    for o in &cfg.zenoh.overrides {
        opts.overrides
            .extend(crate::ros::context::parse_overrides(o)?);
    }
    if let Some(n) = cfg.zenoh.router_check_attempts {
        opts.router_check_attempts = Some(n);
    }
    Ok(opts)
}

/// Starts the configured source.
///
/// # Errors
/// A message when the source cannot start.
pub fn start_source(cfg: &Config) -> Result<SourceHandle, BridgeError> {
    let queue = cfg.source.queue;
    let spawned = match cfg.source.kind {
        SourceKind::Live => {
            let c = cfg.source.live.clone();
            SourceHandle::spawn("natnet-live", queue, move |ctx, cmds| {
                source::live::run(&c, ctx, &cmds);
            })
        }
        SourceKind::Replay => {
            let c = cfg.source.replay.clone();
            SourceHandle::spawn("natnet-replay", queue, move |ctx, cmds| {
                source::replay::run(&c, &ctx, &cmds);
            })
        }
        SourceKind::Synthetic => {
            let scene = source::synthetic::Scene::new(cfg.source.synthetic.clone())
                .map_err(BridgeError::Source)?;
            SourceHandle::spawn("natnet-synthetic", queue, move |ctx, cmds| {
                source::synthetic::run(&scene, &ctx, &cmds);
            })
        }
    };
    spawned.map_err(|e| BridgeError::Source(e.to_string()))
}

/// The SERVERINFO as a JSON document.
#[must_use]
pub fn server_info_json(info: &ServerInfo) -> serde_json::Value {
    serde_json::json!({
        "application": info.application_name,
        "server_version": info.server_version.to_string(),
        "protocol_version": info.protocol_version.to_string(),
        "clock_frequency": info.high_res_clock_frequency,
        "multicast": info.connection.multicast,
        "multicast_group": std::net::Ipv4Addr::from(info.connection.multicast_group).to_string(),
        "data_port": info.connection.data_port,
    })
}

/// The decoded model as a JSON document.
#[must_use]
pub fn model_json(model: &crate::mocap::model::Model) -> serde_json::Value {
    serde_json::to_value(model).unwrap_or_default()
}

/// The stream statistics as a JSON document, with the bridge `state` and
/// the frames dropped between the source and the bridge.
#[must_use]
pub fn stats_json(
    stats: &crate::mocap::stats::StreamStats,
    state: &str,
    queue_drops: u64,
) -> serde_json::Value {
    let mut v = serde_json::to_value(stats).unwrap_or_default();
    if let Some(m) = v.as_object_mut() {
        m.insert("state".into(), state.into());
        m.insert("queue_drops".into(), queue_drops.into());
    }
    v
}

const MIN_REFRESH_INTERVAL: Duration = Duration::from_secs(2);
const MAX_REFRESH_INTERVAL: Duration = Duration::from_secs(60);

/// What a finished run observed.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct RunSummary {
    /// Frames processed.
    pub frames: u64,
    /// Frames dropped between the source and the bridge.
    pub queue_drops: u64,
    /// Frames missing from the sequence.
    pub dropped_frames: u64,
    /// Reordered frames that arrived too late and were discarded.
    pub late_frames: u64,
    /// Datagrams that failed to decode.
    pub decode_errors: u64,
    /// The source reported its end.
    pub finished: bool,
}

/// The running bridge.
pub struct Bridge {
    cfg: Config,
    processor: Processor,
    shared: Arc<Shared>,
    source: SourceHandle,
    context: Option<Context>,
    ros: Option<ros_out::RosOutputs>,
    native: Option<native::NativeOutputs>,
    version: Option<Version>,
    state: &'static str,
    next_publish: Option<Instant>,
    last_refresh: Option<Instant>,
    refresh_interval: Duration,
    last_unknown: std::collections::BTreeSet<String>,
    last_diag: Instant,
    diag_frames: u64,
    diag_repeats: u64,
    diag_dropped: u64,
    last_snapshot: Option<Snapshot>,
    finished: bool,
    config_updates: Option<std::sync::mpsc::Receiver<Config>>,
    _rosout: Option<RosoutWorker>,
}

impl std::fmt::Debug for Bridge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Bridge")
            .field("state", &self.state)
            .finish_non_exhaustive()
    }
}

impl Bridge {
    /// Opens zenoh, declares the outputs and starts the source.
    ///
    /// # Errors
    /// Zenoh, naming or source errors.
    pub fn new(
        cfg: Config,
        context: Option<Context>,
        rosout: Option<&mut RosoutHandle>,
    ) -> Result<Self, BridgeError> {
        let mut cfg = cfg;
        // A replay off real time stretches the server clock against the
        // host's, so clock synchronization would stamp ever further behind.
        if cfg.source.kind == SourceKind::Replay
            && cfg.source.replay.rate != 1.0
            && cfg.timing.policy == StampPolicy::ClockSync
        {
            warn!(
                "replaying at rate {}: stamping on receipt instead of by clock synchronization",
                cfg.source.replay.rate
            );
            cfg.timing.policy = StampPolicy::Receive;
        }
        let shared = Arc::new(Shared::default());
        // Open zenoh (which may wait for a router) before the source starts
        // queueing frames.
        let context = match context {
            Some(c) => Some(c),
            None if cfg.ros.enabled || cfg.native.enabled => Some(open_context(&cfg)?),
            None => None,
        };
        let source = start_source(&cfg)?;
        let commands = source.commander();
        let mut config_updates = None;
        let mut rosout_worker = None;
        let ros = match (&context, cfg.ros.enabled) {
            (Some(ctx), true) => {
                let node = ctx.create_node(&cfg.ros.node_name, &cfg.ros.namespace)?;
                if cfg.ros.rosout
                    && let Some(h) = rosout
                {
                    rosout_worker = h.attach(&node)?;
                }
                info!(
                    "ROS 2 node {} on domain {} (zenoh {})",
                    node.fully_qualified_name(),
                    ctx.domain_id(),
                    ctx.zid()
                );
                let (tx, rx) = std::sync::mpsc::channel();
                config_updates = Some(rx);
                let on_set = ros_out::config_setter(&cfg, tx);
                Some(ros_out::RosOutputs::new(
                    node,
                    &cfg,
                    &shared,
                    &commands,
                    Some(on_set),
                )?)
            }
            _ => None,
        };
        let native = match (&context, cfg.native.enabled) {
            (Some(ctx), true) => {
                info!("native zenoh outputs under {}/**", cfg.native.prefix);
                Some(native::NativeOutputs::new(
                    ctx.session(),
                    &cfg.native,
                    &shared,
                    &commands,
                )?)
            }
            _ => None,
        };
        Ok(Self {
            processor: Processor::new(&cfg),
            cfg,
            shared,
            source,
            context,
            ros,
            native,
            version: None,
            state: "waiting",
            next_publish: None,
            last_refresh: None,
            refresh_interval: MIN_REFRESH_INTERVAL,
            last_unknown: std::collections::BTreeSet::new(),
            last_diag: Instant::now(),
            diag_frames: 0,
            diag_repeats: 0,
            diag_dropped: 0,
            last_snapshot: None,
            finished: false,
            config_updates,
            _rosout: rosout_worker,
        })
    }

    /// The zenoh context, when one is open.
    #[must_use]
    pub fn context(&self) -> Option<&Context> {
        self.context.as_ref()
    }

    /// The ROS outputs, when enabled.
    #[must_use]
    pub fn ros(&self) -> Option<&ros_out::RosOutputs> {
        self.ros.as_ref()
    }

    /// Runs until `stop` is set, the source ends (with `exit_on_end`), or
    /// the source thread dies.
    pub fn run(&mut self, stop: &std::sync::atomic::AtomicBool) -> RunSummary {
        let period = Duration::from_secs_f64(self.cfg.diagnostics.period_secs);
        while !stop.load(Ordering::Relaxed) {
            let wait = period
                .saturating_sub(self.last_diag.elapsed())
                .min(Duration::from_millis(100));
            match self.source.next_event(wait) {
                Ok(event) => {
                    if self.handle(event) {
                        break;
                    }
                }
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                    if !self.finished {
                        warn!("the source stopped");
                    }
                    break;
                }
            }
            while let Some(cfg) = self
                .config_updates
                .as_ref()
                .and_then(|rx| rx.try_recv().ok())
            {
                self.reconfigure(cfg);
            }
            if self.shared.reset_requested.swap(false, Ordering::Relaxed) {
                info!("resetting clock synchronization and velocity history");
                self.processor.reset();
                if let Some(r) = &mut self.ros {
                    r.reset();
                }
            }
            // Decode errors the queue had no room for still count.
            self.processor.stats.decode_errors += self.source.take_dropped_reports();
            if self.last_diag.elapsed() >= period {
                self.diagnostics();
            }
        }
        self.processor.stats.decode_errors += self.source.take_dropped_reports();
        self.diagnostics();
        self.summary()
    }

    /// Adopts a configuration changed through ROS parameters.
    fn reconfigure(&mut self, cfg: Config) {
        info!("parameters changed; reconfiguring");
        self.processor.reconfigure(&cfg);
        if let Some(r) = &mut self.ros {
            r.reconfigure(&cfg);
        }
        if let Some(n) = &mut self.native {
            n.model_changed();
        }
        if cfg.timing.max_rate_hz != self.cfg.timing.max_rate_hz {
            self.next_publish = None;
        }
        self.cfg = cfg;
    }

    /// Counters so far.
    #[must_use]
    pub fn summary(&self) -> RunSummary {
        RunSummary {
            frames: self.processor.stats.frames,
            queue_drops: self.source.dropped_frames(),
            dropped_frames: self.processor.stats.dropped_frames,
            late_frames: self.processor.stats.late_frames,
            decode_errors: self.processor.stats.decode_errors,
            finished: self.finished,
        }
    }

    /// Handles one event; true when the bridge should stop.
    fn handle(&mut self, event: SourceEvent) -> bool {
        match event {
            SourceEvent::ServerInfo(info) => {
                self.processor.set_server_info(&info);
                self.version = Some(info.protocol_version);
                let json = server_info_json(&info);
                let text = json.to_string();
                self.shared.set("server_info", json);
                if let Some(r) = &self.ros {
                    r.publish_server_info(&text);
                }
                if let Some(n) = &self.native {
                    n.put_document("server_info", &text);
                }
            }
            SourceEvent::ModelDef(md, version) => {
                let previous = self.processor.model().clone();
                for (type_id, e) in self.processor.set_model(&md, version) {
                    warn!("model dataset of type {type_id} could not be decoded: {e}");
                }
                let model = self.processor.model();
                if self.shared.model.lock().is_ok_and(|m| !m.is_null()) && *model == previous {
                    debug!("model unchanged");
                    return false;
                }
                info!(
                    "model: {} rigid bodies, {} skeletons, {} force plates, {} devices, {} marker sets",
                    model.rigid_bodies.len(),
                    model.skeletons.len(),
                    model.force_plates.len(),
                    model.devices.len(),
                    model.marker_sets.len()
                );
                let json = model_json(model);
                let text = json.to_string();
                self.shared.set("model", json);
                if let Some(r) = &mut self.ros {
                    r.publish_model(&text);
                }
                if let Some(n) = &mut self.native {
                    n.put_document("model", &text);
                    n.model_changed();
                }
            }
            SourceEvent::Frame {
                frame,
                received,
                received_mono,
            } => {
                let resets = self.processor.stats.resets;
                let Some(snap) = self.processor.process(&frame, received, received_mono) else {
                    let late = self.processor.stats.late_frames;
                    if late <= 10 || late.is_power_of_two() {
                        debug!("discarded late frame {} (#{late})", frame.frame_number);
                    }
                    return false;
                };
                if self.processor.stats.resets != resets {
                    info!(
                        "frame numbers restarted at {}; timing and velocity history reset",
                        snap.frame_number
                    );
                    if let Some(r) = &mut self.ros {
                        r.reset();
                    }
                }
                self.maybe_refresh();
                // A repeated frame number (a paused server) carries the same
                // data and stamp; republishing it only makes tf2 complain.
                let repeated = self
                    .last_snapshot
                    .as_ref()
                    .is_some_and(|s| s.frame_number == snap.frame_number);
                let max_rate = self.cfg.timing.max_rate_hz;
                let due = !repeated
                    && (max_rate <= 0.0 || self.next_publish.is_none_or(|d| received_mono >= d));
                if due {
                    // Deadlines advance by whole periods, so arrival jitter
                    // does not pull the output rate below `max_rate_hz`; after
                    // a gap they resynchronize instead of bursting.
                    if max_rate > 0.0 {
                        let period = Duration::from_secs_f64(1.0 / max_rate);
                        let mut next = self.next_publish.unwrap_or(received_mono) + period;
                        if next <= received_mono {
                            next = received_mono + period;
                        }
                        self.next_publish = Some(next);
                    }
                    if let Some(r) = &mut self.ros {
                        r.publish_frame(&snap);
                    }
                    if let Some(n) = &mut self.native {
                        n.publish_frame(&snap, &frame, self.version);
                    }
                }
                self.last_snapshot = Some(snap);
            }
            SourceEvent::ServerMessage(text) => info!("server: {text}"),
            SourceEvent::DecodeError(e) => {
                self.processor.stats.decode_errors += 1;
                let n = self.processor.stats.decode_errors;
                if n <= 10 || n.is_power_of_two() {
                    warn!("decode error #{n}: {e}");
                }
            }
            SourceEvent::Connection(state) => {
                self.state = match state {
                    ConnectionState::Streaming => "streaming",
                    ConnectionState::Lost => "lost",
                    ConnectionState::Finished => "finished",
                };
                if state == ConnectionState::Finished {
                    self.finished = true;
                    info!("source finished");
                    return self.cfg.source.exit_on_end;
                }
            }
        }
        false
    }

    /// Requests the model when frames carry assets it does not describe,
    /// backing off while a refreshed model still lacks the same assets.
    fn maybe_refresh(&mut self) {
        if self
            .last_refresh
            .is_some_and(|t| t.elapsed() < self.refresh_interval)
        {
            return;
        }
        let unknown = self.processor.take_unknown();
        if unknown.is_empty() {
            self.refresh_interval = MIN_REFRESH_INTERVAL;
            self.last_unknown.clear();
            return;
        }
        if unknown == self.last_unknown {
            if self.refresh_interval == MIN_REFRESH_INTERVAL {
                warn!(
                    "the model does not describe {unknown:?}; publishing them under generic names"
                );
            }
            self.refresh_interval = (self.refresh_interval * 2).min(MAX_REFRESH_INTERVAL);
        } else {
            self.refresh_interval = MIN_REFRESH_INTERVAL;
        }
        debug!("unknown assets {unknown:?}; requesting the model");
        self.last_unknown = unknown;
        self.last_refresh = Some(Instant::now());
        self.source.command(source::SourceCommand::RefreshModel);
    }

    fn stats_json(&self) -> serde_json::Value {
        stats_json(
            &self.processor.stats,
            self.state,
            self.source.dropped_frames(),
        )
    }

    fn diagnostics(&mut self) {
        let elapsed = self.last_diag.elapsed().as_secs_f64().max(1e-3);
        self.last_diag = Instant::now();
        let stats = &self.processor.stats;
        // Late frames are already counted as gaps, and repeated ones (a
        // paused server) carry nothing new: neither counts towards the rate.
        let frames = stats.new_frames() - self.diag_frames;
        let repeats = stats.duplicate_frames - self.diag_repeats;
        // A frame dropped in the queue also leaves a gap in the frame
        // numbers the processor sees, so the gaps alone count every loss.
        let dropped = stats.dropped_frames - self.diag_dropped;
        self.diag_frames = stats.new_frames();
        self.diag_repeats = stats.duplicate_frames;
        self.diag_dropped = stats.dropped_frames;
        let json = self.stats_json();
        let text = json.to_string();
        self.shared.set("stats", json);
        if let Some(n) = &self.native {
            n.put_document("stats", &text);
        }
        let Some(ros) = &self.ros else { return };
        if !self.cfg.diagnostics.enabled {
            return;
        }
        let d = &self.cfg.diagnostics;
        let ratio = if frames + dropped > 0 {
            dropped as f64 / (frames + dropped) as f64
        } else {
            0.0
        };
        let rate = frames as f64 / elapsed;
        let (level, message) = match self.state {
            "lost" => (DiagnosticStatus::ERROR, "NatNet stream lost".to_owned()),
            "waiting" => (DiagnosticStatus::WARN, "waiting for NatNet data".to_owned()),
            "finished" => (DiagnosticStatus::STALE, "replay finished".to_owned()),
            _ if frames == 0 && repeats > 0 => (
                DiagnosticStatus::WARN,
                format!(
                    "stream paused, repeating frame {}",
                    stats.last_frame_number.unwrap_or_default()
                ),
            ),
            _ if frames == 0 => (
                DiagnosticStatus::ERROR,
                "no frames in the last period".to_owned(),
            ),
            _ if ratio > d.drop_warn_ratio => (
                DiagnosticStatus::WARN,
                format!("{:.1}% frames dropped", ratio * 100.0),
            ),
            _ if stats.total_latency.is_some_and(|l| l > d.latency_warn_secs) => (
                DiagnosticStatus::WARN,
                format!(
                    "latency {:.1} ms",
                    stats.total_latency.unwrap_or_default() * 1e3
                ),
            ),
            _ => (DiagnosticStatus::OK, format!("streaming at {rate:.1} Hz")),
        };
        let ms = |v: Option<f64>| v.map_or_else(|| "n/a".to_owned(), |v| format!("{:.3}", v * 1e3));
        let info = self.processor.server_info();
        let model = self.processor.model();
        let hardware_id = info.map_or_else(|| "natnet".to_owned(), |i| i.application_name.clone());
        let mut values = vec![
            KeyValue::new("state", self.state),
            KeyValue::new("frame rate (Hz)", format!("{rate:.2}")),
            KeyValue::new("frames", stats.frames),
            KeyValue::new("dropped frames (sequence gaps)", stats.dropped_frames),
            KeyValue::new(
                "of which dropped in the bridge queue",
                self.source.dropped_frames(),
            ),
            KeyValue::new("late frames (discarded)", stats.late_frames),
            KeyValue::new("duplicate frames", stats.duplicate_frames),
            KeyValue::new("decode errors", stats.decode_errors),
            KeyValue::new("stream restarts", stats.resets),
            KeyValue::new("server latency (ms)", ms(stats.server_latency)),
            KeyValue::new("total latency (ms)", ms(stats.total_latency)),
            KeyValue::new("clock offset (ms)", ms(stats.clock_offset)),
            KeyValue::new(
                "rigid bodies tracked",
                format!("{}/{}", stats.tracked_rigid_bodies, stats.rigid_bodies),
            ),
            KeyValue::new("labeled markers", stats.labeled_markers),
            KeyValue::new("model rigid bodies", model.rigid_bodies.len()),
            KeyValue::new("model skeletons", model.skeletons.len()),
            KeyValue::new("stamp policy", format!("{:?}", self.cfg.timing.policy)),
        ];
        if let Some(i) = info {
            values.push(KeyValue::new("server version", i.server_version));
            values.push(KeyValue::new("protocol version", i.protocol_version));
            values.push(KeyValue::new(
                "clock frequency (Hz)",
                i.high_res_clock_frequency,
            ));
            values.push(KeyValue::new(
                "transport",
                if i.connection.multicast {
                    "multicast"
                } else {
                    "unicast"
                },
            ));
        }
        let prefix = format!("{}: ", ros.node().name());
        let mut status = vec![DiagnosticStatus {
            level,
            name: format!("{prefix}NatNet stream"),
            message,
            hardware_id: hardware_id.clone(),
            values,
        }];
        // Without new frames the last snapshot is old news.
        let stale = frames == 0;
        if let Some(snap) = &self.last_snapshot {
            for b in &snap.bodies {
                let (level, message) = match (stale, b.tracked) {
                    (true, _) => (DiagnosticStatus::STALE, "no new frames"),
                    (false, true) => (DiagnosticStatus::OK, "tracked"),
                    (false, false) => (DiagnosticStatus::WARN, "not tracked"),
                };
                status.push(DiagnosticStatus {
                    level,
                    name: format!("{prefix}rigid body {}", b.name),
                    message: message.into(),
                    hardware_id: hardware_id.clone(),
                    values: vec![
                        KeyValue::new("id", b.id),
                        KeyValue::new("motive name", &b.motive_name),
                        KeyValue::new(
                            "mean marker error (mm)",
                            format!("{:.3}", f64::from(b.mean_error) * 1e3),
                        ),
                    ],
                });
            }
        }
        ros.publish_diagnostics(status);
    }
}
