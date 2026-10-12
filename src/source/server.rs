//! A simulated NatNet server (Motive stand-in) streaming the synthetic scene
//! or a recorded capture, over multicast or unicast.
//!
//! It answers `NAT_CONNECT` with SERVERINFO, `NAT_REQUEST_MODELDEF` with the
//! model definition, `NAT_REQUEST_FRAMEOFDATA` with the latest frame and
//! `NAT_REQUEST` with a success response. Unicast clients (up to 64) receive
//! frames on their command socket and are forgotten when their keepalives
//! stop.
//!
//! A unicast scene server honours the bitstream version a `NAT_CONNECT`
//! requests, as the vendor SDK client expects of a NatNet 3+ server: frames
//! and the model are then encoded with it while SERVERINFO keeps reporting
//! the scene's own version. It streams one version at a time, so the latest
//! hello decides, and a plain one restores the scene's version; a capture, a
//! `model_file` and multicast are sent as they are.

use std::borrow::Cow;
use std::io;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use natnet::command::ConnectOptions;
use serde::{Deserialize, Serialize};
use socket2::{Domain, Protocol, Socket, Type};
use tracing::{debug, info, warn};

use super::NAT_KEEPALIVE;
use super::capture::{CaptureReader, MAX_PAUSE_NS};
use super::synthetic::{Pacer, Scene, SyntheticConfig};

/// Unicast clients streamed to at once.
const MAX_CLIENTS: usize = 64;

/// Simulated server settings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ServerConfig {
    /// Address to bind the command socket to.
    pub bind: Ipv4Addr,
    /// Command port (0: ephemeral).
    pub command_port: u16,
    /// Data port announced in SERVERINFO and used for multicast.
    pub data_port: u16,
    /// Stream by multicast rather than unicast.
    pub multicast: bool,
    /// Multicast group.
    pub multicast_group: Ipv4Addr,
    /// Outgoing multicast interface (unspecified: the system default).
    pub multicast_interface: Ipv4Addr,
    /// Multicast TTL.
    pub multicast_ttl: u32,
    /// Forget unicast clients without a keepalive for this long, ms.
    pub client_timeout_ms: u64,
    /// Stream this capture's FRAMEOFDATA datagrams (verbatim, at recorded
    /// pace) instead of the synthetic scene. The scene still provides
    /// SERVERINFO; set its protocol version to the capture's. The model
    /// served is `model_file`, else the capture's recorded one, else empty.
    pub capture: Option<PathBuf>,
    /// A raw MODELDEF datagram to serve instead of the scene's or the
    /// capture's model.
    pub model_file: Option<PathBuf>,
    /// Restart the capture at its end.
    pub loop_capture: bool,
    /// The synthetic scene.
    pub scene: SyntheticConfig,
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            bind: Ipv4Addr::UNSPECIFIED,
            command_port: 1510,
            data_port: 1511,
            multicast: true,
            multicast_group: Ipv4Addr::new(239, 255, 42, 99),
            multicast_interface: Ipv4Addr::UNSPECIFIED,
            multicast_ttl: 1,
            client_timeout_ms: 5000,
            capture: None,
            model_file: None,
            loop_capture: true,
            scene: SyntheticConfig::default(),
        }
    }
}

/// A running simulated server.
#[derive(Debug)]
pub struct SimServer {
    command_addr: SocketAddrV4,
    stop: Arc<AtomicBool>,
    frames: Arc<AtomicU64>,
    thread: Option<JoinHandle<()>>,
}

impl SimServer {
    /// Binds the sockets and starts streaming.
    ///
    /// # Errors
    /// Socket errors, an invalid scene, or an unreadable capture.
    pub fn spawn(cfg: ServerConfig) -> io::Result<Self> {
        let invalid = |e: String| io::Error::new(io::ErrorKind::InvalidInput, e);
        let scene = Scene::new(cfg.scene.clone()).map_err(invalid)?;
        let command = UdpSocket::bind(SocketAddrV4::new(cfg.bind, cfg.command_port))?;
        let SocketAddr::V4(command_addr) = command.local_addr()? else {
            return Err(invalid("not an IPv4 socket".into()));
        };
        let data = if cfg.multicast {
            let s = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
            s.set_multicast_ttl_v4(cfg.multicast_ttl)?;
            s.set_multicast_loop_v4(true)?;
            if !cfg.multicast_interface.is_unspecified() {
                s.set_multicast_if_v4(&cfg.multicast_interface)?;
            }
            s.bind(&SocketAddrV4::new(cfg.bind, 0).into())?;
            Some(UdpSocket::from(s))
        } else {
            None
        };
        let feed = match &cfg.capture {
            Some(path) => Feed::capture(path, cfg.loop_capture)?,
            None => Feed::Scene {
                pacer: Pacer::new(scene.config().rate_hz),
            },
        };
        let stop = Arc::new(AtomicBool::new(false));
        let frames = Arc::new(AtomicU64::new(0));
        let mut state = State {
            cfg,
            stream: scene.clone(),
            restreamed: false,
            scene,
            command,
            data,
            clients: Vec::new(),
            feed,
            latest: None,
            model: None,
            frames: Arc::clone(&frames),
            send_failures: 0,
        };
        let version = state.scene.version();
        state.model = if let Some(path) = &state.cfg.model_file {
            Some(std::fs::read(path)?)
        } else {
            match &mut state.feed {
                Feed::Scene { .. } => state.scene.model_def().to_datagram(version).ok(),
                Feed::Capture { model, .. } => model.take().or_else(|| {
                    // The scene's model would misname the capture's assets.
                    natnet::ModelDef {
                        datasets: Vec::new(),
                    }
                    .to_datagram(version)
                    .ok()
                }),
            }
        };
        let thread = {
            let stop = Arc::clone(&stop);
            std::thread::Builder::new()
                .name("natnet-sim".into())
                .spawn(move || state.run(&stop))?
        };
        info!("simulated NatNet server on {command_addr}");
        Ok(Self {
            command_addr,
            stop,
            frames,
            thread: Some(thread),
        })
    }

    /// The bound command address.
    #[must_use]
    pub fn command_addr(&self) -> SocketAddrV4 {
        self.command_addr
    }

    /// Frames sent so far.
    #[must_use]
    pub fn frames_sent(&self) -> u64 {
        self.frames.load(Ordering::Relaxed)
    }

    /// Whether the server thread is still running.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.thread.as_ref().is_some_and(|t| !t.is_finished())
    }

    /// Stops streaming and waits for the thread.
    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for SimServer {
    fn drop(&mut self) {
        self.stop();
    }
}

enum Feed {
    Scene {
        pacer: Pacer,
    },
    Capture {
        path: PathBuf,
        looping: bool,
        reader: CaptureReader,
        model: Option<Vec<u8>>,
        /// Recorded time and wall time the pacing counts from.
        origin: Option<(u64, Instant)>,
        /// Recorded time of the previous frame.
        last: Option<u64>,
        pending: Option<(u64, Vec<u8>)>,
        /// Whether this pass over the file found a frame, so looping over
        /// one without any stops.
        pass_had_frames: bool,
        /// Set once the capture ended (or failed) for good.
        finished: bool,
    },
}

impl Feed {
    fn capture(path: &std::path::Path, looping: bool) -> io::Result<Self> {
        // Look ahead for a MODELDEF datagram to serve.
        let mut model = None;
        for record in CaptureReader::open(path)?.take(100_000) {
            // A truncated tail ends the look-ahead, not the server.
            let Ok(record) = record else { break };
            if record.bytes.get(..2) == Some(&natnet::NAT_MODELDEF.to_le_bytes()) {
                model = Some(record.bytes);
                break;
            }
        }
        Ok(Self::Capture {
            path: path.to_owned(),
            looping,
            reader: CaptureReader::open(path)?,
            model,
            origin: None,
            last: None,
            pending: None,
            pass_had_frames: false,
            finished: false,
        })
    }
}

struct State {
    cfg: ServerConfig,
    scene: Scene,
    /// The scene frames and the model are encoded from: `scene`, or a copy
    /// at a client's requested bitstream version.
    stream: Scene,
    /// The stream changed version since the pending frame was encoded.
    restreamed: bool,
    command: UdpSocket,
    data: Option<UdpSocket>,
    clients: Vec<(SocketAddr, Instant)>,
    feed: Feed,
    latest: Option<Vec<u8>>,
    model: Option<Vec<u8>>,
    frames: Arc<AtomicU64>,
    send_failures: u64,
}

impl State {
    fn run(&mut self, stop: &AtomicBool) {
        let mut buf = vec![0_u8; 65_536];
        while !stop.load(Ordering::Relaxed) {
            let (due, frame) = match self.next_frame() {
                Some(next) => next,
                None => {
                    // Capture finished without looping: keep answering commands.
                    (Instant::now() + Duration::from_millis(50), None)
                }
            };
            // Serve commands until the frame is due.
            loop {
                let now = Instant::now();
                if now >= due {
                    break;
                }
                let wait = (due - now)
                    .min(Duration::from_millis(50))
                    .max(Duration::from_micros(100));
                let _ = self.command.set_read_timeout(Some(wait));
                match self.command.recv_from(&mut buf) {
                    Ok((n, from)) => {
                        let request = buf.get(..n).unwrap_or_default().to_vec();
                        self.on_command(&request, from);
                    }
                    Err(e)
                        if matches!(
                            e.kind(),
                            io::ErrorKind::WouldBlock
                                | io::ErrorKind::TimedOut
                                | io::ErrorKind::Interrupted
                        ) => {}
                    Err(e) => {
                        // ICMP errors from departed unicast clients.
                        debug!("command receive: {e}");
                    }
                }
                if stop.load(Ordering::Relaxed) {
                    return;
                }
            }
            // A frame encoded before the stream changed version is dropped.
            let stale = std::mem::take(&mut self.restreamed);
            if let Some(frame) = frame.filter(|_| !stale) {
                self.broadcast(&frame);
                self.latest = Some(frame);
            }
        }
    }

    /// The next frame and when it is due.
    fn next_frame(&mut self) -> Option<(Instant, Option<Vec<u8>>)> {
        match &mut self.feed {
            Feed::Scene { pacer } => {
                let (n, due) = pacer.next();
                match self.stream.frame_datagram(n) {
                    Ok(d) => Some((due, Some(d))),
                    Err(e) => {
                        warn!("cannot encode frame {n}: {e}");
                        Some((due, None))
                    }
                }
            }
            Feed::Capture {
                path,
                looping,
                reader,
                origin,
                last,
                pending,
                pass_had_frames,
                finished,
                ..
            } => {
                if *finished {
                    return None;
                }
                let (t, bytes) = if let Some(p) = pending.take() {
                    p
                } else {
                    loop {
                        match reader.next() {
                            Some(Ok(r))
                                if r.bytes.get(..2)
                                    == Some(&natnet::NAT_FRAMEOFDATA.to_le_bytes()) =>
                            {
                                *pass_had_frames = true;
                                break (r.time_ns, r.bytes);
                            }
                            Some(Ok(_)) => {}
                            Some(Err(e)) => {
                                warn!("capture read failed: {e}");
                                *finished = true;
                                return None;
                            }
                            None => {
                                // Clean EOF can restart a nonempty looping
                                // capture; a read error terminates above.
                                if !(*looping && *pass_had_frames) {
                                    if *looping {
                                        warn!("the capture has no frames to loop over");
                                    } else {
                                        info!("capture finished");
                                    }
                                    *finished = true;
                                    return None;
                                }
                                match CaptureReader::open(path) {
                                    Ok(r) => {
                                        *reader = r;
                                        *origin = None;
                                        *last = None;
                                        *pass_had_frames = false;
                                    }
                                    Err(e) => {
                                        warn!("cannot reopen the capture: {e}");
                                        *finished = true;
                                        return None;
                                    }
                                }
                            }
                        }
                    }
                };
                // Long pauses in the recording are skipped, not waited out;
                // after a step back, pacing restarts.
                if let Some(l) = *last {
                    if t.saturating_sub(l) > MAX_PAUSE_NS {
                        warn!(
                            "skipping a {:.0} s pause in the capture",
                            t.saturating_sub(l) as f64 * 1e-9
                        );
                        *origin = None;
                    } else if t < l {
                        warn!("the capture's time steps back; pacing restarts");
                        *origin = None;
                    }
                }
                *last = Some(t);
                let (t0, wall0) = *origin.get_or_insert((t, Instant::now()));
                let due = wall0 + Duration::from_nanos(t.saturating_sub(t0));
                Some((due, Some(bytes)))
            }
        }
    }

    /// Forgets unicast clients that sent no hello or keepalive for
    /// `client_timeout_ms`.
    fn expire_clients(&mut self) {
        let timeout = Duration::from_millis(self.cfg.client_timeout_ms);
        self.clients.retain(|(addr, seen)| {
            let alive = seen.elapsed() < timeout;
            if !alive {
                info!("unicast client {addr} timed out");
            }
            alive
        });
    }

    fn broadcast(&mut self, frame: &[u8]) {
        self.expire_clients();
        // Only frames that left the host count as sent.
        let mut sent = false;
        let mut failure = None;
        if let Some(data) = &self.data {
            let group = SocketAddrV4::new(self.cfg.multicast_group, self.cfg.data_port);
            match data.send_to(frame, group) {
                Ok(_) => sent = true,
                Err(e) => failure = Some(format!("multicast send failed: {e}")),
            }
        } else {
            for (addr, _) in &self.clients {
                match self.command.send_to(frame, addr) {
                    Ok(_) => sent = true,
                    // Departed clients time out; not worth a warning.
                    Err(e) => debug!("unicast send to {addr} failed: {e}"),
                }
            }
        }
        if sent {
            self.frames.fetch_add(1, Ordering::Relaxed);
        }
        if let Some(failure) = failure {
            self.send_failures += 1;
            // Warn at powers of two so a persistent failure does not flood.
            if self.send_failures.is_power_of_two() {
                warn!("{failure} ({} times so far)", self.send_failures);
            } else {
                debug!("{failure}");
            }
        }
    }

    /// Records a unicast client's hello or keepalive; false when the client
    /// is new and the server already has [`MAX_CLIENTS`]. Multicast needs
    /// no list.
    fn touch(&mut self, from: SocketAddr) -> bool {
        if self.cfg.multicast {
            info!("client {from} connected");
        } else if let Some(c) = self.clients.iter_mut().find(|(a, _)| *a == from) {
            c.1 = Instant::now();
        } else if self.clients.len() < MAX_CLIENTS {
            info!("client {from} connected");
            self.clients.push((from, Instant::now()));
        } else {
            // A flood of (possibly spoofed) hellos must not multiply the
            // frames this server sends.
            debug!("client {from} ignored: {MAX_CLIENTS} clients connected");
            return false;
        }
        true
    }

    /// Streams the bitstream version a `NAT_CONNECT` asks for, where this
    /// server can; a hello without one (or with 0.0.0.0, "most current")
    /// restores the scene's own.
    fn honour_request(&mut self, request: &[u8], from: SocketAddr) {
        let requested = request
            .get(4..)
            .and_then(ConnectOptions::parse)
            .and_then(|o| o.bitstream_version);
        if self.cfg.multicast || !matches!(self.feed, Feed::Scene { .. }) {
            if let Some(version) = requested {
                debug!("{from} requests bitstream {version}; streaming as is");
            }
            return;
        }
        let version = requested.unwrap_or(self.scene.version());
        if version == self.stream.version() {
            return;
        }
        let stream = if version == self.scene.version() {
            Ok(self.scene.clone())
        } else {
            Scene::new(SyntheticConfig {
                protocol_version: version.to_string(),
                ..self.scene.config().clone()
            })
        };
        match stream {
            Ok(stream) => {
                info!("{from} connected; streaming bitstream {version}");
                if self.cfg.model_file.is_none() {
                    self.model = stream.model_def().to_datagram(version).ok();
                }
                self.stream = stream;
                self.restreamed = true;
                self.latest = None;
            }
            Err(e) => warn!(
                "{from} requests bitstream {version}: {e}; streaming {}",
                self.stream.version()
            ),
        }
    }

    fn on_command(&mut self, request: &[u8], from: SocketAddr) {
        if request.len() < 4 {
            return;
        }
        // Not only when a frame goes out: a server with no frames to send
        // must still free the places of departed clients.
        self.expire_clients();
        let id = u16::from_le_bytes([request[0], request[1]]);
        let reply = match id {
            // A client over the limit is not answered, so it retries.
            natnet::NAT_CONNECT if !self.touch(from) => None,
            natnet::NAT_CONNECT => {
                self.honour_request(request, from);
                let group = self.cfg.multicast_group.octets();
                self.scene
                    .server_info(self.cfg.multicast, self.cfg.data_port, group)
                    .to_datagram()
                    .ok()
                    .map(Cow::Owned)
            }
            natnet::NAT_REQUEST_MODELDEF => self.model.as_deref().map(Cow::Borrowed),
            natnet::NAT_REQUEST_FRAMEOFDATA => self.latest.as_deref().map(Cow::Borrowed),
            natnet::NAT_REQUEST => {
                let text = String::from_utf8_lossy(request.get(4..).unwrap_or_default());
                debug!("request {:?} from {from}", text.trim_end_matches('\0'));
                let mut r = natnet::NAT_RESPONSE.to_le_bytes().to_vec();
                r.extend_from_slice(&4_u16.to_le_bytes());
                r.extend_from_slice(&0_i32.to_le_bytes());
                Some(Cow::Owned(r))
            }
            NAT_KEEPALIVE => {
                if self.clients.iter().any(|(a, _)| *a == from) {
                    self.touch(from);
                }
                None
            }
            other => {
                debug!("ignoring message {other} from {from}");
                None
            }
        };
        if let Some(reply) = reply
            && let Err(e) = self.command.send_to(reply.as_ref(), from)
        {
            debug!("reply to {from} failed: {e}");
        }
    }
}

#[cfg(test)]
mod lifecycle_tests {
    use std::fs::{self, OpenOptions};
    use std::io::Write;
    use std::time::{SystemTime, UNIX_EPOCH};

    use super::*;

    #[test]
    fn repeated_raw_latest_frame_requests_preserve_retained_bytes() {
        let scene = Scene::new(SyntheticConfig::default()).unwrap();
        let command = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0)).unwrap();
        let command_addr = command.local_addr().unwrap();
        let client = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0)).unwrap();
        client
            .set_read_timeout(Some(Duration::from_secs(1)))
            .unwrap();
        let client_addr = client.local_addr().unwrap();
        let expected = scene.frame_datagram(42).unwrap();
        let pacer = Pacer::new(scene.config().rate_hz);
        let mut state = State {
            cfg: ServerConfig {
                bind: Ipv4Addr::LOCALHOST,
                multicast: false,
                ..ServerConfig::default()
            },
            stream: scene.clone(),
            scene,
            restreamed: false,
            command,
            data: None,
            clients: Vec::new(),
            feed: Feed::Scene { pacer },
            latest: Some(expected.clone()),
            model: None,
            frames: Arc::new(AtomicU64::new(0)),
            send_failures: 0,
        };
        let request = [
            natnet::NAT_REQUEST_FRAMEOFDATA.to_le_bytes(),
            0_u16.to_le_bytes(),
        ]
        .concat();
        let mut reply = vec![0_u8; 65_536];

        for _ in 0..3 {
            state.on_command(&request, client_addr);
            let (len, from) = client.recv_from(&mut reply).unwrap();
            assert_eq!(from, command_addr);
            assert_eq!(&reply[..len], expected.as_slice());
            assert_eq!(state.latest.as_deref(), Some(expected.as_slice()));
        }
    }

    #[test]
    fn looping_capture_feed_stops_at_corrupt_tail() {
        let path = std::env::temp_dir().join(format!(
            "natnet-zenoh-corrupt-server-{}-{}.natnet",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let mut writer = super::super::capture::CaptureWriter::create(
            &path,
            super::super::capture::Format::NatNet,
        )
        .unwrap();
        writer
            .write(&super::super::capture::Record {
                time_ns: 0,
                source: None,
                bytes: [
                    natnet::NAT_FRAMEOFDATA.to_le_bytes(),
                    0_u16.to_le_bytes(),
                    0_u16.to_le_bytes(),
                ]
                .concat(),
            })
            .unwrap();
        drop(writer);
        OpenOptions::new()
            .append(true)
            .open(&path)
            .unwrap()
            .write_all(&[1, 2, 3])
            .unwrap();

        let feed = Feed::capture(&path, true).unwrap();
        let scene = Scene::new(SyntheticConfig::default()).unwrap();
        let command = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0)).unwrap();
        let mut state = State {
            cfg: ServerConfig::default(),
            stream: scene.clone(),
            scene,
            restreamed: false,
            command,
            data: None,
            clients: Vec::new(),
            feed,
            latest: None,
            model: None,
            frames: Arc::new(AtomicU64::new(0)),
            send_failures: 0,
        };

        let first = state.next_frame();
        let second = state.next_frame();
        let finished = matches!(&state.feed, Feed::Capture { finished: true, .. });
        let first_is_frame = first
            .and_then(|(_, frame)| frame)
            .is_some_and(|frame| frame.starts_with(&natnet::NAT_FRAMEOFDATA.to_le_bytes()));
        let cleanup = fs::remove_file(path);

        assert!(
            cleanup.is_ok(),
            "capture fixture cleanup failed: {cleanup:?}"
        );
        assert!(first_is_frame, "first capture record was not served");
        assert!(second.is_none(), "corrupt capture tail restarted the feed");
        assert!(finished, "corrupt capture feed was not marked finished");
    }
}
