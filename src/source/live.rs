//! A live NatNet client.
//!
//! With a server address the client performs the command handshake
//! (`NAT_CONNECT`, SERVERINFO, `NAT_REQUEST_MODELDEF`), follows the transport
//! the server reports, and repeats the handshake whenever the stream stops:
//!
//! * multicast: frames arrive on the joined group; a dedicated thread
//!   receives them;
//! * unicast: frames arrive on the command socket, which is kept alive with
//!   `NAT_KEEPALIVE`.
//!
//! With multicast possible, the configured group is joined at once, so frames
//! flow even while Motive's command engine does not answer; they are decoded
//! as protocol 4.1 (or `protocol_version`) until a SERVERINFO arrives.
//! Handshake retries back off, and the model is polled while streaming,
//! since Motive does not flag model changes in its frames.
//!
//! Without a server address the client listens to the multicast group and,
//! unless passive, discovers the server from the first frame's source and
//! then performs the handshake with it; a lost stream restarts discovery. A
//! passive client never sends; it decodes with the configured protocol
//! version and clock frequency.

use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime};

use natnet::command::ConnectOptions;
use natnet::net::NAT_UNRECOGNIZED_REQUEST;
use natnet::{ConnectionInfo, ServerInfo, Version};
use serde::{Deserialize, Serialize};
use socket2::{Domain, Protocol, Socket, Type};
use tracing::{debug, info, warn};

use super::dispatch::{self, Dispatcher, Outcome, Policy};
use super::synthetic::parse_version;
use super::{ConnectionState, NAT_KEEPALIVE, SourceCommand, SourceContext, SourceEvent};

/// How frames are delivered.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Transport {
    /// Whatever SERVERINFO reports.
    #[default]
    Auto,
    /// Join the multicast group.
    Multicast,
    /// Receive on the command socket.
    Unicast,
}

/// Live client settings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LiveConfig {
    /// Motive's address; empty to discover it from the multicast stream.
    pub server: Option<Ipv4Addr>,
    /// Local IPv4 interface address; `0.0.0.0` automatically joins the group
    /// on every eligible active IPv4 address, while a nonzero address selects
    /// only that interface.
    pub interface: Ipv4Addr,
    /// Server command port.
    pub command_port: u16,
    /// Data port (overridden by SERVERINFO when it reports one).
    pub data_port: u16,
    /// Multicast group (overridden by SERVERINFO when it reports one).
    pub multicast_group: Ipv4Addr,
    /// Frame transport.
    pub transport: Transport,
    /// Never send anything; requires multicast.
    pub passive: bool,
    /// Protocol version; forces the decoder when set. Passive mode defaults
    /// to 4.1.
    pub protocol_version: Option<String>,
    /// Bitstream version to request in `NAT_CONNECT`, as the vendor SDK
    /// client does; frames are then decoded with it whenever the server
    /// speaks NatNet 3 or later. Unicast only: a multicast stream has one
    /// version for every client. Unset sends the plain hello. Tested against
    /// the simulated server only, not yet against live Motive.
    pub request_bitstream: Option<String>,
    /// Camera clock frequency for passive mode, Hz.
    pub clock_frequency: u64,
    /// Validation policy.
    pub parse_policy: Policy,
    /// A raw MODELDEF datagram to use until (or instead of) the server's.
    pub model_file: Option<PathBuf>,
    /// Initial handshake retry interval, ms; retries back off to 30 s (or
    /// this interval, when longer).
    pub connect_interval_ms: u64,
    /// Unicast keepalive interval, ms.
    pub keepalive_interval_ms: u64,
    /// No frame for this long means the stream is lost, ms.
    pub data_timeout_ms: u64,
    /// Accept multicast datagrams from the server address only.
    pub filter_source: bool,
    /// Requested socket receive buffer, bytes (0: system default).
    pub receive_buffer: usize,
    /// Request the model this often while streaming, s (0: never); an
    /// unchanged model is not republished.
    pub model_poll_secs: u64,
}

impl Default for LiveConfig {
    fn default() -> Self {
        Self {
            server: None,
            interface: Ipv4Addr::UNSPECIFIED,
            command_port: 1510,
            data_port: 1511,
            multicast_group: Ipv4Addr::new(239, 255, 42, 99),
            transport: Transport::Auto,
            passive: false,
            protocol_version: None,
            request_bitstream: None,
            clock_frequency: 10_000_000,
            parse_policy: Policy::Compatible,
            model_file: None,
            connect_interval_ms: 1000,
            keepalive_interval_ms: 250,
            data_timeout_ms: 1000,
            filter_source: false,
            receive_buffer: 4 << 20,
            model_poll_secs: 30,
        }
    }
}

impl LiveConfig {
    /// Checks the settings for contradictions.
    ///
    /// # Errors
    /// A message naming the problem.
    pub fn validate(&self) -> Result<(), String> {
        if self.passive && self.transport == Transport::Unicast {
            return Err("passive mode requires multicast".into());
        }
        if self.server.is_none() && self.transport == Transport::Unicast {
            // Discovery listens for multicast frames, which a unicast
            // server never sends.
            return Err("unicast transport requires `server`".into());
        }
        if self.command_port == 0 || self.data_port == 0 {
            return Err("command_port and data_port must be non-zero".into());
        }
        if !self.multicast_group.is_multicast() {
            return Err(format!("{} is not a multicast group", self.multicast_group));
        }
        if let Some(v) = &self.protocol_version {
            parse_version(v)?;
        }
        if let Some(v) = &self.request_bitstream {
            parse_version(v)?;
            if self.passive {
                return Err(
                    "request_bitstream needs a handshake, which passive mode never sends".into(),
                );
            }
            if self.transport != Transport::Unicast {
                return Err("request_bitstream requires unicast transport".into());
            }
            if self.protocol_version.is_some() {
                return Err("set request_bitstream or protocol_version, not both".into());
            }
        }
        if self.connect_interval_ms == 0
            || self.data_timeout_ms == 0
            || self.keepalive_interval_ms == 0
        {
            return Err("intervals and timeouts must be positive".into());
        }
        Ok(())
    }
}

/// The NAT_KEEPALIVE datagram (id 10, empty payload, trailing NUL as the
/// vendor client sends it).
#[must_use]
pub fn keepalive_datagram() -> [u8; 5] {
    let id = NAT_KEEPALIVE.to_le_bytes();
    [id[0], id[1], 0, 0, 0]
}

fn tune(socket: &Socket, receive_buffer: usize) {
    if receive_buffer > 0
        && let Err(e) = socket.set_recv_buffer_size(receive_buffer)
    {
        debug!("could not set receive buffer to {receive_buffer} bytes: {e}");
    }
}

/// Filters and orders candidate addresses independently of host enumeration
/// order so automatic membership attempts are predictable.
#[cfg(any(unix, windows, test))]
fn eligible_ipv4_addresses(
    interfaces: impl IntoIterator<Item = (Ipv4Addr, bool)>,
) -> Vec<Ipv4Addr> {
    let mut addresses: Vec<_> = interfaces
        .into_iter()
        .filter_map(|(ip, up)| {
            (up && !ip.is_unspecified() && !ip.is_multicast() && !ip.is_broadcast()).then_some(ip)
        })
        .collect();
    addresses.sort_unstable();
    addresses.dedup();
    addresses
}

fn select_interfaces(
    interface: Ipv4Addr,
    enumerate: impl FnOnce() -> io::Result<Vec<Ipv4Addr>>,
) -> io::Result<(Vec<Ipv4Addr>, bool)> {
    if !interface.is_unspecified() {
        return Ok((vec![interface], false));
    }
    let interfaces = enumerate()?;
    Ok((interfaces, true))
}

fn join_candidates<E>(
    interfaces: &[Ipv4Addr],
    mut join: impl FnMut(Ipv4Addr) -> Result<(), E>,
) -> (Vec<Ipv4Addr>, Vec<(Ipv4Addr, E)>) {
    let mut joined = Vec::new();
    let mut failures = Vec::new();
    for interface in interfaces {
        match join(*interface) {
            Ok(()) => joined.push(*interface),
            Err(e) => failures.push((*interface, e)),
        }
    }
    (joined, failures)
}

#[cfg(any(unix, windows))]
fn automatic_interfaces() -> io::Result<Vec<Ipv4Addr>> {
    if_addrs::get_if_addrs().map(|interfaces| {
        eligible_ipv4_addresses(interfaces.into_iter().filter_map(|interface| {
            let is_up = interface.is_oper_up();
            match interface.addr {
                if_addrs::IfAddr::V4(addr) => Some((addr.ip, is_up)),
                if_addrs::IfAddr::V6(_) => None,
            }
        }))
    })
}

#[cfg(not(any(unix, windows)))]
fn automatic_interfaces() -> io::Result<Vec<Ipv4Addr>> {
    Err(io::Error::new(
        io::ErrorKind::Unsupported,
        "automatic IPv4 interface enumeration is unsupported on this platform",
    ))
}

fn local_multicast_sender(
    automatic: bool,
    server_route: Option<Ipv4Addr>,
    group_route: Option<Ipv4Addr>,
) -> Option<Ipv4Addr> {
    (automatic && server_route.is_some_and(|ip| ip.is_loopback()))
        .then_some(group_route)
        .flatten()
}

fn accepts_multicast_source(
    filter: bool,
    server: Option<Ipv4Addr>,
    source: Ipv4Addr,
    local_sender: Option<Ipv4Addr>,
) -> bool {
    !filter || server.is_none_or(|server| server == source || local_sender == Some(source))
}

/// Longest handshake retry interval, unless `connect_interval_ms` is longer.
const MAX_CONNECT_BACKOFF: Duration = Duration::from_secs(30);

/// How long to wait for a SERVERINFO before decoding frames anyway.
const SERVER_INFO_GRACE: Duration = Duration::from_secs(3);

/// The local address the host routes `server` through; no packet is sent.
fn route_to(server: Ipv4Addr) -> Option<Ipv4Addr> {
    let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).ok()?;
    socket.connect((server, 1510)).ok()?;
    match socket.local_addr().ok()?.ip() {
        IpAddr::V4(ip) if !ip.is_unspecified() => Some(ip),
        _ => None,
    }
}

/// Binds a multicast receiver with `SO_REUSEADDR`, plus `SO_REUSEPORT` on
/// macOS and the BSDs. It shares the port only with sockets that set the
/// same options, such as another bridge. A
/// socket bound to the port without it, such as one from std's
/// [`UdpSocket::bind`] (which `natnet`'s `DataClient::bind_multicast`
/// uses), makes whichever bind comes second fail with
/// [`io::ErrorKind::AddrInUse`]; seen on Linux 6.8 at port 1511 on
/// 2026-10-10. On Linux it binds the group address, so datagrams for other
/// groups on the same port stay out.
///
/// # Errors
/// OS errors from creating, binding or joining. A failed bind or join
/// names its step and address, and keeps the OS error's kind.
pub fn multicast_socket(
    interface: Ipv4Addr,
    group: Ipv4Addr,
    port: u16,
    receive_buffer: usize,
) -> io::Result<UdpSocket> {
    let socket = multicast_receiver(group, port, receive_buffer)?;
    socket
        .join_multicast_v4(&group, &interface)
        .map_err(|e| step_error(&format!("joining {group} on {interface}"), &e))?;
    Ok(socket.into())
}

fn multicast_receiver(group: Ipv4Addr, port: u16, receive_buffer: usize) -> io::Result<Socket> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_reuse_address(true)?;
    // BSD kernels, macOS included, share a wildcard-bound UDP port only
    // between sockets that all set SO_REUSEPORT as well.
    #[cfg(any(
        target_vendor = "apple",
        target_os = "freebsd",
        target_os = "netbsd",
        target_os = "openbsd",
        target_os = "dragonfly"
    ))]
    socket.set_reuse_port(true)?;
    tune(&socket, receive_buffer);
    let bind = if cfg!(target_os = "linux") {
        group
    } else {
        Ipv4Addr::UNSPECIFIED
    };
    socket
        .bind(&SocketAddrV4::new(bind, port).into())
        .map_err(|e| step_error(&format!("binding {bind}:{port}"), &e))?;
    Ok(socket)
}

fn multicast_socket_automatic(
    interfaces: &[Ipv4Addr],
    group: Ipv4Addr,
    port: u16,
    receive_buffer: usize,
) -> io::Result<(UdpSocket, Vec<Ipv4Addr>)> {
    if interfaces.is_empty() {
        return Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            "automatic multicast interface selection found no eligible active IPv4 addresses; set source.live.interface or --interface",
        ));
    }
    let socket = multicast_receiver(group, port, receive_buffer)?;
    let (joined, failures) = join_candidates(interfaces, |interface| {
        socket.join_multicast_v4(&group, &interface)
    });
    for (interface, error) in &failures {
        warn!("cannot join multicast {group} on {interface}: {error}");
    }
    if joined.is_empty() {
        return Err(io::Error::new(
            io::ErrorKind::AddrNotAvailable,
            format!(
                "automatic multicast join failed for {group}:{port} on all {} eligible interfaces ({}); set source.live.interface or --interface",
                interfaces.len(),
                failures
                    .iter()
                    .map(|(interface, error)| format!("{interface}: {error}"))
                    .collect::<Vec<_>>()
                    .join("; ")
            ),
        ));
    }
    Ok((socket.into(), joined))
}

/// `e` with the step that failed in front, keeping its kind.
fn step_error(step: &str, e: &io::Error) -> io::Error {
    io::Error::new(e.kind(), format!("{step}: {e}"))
}

fn command_socket(interface: Ipv4Addr, receive_buffer: usize) -> io::Result<UdpSocket> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    tune(&socket, receive_buffer);
    socket.bind(&SocketAddrV4::new(interface, 0).into())?;
    Ok(socket.into())
}

/// Where frames come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DataPath {
    Multicast { group: Ipv4Addr, port: u16 },
    Unicast,
}

/// Receives every datagram before decoding: its bytes, source address and
/// monotonic receive time.
pub type Tap = Arc<dyn Fn(&[u8], SocketAddrV4, Instant) + Send + Sync>;

struct Shared {
    ctx: SourceContext,
    tap: Option<Tap>,
    dispatcher: Mutex<Dispatcher>,
    last_frame: Mutex<Option<Instant>>,
    models_changed: AtomicBool,
    discovered: Mutex<Option<Ipv4Addr>>,
    closed: AtomicBool,
    warned_unrecognized: AtomicBool,
}

impl Shared {
    fn handle(
        &self,
        datagram: &[u8],
        from: SocketAddrV4,
        received: SystemTime,
        mono: Instant,
    ) -> Outcome {
        if let Some(tap) = &self.tap {
            tap(datagram, from, mono);
        }
        let outcome = self
            .dispatcher
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .dispatch(&self.ctx, datagram, received, mono);
        match &outcome {
            Outcome::Frame { models_changed } => {
                *self
                    .last_frame
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner) = Some(mono);
                if *models_changed {
                    self.models_changed.store(true, Ordering::Relaxed);
                }
            }
            Outcome::Closed => self.closed.store(true, Ordering::Relaxed),
            Outcome::Ignored(NAT_UNRECOGNIZED_REQUEST) => {
                if self.warned_unrecognized.swap(true, Ordering::Relaxed) {
                    debug!("the server did not recognize a request");
                } else {
                    warn!("the server did not recognize a request (a keepalive or model request?)");
                }
            }
            Outcome::Ignored(id) => debug!("ignoring NatNet message {id}"),
            _ => {}
        }
        outcome
    }

    fn done(&self) -> bool {
        self.ctx.stopped() || self.closed.load(Ordering::Relaxed)
    }
}

/// The multicast receiver thread.
struct DataThread {
    path: DataPath,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl DataThread {
    fn spawn(
        shared: &Arc<Shared>,
        cfg: &LiveConfig,
        interfaces: &[Ipv4Addr],
        automatic: bool,
        group: Ipv4Addr,
        port: u16,
    ) -> io::Result<Self> {
        let (socket, joined) = if automatic {
            multicast_socket_automatic(interfaces, group, port, cfg.receive_buffer)?
        } else {
            (
                multicast_socket(cfg.interface, group, port, cfg.receive_buffer)?,
                vec![cfg.interface],
            )
        };
        socket.set_read_timeout(Some(Duration::from_millis(100)))?;
        for interface in &joined {
            info!("joined multicast {group}:{port} on {interface}");
        }
        // A server on this host sends its multicast from the address the
        // kernel picks for the group, which the source filter accepts too.
        let local_sender =
            local_multicast_sender(automatic, cfg.server.and_then(route_to), route_to(group));
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let shared = Arc::clone(shared);
            let stop = Arc::clone(&stop);
            let (filter, server) = (cfg.filter_source, cfg.server);
            std::thread::Builder::new()
                .name("natnet-data".into())
                .spawn(move || {
                    let mut buf = vec![0_u8; 65_536];
                    while !stop.load(Ordering::Relaxed) && !shared.done() {
                        match socket.recv_from(&mut buf) {
                            Ok((n, from)) => {
                                let received = SystemTime::now();
                                let mono = Instant::now();
                                let std::net::SocketAddr::V4(from) = from else {
                                    continue;
                                };
                                let ip = *from.ip();
                                let is_frame = buf.get(..2)
                                    == Some(&natnet::NAT_FRAMEOFDATA.to_le_bytes()[..]);
                                let source = {
                                    let mut d = shared
                                        .discovered
                                        .lock()
                                        .unwrap_or_else(PoisonError::into_inner);
                                    // Only a frame identifies a server.
                                    if server.is_none() && d.is_none() && is_frame {
                                        *d = Some(ip);
                                    }
                                    server.or(*d)
                                };
                                if !accepts_multicast_source(filter, source, ip, local_sender) {
                                    continue;
                                }
                                shared.handle(&buf[..n], from, received, mono);
                            }
                            Err(e)
                                if matches!(
                                    e.kind(),
                                    io::ErrorKind::WouldBlock
                                        | io::ErrorKind::TimedOut
                                        | io::ErrorKind::Interrupted
                                ) => {}
                            Err(e) => {
                                warn!("multicast receive failed: {e}");
                                std::thread::sleep(Duration::from_millis(100));
                            }
                        }
                    }
                })?
        };
        Ok(Self {
            path: DataPath::Multicast { group, port },
            stop,
            thread: Some(thread),
        })
    }
}

impl Drop for DataThread {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// Runs the live client until stopped.
pub fn run(cfg: &LiveConfig, ctx: SourceContext, commands: &Receiver<SourceCommand>) {
    run_with_tap(cfg, ctx, commands, None);
}

/// Runs the live client, showing every received datagram to `tap`.
pub fn run_with_tap(
    cfg: &LiveConfig,
    ctx: SourceContext,
    commands: &Receiver<SourceCommand>,
    tap: Option<Tap>,
) {
    let forced = cfg
        .protocol_version
        .as_deref()
        .and_then(|v| parse_version(v).ok())
        .or(cfg.passive.then_some(Version::V4_1));
    let requested = cfg
        .request_bitstream
        .as_deref()
        .and_then(|v| parse_version(v).ok());
    let shared = Arc::new(Shared {
        ctx,
        tap,
        dispatcher: Mutex::new(Dispatcher::new(cfg.parse_policy, forced).with_request(requested)),
        last_frame: Mutex::new(None),
        models_changed: AtomicBool::new(false),
        discovered: Mutex::new(None),
        closed: AtomicBool::new(false),
        warned_unrecognized: AtomicBool::new(false),
    });
    if let Some(path) = &cfg.model_file {
        let version = forced.unwrap_or(Version::V4_1);
        match dispatch::load_modeldef(path, version, cfg.parse_policy) {
            Ok(md) => {
                shared.ctx.send(SourceEvent::ModelDef(md, version));
            }
            Err(e) => warn!("model file: {e}"),
        }
    }
    Client::new(cfg, shared).run(commands);
}

struct Client<'a> {
    cfg: &'a LiveConfig,
    shared: Arc<Shared>,
    command: Option<(UdpSocket, SocketAddrV4)>,
    data: Option<DataThread>,
    path: Option<DataPath>,
    info: Option<ServerInfo>,
    last_connect: Option<Instant>,
    last_keepalive: Instant,
    last_model_request: Option<Instant>,
    connect_backoff: Duration,
    need_model: bool,
    decoding_without_info: bool,
    streaming: bool,
    /// When the client started or last opened its command channel: the
    /// SERVERINFO grace period counts from the handshake it enables.
    started: Instant,
}

impl<'a> Client<'a> {
    fn new(cfg: &'a LiveConfig, shared: Arc<Shared>) -> Self {
        Self {
            cfg,
            shared,
            command: None,
            data: None,
            path: None,
            info: None,
            last_connect: None,
            last_keepalive: Instant::now(),
            last_model_request: None,
            connect_backoff: Self::ms(cfg.connect_interval_ms),
            need_model: true,
            decoding_without_info: false,
            streaming: false,
            started: Instant::now(),
        }
    }

    fn ms(v: u64) -> Duration {
        Duration::from_millis(v)
    }

    /// Sleeps up to `d`, returning early when the source stops, so a long
    /// retry interval does not hold up shutdown.
    fn pause(&self, d: Duration) {
        let until = Instant::now() + d;
        while !self.shared.done() {
            let left = until.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            std::thread::sleep(left.min(Duration::from_millis(50)));
        }
    }

    /// Whether frames may arrive by multicast before any SERVERINFO.
    fn listens_first(&self) -> bool {
        self.cfg.server.is_none() || self.cfg.passive || self.cfg.transport != Transport::Unicast
    }

    fn run(mut self, commands: &Receiver<SourceCommand>) {
        if self.listens_first() {
            // Listen first: passive, discovering the server, or ready for
            // multicast frames before the handshake completes.
            if self.cfg.passive {
                let info = self.passive_info();
                self.shared.ctx.send(SourceEvent::ServerInfo(info.clone()));
                self.info = Some(info);
            }
            let (group, port) = (self.cfg.multicast_group, self.cfg.data_port);
            if !self.ensure_path(DataPath::Multicast { group, port }) {
                return;
            }
        }
        let mut buf = vec![0_u8; 65_536];
        while !self.shared.done() {
            if !self.cfg.passive && self.command.is_none() {
                let server = self.cfg.server.or_else(|| {
                    *self
                        .shared
                        .discovered
                        .lock()
                        .unwrap_or_else(PoisonError::into_inner)
                });
                if let Some(server) = server {
                    self.open_command(server);
                }
            }
            for cmd in commands.try_iter() {
                match cmd {
                    SourceCommand::RefreshModel => {
                        self.request_model(true);
                    }
                }
            }
            // A change flagged just after a request is kept until the
            // debounce lets another request out.
            if self.shared.models_changed.swap(false, Ordering::Relaxed)
                && !self.request_model(false)
            {
                self.shared.models_changed.store(true, Ordering::Relaxed);
            }
            self.maybe_poll_model();
            self.maybe_decode_without_info();
            self.check_stream();
            self.maybe_connect();
            self.maybe_keepalive();
            if !self.receive_command(&mut buf) {
                std::thread::sleep(Duration::from_millis(50));
            }
        }
        if self.streaming && !self.shared.closed.load(Ordering::Relaxed) {
            debug!("live source stopping");
        }
    }

    fn passive_info(&self) -> ServerInfo {
        let version = self
            .shared
            .dispatcher
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .version()
            .unwrap_or(Version::V4_1);
        ServerInfo {
            application_name: "passive multicast".into(),
            server_version: Version::new(0, 0, 0, 0),
            protocol_version: version,
            high_res_clock_frequency: self.cfg.clock_frequency,
            connection: ConnectionInfo {
                data_port: self.cfg.data_port,
                multicast: true,
                multicast_group: self.cfg.multicast_group.octets(),
            },
        }
    }

    /// Interfaces to join, and whether they were selected automatically.
    fn interfaces(&self) -> io::Result<(Vec<Ipv4Addr>, bool)> {
        select_interfaces(self.cfg.interface, automatic_interfaces).map_err(|e| {
            io::Error::new(
                e.kind(),
                format!("cannot enumerate active IPv4 multicast interfaces: {e}; set source.live.interface or --interface"),
            )
        })
    }

    /// Requests the model periodically while streaming: Motive does not set
    /// the frames' model-change bit, so renamed or edited assets are only
    /// noticed this way.
    fn maybe_poll_model(&mut self) {
        let poll = Duration::from_secs(self.cfg.model_poll_secs);
        if self.cfg.model_poll_secs > 0
            && self.streaming
            && self.command.is_some()
            && self.info.is_some()
            && self.last_model_request.is_none_or(|t| t.elapsed() >= poll)
        {
            self.request_model(false);
        }
    }

    /// Decodes multicast frames with protocol 4.1 when a server does not
    /// answer the handshake: its command engine can stall while it keeps
    /// streaming. A later SERVERINFO takes over.
    fn maybe_decode_without_info(&mut self) {
        let known_server = self.cfg.server.is_some()
            || self
                .shared
                .discovered
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .is_some();
        if self.decoding_without_info
            || self.info.is_some()
            || !known_server
            || !matches!(self.path, Some(DataPath::Multicast { .. }))
            || self.started.elapsed() < SERVER_INFO_GRACE
        {
            return;
        }
        self.decoding_without_info = true;
        let mut dispatcher = self
            .shared
            .dispatcher
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if dispatcher.version().is_none() && dispatcher.adopt(Version::V4_1).is_err() {
            return;
        }
        drop(dispatcher);
        warn!(
            "no SERVERINFO after {} s; decoding multicast frames as protocol {} until one arrives",
            SERVER_INFO_GRACE.as_secs(),
            self.passive_info().protocol_version
        );
        let info = self.passive_info();
        self.shared.ctx.send(SourceEvent::ServerInfo(info));
    }

    fn open_command(&mut self, server: Ipv4Addr) {
        let interface = if self.cfg.interface.is_unspecified() {
            route_to(server).unwrap_or(Ipv4Addr::UNSPECIFIED)
        } else {
            self.cfg.interface
        };
        match command_socket(interface, self.cfg.receive_buffer).and_then(|s| {
            s.set_read_timeout(Some(Duration::from_millis(50)))?;
            Ok(s)
        }) {
            Ok(socket) => {
                let addr = SocketAddrV4::new(server, self.cfg.command_port);
                info!("NatNet command channel to {addr} from {interface}");
                self.command = Some((socket, addr));
                self.started = Instant::now();
            }
            Err(e) => {
                warn!("cannot open the command socket: {e}");
                self.pause(Self::ms(self.cfg.connect_interval_ms));
            }
        }
    }

    fn send(&self, datagram: &[u8]) {
        if let Some((socket, addr)) = &self.command
            && let Err(e) = socket.send_to(datagram, addr)
        {
            debug!("send to {addr} failed: {e}");
        }
    }

    /// Sends NAT_CONNECT while there is no SERVERINFO or no stream, backing
    /// off so a paused or unreachable server is not polled every second.
    fn maybe_connect(&mut self) {
        if self.command.is_none() || (self.info.is_some() && self.streaming) {
            return;
        }
        if self
            .last_connect
            .is_some_and(|t| t.elapsed() < self.connect_backoff)
        {
            return;
        }
        let requested = self
            .cfg
            .request_bitstream
            .as_deref()
            .and_then(|v| parse_version(v).ok());
        let request = match requested {
            Some(v) => natnet::command::connect_request_with(ConnectOptions::requesting(v)),
            None => natnet::command::connect_request(),
        };
        if let Ok(req) = request {
            if self.last_connect.is_some() {
                debug!(
                    "no {} yet; retrying NAT_CONNECT, next in at most {:?}",
                    if self.info.is_none() {
                        "SERVERINFO"
                    } else {
                        "frame"
                    },
                    self.connect_backoff * 2
                );
                let max = MAX_CONNECT_BACKOFF.max(Self::ms(self.cfg.connect_interval_ms));
                self.connect_backoff = (self.connect_backoff * 2).min(max);
            }
            self.send(req.as_bytes());
        }
        self.last_connect = Some(Instant::now());
    }

    fn maybe_keepalive(&mut self) {
        if self.path == Some(DataPath::Unicast)
            && self.last_keepalive.elapsed() >= Self::ms(self.cfg.keepalive_interval_ms)
        {
            self.send(&keepalive_datagram());
            self.last_keepalive = Instant::now();
        }
    }

    /// Requests the model definitions; false when the request was
    /// debounced and should be retried later.
    fn request_model(&mut self, force: bool) -> bool {
        if self.command.is_none() {
            debug!("model refresh requested but there is no command channel");
            return true;
        }
        // Debounce server-flagged changes; explicit requests always go out.
        if !force
            && self
                .last_model_request
                .is_some_and(|t| t.elapsed() < Duration::from_secs(1))
        {
            return false;
        }
        if let Ok(req) = natnet::command::request_modeldef() {
            self.send(req.as_bytes());
            self.last_model_request = Some(Instant::now());
        }
        true
    }

    /// Receives one datagram on the command socket; false when idle.
    fn receive_command(&mut self, buf: &mut [u8]) -> bool {
        let Some((socket, server)) = &self.command else {
            return false;
        };
        let server = *server;
        match socket.recv_from(buf) {
            Ok((n, from)) => {
                let received = SystemTime::now();
                let mono = Instant::now();
                if from.ip() != IpAddr::V4(*server.ip()) {
                    debug!("ignoring a command-socket datagram from {from}");
                    return true;
                }
                let data = buf.get(..n).unwrap_or_default();
                // Record the real source port: unicast frames come from the
                // server's data port, replies from its command port.
                let source = match from {
                    SocketAddr::V4(a) => a,
                    SocketAddr::V6(_) => server,
                };
                if let Outcome::ServerInfo(info) = self.shared.handle(data, source, received, mono)
                {
                    self.on_server_info(info, server);
                }
                true
            }
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                ) =>
            {
                true
            }
            Err(e) => {
                // ICMP port unreachable surfaces here on some systems while
                // the server is down.
                debug!("command receive failed: {e}");
                false
            }
        }
    }

    fn on_server_info(&mut self, info: ServerInfo, server: SocketAddrV4) {
        let changed = self.info.as_ref() != Some(&info) || self.decoding_without_info;
        self.decoding_without_info = false;
        if changed {
            info!(
                "{} {} at {server}: protocol {}, clock {} Hz, {}",
                info.application_name,
                info.server_version,
                info.protocol_version,
                info.high_res_clock_frequency,
                if info.connection.multicast {
                    "multicast"
                } else {
                    "unicast"
                }
            );
        } else {
            debug!("SERVERINFO from {server} unchanged");
        }
        let multicast = match self.cfg.transport {
            Transport::Auto => info.connection.multicast,
            Transport::Multicast => true,
            Transport::Unicast => false,
        };
        if self.cfg.transport != Transport::Auto && multicast != info.connection.multicast {
            warn!(
                "the server reports {} streaming but {:?} was configured",
                if info.connection.multicast {
                    "multicast"
                } else {
                    "unicast"
                },
                self.cfg.transport
            );
        }
        let path = if multicast {
            let group = Ipv4Addr::from(info.connection.multicast_group);
            let group = if group.is_multicast() {
                group
            } else {
                self.cfg.multicast_group
            };
            let port = if info.connection.data_port == 0 {
                self.cfg.data_port
            } else {
                info.connection.data_port
            };
            DataPath::Multicast { group, port }
        } else {
            DataPath::Unicast
        };
        self.info = Some(info);
        if !self.ensure_path(path) {
            return;
        }
        if changed || self.need_model {
            self.need_model = false;
            self.request_model(true);
        }
    }

    /// Switches the frame path; false when the source must stop.
    fn ensure_path(&mut self, path: DataPath) -> bool {
        if self.path == Some(path) {
            return true;
        }
        self.data = None;
        self.path = Some(path);
        match path {
            DataPath::Unicast => {
                info!("receiving unicast frames on the command socket");
                self.send(&keepalive_datagram());
                self.last_keepalive = Instant::now();
                true
            }
            DataPath::Multicast { group, port } => {
                let result = self.interfaces().and_then(|(interfaces, automatic)| {
                    DataThread::spawn(&self.shared, self.cfg, &interfaces, automatic, group, port)
                });
                match result {
                    Ok(t) => {
                        debug_assert_eq!(t.path, path);
                        self.data = Some(t);
                        true
                    }
                    Err(e) => {
                        let hint = if e.kind() == io::ErrorKind::AddrInUse {
                            " (another receiver on this host may hold the port without \
                             SO_REUSEADDR, or on macOS and the BSDs SO_REUSEPORT)"
                        } else {
                            ""
                        };
                        let msg = format!("cannot receive multicast {group}:{port}: {e}{hint}");
                        warn!("{msg}");
                        self.shared.ctx.report(SourceEvent::DecodeError(msg));
                        // Retry on the next SERVERINFO or loop.
                        self.path = None;
                        self.pause(Self::ms(self.cfg.connect_interval_ms));
                        !self.shared.done()
                    }
                }
            }
        }
    }

    fn check_stream(&mut self) {
        if self.path.is_none() && self.listens_first() {
            let (group, port) = (self.cfg.multicast_group, self.cfg.data_port);
            self.ensure_path(DataPath::Multicast { group, port });
        }
        let last = *self
            .shared
            .last_frame
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        let timeout = Self::ms(self.cfg.data_timeout_ms);
        let fresh = last.is_some_and(|t| t.elapsed() < timeout);
        if fresh && !self.streaming {
            self.streaming = true;
            self.connect_backoff = Self::ms(self.cfg.connect_interval_ms);
            info!("NatNet stream up");
            self.shared
                .ctx
                .send(SourceEvent::Connection(ConnectionState::Streaming));
        } else if !fresh && self.streaming {
            self.streaming = false;
            warn!("no NatNet frame for {} ms", self.cfg.data_timeout_ms);
            self.shared
                .ctx
                .send(SourceEvent::Connection(ConnectionState::Lost));
            // Re-handshake immediately; a restarted server has forgotten us
            // and may carry a different model.
            self.last_connect = None;
            self.connect_backoff = Self::ms(self.cfg.connect_interval_ms);
            self.need_model = true;
            if self.cfg.server.is_none() {
                // The server may have moved (DHCP); discover it again.
                *self
                    .shared
                    .discovered
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner) = None;
                // Passive mode keeps its synthesized server info.
                if !self.cfg.passive {
                    self.command = None;
                    self.info = None;
                    self.decoding_without_info = false;
                }
            }
        } else if last.is_none()
            && !self.streaming
            && self.started.elapsed() > timeout * 5
            && self.command.is_none()
            && !self.cfg.passive
            && self.cfg.server.is_none()
        {
            debug!("still waiting for a multicast datagram to discover the server");
            self.started = Instant::now();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eligible_interfaces_are_sorted_deduplicated_and_keep_loopback_and_link_local() {
        let candidates = eligible_ipv4_addresses([
            (Ipv4Addr::new(192, 0, 2, 4), true),
            (Ipv4Addr::new(169, 254, 2, 3), true),
            (Ipv4Addr::LOCALHOST, true),
            (Ipv4Addr::new(192, 0, 2, 4), true),
            (Ipv4Addr::new(203, 0, 113, 8), false),
            (Ipv4Addr::UNSPECIFIED, true),
            (Ipv4Addr::new(224, 0, 0, 1), true),
        ]);
        assert_eq!(
            candidates,
            [
                Ipv4Addr::LOCALHOST,
                Ipv4Addr::new(169, 254, 2, 3),
                Ipv4Addr::new(192, 0, 2, 4),
            ]
        );
    }

    #[test]
    fn explicit_interface_skips_automatic_enumeration() {
        let explicit = Ipv4Addr::new(192, 0, 2, 9);
        let (interfaces, automatic) =
            select_interfaces(explicit, || panic!("explicit interface must not enumerate"))
                .unwrap();
        assert_eq!(interfaces, [explicit]);
        assert!(!automatic);
    }

    #[test]
    fn source_filter_keeps_local_route_exception_only_for_local_configured_server() {
        let group_sender = Ipv4Addr::new(192, 0, 2, 10);
        let remote_route = Ipv4Addr::new(198, 51, 100, 20);
        let remote_exception = local_multicast_sender(true, Some(remote_route), Some(group_sender));
        assert_eq!(remote_exception, None);
        assert!(!accepts_multicast_source(
            true,
            Some(remote_route),
            group_sender,
            remote_exception
        ));

        let local_exception =
            local_multicast_sender(true, Some(Ipv4Addr::LOCALHOST), Some(group_sender));
        assert_eq!(local_exception, Some(group_sender));
        assert!(accepts_multicast_source(
            true,
            Some(Ipv4Addr::LOCALHOST),
            group_sender,
            local_exception
        ));
        assert!(!accepts_multicast_source(
            true,
            Some(Ipv4Addr::LOCALHOST),
            group_sender,
            None
        ));
    }

    #[test]
    fn multicast_membership_continues_after_failures_and_reports_all_failed() {
        let candidates = [Ipv4Addr::LOCALHOST, Ipv4Addr::new(192, 0, 2, 3)];
        let (joined, failed) = join_candidates(&candidates, |ip| {
            if ip.is_loopback() {
                Err("simulated join failure")
            } else {
                Ok(())
            }
        });
        assert_eq!(joined, [Ipv4Addr::new(192, 0, 2, 3)]);
        assert_eq!(failed, [(Ipv4Addr::LOCALHOST, "simulated join failure")]);

        let (joined, failed) = join_candidates(&candidates, |_| Err("join failed"));
        assert_eq!(joined, Vec::<Ipv4Addr>::new());
        assert_eq!(failed.len(), candidates.len());
        let (joined, failed): (Vec<Ipv4Addr>, Vec<(Ipv4Addr, &str)>) =
            join_candidates(&[], |_| Err("unreachable"));
        assert!(joined.is_empty() && failed.is_empty());
    }

    #[test]
    fn automatic_socket_reports_empty_candidates() {
        let error =
            multicast_socket_automatic(&[], Ipv4Addr::new(239, 255, 42, 99), 0, 0).unwrap_err();
        assert!(
            error
                .to_string()
                .contains("no eligible active IPv4 addresses")
        );
    }

    #[test]
    fn route_selected_command_binding_uses_server_route() {
        let route = route_to(Ipv4Addr::LOCALHOST).unwrap();
        let socket = command_socket(route, 0).unwrap();
        assert_eq!(socket.local_addr().unwrap().ip(), IpAddr::V4(route));
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn a_port_held_without_reuse_fails_at_the_bind() {
        // The holder is bound but never joins, and nothing is sent.
        let group = Ipv4Addr::new(239, 255, 42, 99);
        let holder = UdpSocket::bind((group, 0)).unwrap();
        let port = holder.local_addr().unwrap().port();
        let e = multicast_socket(Ipv4Addr::LOCALHOST, group, port, 0).unwrap_err();
        assert_eq!(e.kind(), io::ErrorKind::AddrInUse);
        assert!(
            e.to_string()
                .starts_with(&format!("binding {group}:{port}: ")),
            "{e}"
        );
    }
}
