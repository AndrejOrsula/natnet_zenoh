//! `natnet_zenoh`: bridge OptiTrack NatNet motion capture to Zenoh and ROS 2.

use std::io::IsTerminal;
use std::net::{Ipv4Addr, SocketAddrV4};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use clap::{Args, Parser, Subcommand, ValueEnum};
use tracing::{error, info, warn};
use tracing_subscriber::EnvFilter;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;

use natnet_zenoh::bridge::snapshot::Processor;
use natnet_zenoh::bridge::{Bridge, context_options, server_info_json};
use natnet_zenoh::config::{Config, SourceKind};
use natnet_zenoh::ros::context::Context;
use natnet_zenoh::ros::rosout::{RosoutHandle, RosoutLayer};
use natnet_zenoh::source::capture::{CaptureWriter, Format, Record};
use natnet_zenoh::source::live::{self, Transport};
use natnet_zenoh::source::server::{ServerConfig, SimServer};
use natnet_zenoh::source::{ConnectionState, SourceEvent, SourceHandle};

#[derive(Debug, Parser)]
#[command(version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Run the bridge.
    Run(RunArgs),
    /// Connect to a NatNet server, report what it streams, and exit.
    Probe(ProbeArgs),
    /// Record raw NatNet datagrams to a capture file.
    Record(RecordArgs),
    /// Run a simulated NatNet server (synthetic scene or capture playback).
    Serve(ServeArgs),
    /// Print the default configuration (TOML).
    Config,
    /// Validate a configuration file.
    Check {
        /// The configuration file.
        config: PathBuf,
    },
    /// List the ROS 2 types the bridge uses, with their RIHS01 hashes.
    Types,
}

/// `--up-axis` values.
#[derive(Debug, Clone, Copy, ValueEnum)]
enum UpAxisArg {
    /// Y up (Motive's default).
    Y,
    /// Z up.
    Z,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum TransportArg {
    Auto,
    Multicast,
    Unicast,
}

impl From<TransportArg> for Transport {
    fn from(t: TransportArg) -> Self {
        match t {
            TransportArg::Auto => Self::Auto,
            TransportArg::Multicast => Self::Multicast,
            TransportArg::Unicast => Self::Unicast,
        }
    }
}

/// Live-source options shared by several subcommands.
#[derive(Debug, Args, Clone)]
struct LiveArgs {
    /// Configuration file (TOML); defaults apply without one. Falls back to
    /// a non-empty NATNET_ZENOH_CONFIG environment variable.
    #[arg(long, short = 'c')]
    config: Option<PathBuf>,
    /// Motive's address (omit to discover it from the multicast stream).
    #[arg(long, short = 's')]
    server: Option<Ipv4Addr>,
    /// Local interface address.
    #[arg(long, short = 'i')]
    interface: Option<Ipv4Addr>,
    /// Frame transport.
    #[arg(long, value_enum)]
    transport: Option<TransportArg>,
    /// Listen to multicast only and never send anything.
    #[arg(long)]
    passive: bool,
    /// Multicast group.
    #[arg(long)]
    multicast_group: Option<Ipv4Addr>,
    /// Server command port.
    #[arg(long)]
    command_port: Option<u16>,
    /// Data port.
    #[arg(long)]
    data_port: Option<u16>,
    /// Force the NatNet protocol version (e.g. 4.1) instead of the one the
    /// server announces; `run --replay` decodes the capture with it too.
    #[arg(long)]
    protocol_version: Option<String>,
    /// Ask the server for this bitstream version in NAT_CONNECT (e.g. 4.1)
    /// and decode with it, as the vendor SDK client does; unicast only.
    #[arg(long, conflicts_with_all = ["passive", "protocol_version"])]
    request_bitstream: Option<String>,
}

impl LiveArgs {
    /// `--config`, else `NATNET_ZENOH_CONFIG` unless it is empty (as
    /// `${CFG:-}` in a compose file leaves it).
    fn config_path(&self) -> Option<PathBuf> {
        self.config.clone().or_else(|| {
            std::env::var_os("NATNET_ZENOH_CONFIG")
                .filter(|v| !v.is_empty())
                .map(PathBuf::from)
        })
    }

    fn apply(&self, cfg: &mut Config) {
        let live = &mut cfg.source.live;
        if self.server.is_some() {
            live.server = self.server;
        }
        if let Some(i) = self.interface {
            live.interface = i;
        }
        if let Some(t) = self.transport {
            live.transport = t.into();
        }
        live.passive |= self.passive;
        if let Some(g) = self.multicast_group {
            live.multicast_group = g;
        }
        if let Some(p) = self.command_port {
            live.command_port = p;
        }
        if let Some(p) = self.data_port {
            live.data_port = p;
        }
        if self.protocol_version.is_some() {
            live.protocol_version.clone_from(&self.protocol_version);
        }
        if self.request_bitstream.is_some() {
            live.request_bitstream.clone_from(&self.request_bitstream);
        }
    }
}

#[derive(Debug, Args)]
struct RunArgs {
    #[command(flatten)]
    live: LiveArgs,
    /// Replay this capture instead of connecting to Motive.
    #[arg(long, conflicts_with_all = [
        "synthetic", "server", "passive", "transport", "interface", "multicast_group",
        "command_port", "data_port", "request_bitstream",
    ])]
    replay: Option<PathBuf>,
    /// Replay speed (0: as fast as possible).
    #[arg(long)]
    rate: Option<f64>,
    /// Loop the replay.
    #[arg(long = "loop")]
    looping: bool,
    /// A raw MODELDEF datagram for captures or passive streams without one.
    #[arg(long)]
    model_file: Option<PathBuf>,
    /// Stream the built-in synthetic scene.
    #[arg(long, conflicts_with_all = [
        "server", "passive", "transport", "interface", "multicast_group", "command_port",
        "data_port", "protocol_version", "model_file", "request_bitstream",
    ])]
    synthetic: bool,
    /// ROS domain id.
    #[arg(long)]
    domain_id: Option<u32>,
    /// ROS node name.
    #[arg(long)]
    node_name: Option<String>,
    /// ROS namespace.
    #[arg(long)]
    namespace: Option<String>,
    /// Fixed mocap world frame.
    #[arg(long)]
    world_frame: Option<String>,
    /// Up axis of the Motive stream (Motive's streaming "Up Axis" setting).
    #[arg(long)]
    up_axis: Option<UpAxisArg>,
    /// Disable the ROS 2 outputs.
    #[arg(long)]
    no_ros: bool,
    /// Enable the native zenoh outputs.
    #[arg(long)]
    native: bool,
    /// Zenoh session configuration file.
    #[arg(long)]
    zenoh_config: Option<PathBuf>,
    /// Zenoh endpoint to connect to (repeatable), e.g. tcp/10.0.0.2:7447.
    #[arg(long)]
    connect: Vec<String>,
    /// Stop after this many seconds; exits with an error if no frame came.
    #[arg(long, value_parser = parse_seconds)]
    duration: Option<Duration>,
}

#[derive(Debug, Args)]
struct ProbeArgs {
    #[command(flatten)]
    live: LiveArgs,
    /// Seconds to listen for frames.
    #[arg(long, short = 't', default_value = "3", value_parser = parse_seconds)]
    duration: Duration,
    /// Print JSON.
    #[arg(long)]
    json: bool,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum FormatArg {
    Natnet,
    Aucap,
}

#[derive(Debug, Args)]
struct RecordArgs {
    /// Output capture file.
    #[arg(long, short = 'o')]
    output: PathBuf,
    /// Capture format (NatNet by default).
    #[arg(long, value_enum, default_value_t = FormatArg::Natnet)]
    format: FormatArg,
    /// Stop after this many seconds (default: until Ctrl-C).
    #[arg(long, short = 't', value_parser = parse_seconds)]
    duration: Option<Duration>,
    /// Overwrite an existing output file.
    #[arg(long)]
    force: bool,
    #[command(flatten)]
    live: LiveArgs,
}

#[derive(Debug, Args)]
struct ServeArgs {
    /// Address to bind.
    #[arg(long, default_value_t = Ipv4Addr::UNSPECIFIED)]
    bind: Ipv4Addr,
    /// Command port.
    #[arg(long, default_value_t = 1510)]
    command_port: u16,
    /// Data port.
    #[arg(long, default_value_t = 1511)]
    data_port: u16,
    /// Stream by unicast instead of multicast.
    #[arg(long)]
    unicast: bool,
    /// Multicast group.
    #[arg(long, default_value_t = Ipv4Addr::new(239, 255, 42, 99))]
    multicast_group: Ipv4Addr,
    /// Outgoing multicast interface.
    #[arg(long, default_value_t = Ipv4Addr::UNSPECIFIED)]
    multicast_interface: Ipv4Addr,
    /// Frame rate of the synthetic scene.
    #[arg(long, default_value_t = 120.0)]
    rate: f64,
    /// Rigid bodies in the synthetic scene.
    #[arg(long, default_value_t = 3)]
    rigid_bodies: usize,
    /// Protocol version to emit.
    #[arg(long, default_value = "4.1")]
    protocol_version: String,
    /// Stream this capture's frames instead of the synthetic scene.
    #[arg(long)]
    capture: Option<PathBuf>,
    /// Serve this raw MODELDEF datagram as the model definition.
    #[arg(long)]
    model_file: Option<PathBuf>,
}

fn stop_flag() -> Arc<AtomicBool> {
    let stop = Arc::new(AtomicBool::new(false));
    let s = Arc::clone(&stop);
    if let Err(e) = ctrlc::set_handler(move || {
        if s.swap(true, Ordering::Relaxed) {
            // A second Ctrl-C exits immediately.
            std::process::exit(130);
        }
    }) {
        warn!("cannot install the Ctrl-C handler: {e}");
    }
    stop
}

fn init_logging() -> RosoutHandle {
    let filter =
        EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info,zenoh=warn"));
    let (rosout, handle) = RosoutLayer::new(tracing::Level::INFO, &["natnet_zenoh"]);
    // Logs go to stderr, so that stdout carries only command output
    // (`probe --json`, `config`).
    tracing_subscriber::registry()
        .with(filter)
        .with(
            tracing_subscriber::fmt::layer()
                .with_target(false)
                .with_writer(std::io::stderr)
                .with_ansi(std::io::stderr().is_terminal()),
        )
        .with(rosout)
        .init();
    handle
}

/// Writes command output to stdout. A closed reader (`| head`) is not an
/// error: the output is simply not wanted any more.
fn emit(text: &str) -> Result<(), String> {
    use std::io::Write as _;
    let mut out = std::io::stdout().lock();
    match out.write_all(text.as_bytes()).and_then(|()| out.flush()) {
        Err(e) if e.kind() != std::io::ErrorKind::BrokenPipe => Err(format!("stdout: {e}")),
        _ => Ok(()),
    }
}

/// Reads the configuration file, unvalidated: options override it first.
fn load_config(path: Option<&PathBuf>) -> Result<Config, String> {
    match path {
        Some(p) => Config::read(p).map_err(|e| e.to_string()),
        None => Ok(Config::default()),
    }
}

/// Fails for files the selected source will need but cannot read, which
/// the sources would otherwise only report once running.
fn check_files(cfg: &Config) -> Result<(), String> {
    let exists = |what: &str, path: &Path| {
        if path.is_file() {
            Ok(())
        } else {
            Err(format!("{what} {}: no such file", path.display()))
        }
    };
    match cfg.source.kind {
        SourceKind::Replay => {
            exists("source.replay.path", &cfg.source.replay.path)?;
            if let Some(m) = &cfg.source.replay.model_file {
                exists("source.replay.model_file", m)?;
            }
        }
        SourceKind::Live => {
            if let Some(m) = &cfg.source.live.model_file {
                exists("source.live.model_file", m)?;
            }
        }
        SourceKind::Synthetic => {}
    }
    Ok(())
}

/// Loads the configuration of a live-only command (`probe`, `record`).
fn live_config(args: &LiveArgs) -> Result<Config, String> {
    let mut cfg = load_config(args.config_path().as_ref())?;
    args.apply(&mut cfg);
    cfg.source.kind = SourceKind::Live;
    cfg.validate().map_err(|e| e.to_string())?;
    check_files(&cfg)?;
    Ok(cfg)
}

fn run(args: &RunArgs, mut rosout: RosoutHandle) -> Result<(), String> {
    let mut cfg = load_config(args.live.config_path().as_ref())?;
    args.live.apply(&mut cfg);
    // `--server` and `--passive` only make sense for a live source.
    if args.live.server.is_some() || args.live.passive {
        cfg.source.kind = SourceKind::Live;
    }
    if args.live.protocol_version.is_some() {
        cfg.source
            .replay
            .protocol_version
            .clone_from(&args.live.protocol_version);
    }
    if let Some(r) = args.rate {
        cfg.source.replay.rate = r;
    }
    cfg.source.replay.looping |= args.looping;
    if let Some(path) = &args.replay {
        cfg.source.kind = SourceKind::Replay;
        cfg.source.replay.path.clone_from(path);
    } else if args.synthetic {
        cfg.source.kind = SourceKind::Synthetic;
    }
    if args.model_file.is_some() {
        cfg.source.replay.model_file.clone_from(&args.model_file);
        cfg.source.live.model_file.clone_from(&args.model_file);
    }
    if args.domain_id.is_some() {
        cfg.ros.domain_id = args.domain_id;
    }
    if let Some(n) = &args.node_name {
        cfg.ros.node_name.clone_from(n);
    }
    if let Some(n) = &args.namespace {
        cfg.ros.namespace.clone_from(n);
    }
    if let Some(w) = &args.world_frame {
        cfg.ros.world_frame.clone_from(w);
    }
    if let Some(a) = args.up_axis {
        cfg.transform.up_axis = match a {
            UpAxisArg::Y => natnet_zenoh::mocap::transform::UpAxis::Y,
            UpAxisArg::Z => natnet_zenoh::mocap::transform::UpAxis::Z,
        };
    }
    cfg.ros.enabled &= !args.no_ros;
    cfg.native.enabled |= args.native;
    if args.zenoh_config.is_some() {
        cfg.zenoh.config_file.clone_from(&args.zenoh_config);
    }
    if !args.connect.is_empty() {
        let endpoints = serde_json::to_string(&args.connect).map_err(|e| e.to_string())?;
        cfg.zenoh
            .overrides
            .push(format!("connect/endpoints={endpoints}"));
    }
    cfg.validate().map_err(|e| e.to_string())?;
    check_files(&cfg)?;

    let stop = stop_flag();
    if let Some(d) = args.duration {
        let s = Arc::clone(&stop);
        std::thread::spawn(move || {
            std::thread::sleep(d);
            s.store(true, Ordering::Relaxed);
        });
    }
    // Open zenoh here rather than in the bridge, so that a stop request
    // ends the wait for a router.
    let context = if cfg.ros.enabled || cfg.native.enabled {
        let mut opts = context_options(&cfg).map_err(|e| e.to_string())?;
        let attempts = opts.router_check_attempts.take();
        let ctx = Context::new(&opts).map_err(|e| e.to_string())?;
        if let Some(attempts) = attempts {
            ctx.wait_for_router(attempts, &stop);
        }
        if stop.load(Ordering::Relaxed) {
            return Ok(());
        }
        Some(ctx)
    } else {
        None
    };
    let mut bridge = Bridge::new(cfg, context, Some(&mut rosout)).map_err(|e| e.to_string())?;
    let summary = bridge.run(&stop);
    info!(
        "stopped: {} frames, {} dropped in sequence ({} of them arrived late), {} dropped in queue, {} decode errors",
        summary.frames,
        summary.dropped_frames,
        summary.late_frames,
        summary.queue_drops,
        summary.decode_errors
    );
    // A bounded run (a replay's end, `--duration`) that saw nothing failed.
    if summary.frames == 0 && (summary.finished || args.duration.is_some()) {
        return Err("no frames received".into());
    }
    Ok(())
}

fn probe(args: &ProbeArgs) -> Result<(), String> {
    let cfg = live_config(&args.live)?;
    let live = cfg.source.live.clone();
    let handle = SourceHandle::spawn("natnet-live", 1024, move |ctx, cmds| {
        live::run(&live, ctx, &cmds);
    })
    .map_err(|e| e.to_string())?;
    let stop = stop_flag();
    let mut processor = Processor::new(&cfg);
    let mut info_json = serde_json::Value::Null;
    let mut messages = Vec::new();
    let mut errors = Vec::new();
    let mut first_frame: Option<Instant> = None;
    let duration = args.duration.max(Duration::from_millis(100));
    let deadline = Instant::now() + duration + Duration::from_secs(2);
    let mut last_snapshot = None;
    while !stop.load(Ordering::Relaxed) && Instant::now() < deadline {
        if first_frame.is_some_and(|t| t.elapsed() >= duration) {
            break;
        }
        match handle.events.recv_timeout(Duration::from_millis(100)) {
            Ok(SourceEvent::ServerInfo(i)) => {
                processor.set_server_info(&i);
                info_json = server_info_json(&i);
            }
            Ok(SourceEvent::ModelDef(md, v)) => {
                for (t, e) in processor.set_model(&md, v) {
                    errors.push(format!("model dataset type {t}: {e}"));
                }
            }
            Ok(SourceEvent::Frame {
                frame,
                received,
                received_mono,
            }) => {
                first_frame.get_or_insert(received_mono);
                if let Some(snap) = processor.process(&frame, received, received_mono) {
                    last_snapshot = Some(snap);
                }
            }
            Ok(SourceEvent::ServerMessage(m)) => messages.push(m),
            Ok(SourceEvent::DecodeError(e)) => errors.push(e),
            Ok(SourceEvent::Connection(_)) | Err(_) => {}
        }
    }
    let elapsed = first_frame.map_or(0.0, |t| t.elapsed().as_secs_f64());
    let stats = &processor.stats;
    let report = serde_json::json!({
        "server": info_json,
        "model": processor.model(),
        "frames": stats.frames,
        "frame_rate": if elapsed > 0.0 { stats.frames as f64 / elapsed } else { 0.0 },
        "dropped_frames": stats.dropped_frames,
        "late_frames": stats.late_frames,
        "duplicate_frames": stats.duplicate_frames,
        "restarts": stats.resets,
        "server_latency_ms": stats.server_latency.map(|v| v * 1e3),
        "total_latency_ms": stats.total_latency.map(|v| v * 1e3),
        "clock_offset_ms": stats.clock_offset.map(|v| v * 1e3),
        "last_frame": last_snapshot,
        "server_messages": messages,
        "errors": errors,
    });
    if args.json {
        let json = serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?;
        emit(&format!("{json}\n"))?;
    } else {
        let mut lines = vec![format!("server:   {info_json}")];
        let m = processor.model();
        lines.push(format!(
            "model:    {} rigid bodies, {} skeletons, {} force plates, {} devices, {} cameras, {} assets",
            m.rigid_bodies.len(),
            m.skeletons.len(),
            m.force_plates.len(),
            m.devices.len(),
            m.cameras.len(),
            m.assets.len()
        ));
        for rb in m.rigid_bodies.values() {
            lines.push(format!(
                "  rigid body {:>4}  {:<24} {} markers",
                rb.id,
                rb.name,
                rb.markers.len()
            ));
        }
        lines.push(format!(
            "frames:   {} ({:.1} Hz, {} dropped, {} late, {} restarts)",
            stats.frames,
            report["frame_rate"].as_f64().unwrap_or_default(),
            stats.dropped_frames,
            stats.late_frames,
            stats.resets
        ));
        if let Some(s) = &last_snapshot {
            for b in &s.bodies {
                lines.push(match b.pose {
                    Some(p) => format!(
                        "  {:<24} [{:+.4}, {:+.4}, {:+.4}] m",
                        b.name, p.position[0], p.position[1], p.position[2]
                    ),
                    None => format!("  {:<24} not tracked", b.name),
                });
            }
            lines.push(format!(
                "markers:  {} labeled",
                s.markers.iter().filter(|m| m.labeled()).count()
            ));
        }
        let ms =
            |v: Option<f64>| v.map_or_else(|| "n/a".to_owned(), |v| format!("{:.2} ms", v * 1e3));
        lines.push(format!(
            "latency:  server {}, total {}, clock offset {}",
            ms(stats.server_latency),
            ms(stats.total_latency),
            ms(stats.clock_offset)
        ));
        lines.extend(errors.iter().map(|e| format!("error:    {e}")));
        emit(&(lines.join("\n") + "\n"))?;
    }
    if stats.frames == 0 {
        return Err("no frames received".into());
    }
    Ok(())
}

/// Datagrams `record` holds in memory while the disk catches up.
const RECORD_BACKLOG: usize = 16_384;

/// What `record` has written so far.
struct Recording {
    writer: CaptureWriter<std::io::BufWriter<std::fs::File>>,
    datagrams: u64,
    frames: u64,
    bytes: u64,
    error: Option<String>,
}

fn record(args: &RecordArgs) -> Result<(), String> {
    let cfg = live_config(&args.live)?;
    let format = match args.format {
        FormatArg::Natnet => Format::NatNet,
        FormatArg::Aucap => Format::Aucap,
    };
    let created = if args.force {
        CaptureWriter::create(&args.output, format)
    } else {
        CaptureWriter::create_new(&args.output, format)
    };
    let writer = created.map_err(|e| {
        if e.kind() == std::io::ErrorKind::AlreadyExists {
            format!("{} exists (--force overwrites it)", args.output.display())
        } else {
            format!("{}: {e}", args.output.display())
        }
    })?;
    let writer = Arc::new(Mutex::new(Recording {
        writer,
        datagrams: 0,
        frames: 0,
        bytes: 0,
        error: None,
    }));
    let start = Instant::now();
    // The receive thread only queues datagrams: a slow disk must not hold up
    // the socket, whose buffer would then overflow without a trace.
    let (records, queued) = std::sync::mpsc::sync_channel::<Record>(RECORD_BACKLOG);
    let dropped = Arc::new(AtomicU64::new(0));
    let disk = {
        let writer = Arc::clone(&writer);
        std::thread::Builder::new()
            .name("natnet-record".into())
            .spawn(move || {
                for record in queued {
                    let mut w = writer.lock().unwrap_or_else(PoisonError::into_inner);
                    match w.writer.write(&record) {
                        Ok(()) => {
                            w.datagrams += 1;
                            w.bytes += record.bytes.len() as u64;
                            if record.bytes.get(..2)
                                == Some(&natnet::NAT_FRAMEOFDATA.to_le_bytes()[..])
                            {
                                w.frames += 1;
                            }
                        }
                        Err(e) => {
                            w.error = Some(e.to_string());
                            return;
                        }
                    }
                }
            })
            .map_err(|e| e.to_string())?
    };
    let tap: live::Tap = {
        let dropped = Arc::clone(&dropped);
        Arc::new(move |bytes: &[u8], from: SocketAddrV4, at: Instant| {
            let record = Record {
                time_ns: at.saturating_duration_since(start).as_nanos() as u64,
                source: Some(from),
                bytes: bytes.to_vec(),
            };
            if records.try_send(record).is_err() {
                dropped.fetch_add(1, Ordering::Relaxed);
            }
        })
    };
    let live = cfg.source.live.clone();
    let mut handle = SourceHandle::spawn("natnet-live", 4096, move |ctx, cmds| {
        live::run_with_tap(&live, ctx, &cmds, Some(tap));
    })
    .map_err(|e| e.to_string())?;
    let stop = stop_flag();
    info!("recording to {} (Ctrl-C to stop)", args.output.display());
    let mut last_report = Instant::now();
    while !stop.load(Ordering::Relaxed) {
        if args.duration.is_some_and(|d| start.elapsed() >= d) {
            break;
        }
        // Drain decoded events; the tap has already written the datagrams.
        while let Ok(e) = handle.events.try_recv() {
            match e {
                SourceEvent::Connection(ConnectionState::Lost) => warn!("stream lost"),
                SourceEvent::ServerInfo(i) => info!("server: {}", server_info_json(&i)),
                _ => {}
            }
        }
        if last_report.elapsed() >= Duration::from_secs(5) {
            last_report = Instant::now();
            let w = writer.lock().unwrap_or_else(PoisonError::into_inner);
            info!(
                "{} datagrams ({} frames), {} bytes",
                w.datagrams, w.frames, w.bytes
            );
        }
        let w = writer.lock().unwrap_or_else(PoisonError::into_inner);
        if let Some(e) = &w.error {
            return Err(format!("write failed: {e}"));
        }
        drop(w);
        std::thread::sleep(Duration::from_millis(50));
    }
    // Stopping the source drops the tap, which ends the writer's queue.
    handle.stop();
    drop(handle);
    if disk.join().is_err() {
        return Err("the capture writer panicked".into());
    }
    let mut w = writer.lock().unwrap_or_else(PoisonError::into_inner);
    if let Some(e) = &w.error {
        return Err(format!("write failed: {e}"));
    }
    w.writer.flush().map_err(|e| e.to_string())?;
    emit(&format!(
        "{} datagrams ({} frames), {} bytes written to {}\n",
        w.datagrams,
        w.frames,
        w.bytes,
        args.output.display()
    ))?;
    let dropped = dropped.load(Ordering::Relaxed);
    if dropped > 0 {
        warn!("{dropped} datagrams were not recorded: the disk did not keep up");
    }
    if w.frames == 0 {
        return Err("no frames received".into());
    }
    Ok(())
}

/// Parses a `--duration` in seconds.
fn parse_seconds(s: &str) -> Result<Duration, String> {
    let secs: f64 = s.parse().map_err(|e| format!("{e}"))?;
    Duration::try_from_secs_f64(secs)
        .map_err(|_| format!("{s} is not a non-negative number of seconds"))
}

fn serve(args: &ServeArgs) -> Result<(), String> {
    let mut cfg = ServerConfig {
        bind: args.bind,
        command_port: args.command_port,
        data_port: args.data_port,
        multicast: !args.unicast,
        multicast_group: args.multicast_group,
        multicast_interface: args.multicast_interface,
        capture: args.capture.clone(),
        model_file: args.model_file.clone(),
        ..ServerConfig::default()
    };
    cfg.scene.rate_hz = args.rate;
    cfg.scene.rigid_bodies = args.rigid_bodies;
    cfg.scene
        .protocol_version
        .clone_from(&args.protocol_version);
    let server = SimServer::spawn(cfg).map_err(|e| e.to_string())?;
    emit(&format!(
        "simulated NatNet server on {} ({}); Ctrl-C to stop\n",
        server.command_addr(),
        if args.unicast {
            "unicast".to_owned()
        } else {
            format!("multicast {}:{}", args.multicast_group, args.data_port)
        }
    ))?;
    let stop = stop_flag();
    let mut last = Instant::now();
    while !stop.load(Ordering::Relaxed) && server.is_running() {
        std::thread::sleep(Duration::from_millis(100));
        if last.elapsed() >= Duration::from_secs(10) {
            last = Instant::now();
            info!("{} frames sent", server.frames_sent());
        }
    }
    if !stop.load(Ordering::Relaxed) {
        return Err("the server thread stopped unexpectedly".into());
    }
    Ok(())
}

fn check(path: &Path) -> Result<(), String> {
    let cfg = Config::load(path).map_err(|e| e.to_string())?;
    cfg.validate_all().map_err(|e| e.to_string())?;
    check_files(&cfg)?;
    emit(&format!("{}: ok\n", path.display()))
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let rosout = init_logging();
    let result = match &cli.command {
        Command::Run(args) => run(args, rosout),
        Command::Probe(args) => probe(args),
        Command::Record(args) => record(args),
        Command::Serve(args) => serve(args),
        Command::Config => emit(&Config::default_toml()),
        Command::Check { config } => check(config),
        Command::Types => {
            let lines: Vec<String> = natnet_zenoh::ros::msgs::catalog()
                .into_iter()
                .map(|(name, hash)| format!("{name:<40} {hash}"))
                .collect();
            emit(&(lines.join("\n") + "\n"))
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            error!("{e}");
            ExitCode::FAILURE
        }
    }
}
