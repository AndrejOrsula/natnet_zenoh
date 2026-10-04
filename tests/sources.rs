//! Sources end to end: the live client against the simulated server, and
//! capture replay.

// Shared helpers use panicking assertions.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::net::Ipv4Addr;
use std::time::{Duration, Instant};

use natnet::Version;

use natnet_zenoh::source::capture::{CaptureWriter, Format, Record};
use natnet_zenoh::source::live::{self, LiveConfig, Transport};
use natnet_zenoh::source::replay::{self, ReplayConfig};
use natnet_zenoh::source::server::{ServerConfig, SimServer};
use natnet_zenoh::source::synthetic::{Scene, SyntheticConfig};
use natnet_zenoh::source::{ConnectionState, SourceCommand, SourceEvent, SourceHandle};

#[derive(Default, Debug)]
struct Seen {
    info: usize,
    model: usize,
    frames: usize,
    streaming: usize,
    lost: usize,
    finished: usize,
    errors: Vec<String>,
    last_frame: i32,
}

fn collect(handle: &SourceHandle, until: impl Fn(&Seen) -> bool, timeout: Duration) -> Seen {
    let mut seen = Seen::default();
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline && !until(&seen) {
        match handle.events.recv_timeout(Duration::from_millis(50)) {
            Ok(SourceEvent::ServerInfo(_)) => seen.info += 1,
            Ok(SourceEvent::ModelDef(..)) => seen.model += 1,
            Ok(SourceEvent::Frame { frame, .. }) => {
                seen.frames += 1;
                seen.last_frame = frame.frame_number;
            }
            Ok(SourceEvent::Connection(ConnectionState::Streaming)) => seen.streaming += 1,
            Ok(SourceEvent::Connection(ConnectionState::Lost)) => seen.lost += 1,
            Ok(SourceEvent::Connection(ConnectionState::Finished)) => seen.finished += 1,
            Ok(SourceEvent::DecodeError(e)) => seen.errors.push(e),
            Ok(SourceEvent::ServerMessage(_)) | Err(_) => {}
        }
    }
    seen
}

fn free_port() -> u16 {
    std::net::UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

fn scene() -> SyntheticConfig {
    SyntheticConfig {
        rate_hz: 200.0,
        ..SyntheticConfig::default()
    }
}

#[test]
fn live_unicast_handshake_keepalive_and_reconnect() {
    let mut server = SimServer::spawn(ServerConfig {
        bind: Ipv4Addr::LOCALHOST,
        command_port: 0,
        multicast: false,
        client_timeout_ms: 500,
        scene: scene(),
        ..ServerConfig::default()
    })
    .unwrap();
    let port = server.command_addr().port();
    let cfg = LiveConfig {
        server: Some(Ipv4Addr::LOCALHOST),
        interface: Ipv4Addr::LOCALHOST,
        command_port: port,
        transport: Transport::Auto,
        keepalive_interval_ms: 100,
        data_timeout_ms: 1000,
        connect_interval_ms: 200,
        ..LiveConfig::default()
    };
    let handle =
        SourceHandle::spawn("live", 256, move |ctx, cmds| live::run(&cfg, ctx, &cmds)).unwrap();
    let seen = collect(
        &handle,
        |s| s.frames >= 50 && s.model >= 1,
        Duration::from_secs(5),
    );
    assert_eq!(seen.info, 1, "{seen:?}");
    assert!(seen.model >= 1, "{seen:?}");
    assert_eq!(seen.streaming, 1, "{seen:?}");
    assert!(seen.errors.is_empty(), "{seen:?}");
    // Keepalives hold the subscription well past the server's client timeout.
    let seen = collect(&handle, |_| false, Duration::from_millis(1200));
    assert!(seen.frames > 150, "unicast stream stalled: {seen:?}");
    assert_eq!(seen.lost, 0);

    handle.command(SourceCommand::RefreshModel);
    let seen = collect(&handle, |s| s.model >= 1, Duration::from_secs(2));
    assert_eq!(seen.model, 1);

    // Restart the server on the same port: the client notices and reconnects.
    server.stop();
    let seen = collect(&handle, |s| s.lost >= 1, Duration::from_secs(3));
    assert_eq!(seen.lost, 1, "{seen:?}");
    let _server = SimServer::spawn(ServerConfig {
        bind: Ipv4Addr::LOCALHOST,
        command_port: port,
        multicast: false,
        scene: scene(),
        ..ServerConfig::default()
    })
    .unwrap();
    let seen = collect(
        &handle,
        |s| s.streaming >= 1 && s.frames >= 20,
        Duration::from_secs(5),
    );
    assert_eq!(seen.streaming, 1, "{seen:?}");
    assert!(seen.info >= 1 && seen.model >= 1, "{seen:?}");
}

/// Frames, model and the reported protocol version from a unicast client of
/// a 4.1 scene server, with `cfg` applied on top.
fn unicast_session(cfg: impl FnOnce(LiveConfig) -> LiveConfig) -> (Seen, Option<Version>) {
    let server = SimServer::spawn(ServerConfig {
        bind: Ipv4Addr::LOCALHOST,
        command_port: 0,
        multicast: false,
        scene: scene(),
        ..ServerConfig::default()
    })
    .unwrap();
    let cfg = cfg(LiveConfig {
        server: Some(Ipv4Addr::LOCALHOST),
        interface: Ipv4Addr::LOCALHOST,
        command_port: server.command_addr().port(),
        transport: Transport::Unicast,
        connect_interval_ms: 200,
        ..LiveConfig::default()
    });
    cfg.validate().unwrap();
    let handle =
        SourceHandle::spawn("live", 256, move |ctx, cmds| live::run(&cfg, ctx, &cmds)).unwrap();
    let mut version = None;
    let mut seen = Seen::default();
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline && !(seen.frames >= 50 && seen.model >= 1) {
        match handle.events.recv_timeout(Duration::from_millis(50)) {
            Ok(SourceEvent::ServerInfo(info)) => version = Some(info.protocol_version),
            Ok(SourceEvent::ModelDef(..)) => seen.model += 1,
            Ok(SourceEvent::Frame { .. }) => seen.frames += 1,
            Ok(SourceEvent::DecodeError(e)) => seen.errors.push(e),
            _ => {}
        }
    }
    (seen, version)
}

#[test]
fn live_unicast_requests_a_bitstream_version() {
    // The server reports 4.1 but streams the requested 3.1, which the client
    // decodes: SERVERINFO downstream carries the decoding version.
    let (seen, version) = unicast_session(|cfg| LiveConfig {
        request_bitstream: Some("3.1".into()),
        ..cfg
    });
    assert!(seen.frames >= 50 && seen.model >= 1, "{seen:?}");
    assert!(seen.errors.is_empty(), "{seen:?}");
    assert_eq!(version, Some(Version::new(3, 1, 0, 0)));

    // Control: the same 4.1 stream decoded as 3.1 without the request fails,
    // so the request is what made the session above decode.
    let (seen, _) = unicast_session(|cfg| LiveConfig {
        protocol_version: Some("3.1".into()),
        ..cfg
    });
    assert!(!seen.errors.is_empty(), "{seen:?}");
}

#[test]
fn request_bitstream_needs_an_active_unicast_client() {
    let base = LiveConfig {
        server: Some(Ipv4Addr::LOCALHOST),
        transport: Transport::Unicast,
        request_bitstream: Some("4.1".into()),
        ..LiveConfig::default()
    };
    base.validate().unwrap();
    for (cfg, problem) in [
        (
            LiveConfig {
                request_bitstream: Some("four".into()),
                ..base.clone()
            },
            "invalid version",
        ),
        (
            LiveConfig {
                transport: Transport::Auto,
                ..base.clone()
            },
            "unicast",
        ),
        (
            LiveConfig {
                transport: Transport::Multicast,
                passive: true,
                ..base.clone()
            },
            "passive",
        ),
        (
            LiveConfig {
                protocol_version: Some("4.1".into()),
                ..base.clone()
            },
            "not both",
        ),
    ] {
        let e = cfg.validate().unwrap_err();
        assert!(e.contains(problem), "{e}");
    }
}

#[test]
fn live_multicast_with_server_and_passive() {
    let data_port = free_port();
    let group = Ipv4Addr::new(239, 255, 42, 199);
    let server = SimServer::spawn(ServerConfig {
        bind: Ipv4Addr::LOCALHOST,
        command_port: 0,
        data_port,
        multicast: true,
        multicast_group: group,
        multicast_interface: Ipv4Addr::LOCALHOST,
        scene: scene(),
        ..ServerConfig::default()
    })
    .unwrap();
    let port = server.command_addr().port();
    let active = LiveConfig {
        server: Some(Ipv4Addr::LOCALHOST),
        interface: Ipv4Addr::LOCALHOST,
        command_port: port,
        model_poll_secs: 1,
        ..LiveConfig::default()
    };
    let passive = LiveConfig {
        interface: Ipv4Addr::LOCALHOST,
        data_port,
        multicast_group: group,
        passive: true,
        ..LiveConfig::default()
    };
    let a = SourceHandle::spawn("active", 1024, move |ctx, cmds| {
        live::run(&active, ctx, &cmds);
    })
    .unwrap();
    // Both clients share the data port (SO_REUSEADDR).
    let p = SourceHandle::spawn("passive", 1024, move |ctx, cmds| {
        live::run(&passive, ctx, &cmds);
    })
    .unwrap();
    // The model is polled while streaming.
    let seen_a = collect(
        &a,
        |s| s.frames >= 50 && s.model >= 2,
        Duration::from_secs(5),
    );
    let seen_p = collect(&p, |s| s.frames >= 50, Duration::from_secs(5));
    assert!(
        seen_a.frames >= 50 && seen_a.model >= 2 && seen_a.info == 1,
        "{seen_a:?}"
    );
    assert!(
        seen_p.frames >= 50 && seen_p.info == 1 && seen_p.model == 0,
        "{seen_p:?}"
    );
    assert!(seen_a.errors.is_empty() && seen_p.errors.is_empty());
}

#[test]
fn live_discovers_server_from_multicast() {
    let data_port = free_port();
    let group = Ipv4Addr::new(239, 255, 42, 198);
    let server = SimServer::spawn(ServerConfig {
        bind: Ipv4Addr::LOCALHOST,
        command_port: 0,
        data_port,
        multicast_group: group,
        multicast_interface: Ipv4Addr::LOCALHOST,
        scene: scene(),
        ..ServerConfig::default()
    })
    .unwrap();
    let cfg = LiveConfig {
        interface: Ipv4Addr::LOCALHOST,
        command_port: server.command_addr().port(),
        data_port,
        multicast_group: group,
        ..LiveConfig::default()
    };
    let h = SourceHandle::spawn("discover", 1024, move |ctx, cmds| {
        live::run(&cfg, ctx, &cmds);
    })
    .unwrap();
    let seen = collect(
        &h,
        |s| s.frames >= 20 && s.model >= 1,
        Duration::from_secs(5),
    );
    assert!(
        seen.info == 1 && seen.model >= 1 && seen.frames >= 20,
        "{seen:?}"
    );
}

/// Motive's command engine can stall while it keeps streaming: frames must
/// still flow, decoded as protocol 4.1.
#[test]
fn live_multicast_flows_without_a_serverinfo() {
    let data_port = free_port();
    let group = Ipv4Addr::new(239, 255, 42, 197);
    let _server = SimServer::spawn(ServerConfig {
        bind: Ipv4Addr::LOCALHOST,
        command_port: 0,
        data_port,
        multicast: true,
        multicast_group: group,
        multicast_interface: Ipv4Addr::LOCALHOST,
        scene: scene(),
        ..ServerConfig::default()
    })
    .unwrap();
    // Nobody answers on this command port.
    let silent = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    let cfg = LiveConfig {
        server: Some(Ipv4Addr::LOCALHOST),
        command_port: silent.local_addr().unwrap().port(),
        data_port,
        multicast_group: group,
        ..LiveConfig::default()
    };
    let h = SourceHandle::spawn("silent", 1024, move |ctx, cmds| {
        live::run(&cfg, ctx, &cmds);
    })
    .unwrap();
    let seen = collect(&h, |s| s.frames >= 20, Duration::from_secs(10));
    assert!(
        seen.info == 1 && seen.frames >= 20 && seen.model == 0,
        "{seen:?}"
    );
    assert!(seen.errors.is_empty(), "{seen:?}");
}

fn temp_path(name: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!("natnet_zenoh-{}-{name}", std::process::id()))
}

#[test]
fn replay_both_containers() {
    let scene = Scene::new(SyntheticConfig::default()).unwrap();
    for format in [Format::NatNet, Format::Aucap] {
        let path = temp_path(&format!("{format:?}.cap"));
        let mut w = CaptureWriter::create(&path, format).unwrap();
        let info = scene.server_info(false, 0, [0; 4]).to_datagram().unwrap();
        w.write(&Record {
            time_ns: 0,
            source: None,
            bytes: info,
        })
        .unwrap();
        let md = scene.model_def().to_datagram(scene.version()).unwrap();
        w.write(&Record {
            time_ns: 1,
            source: None,
            bytes: md,
        })
        .unwrap();
        for n in 0..100 {
            let bytes = scene.frame_datagram(n).unwrap();
            w.write(&Record {
                time_ns: 2 + u64::try_from(n).unwrap() * 1_000_000,
                source: None,
                bytes,
            })
            .unwrap();
        }
        w.flush().unwrap();
        drop(w);

        // Unpaced replay delivers every frame, even through a tiny queue.
        let cfg = ReplayConfig {
            path: path.clone(),
            rate: 0.0,
            ..ReplayConfig::default()
        };
        let h = SourceHandle::spawn("replay", 4, move |ctx, cmds| replay::run(&cfg, &ctx, &cmds))
            .unwrap();
        let seen = collect(&h, |s| s.finished >= 1, Duration::from_secs(10));
        assert_eq!(seen.frames, 100, "{format:?}: {seen:?}");
        assert_eq!(seen.last_frame, 99);
        assert_eq!(seen.info, 2, "synthesized-from-scan plus the recorded one");
        assert!(seen.model >= 1);
        assert!(seen.errors.is_empty(), "{:?}", seen.errors);

        // Paced at 1x: 100 frames 1 ms apart take about 0.1 s.
        let cfg = ReplayConfig {
            path: path.clone(),
            rate: 1.0,
            start_offset_secs: 0.05,
            ..ReplayConfig::default()
        };
        let start = Instant::now();
        let h = SourceHandle::spawn("replay", 256, move |ctx, cmds| {
            replay::run(&cfg, &ctx, &cmds);
        })
        .unwrap();
        let seen = collect(&h, |s| s.finished >= 1, Duration::from_secs(10));
        assert!(
            start.elapsed() >= Duration::from_millis(45),
            "{:?}",
            start.elapsed()
        );
        assert_eq!(seen.frames, 50, "{seen:?}");
        std::fs::remove_file(&path).unwrap();
    }
}

#[test]
fn replay_skips_long_pauses() {
    let scene = Scene::new(SyntheticConfig::default()).unwrap();
    let path = temp_path("pause.cap");
    let mut w = CaptureWriter::create(&path, Format::NatNet).unwrap();
    // Two frames, a two-minute pause (a stopped recording), two frames.
    for (n, time_ns) in [0, 1_000_000, 120_000_000_000, 120_001_000_000]
        .into_iter()
        .enumerate()
    {
        w.write(&Record {
            time_ns,
            source: None,
            bytes: scene.frame_datagram(i32::try_from(n).unwrap()).unwrap(),
        })
        .unwrap();
    }
    w.flush().unwrap();
    drop(w);
    let cfg = ReplayConfig {
        path: path.clone(),
        rate: 1.0,
        ..ReplayConfig::default()
    };
    let h = SourceHandle::spawn("replay", 16, move |ctx, cmds| {
        replay::run(&cfg, &ctx, &cmds);
    })
    .unwrap();
    let seen = collect(&h, |s| s.finished >= 1, Duration::from_secs(10));
    assert_eq!(seen.frames, 4, "{seen:?}");
    assert_eq!(seen.finished, 1, "{seen:?}");
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn replay_paces_after_a_corrupt_timestamp() {
    let scene = Scene::new(SyntheticConfig::default()).unwrap();
    let path = temp_path("step_back.cap");
    let mut w = CaptureWriter::create(&path, Format::NatNet).unwrap();
    // One corrupt timestamp (a 90 s spike), then time steps back to normal:
    // the last three frames still play 300 ms apart.
    let times = [
        0,
        300_000_000,
        90_000_000_000,
        400_000_000,
        700_000_000,
        1_000_000_000,
    ];
    for (n, time_ns) in times.into_iter().enumerate() {
        w.write(&Record {
            time_ns,
            source: None,
            bytes: scene.frame_datagram(i32::try_from(n).unwrap()).unwrap(),
        })
        .unwrap();
    }
    w.flush().unwrap();
    drop(w);
    let cfg = ReplayConfig {
        path: path.clone(),
        rate: 1.0,
        ..ReplayConfig::default()
    };
    let start = Instant::now();
    let h = SourceHandle::spawn("replay", 16, move |ctx, cmds| {
        replay::run(&cfg, &ctx, &cmds);
    })
    .unwrap();
    let seen = collect(&h, |s| s.finished >= 1, Duration::from_secs(10));
    assert_eq!(seen.frames, 6, "{seen:?}");
    assert!(
        start.elapsed() >= Duration::from_millis(850),
        "{:?}",
        start.elapsed()
    );
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn server_capture_feed_serves_matching_model() {
    let scene = Scene::new(SyntheticConfig::default()).unwrap();
    let capture = temp_path("server_feed.natnet");
    let mut w = CaptureWriter::create(&capture, Format::NatNet).unwrap();
    for n in 0..200 {
        let bytes = scene.frame_datagram(n).unwrap();
        w.write(&Record {
            time_ns: u64::try_from(n).unwrap() * 5_000_000,
            source: None,
            bytes,
        })
        .unwrap();
    }
    drop(w);
    let model_file = temp_path("server_feed.modeldef");
    std::fs::write(
        &model_file,
        scene.model_def().to_datagram(scene.version()).unwrap(),
    )
    .unwrap();

    // Without a recorded model the server serves an empty one rather than
    // the synthetic scene's; a model file takes precedence.
    for (file, datasets) in [
        (None, 0),
        (Some(model_file.clone()), scene.model_def().datasets.len()),
    ] {
        let server = SimServer::spawn(ServerConfig {
            bind: Ipv4Addr::LOCALHOST,
            command_port: 0,
            multicast: false,
            capture: Some(capture.clone()),
            model_file: file,
            ..ServerConfig::default()
        })
        .unwrap();
        let cfg = LiveConfig {
            server: Some(Ipv4Addr::LOCALHOST),
            interface: Ipv4Addr::LOCALHOST,
            command_port: server.command_addr().port(),
            ..LiveConfig::default()
        };
        let handle =
            SourceHandle::spawn("live", 256, move |ctx, cmds| live::run(&cfg, ctx, &cmds)).unwrap();
        let (mut model, mut frames) = (None, 0);
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline && (model.is_none() || frames < 20) {
            match handle.events.recv_timeout(Duration::from_millis(50)) {
                Ok(SourceEvent::ModelDef(md, _)) => model = Some(md.datasets.len()),
                Ok(SourceEvent::Frame { .. }) => frames += 1,
                Ok(SourceEvent::DecodeError(e)) => panic!("{e}"),
                _ => {}
            }
        }
        assert_eq!(model, Some(datasets));
        assert!(frames >= 20, "{frames}");
    }
    std::fs::remove_file(&capture).unwrap();
    std::fs::remove_file(&model_file).unwrap();
}

#[test]
fn stopping_a_blocked_lossless_replay_returns() {
    let scene = Scene::new(SyntheticConfig::default()).unwrap();
    let path = temp_path("blocked.natnet");
    let mut w = CaptureWriter::create(&path, Format::NatNet).unwrap();
    for n in 0..100 {
        let bytes = scene.frame_datagram(n).unwrap();
        w.write(&Record {
            time_ns: u64::try_from(n).unwrap(),
            source: None,
            bytes,
        })
        .unwrap();
    }
    drop(w);
    // Unpaced replay blocks on its full one-slot queue; nobody reads it.
    let cfg = ReplayConfig {
        path: path.clone(),
        rate: 0.0,
        looping: true,
        ..ReplayConfig::default()
    };
    let mut h =
        SourceHandle::spawn("replay", 1, move |ctx, cmds| replay::run(&cfg, &ctx, &cmds)).unwrap();
    std::thread::sleep(Duration::from_millis(100));
    let start = Instant::now();
    h.stop();
    assert!(
        start.elapsed() < Duration::from_secs(2),
        "{:?}",
        start.elapsed()
    );
    std::fs::remove_file(&path).unwrap();
}
