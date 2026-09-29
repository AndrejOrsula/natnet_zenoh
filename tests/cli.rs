//! The command line: exit codes, argument checks and machine-readable output
//! of the real binary.

// Helpers outside `#[test]` functions share the tests' panicking style.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::process::{Command, Output, Stdio};
use std::time::{Duration, Instant};

use natnet_zenoh::source::capture::{CaptureWriter, Format, Record};
use natnet_zenoh::source::synthetic::{Scene, SyntheticConfig};

fn bin() -> Command {
    let mut c = Command::new(env!("CARGO_BIN_EXE_natnet_zenoh"));
    c.env_remove("NATNET_ZENOH_CONFIG").env("RUST_LOG", "info");
    c
}

fn run(c: &mut Command) -> Output {
    c.stdin(Stdio::null()).output().unwrap()
}

fn stderr(o: &Output) -> String {
    String::from_utf8_lossy(&o.stderr).into_owned()
}

/// A per-test temporary path.
fn temp(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("natnet_zenoh_cli_{}_{name}", std::process::id()))
}

/// A capture with the synthetic model and `frames` frames, 1 ms apart.
fn capture(path: &Path, frames: i32) {
    let scene = Scene::new(SyntheticConfig::default()).unwrap();
    let mut w = CaptureWriter::create(path, Format::NatNet).unwrap();
    let md = scene
        .model_def()
        .to_datagram_version(scene.version())
        .unwrap();
    w.write(&Record {
        time_ns: 0,
        source: None,
        bytes: md,
    })
    .unwrap();
    for n in 0..frames {
        w.write(&Record {
            time_ns: 1 + u64::try_from(n).unwrap() * 1_000_000,
            source: None,
            bytes: scene.frame_datagram(n).unwrap(),
        })
        .unwrap();
    }
}

fn free_udp_port() -> u16 {
    UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

#[test]
fn default_configuration_checks_and_bad_values_fail() {
    let out = run(bin().arg("config"));
    assert!(out.status.success(), "{}", stderr(&out));
    let path = temp("default.toml");
    std::fs::write(&path, &out.stdout).unwrap();
    let out = run(bin().arg("check").arg(&path));
    assert!(out.status.success(), "{}", stderr(&out));
    assert!(String::from_utf8_lossy(&out.stdout).ends_with(": ok\n"));

    // An unused section still fails `check`.
    std::fs::write(&path, "[native]\nprefix = \"natnet/\"\n").unwrap();
    let out = run(bin().arg("check").arg(&path));
    assert_eq!(out.status.code(), Some(1));
    assert!(stderr(&out).contains("native.prefix"), "{}", stderr(&out));

    std::fs::write(
        &path,
        "[source]\nkind = \"replay\"\n[source.replay]\npath = \"/nonexistent.natnet\"\n",
    )
    .unwrap();
    let out = run(bin().arg("check").arg(&path));
    assert_eq!(out.status.code(), Some(1));
    assert!(stderr(&out).contains("no such file"), "{}", stderr(&out));
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn arguments_are_checked() {
    for args in [
        &["run", "--synthetic", "--data-port", "5000"][..],
        &["run", "--replay", "x.natnet", "--transport", "unicast"],
        &["record", "-o", "x.natnet", "--duration", "NaN"],
        &["probe", "--duration", "-1"],
        &["run", "--synthetic", "--request-bitstream", "4.1"],
        &["probe", "--passive", "--request-bitstream", "4.1"],
        &[
            "probe",
            "--protocol-version",
            "4.1",
            "--request-bitstream",
            "4.1",
        ],
    ] {
        let out = run(bin().args(args));
        assert_eq!(out.status.code(), Some(2), "{args:?}: {}", stderr(&out));
    }
}

#[test]
fn replays_exit_by_outcome() {
    let missing = run(bin().args(["run", "--no-ros", "--replay", "/nonexistent.natnet"]));
    assert_eq!(missing.status.code(), Some(1));
    assert!(
        stderr(&missing).contains("no such file"),
        "{}",
        stderr(&missing)
    );

    let empty = temp("empty.natnet");
    capture(&empty, 0);
    let out = run(bin()
        .args(["run", "--no-ros", "--rate", "0", "--replay"])
        .arg(&empty));
    assert_eq!(out.status.code(), Some(1), "{}", stderr(&out));
    assert!(
        stderr(&out).contains("no frames received"),
        "{}",
        stderr(&out)
    );
    std::fs::remove_file(&empty).unwrap();

    let full = temp("full.natnet");
    capture(&full, 50);
    // An empty NATNET_ZENOH_CONFIG counts as unset; a set one is read.
    let out = run(bin()
        .env("NATNET_ZENOH_CONFIG", "")
        .args(["run", "--no-ros", "--rate", "0", "--replay"])
        .arg(&full));
    assert!(out.status.success(), "{}", stderr(&out));
    assert!(
        stderr(&out).contains("stopped: 50 frames"),
        "{}",
        stderr(&out)
    );
    let out = run(bin()
        .env("NATNET_ZENOH_CONFIG", "/nonexistent.toml")
        .args(["run", "--no-ros", "--rate", "0", "--replay"])
        .arg(&full));
    assert_eq!(out.status.code(), Some(1), "{}", stderr(&out));
    std::fs::remove_file(&full).unwrap();
}

#[test]
fn probe_and_record_against_the_simulated_server() {
    let (command, data) = (free_udp_port(), free_udp_port());
    let (command, data) = (command.to_string(), data.to_string());
    let mut server = bin()
        .args(["serve", "--bind", "127.0.0.1", "--unicast", "--rate", "100"])
        .args(["--command-port", &command, "--data-port", &data])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    std::thread::sleep(Duration::from_millis(300));
    let live = [
        "--server",
        "127.0.0.1",
        "--transport",
        "unicast",
        "--command-port",
        &command,
        "--data-port",
        &data,
    ];

    // Logs go to stderr, so stdout is exactly the JSON report.
    let out = run(bin().args(["probe", "--json", "-t", "0.5"]).args(live));
    assert!(out.status.success(), "{}", stderr(&out));
    let report: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert!(report["frames"].as_u64().is_some_and(|n| n > 0), "{report}");
    assert_eq!(report["server"]["protocol_version"], "4.1.0.0", "{report}");

    // The lab check of a versioned hello: the 4.1 server streams 3.1.
    let out = run(bin()
        .args(["probe", "--json", "-t", "0.5", "--request-bitstream", "3.1"])
        .args(live));
    assert!(out.status.success(), "{}", stderr(&out));
    let report: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert!(report["frames"].as_u64().is_some_and(|n| n > 0), "{report}");
    assert_eq!(report["server"]["protocol_version"], "3.1.0.0", "{report}");
    assert_eq!(report["errors"], serde_json::json!([]), "{report}");
    assert!(
        stderr(&out)
            .contains("the server reports protocol 4.1.0.0; decoding the requested 3.1.0.0"),
        "{}",
        stderr(&out)
    );

    let path = temp("record.natnet");
    let _ = std::fs::remove_file(&path);
    let record = |extra: &[&str]| {
        run(bin()
            .args(["record", "-t", "0.5", "-o"])
            .arg(&path)
            .args(extra)
            .args(live))
    };
    let out = record(&[]);
    assert!(out.status.success(), "{}", stderr(&out));
    assert!(std::fs::metadata(&path).unwrap().len() > 0);
    let out = record(&[]);
    assert_eq!(out.status.code(), Some(1), "an existing file is kept");
    assert!(stderr(&out).contains("--force"), "{}", stderr(&out));
    let out = record(&["--force"]);
    assert!(out.status.success(), "{}", stderr(&out));
    std::fs::remove_file(&path).unwrap();

    server.kill().unwrap();
    server.wait().unwrap();

    // Nothing answers any more: no frames is a failure.
    let started = Instant::now();
    let out = run(bin().args(["probe", "-t", "0.2"]).args(live));
    assert_eq!(out.status.code(), Some(1), "{}", stderr(&out));
    assert!(started.elapsed() < Duration::from_secs(10));
}

#[test]
fn types_lists_hashes() {
    let out = run(bin().arg("types"));
    assert!(out.status.success());
    let text = String::from_utf8(out.stdout).unwrap();
    assert!(
        text.lines()
            .any(|l| l.starts_with("geometry_msgs/msg/PoseStamped") && l.contains("RIHS01_"))
    );
}
