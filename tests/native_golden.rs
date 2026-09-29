//! The native zenoh JSON documents against the references in
//! `tests/golden/native/`.
//!
//! The references pin format `natnet_zenoh.v1`: renaming, removing or
//! retyping a field fails here, and so does a changed value. Each document is
//! built from the synthetic scene with fixed receive times, so it is
//! deterministic. Floats are compared after rounding to six significant
//! digits, so a last-bit difference in a platform's `sin` does not fail the
//! test. After an intended, additive change, rewrite the references with
//! `NATNET_ZENOH_BLESS=1 cargo test --test native_golden` and review the diff.

// Helpers outside `#[test]` functions share the tests' panicking style.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime};

use natnet_zenoh::bridge::native::{
    FORMAT_SCHEMA, frame_json, raw_frame_encoding, rigid_body_json,
};
use natnet_zenoh::bridge::snapshot::{Processor, Snapshot};
use natnet_zenoh::bridge::{model_json, server_info_json, stats_json};
use natnet_zenoh::config::Config;
use natnet_zenoh::source::synthetic::{Scene, SyntheticConfig};
use serde_json::Value;

/// Rounds every non-integer number to six significant digits.
fn normalize(value: &mut Value) {
    match value {
        Value::Number(n) if !(n.is_i64() || n.is_u64()) => {
            let x = n.as_f64().unwrap();
            let rounded: f64 = format!("{x:.5e}").parse().unwrap();
            *n = serde_json::Number::from_f64(rounded).unwrap();
        }
        Value::Array(items) => items.iter_mut().for_each(normalize),
        Value::Object(map) => map.values_mut().for_each(normalize),
        _ => {}
    }
}

fn check(name: &str, mut document: Value) {
    normalize(&mut document);
    let text = serde_json::to_string_pretty(&document).unwrap() + "\n";
    let path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"),
        "tests",
        "golden",
        "native",
        name,
    ]
    .iter()
    .collect();
    if std::env::var_os("NATNET_ZENOH_BLESS").is_some() {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, &text).unwrap();
        return;
    }
    let golden = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}; bless it first", path.display()));
    assert!(
        golden == text,
        "{name} differs from its reference; a breaking change needs a new format version\n\
         --- reference\n{golden}\n--- now\n{text}"
    );
}

/// The synthetic scene's model and two consecutive frames, processed as the
/// bridge does with native outputs enabled.
fn scene() -> (Scene, Processor, Snapshot) {
    let scene = Scene::new(SyntheticConfig::default()).unwrap();
    let mut cfg = Config::default();
    cfg.native.enabled = true;
    let mut processor = Processor::new(&cfg);
    processor.set_server_info(&scene.server_info(true, 1511, [239, 255, 42, 99]));
    // The model goes through the wire, as a live server's does.
    let version = scene.version();
    let datagram = scene.model_def().to_datagram_version(version).unwrap();
    let natnet::Message::ModelDef(model) = natnet::Decoder::new(version)
        .unwrap()
        .parse(&datagram)
        .unwrap()
        .into_message()
    else {
        panic!("not a MODELDEF");
    };
    let errors = processor.set_model(&model, version);
    assert!(errors.is_empty(), "{errors:?}");
    let wall = SystemTime::UNIX_EPOCH + Duration::from_secs(1_800_000_007);
    let mono = Instant::now();
    let period = Duration::from_secs_f64(1.0 / scene.config().rate_hz);
    let mut snap = None;
    for (i, frame_number) in [240, 241].into_iter().enumerate() {
        let late = period * u32::try_from(i).unwrap();
        snap = processor.process(&scene.frame(frame_number), wall + late, mono + late);
    }
    (scene, processor, snap.unwrap())
}

#[test]
fn schema_names_the_format_version() {
    assert_eq!(FORMAT_SCHEMA, "natnet_zenoh.v1");
    assert_eq!(
        natnet_zenoh::bridge::native::json_encoding().to_string(),
        "application/json;natnet_zenoh.v1"
    );
    assert_eq!(
        raw_frame_encoding(natnet::Version::V4_1).to_string(),
        "application/octet-stream;natnet.4.1.0.0"
    );
    assert_eq!(
        raw_frame_encoding(natnet::Version::new(3, 1, 0, 0)).to_string(),
        "application/octet-stream;natnet.3.1.0.0"
    );
}

#[test]
fn server_info() {
    let scene = Scene::new(SyntheticConfig::default()).unwrap();
    check(
        "server_info.json",
        server_info_json(&scene.server_info(true, 1511, [239, 255, 42, 99])),
    );
}

#[test]
fn model() {
    let (_, processor, _) = scene();
    check("model.json", model_json(processor.model()));
}

#[test]
fn frame() {
    let (_, _, snap) = scene();
    check(
        "frame.json",
        serde_json::from_slice(&frame_json(&snap).unwrap()).unwrap(),
    );
}

#[test]
fn rigid_body() {
    let (_, _, snap) = scene();
    check(
        "rigid_body.json",
        serde_json::from_slice(&rigid_body_json(&snap, &snap.bodies[0]).unwrap()).unwrap(),
    );
}

#[test]
fn stats() {
    let (_, processor, _) = scene();
    check("stats.json", stats_json(&processor.stats, "streaming", 0));
}
