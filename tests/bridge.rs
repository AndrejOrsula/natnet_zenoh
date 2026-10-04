//! End-to-end: a synthetic or replayed NatNet source through the bridge to
//! ROS 2 and native zenoh consumers in a second in-process zenoh session.

// Shared helpers use panicking assertions.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::time::{Duration, Instant};

use natnet_zenoh::bridge::Bridge;
use natnet_zenoh::config::{Config, SourceKind};
use natnet_zenoh::ros::msgs::geometry_msgs::PoseStamped;
use natnet_zenoh::ros::msgs::rcl_interfaces::{
    GetParameters, GetParametersRequest, Parameter, ParameterValue, SetParameters,
    SetParametersRequest,
};
use natnet_zenoh::ros::msgs::std_msgs;
use natnet_zenoh::ros::msgs::std_srvs::{Trigger, TriggerRequest};
use natnet_zenoh::ros::msgs::tf2_msgs::TFMessage;
use natnet_zenoh::ros::msgs::type_description_interfaces::{
    GetTypeDescription, GetTypeDescriptionRequest,
};
use natnet_zenoh::ros::{Context, ContextOptions, Message, QosProfile, SessionConfig};
use natnet_zenoh::source::capture::{CaptureWriter, Format, Record};
use natnet_zenoh::source::synthetic::{Scene, SyntheticConfig};
use zenoh::Wait;

fn free_port() -> u16 {
    TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

/// Two peer contexts linked over loopback TCP.
fn pair(domain_id: u32) -> (Context, Context) {
    let base = |extra: &str| {
        SessionConfig::Json5(format!(
            r#"{{ mode: "peer", scouting: {{ multicast: {{ enabled: false }}, gossip: {{ enabled: false }} }}, {extra} }}"#
        ))
    };
    let open = |extra: String| {
        Context::new(&ContextOptions {
            domain_id,
            config: base(&extra),
            router_check_attempts: None,
            ..ContextOptions::default()
        })
    };
    // The probed port is free only until zenoh binds it; another test may
    // take it in between, so retry with a fresh one.
    let mut last_error = None;
    let (a, port) = (0..5)
        .find_map(|_| {
            let port = free_port();
            open(format!(
                r#"listen: {{ endpoints: ["tcp/127.0.0.1:{port}"], exit_on_failure: true }}"#
            ))
            .map_err(|e| last_error = Some(e))
            .ok()
            .map(|a| (a, port))
        })
        .unwrap_or_else(|| panic!("no listening peer after 5 ports: {last_error:?}"));
    let b = open(format!(
        r#"listen: {{ endpoints: [] }}, connect: {{ endpoints: ["tcp/127.0.0.1:{port}"] }}"#
    ))
    .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    while b.session().info().peers_zid().wait().next().is_none() {
        assert!(Instant::now() < deadline, "peers did not connect");
        std::thread::sleep(Duration::from_millis(20));
    }
    (a, b)
}

fn wait_for<T>(rx: &mpsc::Receiver<T>, what: &str) -> T {
    rx.recv_timeout(Duration::from_secs(10))
        .unwrap_or_else(|_| panic!("no {what}"))
}

#[test]
fn synthetic_to_ros_and_native() {
    let (a, b) = pair(21);
    let mut cfg = Config::default();
    cfg.source.kind = SourceKind::Synthetic;
    cfg.source.synthetic.rate_hz = 200.0;
    cfg.ros.namespace = "/e2e".into();
    cfg.native.enabled = true;
    cfg.native.prefix = "e2e_natnet".into();
    let mut bridge = Bridge::new(cfg, Some(a), None).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = Arc::clone(&stop);
        std::thread::spawn(move || bridge.run(&stop))
    };

    let node = b.create_node("consumer", "/e2e").unwrap();
    let (tx_tf, rx_tf) = mpsc::channel();
    let _tf = node
        .create_subscription::<TFMessage, _>("/tf", QosProfile::keep_last(100), move |m, _| {
            let _ = tx_tf.send(m);
        })
        .unwrap();
    let (tx_pose, rx_pose) = mpsc::channel();
    let _pose = node
        .create_subscription::<PoseStamped, _>(
            "natnet/rigid_bodies/body_1/pose",
            QosProfile::default(),
            move |m, _| {
                let _ = tx_pose.send(m);
            },
        )
        .unwrap();
    let (tx_info, rx_info) = mpsc::channel();
    let _info = node
        .create_subscription::<std_msgs::String, _>(
            "natnet/server_info",
            QosProfile::transient_local(1),
            move |m, _| {
                let _ = tx_info.send(m);
            },
        )
        .unwrap();
    let native_seen = Arc::new(Mutex::new(Vec::new()));
    let _native = {
        let seen = Arc::clone(&native_seen);
        b.session()
            .declare_subscriber("e2e_natnet/rigid_bodies/body_1")
            .callback(move |s| seen.lock().unwrap().push(s.payload().to_bytes().to_vec()))
            .wait()
            .unwrap()
    };

    let tf = wait_for(&rx_tf, "tf");
    let children: Vec<&str> = tf
        .transforms
        .iter()
        .map(|t| t.child_frame_id.as_str())
        .collect();
    assert!(children.contains(&"body_1"), "{children:?}");
    assert!(tf.transforms.iter().all(|t| t.header.frame_id == "mocap"));

    let pose = wait_for(&rx_pose, "pose");
    assert_eq!(pose.header.frame_id, "mocap");
    let q = pose.pose.orientation;
    assert!(((q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) - 1.0).abs() < 1e-6);

    let info: serde_json::Value =
        serde_json::from_str(&wait_for(&rx_info, "server info").data).unwrap();
    assert_eq!(info["protocol_version"], "4.1.0.0", "{info}");

    // Services: parameters, type descriptions and a trigger.
    let get = node
        .create_client::<GetParameters>("natnet/get_parameters")
        .unwrap();
    let ask = |names: &[&str]| {
        get.call(
            &GetParametersRequest {
                names: names.iter().map(|n| (*n).to_owned()).collect(),
            },
            Duration::from_secs(5),
        )
        .unwrap()
        .values
    };
    let values = ask(&["ros.world_frame", "rigid_bodies.path_length"]);
    assert_eq!(values[0].string_value, "mocap");
    assert_eq!(values[1].r#type, ParameterValue::INTEGER);
    // Like rclcpp, an undeclared name empties the whole answer.
    let answer = ask(&["ros.world_frame", "nope"]);
    assert!(answer.is_empty(), "{answer:?}");
    let set = node
        .create_client::<SetParameters>("natnet/set_parameters")
        .unwrap();
    let reply = set
        .call(
            &SetParametersRequest {
                parameters: vec![Parameter {
                    name: "ros.world_frame".into(),
                    value: ParameterValue::default(),
                }],
            },
            Duration::from_secs(5),
        )
        .unwrap();
    assert!(!reply.results[0].successful);
    // A writable key: excluding a body withdraws it from /tf.
    let string_array = |items: &[&str]| ParameterValue {
        r#type: ParameterValue::STRING_ARRAY,
        string_array_value: items.iter().map(|s| (*s).to_owned()).collect(),
        ..ParameterValue::default()
    };
    let reply = set
        .call(
            &SetParametersRequest {
                parameters: vec![
                    Parameter {
                        name: "rigid_bodies.exclude".into(),
                        value: string_array(&["body_1"]),
                    },
                    Parameter {
                        name: "timing.max_rate_hz".into(),
                        value: ParameterValue {
                            r#type: ParameterValue::INTEGER,
                            integer_value: 30,
                            ..ParameterValue::default()
                        },
                    },
                ],
            },
            Duration::from_secs(5),
        )
        .unwrap();
    assert!(reply.results[0].successful, "{}", reply.results[0].reason);
    assert!(
        reply.results[1].reason.contains("is of type {double}"),
        "{}",
        reply.results[1].reason
    );
    assert_eq!(
        ask(&["rigid_bodies.exclude"])[0].string_array_value,
        vec!["body_1".to_owned()]
    );
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let tf = wait_for(&rx_tf, "tf after exclusion");
        if tf.transforms.iter().all(|t| t.child_frame_id != "body_1") {
            break;
        }
        assert!(Instant::now() < deadline, "body_1 is still published");
    }
    let describe = node
        .create_client::<GetTypeDescription>("natnet/get_type_description")
        .unwrap();
    let reply = describe
        .call(
            &GetTypeDescriptionRequest {
                type_name: PoseStamped::TYPE_NAME.into(),
                type_hash: PoseStamped::type_hash(),
                include_type_sources: false,
            },
            Duration::from_secs(5),
        )
        .unwrap();
    assert!(reply.successful, "{}", reply.failure_reason);
    assert_eq!(
        reply.type_description.type_description.type_name.0,
        PoseStamped::TYPE_NAME
    );
    let refresh = node
        .create_client::<Trigger>("natnet/refresh_model")
        .unwrap();
    assert!(
        refresh
            .call(&TriggerRequest::default(), Duration::from_secs(5))
            .unwrap()
            .success
    );

    let deadline = Instant::now() + Duration::from_secs(10);
    while native_seen.lock().unwrap().is_empty() {
        assert!(Instant::now() < deadline, "no native rigid body sample");
        std::thread::sleep(Duration::from_millis(20));
    }
    let body: serde_json::Value = serde_json::from_slice(&native_seen.lock().unwrap()[0]).unwrap();
    assert_eq!(body["body"]["name"], "body_1", "{body}");

    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    assert!(summary.frames > 0, "{summary:?}");
    assert_eq!(summary.decode_errors, 0);
    assert!(!summary.finished);
}

#[test]
fn replay_runs_to_the_end_without_ros() {
    let scene = Scene::new(SyntheticConfig::default()).unwrap();
    let path =
        std::env::temp_dir().join(format!("natnet_zenoh_bridge_{}.natnet", std::process::id()));
    let mut w = CaptureWriter::create(&path, Format::NatNet).unwrap();
    let md = scene.model_def().to_datagram(scene.version()).unwrap();
    w.write(&Record {
        time_ns: 0,
        source: None,
        bytes: md,
    })
    .unwrap();
    for n in 0..300 {
        let bytes = scene.frame_datagram(n).unwrap();
        w.write(&Record {
            time_ns: 1 + u64::try_from(n).unwrap() * 1_000_000,
            source: None,
            bytes,
        })
        .unwrap();
    }
    drop(w);

    let mut cfg = Config::default();
    cfg.source.kind = SourceKind::Replay;
    cfg.source.replay.path = path.clone();
    cfg.source.replay.rate = 0.0;
    cfg.source.replay.protocol_version = Some("4.1".into());
    cfg.ros.enabled = false;
    let mut bridge = Bridge::new(cfg, None, None).unwrap();
    assert!(bridge.context().is_none());
    let summary = bridge.run(&AtomicBool::new(false));
    std::fs::remove_file(&path).unwrap();
    assert!(summary.finished, "{summary:?}");
    assert_eq!(summary.frames, 300, "{summary:?}");
    assert_eq!(
        (
            summary.queue_drops,
            summary.dropped_frames,
            summary.decode_errors
        ),
        (0, 0, 0)
    );
}

#[test]
fn native_outputs_without_ros() {
    let (a, b) = pair(22);
    let mut cfg = Config::default();
    cfg.source.kind = SourceKind::Synthetic;
    cfg.source.synthetic.rate_hz = 200.0;
    cfg.ros.enabled = false;
    cfg.native.enabled = true;
    cfg.native.raw = true;
    cfg.native.prefix = "e2e_native".into();
    let mut bridge = Bridge::new(cfg, Some(a), None).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = Arc::clone(&stop);
        std::thread::spawn(move || bridge.run(&stop))
    };

    let subscribe = |key: &str| {
        let (tx, rx) = mpsc::channel();
        let (etx, erx) = mpsc::channel();
        let sub = b
            .session()
            .declare_subscriber(key.to_owned())
            .callback(move |s| {
                let _ = etx.send(s.encoding().to_string());
                let _ = tx.send(s.payload().to_bytes().to_vec());
            })
            .wait()
            .unwrap();
        (sub, rx, erx)
    };
    let (_frame_sub, frames, frame_encodings) = subscribe("e2e_native/frame");
    let (_raw_sub, raws, raw_encodings) = subscribe("e2e_native/raw/frame");

    let frame: serde_json::Value =
        serde_json::from_slice(&wait_for(&frames, "frame JSON")).unwrap();
    assert!(frame["frame_number"].is_i64(), "{frame}");
    assert_eq!(frame["bodies"].as_array().map(Vec::len), Some(3), "{frame}");

    // The raw key carries a NatNet datagram the vendor decoder accepts.
    let raw = wait_for(&raws, "raw frame");
    let decoded = natnet::Decoder::new(natnet::Version::V4_1)
        .unwrap()
        .parse(&raw)
        .unwrap()
        .into_message();
    assert!(
        matches!(decoded, natnet::Message::FrameOfData(_)),
        "{decoded:?}"
    );
    // Samples name the format version, and raw frames their NatNet version.
    assert_eq!(
        wait_for(&frame_encodings, "frame encoding"),
        "application/json;natnet_zenoh.v1"
    );
    assert_eq!(
        wait_for(&raw_encodings, "raw frame encoding"),
        "application/octet-stream;natnet.4.1.0.0"
    );
    let alive: Vec<String> = b
        .session()
        .liveliness()
        .get("e2e_native/alive")
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap()
        .iter()
        .filter_map(|r| r.result().ok().map(|s| s.key_expr().to_string()))
        .collect();
    assert_eq!(alive, ["e2e_native/alive"]);
    let encodings: Vec<String> = b
        .session()
        .get("e2e_native/server_info")
        .timeout(Duration::from_secs(5))
        .wait()
        .unwrap()
        .iter()
        .filter_map(|r| r.result().ok().map(|s| s.encoding().to_string()))
        .collect();
    assert_eq!(encodings, ["application/json;natnet_zenoh.v1"]);

    let get = |key: &str| -> Vec<Vec<u8>> {
        b.session()
            .get(key.to_owned())
            .timeout(Duration::from_secs(5))
            .wait()
            .unwrap()
            .iter()
            .filter_map(|r| r.result().ok().map(|s| s.payload().to_bytes().to_vec()))
            .collect()
    };
    let deadline = Instant::now() + Duration::from_secs(10);
    let model = loop {
        let replies = get("e2e_native/model");
        let model: serde_json::Value = serde_json::from_slice(&replies[0]).unwrap();
        if model["rigid_bodies"]
            .as_object()
            .is_some_and(|m| m.len() == 3)
        {
            break model;
        }
        assert!(Instant::now() < deadline, "no model: {model}");
        std::thread::sleep(Duration::from_millis(50));
    };
    assert_eq!(
        model["skeletons"].as_object().map(serde_json::Map::len),
        Some(1)
    );
    let info: serde_json::Value =
        serde_json::from_slice(&get("e2e_native/server_info")[0]).unwrap();
    assert_eq!(info["protocol_version"], "4.1.0.0", "{info}");
    // Stats are stored every `diagnostics.period_secs`.
    let deadline = Instant::now() + Duration::from_secs(10);
    let stats = loop {
        let stats: serde_json::Value = serde_json::from_slice(&get("e2e_native/stats")[0]).unwrap();
        if !stats.is_null() {
            break stats;
        }
        assert!(Instant::now() < deadline, "no stats");
        std::thread::sleep(Duration::from_millis(100));
    };
    assert!(stats["frames"].as_u64().is_some_and(|n| n > 0), "{stats}");
    assert_eq!(get("e2e_native/refresh_model"), [b"requested".to_vec()]);

    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    assert!(summary.frames > 0, "{summary:?}");
    assert_eq!(summary.decode_errors, 0);
}
