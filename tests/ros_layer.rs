//! In-process tests of the ROS 2 layer over real zenoh sessions linked by
//! TCP on the loopback interface.

// Shared helpers use panicking assertions.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::net::TcpListener;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use natnet_zenoh::ros::attachment::{
    ATTACHMENT_LEN, Attachment, AttachmentFormat, LEGACY_ATTACHMENT_LEN,
};
use natnet_zenoh::ros::context::RmwZenohCompat;
use natnet_zenoh::ros::liveliness::EntityKind;
use natnet_zenoh::ros::msgs::rcl_interfaces::Log;
use natnet_zenoh::ros::msgs::std_msgs;
use natnet_zenoh::ros::msgs::std_srvs::{Trigger, TriggerRequest, TriggerResponse};
use natnet_zenoh::ros::rosout::{RosoutLayer, rosout_qos};
use natnet_zenoh::ros::{
    Context, ContextOptions, Gid, Message, QosProfile, RosError, SessionConfig,
};
use tracing_subscriber::layer::SubscriberExt;
use zenoh::Wait;

fn free_port() -> u16 {
    TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

/// Two peer contexts: the first listens, the second connects to it.
fn pair(domain_id: u32) -> (Context, Context) {
    pair_with(domain_id, RmwZenohCompat::Current)
}

/// Like [`pair`], both speaking the given `rmw_zenoh_cpp` revision.
fn pair_with(domain_id: u32, compat: RmwZenohCompat) -> (Context, Context) {
    let base = |extra: &str| {
        SessionConfig::Json5(format!(
            r#"{{ mode: "peer", scouting: {{ multicast: {{ enabled: false }}, gossip: {{ enabled: false }} }}, timestamping: {{ enabled: true }}, {extra} }}"#
        ))
    };
    let open = |extra: String| {
        Context::new(&ContextOptions {
            domain_id,
            config: base(&extra),
            router_check_attempts: None,
            compat,
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
    // Wait for the link.
    let deadline = Instant::now() + Duration::from_secs(5);
    while b.session().info().peers_zid().wait_list().is_empty() {
        assert!(Instant::now() < deadline, "peers did not connect");
        std::thread::sleep(Duration::from_millis(20));
    }
    (a, b)
}

trait WaitList {
    fn wait_list(self) -> Vec<zenoh::session::ZenohId>;
}

impl<T: zenoh::Wait<To = I>, I: Iterator<Item = zenoh::session::ZenohId>> WaitList for T {
    fn wait_list(self) -> Vec<zenoh::session::ZenohId> {
        self.wait().collect()
    }
}

fn msg(s: &str) -> std_msgs::String {
    std_msgs::String { data: s.into() }
}

#[test]
fn pub_sub_across_sessions() {
    let (a, b) = pair(11);
    let pub_node = a.create_node("talker", "/test").unwrap();
    let sub_node = b.create_node("listener", "/test").unwrap();
    let (tx, rx) = mpsc::channel();
    let _sub = sub_node
        .create_subscription::<std_msgs::String, _>(
            "chatter",
            QosProfile::default(),
            move |m, info| {
                tx.send((m, info.attachment)).unwrap();
            },
        )
        .unwrap();
    let publisher = pub_node
        .create_publisher::<std_msgs::String>("chatter", QosProfile::default())
        .unwrap();
    assert_eq!(publisher.raw().topic_name(), "/test/chatter");

    // Publish until the subscriber is matched and receives.
    let deadline = Instant::now() + Duration::from_secs(5);
    let (m, att) = loop {
        publisher.publish(&msg("hello")).unwrap();
        if let Ok(r) = rx.recv_timeout(Duration::from_millis(100)) {
            break r;
        }
        assert!(Instant::now() < deadline, "no message received");
    };
    assert_eq!(m.data, "hello");
    let att = att.expect("attachment");
    assert_eq!(att.source_gid, publisher.raw().gid());
    assert!(att.sequence_number >= 1);
}

#[test]
fn transient_local_late_joiner() {
    // Advanced pub/sub with and without sample-miss recovery, and the
    // Jazzy 0.2.0 cache.
    for (domain, compat) in [
        (12, RmwZenohCompat::Current),
        (18, RmwZenohCompat::Humble),
        (17, RmwZenohCompat::Jazzy023),
        (15, RmwZenohCompat::Jazzy022),
        (16, RmwZenohCompat::Jazzy020),
    ] {
        late_joiner(domain, compat);
    }
}

fn late_joiner(domain: u32, compat: RmwZenohCompat) {
    let (a, b) = pair_with(domain, compat);
    let pub_node = a.create_node("latched", "/").unwrap();
    let publisher = pub_node
        .create_publisher::<std_msgs::String>("/latched", QosProfile::transient_local(2))
        .unwrap();
    for s in ["one", "two", "three"] {
        publisher.publish(&msg(s)).unwrap();
    }
    std::thread::sleep(Duration::from_millis(200));

    // The token encodes the QoS against the release's default durability,
    // and the other side reads it back as transient local.
    let qos = match compat {
        RmwZenohCompat::Current | RmwZenohCompat::Humble | RmwZenohCompat::Jazzy023 => {
            ":1:,2:,:,:,,"
        }
        RmwZenohCompat::Jazzy022 | RmwZenohCompat::Jazzy020 => "::,2:,:,:,,",
    };
    // Humble has no type hashes; the others carry RIHS01.
    let hash = if compat == RmwZenohCompat::Humble {
        "/TypeHashNotSupported/"
    } else {
        "/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/"
    };
    let tokens: Vec<String> = b
        .session()
        .liveliness()
        .get(format!("@ros2_lv/{domain}/*/*/*/MP/**"))
        .wait()
        .unwrap()
        .iter()
        .filter_map(|r| r.result().ok().map(|s| s.key_expr().to_string()))
        .filter(|k| k.contains("/%latched/"))
        .collect();
    assert_eq!(tokens.len(), 1, "{tokens:?}");
    assert!(
        tokens[0].ends_with(&format!("/{qos}")) && tokens[0].contains(hash),
        "{compat:?}: {tokens:?}"
    );
    let seen = b.graph(Duration::from_secs(2)).unwrap();
    let latched = seen
        .iter()
        .find_map(|e| e.topic.as_ref().filter(|t| t.name == "/latched"))
        .unwrap();
    assert_eq!(
        latched.qos,
        QosProfile::transient_local(2).adapted(compat.qos_defaults())
    );

    let sub_node = b.create_node("late", "/").unwrap();
    let (tx, rx) = mpsc::channel();
    let _sub = sub_node
        .create_subscription::<std_msgs::String, _>(
            "/latched",
            QosProfile::transient_local(10),
            move |m, _| tx.send(m.data).unwrap(),
        )
        .unwrap();
    let mut got = Vec::new();
    while let Ok(s) = rx.recv_timeout(Duration::from_secs(2)) {
        got.push(s);
        if got.len() == 2 {
            break;
        }
    }
    assert_eq!(
        got,
        ["two", "three"],
        "{compat:?}: depth-2 cache replayed in order"
    );
    // A live message still arrives, and no duplicates do.
    publisher.publish(&msg("four")).unwrap();
    assert_eq!(
        rx.recv_timeout(Duration::from_secs(2)).as_deref(),
        Ok("four"),
        "{compat:?}"
    );
    assert!(
        rx.recv_timeout(Duration::from_millis(300)).is_err(),
        "{compat:?}"
    );
}

#[test]
fn service_call_and_graph() {
    let (a, b) = pair(13);
    let server_node = a.create_node("server", "/").unwrap();
    let server = server_node
        .create_service::<Trigger, _>("~/trigger", |_req, info| TriggerResponse {
            success: true,
            message: format!("seq {}", info.attachment.sequence_number),
        })
        .unwrap();
    let client_node = b.create_node("client", "/").unwrap();
    let client = client_node
        .create_client::<Trigger>("/server/trigger")
        .unwrap();

    let deadline = Instant::now() + Duration::from_secs(5);
    while !client.service_is_ready(Duration::from_millis(500)).unwrap() {
        assert!(Instant::now() < deadline, "service never appeared");
    }
    let r = client
        .call(&TriggerRequest::default(), Duration::from_secs(2))
        .unwrap();
    assert!(r.success);
    assert_eq!(r.message, "seq 1");
    let r = client
        .call(&TriggerRequest::default(), Duration::from_secs(2))
        .unwrap();
    assert_eq!(r.message, "seq 2");

    let graph = b.graph(Duration::from_secs(1)).unwrap();
    assert!(
        graph
            .iter()
            .any(|e| e.kind == EntityKind::Node && e.node.name == "server")
    );
    assert!(
        graph.iter().any(|e| e.kind == EntityKind::Service
            && e.topic.as_ref().unwrap().name == "/server/trigger")
    );
    assert!(
        graph
            .iter()
            .any(|e| e.kind == EntityKind::Client && e.node.name == "client")
    );

    // A call to a missing service times out.
    let missing = client_node.create_client::<Trigger>("/nobody").unwrap();
    assert!(matches!(
        missing.call(&TriggerRequest::default(), Duration::from_millis(300)),
        Err(RosError::Timeout)
    ));

    // The server answers each client in its own attachment format.
    let key = server.entity().topic_keyexpr().unwrap();
    for (format, len) in [
        (AttachmentFormat::Current, ATTACHMENT_LEN),
        (AttachmentFormat::Legacy, LEGACY_ATTACHMENT_LEN),
    ] {
        let replies = b
            .session()
            .get(key.clone())
            .payload(TriggerRequest::default().to_cdr().unwrap())
            .attachment(Attachment::now(7, Gid([3; 16])).encode_as(format))
            .timeout(Duration::from_secs(2))
            .wait()
            .unwrap();
        let reply = replies.recv().unwrap();
        let sample = reply.result().unwrap();
        let bytes = sample.attachment().unwrap().to_bytes().into_owned();
        assert_eq!(bytes.len(), len, "{format:?}");
        let (attachment, got) = Attachment::decode_format(&bytes).unwrap();
        assert_eq!(got, format);
        assert_eq!(attachment.sequence_number, 7);
        assert_eq!(attachment.source_gid, Gid([3; 16]));
    }
}

#[test]
fn rosout_worker_lives_with_its_guard() {
    let (a, b) = pair(14);
    let (layer, mut handle) = RosoutLayer::new(tracing::Level::INFO, &["ros_layer"]);
    let subscriber = tracing_subscriber::registry().with(layer);
    let node = a.create_node("logger", "/").unwrap();
    let listener = b.create_node("console", "/").unwrap();
    let (tx, rx) = mpsc::channel();
    let _sub = listener
        .create_subscription::<Log, _>("/rosout", rosout_qos(), move |m, _| {
            let _ = tx.send(m);
        })
        .unwrap();
    let rosout_publishers = || {
        b.graph(Duration::from_millis(500))
            .unwrap()
            .into_iter()
            .filter(|e| {
                e.kind == EntityKind::Publisher
                    && e.node.name == "logger"
                    && e.topic.as_ref().is_some_and(|t| t.name == "/rosout")
            })
            .count()
    };

    tracing::subscriber::with_default(subscriber, || {
        let worker = handle.attach(&node).unwrap().expect("first attach");
        assert!(handle.attach(&node).unwrap().is_none(), "already attached");
        let deadline = Instant::now() + Duration::from_secs(5);
        let log = loop {
            tracing::info!("hello rosout");
            if let Ok(log) = rx.recv_timeout(Duration::from_millis(100)) {
                break log;
            }
            assert!(Instant::now() < deadline, "no /rosout message");
        };
        assert_eq!(log.name, "logger");
        assert_eq!(log.msg, "hello rosout");
        assert_eq!(rosout_publishers(), 1);

        drop(worker);
        let deadline = Instant::now() + Duration::from_secs(5);
        while rosout_publishers() != 0 {
            assert!(
                Instant::now() < deadline,
                "the /rosout publisher outlived its worker"
            );
        }
        // The handle can attach again once the worker is gone.
        let again = handle.attach(&node).unwrap();
        assert!(again.is_some());
    });
}
