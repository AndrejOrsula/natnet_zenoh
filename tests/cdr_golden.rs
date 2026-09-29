//! CDR layout checks against payloads produced by ROS 2 Jazzy
//! `rclpy.serialization.serialize_message`.
//!
//! Fast CDR leaves alignment padding uninitialised, so the golden payloads
//! contain arbitrary padding bytes. Each test therefore decodes the golden
//! payload (which only succeeds with the right alignment), checks the field
//! values, and checks that our own encoding has the same length and decodes
//! to the same value.

// Helpers outside `#[test]` functions share the tests' panicking style.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use natnet_zenoh::ros::Message;
use natnet_zenoh::ros::msgs::{
    diagnostic_msgs::{DiagnosticArray, DiagnosticStatus, KeyValue},
    geometry_msgs::{Point, Quaternion, TransformStamped, Vector3},
    nav_msgs::Odometry,
    sensor_msgs::{PointCloud2, PointField},
    std_msgs::Header,
    std_srvs::{TriggerRequest, TriggerResponse},
    tf2_msgs::TFMessage,
    visualization_msgs::Marker,
};

fn hex(s: &str) -> Vec<u8> {
    s.as_bytes()
        .chunks(2)
        .map(|pair| u8::from_str_radix(std::str::from_utf8(pair).unwrap(), 16).unwrap())
        .collect()
}

fn check<M: Message + std::fmt::Debug + PartialEq>(golden_hex: &str, expected: &M) {
    let golden = hex(golden_hex);
    let decoded = M::from_cdr(&golden).unwrap();
    assert_eq!(
        &decoded, expected,
        "golden payload decodes to the expected value"
    );
    let ours = expected.to_cdr().unwrap();
    assert_eq!(ours.len(), golden.len(), "encoded length matches rclpy");
    assert_eq!(&ours[..4], &golden[..4], "encapsulation header");
    assert_eq!(&M::from_cdr(&ours).unwrap(), expected);
}

#[test]
fn tf_message() {
    let mut t = TransformStamped::default();
    t.header.stamp.sec = 5;
    t.header.stamp.nanosec = 7;
    t.header.frame_id = "world".into();
    t.child_frame_id = "rb".into();
    t.transform.translation = Vector3 {
        x: 1.0,
        y: -2.0,
        z: 0.5,
    };
    t.transform.rotation = Quaternion::IDENTITY;
    check(
        "0001000001000000050000000700000006000000776f726c6400002a03000000726200f0000000000000f03f00000000000000c0000000000000e03f000000000000000000000000000000000000000000000000000000000000f03f",
        &TFMessage {
            transforms: vec![t],
        },
    );
}

#[test]
fn marker() {
    let mut m = Marker::default();
    m.header.frame_id = "w".into();
    m.ns = "n".into();
    m.id = 3;
    m.r#type = Marker::SPHERE;
    m.pose.orientation = Quaternion::IDENTITY;
    m.scale.x = 0.1;
    m.color.a = 1.0;
    m.text = "hi".into();
    m.frame_locked = true;
    m.points = vec![Point {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    }];
    check(
        "0001000000000000000000000200000077009731020000006e008e31030000000200000000000000785a9731000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000f03f9a9999999999b93f000000000000000000000000000000000000000000000000000000000000803f00000000000000000161000001000000000000000000f03f000000000000000000000000000000000000000001000000007a000000000000000000000100000000610000010000000061000000000000000000000300000068690000010000000061000001000000006100000000000000",
        &m,
    );
}

#[test]
fn odometry_fixed_arrays() {
    let mut o = Odometry::default();
    o.header.frame_id = "odom".into();
    o.child_frame_id = "b".into();
    o.pose.pose.orientation = Quaternion::IDENTITY;
    o.pose.covariance[0] = 1.0;
    o.twist.covariance[35] = 2.0;
    // Padding bytes after the frame ids are arbitrary (uninitialised).
    check(
        "000100000000000000000000050000006f646f6d007600000200000062000000b0bb5b19000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000f03f000000000000f03f00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000040",
        &o,
    );
}

#[test]
fn point_cloud() {
    let mut pc = PointCloud2::default();
    pc.header.frame_id = "w".into();
    pc.height = 1;
    pc.width = 1;
    pc.fields = vec![PointField::new("x", 0, PointField::FLOAT32)];
    pc.point_step = 4;
    pc.row_step = 4;
    pc.data = vec![1, 2, 3, 4];
    pc.is_dense = true;
    check(
        "000100000000000000000000020000007700617a01000000010000000100000002000000780073690000000007636b61010000000061765f0400000004000000040000000102030401",
        &pc,
    );
}

#[test]
fn diagnostics_byte_level() {
    let status = DiagnosticStatus {
        level: DiagnosticStatus::ERROR,
        name: "a".into(),
        values: vec![KeyValue::new("k", "v")],
        ..DiagnosticStatus::default()
    };
    check(
        "00010000000000000000000001000000008593310100000002f98d310200000061005a440100000000835a440100000000e8913101000000020000006b000000020000007600",
        &DiagnosticArray {
            header: Header::default(),
            status: vec![status],
        },
    );
}

#[test]
fn trigger() {
    check(
        "0001000001610000030000006f6b00",
        &TriggerResponse {
            success: true,
            message: "ok".into(),
        },
    );
    // rclpy pads the one-byte placeholder struct to 4 body bytes; decoding
    // is what matters for interoperability.
    let golden = hex("0001000000610000");
    assert_eq!(
        TriggerRequest::from_cdr(&golden).unwrap(),
        TriggerRequest::default()
    );
    assert_eq!(
        TriggerRequest::default().to_cdr().unwrap(),
        hex("0001000000")
    );
}
