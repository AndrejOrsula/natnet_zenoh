//! ROS entity identities stay unique across wrappers of one zenoh session.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use natnet_zenoh::ros::msgs::std_msgs;
use natnet_zenoh::ros::{Context, ContextOptions, QosProfile, SessionConfig};

fn options() -> ContextOptions {
    ContextOptions {
        config: SessionConfig::Json5(
            r#"{ mode: "peer", scouting: { multicast: { enabled: false }, gossip: { enabled: false } }, timestamping: { enabled: true }, listen: { endpoints: [] } }"#.into(),
        ),
        router_check_attempts: None,
        ..ContextOptions::default()
    }
}

#[test]
fn wrappers_and_context_clones_get_distinct_ros_entities() {
    let original = Context::new(&options()).unwrap();
    let session = original.session().clone();
    let first = Context::from_session(session.clone(), &options());
    let second = Context::from_session(session, &options());

    let first_node = first.create_node("same", "/identity").unwrap();
    let second_node = second.create_node("same", "/identity").unwrap();
    assert_eq!(first_node.context().zid(), second_node.context().zid());
    assert_ne!(first_node.gid(), second_node.gid());

    let first_pub = first_node
        .create_publisher::<std_msgs::String>("same", QosProfile::default())
        .unwrap();
    let second_pub = second_node
        .create_publisher::<std_msgs::String>("same", QosProfile::default())
        .unwrap();
    let defaults = first.compat().qos_defaults();
    let first_entity = first_pub.raw().entity();
    let second_entity = second_pub.raw().entity();
    assert_ne!(first_entity.id, second_entity.id);
    assert_ne!(
        first_entity.liveliness_keyexpr(defaults),
        second_entity.liveliness_keyexpr(defaults)
    );
    assert_ne!(first_pub.raw().gid(), second_pub.raw().gid());
    // Topic keys identify ROS topics and remain shared across publishers.
    assert_eq!(first_pub.raw().keyexpr(), second_pub.raw().keyexpr());

    let clone_node = first.clone().create_node("same", "/identity").unwrap();
    assert_ne!(first_node.gid(), clone_node.gid());
    assert_eq!(first_node.context().zid(), clone_node.context().zid());
    let clone_pub = clone_node
        .create_publisher::<std_msgs::String>("same", QosProfile::default())
        .unwrap();
    let clone_entity = clone_pub.raw().entity();
    assert_ne!(first_entity.id, clone_entity.id);
    assert_ne!(
        first_entity.liveliness_keyexpr(defaults),
        clone_entity.liveliness_keyexpr(defaults)
    );
    assert_ne!(first_pub.raw().gid(), clone_pub.raw().gid());

    drop((
        clone_pub,
        clone_node,
        first_pub,
        second_pub,
        first_node,
        second_node,
    ));
    drop((first, second, original));
}
