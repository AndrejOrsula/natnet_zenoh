//! A minimal ROS 2 node on the pure-Rust `rmw_zenoh` layer.
//!
//! Publishes `std_msgs/String` on `/chatter`, a transient-local `/greeting`,
//! echoes `/listener_input` and serves `std_srvs/Trigger` on
//! `/talker/trigger`. Start `ros2 run rmw_zenoh_cpp rmw_zenohd`, then
//! `cargo run --example ros_talker` and inspect it with the `ros2` CLI under
//! `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.

use std::time::Duration;

use natnet_zenoh::ros::msgs::std_msgs;
use natnet_zenoh::ros::msgs::std_srvs::{Trigger, TriggerResponse};
use natnet_zenoh::ros::{Context, ContextOptions, QosProfile};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();
    let ctx = Context::new(&ContextOptions::from_env()?)?;
    let node = ctx.create_node("talker", "/")?;
    let chatter = node.create_publisher::<std_msgs::String>("/chatter", QosProfile::default())?;
    println!("/chatter GID: {}", chatter.raw().gid());
    let greeting =
        node.create_publisher::<std_msgs::String>("/greeting", QosProfile::transient_local(1))?;
    greeting.publish(&std_msgs::String {
        data: "hello from natnet_zenoh".into(),
    })?;
    let _echo = node.create_subscription::<std_msgs::String, _>(
        "/listener_input",
        QosProfile::default(),
        |m, _| println!("received: {}", m.data),
    )?;
    let _trigger = node.create_service::<Trigger, _>("~/trigger", |_, info| TriggerResponse {
        success: true,
        message: format!("triggered (request {})", info.attachment.sequence_number),
    })?;
    for i in 0_u64.. {
        chatter.publish(&std_msgs::String {
            data: format!("hello {i}"),
        })?;
        std::thread::sleep(Duration::from_millis(100));
    }
    Ok(())
}
