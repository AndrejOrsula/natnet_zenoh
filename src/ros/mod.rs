//! A pure-Rust ROS 2 client layer that speaks the `rmw_zenoh_cpp` protocol.
//!
//! Stock ROS 2 nodes using `RMW_IMPLEMENTATION=rmw_zenoh_cpp` discover and
//! exchange data with the entities declared here without any ROS 2 install
//! on this side: the key expressions, liveliness tokens, attachments, QoS
//! encoding, type hashes and CDR payloads are reproduced byte for byte.

pub mod attachment;
pub mod cdr;
pub mod context;
pub mod gid;
pub mod introspection;
pub mod liveliness;
pub mod message;
pub mod msgs;
pub mod names;
pub mod node;
pub mod parameters;
pub mod qos;
pub mod rosout;
pub mod type_description;

pub use context::{Context, ContextOptions, RosError, SessionConfig};
pub use gid::Gid;
pub use message::{Message, Service};
pub use node::{
    Client, MessageInfo, Node, Publisher, RawPublisher, RequestInfo, ServiceServer, Subscription,
};
pub use qos::QosProfile;
