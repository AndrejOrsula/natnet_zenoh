//! ROS 2 message and service definitions used by the bridge.
//!
//! Each module mirrors one ROS 2 package; every type's `RIHS01` hash is
//! computed from its declaration and pinned against ROS 2 Jazzy in tests.

pub mod builtin_interfaces;
pub mod diagnostic_msgs;
pub mod geometry_msgs;
pub mod mocap4r2_msgs;
pub mod nav_msgs;
pub mod rcl_interfaces;
pub mod sensor_msgs;
pub mod service_msgs;
pub mod std_msgs;
pub mod std_srvs;
pub mod tf2_msgs;
pub mod type_description_interfaces;
pub mod visualization_msgs;

/// Every message and service type the bridge publishes or serves, with its
/// `RIHS01` hash, for `natnet_zenoh types`.
#[must_use]
pub fn catalog() -> Vec<(&'static str, String)> {
    use super::message::{Message, Service};
    fn m<T: Message>() -> (&'static str, String) {
        (T::TYPE_NAME, T::type_hash())
    }
    fn s<T: Service>() -> (&'static str, String) {
        (T::TYPE_NAME, T::type_hash())
    }
    vec![
        m::<builtin_interfaces::Time>(),
        m::<std_msgs::Header>(),
        m::<std_msgs::String>(),
        m::<std_msgs::Float32MultiArray>(),
        m::<geometry_msgs::PoseStamped>(),
        m::<geometry_msgs::PoseArray>(),
        m::<geometry_msgs::TransformStamped>(),
        m::<geometry_msgs::TwistStamped>(),
        m::<geometry_msgs::WrenchStamped>(),
        m::<nav_msgs::Odometry>(),
        m::<nav_msgs::Path>(),
        m::<tf2_msgs::TFMessage>(),
        m::<sensor_msgs::PointCloud2>(),
        m::<visualization_msgs::MarkerArray>(),
        m::<diagnostic_msgs::DiagnosticArray>(),
        m::<rcl_interfaces::Log>(),
        m::<mocap4r2_msgs::Markers>(),
        m::<mocap4r2_msgs::RigidBodies>(),
        m::<rcl_interfaces::ParameterEvent>(),
        s::<std_srvs::Trigger>(),
        s::<rcl_interfaces::ListParameters>(),
        s::<rcl_interfaces::GetParameters>(),
        s::<rcl_interfaces::GetParameterTypes>(),
        s::<rcl_interfaces::DescribeParameters>(),
        s::<rcl_interfaces::SetParameters>(),
        s::<rcl_interfaces::SetParametersAtomically>(),
        s::<type_description_interfaces::GetTypeDescription>(),
    ]
}
