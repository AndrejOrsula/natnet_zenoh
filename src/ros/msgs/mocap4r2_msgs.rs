//! `mocap4r2_msgs`: the MOCAP4ROS2 marker and rigid-body messages.
//!
//! Layout of the `mocap4r2_msgs` package (version 0.2.0). The package is not
//! part of a stock ROS 2 install; subscribers need it built in their
//! workspace.

use super::geometry_msgs::{Point, Pose};
use super::std_msgs::Header;

crate::ros_message! {
    /// One marker (`mocap4r2_msgs/msg/Marker`).
    pub struct Marker("mocap4r2_msgs/msg/Marker") {
        /// Which identifier is valid, one of the associated constants.
        pub id_type: i8,
        /// Marker index.
        pub marker_index: i32,
        /// Marker name.
        pub marker_name: String,
        /// Position.
        pub translation: Point,
    }
}

crate::ros_message! {
    /// Markers of one frame (`mocap4r2_msgs/msg/Markers`).
    pub struct Markers("mocap4r2_msgs/msg/Markers") {
        /// Stamp and frame.
        pub header: Header,
        /// Motion-capture frame number.
        pub frame_number: u32,
        /// The markers.
        pub markers: Vec<Marker>,
    }
}

crate::ros_message! {
    /// One rigid body (`mocap4r2_msgs/msg/RigidBody`).
    pub struct RigidBody("mocap4r2_msgs/msg/RigidBody") {
        /// Rigid-body name.
        pub rigid_body_name: String,
        /// Markers of the body.
        pub markers: Vec<Marker>,
        /// Pose of the body.
        pub pose: Pose,
    }
}

crate::ros_message! {
    /// Rigid bodies of one frame (`mocap4r2_msgs/msg/RigidBodies`).
    pub struct RigidBodies("mocap4r2_msgs/msg/RigidBodies") {
        /// Stamp and frame.
        pub header: Header,
        /// Motion-capture frame number.
        pub frame_number: u32,
        /// The bodies.
        pub rigidbodies: Vec<RigidBody>,
    }
}

impl Marker {
    /// `marker_name` identifies the marker.
    pub const USE_NAME: i8 = 0;
    /// `marker_index` identifies the marker.
    pub const USE_INDEX: i8 = 1;
    /// Both identify the marker.
    pub const USE_BOTH: i8 = 2;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    /// Hashes computed by `rosidl_generator_type_description` (Jazzy) from
    /// the package's `.msg` files at commit `2bce86f`.
    #[test]
    fn hashes_match_rosidl() {
        assert_eq!(
            Marker::type_hash(),
            "RIHS01_0cadb14b00eb93dfd5d1b774f04f13efc0c2e08a3bee1abbaef9f67f6b10b943"
        );
        assert_eq!(
            Markers::type_hash(),
            "RIHS01_36a59e85fa0dcebed1f0a91d748ca17e259b187f779e725af5c6701e8a03054a"
        );
        assert_eq!(
            RigidBody::type_hash(),
            "RIHS01_7a1bca552c93a8da4f09083ef4327410b6687a95fe14db29292c4619166f5e66"
        );
        assert_eq!(
            RigidBodies::type_hash(),
            "RIHS01_1f2d74f2a1706c59639a2349b7be91e9e4d751fad6fc3048c59d8f1f0576ea70"
        );
    }
}
