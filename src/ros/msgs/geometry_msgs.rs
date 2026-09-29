//! `geometry_msgs`: points, poses, transforms, twists and wrenches.

use super::std_msgs::Header;

crate::ros_message! {
    /// A free vector (`geometry_msgs/msg/Vector3`).
    pub struct Vector3("geometry_msgs/msg/Vector3") {
        /// X.
        pub x: f64,
        /// Y.
        pub y: f64,
        /// Z.
        pub z: f64,
    }
}

crate::ros_message! {
    /// A position (`geometry_msgs/msg/Point`).
    pub struct Point("geometry_msgs/msg/Point") {
        /// X.
        pub x: f64,
        /// Y.
        pub y: f64,
        /// Z.
        pub z: f64,
    }
}

crate::ros_message! {
    /// An orientation as a unit quaternion (`geometry_msgs/msg/Quaternion`).
    /// [`Default`] is the identity, as the IDL defaults say.
    pub struct Quaternion("geometry_msgs/msg/Quaternion") {
        /// X.
        pub x: f64 = 0.0,
        /// Y.
        pub y: f64 = 0.0,
        /// Z.
        pub z: f64 = 0.0,
        /// W (scalar part).
        pub w: f64 = 1.0,
    }
}

crate::ros_message! {
    /// A position and orientation (`geometry_msgs/msg/Pose`).
    pub struct Pose("geometry_msgs/msg/Pose") {
        /// Position.
        pub position: Point,
        /// Orientation.
        pub orientation: Quaternion,
    }
}

crate::ros_message! {
    /// A stamped pose (`geometry_msgs/msg/PoseStamped`).
    pub struct PoseStamped("geometry_msgs/msg/PoseStamped") {
        /// Stamp and frame.
        pub header: Header,
        /// The pose.
        pub pose: Pose,
    }
}

crate::ros_message! {
    /// Poses sharing a header (`geometry_msgs/msg/PoseArray`).
    pub struct PoseArray("geometry_msgs/msg/PoseArray") {
        /// Stamp and frame.
        pub header: Header,
        /// The poses.
        pub poses: Vec<Pose>,
    }
}

crate::ros_message! {
    /// A pose with a 6×6 row-major covariance (`geometry_msgs/msg/PoseWithCovariance`).
    pub struct PoseWithCovariance("geometry_msgs/msg/PoseWithCovariance") {
        /// The pose.
        pub pose: Pose,
        /// Covariance over (x, y, z, rot x, rot y, rot z).
        pub covariance: [f64; 36],
    }
}

crate::ros_message! {
    /// A rigid transform (`geometry_msgs/msg/Transform`).
    pub struct Transform("geometry_msgs/msg/Transform") {
        /// Translation.
        pub translation: Vector3,
        /// Rotation.
        pub rotation: Quaternion,
    }
}

crate::ros_message! {
    /// A transform from `header.frame_id` to `child_frame_id`
    /// (`geometry_msgs/msg/TransformStamped`).
    pub struct TransformStamped("geometry_msgs/msg/TransformStamped") {
        /// Stamp and parent frame.
        pub header: Header,
        /// Child frame.
        pub child_frame_id: String,
        /// The transform.
        pub transform: Transform,
    }
}

crate::ros_message! {
    /// Linear and angular velocity (`geometry_msgs/msg/Twist`).
    pub struct Twist("geometry_msgs/msg/Twist") {
        /// Linear velocity.
        pub linear: Vector3,
        /// Angular velocity.
        pub angular: Vector3,
    }
}

crate::ros_message! {
    /// A stamped twist (`geometry_msgs/msg/TwistStamped`).
    pub struct TwistStamped("geometry_msgs/msg/TwistStamped") {
        /// Stamp and frame.
        pub header: Header,
        /// The twist.
        pub twist: Twist,
    }
}

crate::ros_message! {
    /// A twist with a 6×6 row-major covariance (`geometry_msgs/msg/TwistWithCovariance`).
    pub struct TwistWithCovariance("geometry_msgs/msg/TwistWithCovariance") {
        /// The twist.
        pub twist: Twist,
        /// Covariance over (x, y, z, rot x, rot y, rot z).
        pub covariance: [f64; 36],
    }
}

crate::ros_message! {
    /// Force and torque (`geometry_msgs/msg/Wrench`).
    pub struct Wrench("geometry_msgs/msg/Wrench") {
        /// Force.
        pub force: Vector3,
        /// Torque.
        pub torque: Vector3,
    }
}

crate::ros_message! {
    /// A stamped wrench (`geometry_msgs/msg/WrenchStamped`).
    pub struct WrenchStamped("geometry_msgs/msg/WrenchStamped") {
        /// Stamp and frame.
        pub header: Header,
        /// The wrench.
        pub wrench: Wrench,
    }
}

crate::ros_message! {
    /// A stamped point (`geometry_msgs/msg/PointStamped`).
    pub struct PointStamped("geometry_msgs/msg/PointStamped") {
        /// Stamp and frame.
        pub header: Header,
        /// The point.
        pub point: Point,
    }
}

impl Quaternion {
    /// The identity rotation.
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };
}

impl Pose {
    /// The identity pose.
    pub const IDENTITY: Self = Self {
        position: Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        orientation: Quaternion::IDENTITY,
    };
}

impl Transform {
    /// The identity transform.
    pub const IDENTITY: Self = Self {
        translation: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        rotation: Quaternion::IDENTITY,
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn idl_defaults() {
        assert_eq!(Quaternion::default(), Quaternion::IDENTITY);
        let defaults: Vec<String> = Quaternion::individual_description()
            .fields
            .into_iter()
            .map(|f| f.default_value)
            .collect();
        // share/geometry_msgs/msg/Quaternion.json in ROS 2 Jazzy.
        assert_eq!(defaults, ["0.0", "0.0", "0.0", "1.0"]);
        let pose = Pose::individual_description();
        assert!(pose.fields.iter().all(|f| f.default_value.is_empty()));
    }

    #[test]
    fn hashes_match_jazzy() {
        for (hash, expected) in [
            (
                Vector3::type_hash(),
                "cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d",
            ),
            (
                Point::type_hash(),
                "6963084842a9b04494d6b2941d11444708d892da2f4b09843b9c43f42a7f6881",
            ),
            (
                Quaternion::type_hash(),
                "8a765f66778c8ff7c8ab94afcc590a2ed5325a1d9a076ffff38fbce36f458684",
            ),
            (
                Pose::type_hash(),
                "d501954e9476cea2996984e812054b68026ae0bfae789d9a10b23daf35cc90fa",
            ),
            (
                PoseStamped::type_hash(),
                "10f3786d7d40fd2b54367835614bff85d4ad3b5dab62bf8bca0cc232d73b4cd8",
            ),
            (
                PoseArray::type_hash(),
                "af0cc36d190e104d546d168d6b39df04fa4b4ccecf59cb4c9ed328d3d5004aa0",
            ),
            (
                Transform::type_hash(),
                "beb83fbe698636351461f6f35d1abb20010c43d55374d81bd041f1ba2581fddc",
            ),
            (
                TransformStamped::type_hash(),
                "0a241f87d04668d94099cbb5ba11691d5ad32c2f29682e4eb5653424bd275206",
            ),
            (
                Twist::type_hash(),
                "9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a",
            ),
            (
                TwistStamped::type_hash(),
                "5f0fcd4f81d5d06ad9b4c4c63e3ea51b82d6ae4d0558f1d475229b1121db6f64",
            ),
            (
                PoseWithCovariance::type_hash(),
                "9a7c0fd234b7f45c6098745ecccd773ca1085670e64107135397aee31c02e1bb",
            ),
            (
                TwistWithCovariance::type_hash(),
                "49f574f033f095d8b6cd1beaca5ca7925e296e84af1716d16c89d38b059c8c18",
            ),
            (
                Wrench::type_hash(),
                "018e8519d57c16adbe97c9fe1460ef21fec7e31bc541de3d653a35895677ce52",
            ),
            (
                WrenchStamped::type_hash(),
                "8dc3deaf06b2ab281f9f9a742a8961c328ca7cec16e3fd6586d3a5c83fa78f77",
            ),
            (
                PointStamped::type_hash(),
                "4c0296af86e01e562e9e0405d138a01537247580076c58ea38d7923ac1045897",
            ),
        ] {
            assert_eq!(hash, format!("RIHS01_{expected}"));
        }
    }
}
