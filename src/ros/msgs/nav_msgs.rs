//! `nav_msgs`: odometry and paths.

use super::geometry_msgs::{PoseStamped, PoseWithCovariance, TwistWithCovariance};
use super::std_msgs::Header;

crate::ros_message! {
    /// Pose in `header.frame_id` and twist in `child_frame_id`
    /// (`nav_msgs/msg/Odometry`).
    pub struct Odometry("nav_msgs/msg/Odometry") {
        /// Stamp and pose frame.
        pub header: Header,
        /// Frame of the twist.
        pub child_frame_id: String,
        /// The pose.
        pub pose: PoseWithCovariance,
        /// The twist.
        pub twist: TwistWithCovariance,
    }
}

crate::ros_message! {
    /// A sequence of poses (`nav_msgs/msg/Path`).
    pub struct Path("nav_msgs/msg/Path") {
        /// Stamp and frame.
        pub header: Header,
        /// The poses.
        pub poses: Vec<PoseStamped>,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn hashes_match_jazzy() {
        assert_eq!(
            Odometry::type_hash(),
            "RIHS01_3cc97dc7fb7502f8714462c526d369e35b603cfc34d946e3f2eda2766dfec6e0"
        );
        assert_eq!(
            Path::type_hash(),
            "RIHS01_1957a5bb3cee5da65c4e52e52b65a93df227efce4c20f8458b36e73066ca334b"
        );
    }
}
