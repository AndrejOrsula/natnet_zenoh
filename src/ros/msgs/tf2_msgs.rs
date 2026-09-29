//! `tf2_msgs`: the `/tf` and `/tf_static` message.

use super::geometry_msgs::TransformStamped;

crate::ros_message! {
    /// A batch of transforms (`tf2_msgs/msg/TFMessage`).
    pub struct TFMessage("tf2_msgs/msg/TFMessage") {
        /// The transforms.
        pub transforms: Vec<TransformStamped>,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn hash_matches_jazzy() {
        assert_eq!(
            TFMessage::type_hash(),
            "RIHS01_e369d0f05a23ae52508854b66f6aa0437f3449d652e8cbf22d5abe85d020f087"
        );
    }
}
