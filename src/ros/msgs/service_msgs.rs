//! `service_msgs`: metadata of service introspection events.

use super::builtin_interfaces::Time;

crate::ros_message! {
    /// Metadata of a service event (`service_msgs/msg/ServiceEventInfo`).
    ///
    /// Every service type hash references this type through its generated
    /// `_Event` message, so it is needed even though introspection events are
    /// not published.
    pub struct ServiceEventInfo("service_msgs/msg/ServiceEventInfo") {
        /// Event kind.
        pub event_type: u8,
        /// Event time.
        pub stamp: Time,
        /// GID of the client.
        pub client_gid: [u8; 16],
        /// Request sequence number.
        pub sequence_number: i64,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn hash_matches_jazzy() {
        assert_eq!(
            ServiceEventInfo::type_hash(),
            "RIHS01_41bcbbe07a75c9b52bc96bfd5c24d7f0fc0a08c0cb7921b3373c5732345a6f45"
        );
    }
}
