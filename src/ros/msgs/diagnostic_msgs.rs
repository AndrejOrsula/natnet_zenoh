//! `diagnostic_msgs`: the `/diagnostics` message.

use super::std_msgs::Header;

crate::ros_message! {
    /// A key/value pair (`diagnostic_msgs/msg/KeyValue`).
    pub struct KeyValue("diagnostic_msgs/msg/KeyValue") {
        /// Key.
        pub key: String,
        /// Value.
        pub value: String,
    }
}

crate::ros_message! {
    /// Status of one component (`diagnostic_msgs/msg/DiagnosticStatus`).
    pub struct DiagnosticStatus("diagnostic_msgs/msg/DiagnosticStatus") {
        /// Level, one of the associated constants (a ROS `byte`).
        pub level: u8 [BYTE],
        /// Component name.
        pub name: String,
        /// Human-readable summary.
        pub message: String,
        /// Hardware identifier.
        pub hardware_id: String,
        /// Additional values.
        pub values: Vec<KeyValue>,
    }
}

crate::ros_message! {
    /// Statuses of several components (`diagnostic_msgs/msg/DiagnosticArray`).
    pub struct DiagnosticArray("diagnostic_msgs/msg/DiagnosticArray") {
        /// Stamp.
        pub header: Header,
        /// Statuses.
        pub status: Vec<DiagnosticStatus>,
    }
}

impl KeyValue {
    /// A pair from anything printable.
    pub fn new(key: &str, value: impl std::fmt::Display) -> Self {
        Self {
            key: key.to_owned(),
            value: value.to_string(),
        }
    }
}

impl DiagnosticStatus {
    /// Operating normally.
    pub const OK: u8 = 0;
    /// Degraded.
    pub const WARN: u8 = 1;
    /// Failed.
    pub const ERROR: u8 = 2;
    /// No recent data.
    pub const STALE: u8 = 3;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn hashes_match_jazzy() {
        assert_eq!(
            KeyValue::type_hash(),
            "RIHS01_d68081eaa540288c5440753baecef0c4e16e81a5f78ad68902ded5100413bb42"
        );
        assert_eq!(
            DiagnosticStatus::type_hash(),
            "RIHS01_b0e3e692ea2d54a8af2f4ef1930e81556a2db55216b771f8a7d2724ed47bf0e4"
        );
        assert_eq!(
            DiagnosticArray::type_hash(),
            "RIHS01_5a8a36efb05fb25070fa0fb3810290c0e6cd4862b54a8fb975a1ee8dc55a333e"
        );
    }
}
