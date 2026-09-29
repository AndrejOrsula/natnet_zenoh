//! `std_srvs`: generic services.

use crate::ros::message::Service;

crate::ros_message! {
    /// Request of [`Trigger`] (`std_srvs/srv/Trigger_Request`).
    pub struct TriggerRequest("std_srvs/srv/Trigger_Request") {
        /// Placeholder required by the IDL for empty structures.
        pub structure_needs_at_least_one_member: u8,
    }
}

crate::ros_message! {
    /// Response of [`Trigger`] (`std_srvs/srv/Trigger_Response`).
    pub struct TriggerResponse("std_srvs/srv/Trigger_Response") {
        /// Whether the action succeeded.
        pub success: bool,
        /// Informational message.
        pub message: String,
    }
}

/// Triggers an action (`std_srvs/srv/Trigger`).
#[derive(Debug, Clone, Copy, Default)]
pub struct Trigger;

impl Service for Trigger {
    const TYPE_NAME: &'static str = "std_srvs/srv/Trigger";
    type Request = TriggerRequest;
    type Response = TriggerResponse;
}

crate::ros_message! {
    /// Request of [`Empty`] (`std_srvs/srv/Empty_Request`).
    pub struct EmptyRequest("std_srvs/srv/Empty_Request") {
        /// Placeholder required by the IDL for empty structures.
        pub structure_needs_at_least_one_member: u8,
    }
}

crate::ros_message! {
    /// Response of [`Empty`] (`std_srvs/srv/Empty_Response`).
    pub struct EmptyResponse("std_srvs/srv/Empty_Response") {
        /// Placeholder required by the IDL for empty structures.
        pub structure_needs_at_least_one_member: u8,
    }
}

/// A service without data (`std_srvs/srv/Empty`).
#[derive(Debug, Clone, Copy, Default)]
pub struct Empty;

impl Service for Empty {
    const TYPE_NAME: &'static str = "std_srvs/srv/Empty";
    type Request = EmptyRequest;
    type Response = EmptyResponse;
}

crate::ros_message! {
    /// Request of [`SetBool`] (`std_srvs/srv/SetBool_Request`).
    pub struct SetBoolRequest("std_srvs/srv/SetBool_Request") {
        /// The requested value.
        pub data: bool,
    }
}

crate::ros_message! {
    /// Response of [`SetBool`] (`std_srvs/srv/SetBool_Response`).
    pub struct SetBoolResponse("std_srvs/srv/SetBool_Response") {
        /// Whether the value was applied.
        pub success: bool,
        /// Informational message.
        pub message: String,
    }
}

/// Sets a boolean (`std_srvs/srv/SetBool`).
#[derive(Debug, Clone, Copy, Default)]
pub struct SetBool;

impl Service for SetBool {
    const TYPE_NAME: &'static str = "std_srvs/srv/SetBool";
    type Request = SetBoolRequest;
    type Response = SetBoolResponse;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn hashes_match_jazzy() {
        assert_eq!(
            Trigger::type_hash(),
            "RIHS01_eeff2cd6fa5ad9d27cdf4dec64818317839b62f212a91e6b5304b634b2062c5f"
        );
        assert_eq!(
            TriggerRequest::type_hash(),
            "RIHS01_d010825374ce8918e72bfd826c82603e60f45419e932ea976f807b74a863a199"
        );
        assert_eq!(
            TriggerResponse::type_hash(),
            "RIHS01_2d946c21e2fc3f1e9ca6986a8191d85fcc70097a8bb7771a053564bc47009cdf"
        );
        assert_eq!(
            Empty::type_hash(),
            "RIHS01_5888399dedec5ccc85ea6451949fd2c9f97bfdf963f9a588821639fcd31b5d19"
        );
        assert_eq!(
            SetBool::type_hash(),
            "RIHS01_abe9e4bb6b41b40e6789712c00ec8871923e089af3f667a79992a428cff2da0a"
        );
    }

    #[test]
    fn event_description_matches_jazzy() {
        let td = Trigger::type_description();
        let event = td
            .referenced_type_descriptions
            .iter()
            .find(|t| t.type_name == "std_srvs/srv/Trigger_Event")
            .unwrap()
            .clone();
        let set: crate::ros::type_description::TypeSet = td
            .referenced_type_descriptions
            .iter()
            .map(|t| (t.type_name.clone(), t.clone()))
            .collect();
        let event_td = crate::ros::type_description::TypeDescription::new(event, &set);
        assert_eq!(
            event_td.hash(),
            "RIHS01_ec4a1d26b0575e61906890342aba9523a2360e9845857c5590fa6e23bc39e1c2"
        );
    }
}
