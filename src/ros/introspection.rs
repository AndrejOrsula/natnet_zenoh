//! `~/get_type_description`, as every rclcpp node offers it (REP 2011).

use std::sync::PoisonError;

use super::context::Result;
use super::msgs::type_description_interfaces::{
    GetTypeDescription, GetTypeDescriptionResponse, KeyValue, TypeDescription,
};
use super::node::TypeRegistry;
use super::{Node, ServiceServer};

/// Answers a request from the registry.
#[must_use]
pub fn describe(
    registry: &TypeRegistry,
    type_name: &str,
    type_hash: &str,
) -> GetTypeDescriptionResponse {
    let types = registry.lock().unwrap_or_else(PoisonError::into_inner);
    let fail = |reason: String| GetTypeDescriptionResponse {
        successful: false,
        failure_reason: reason,
        ..GetTypeDescriptionResponse::default()
    };
    // rcl looks types up by hash; the name is accepted as well.
    let found = types.get(type_name).or_else(|| {
        types
            .values()
            .find(|d| !type_hash.is_empty() && d.hash() == type_hash)
    });
    let Some(description) = found else {
        return fail(format!(
            "type {type_name:?} ({type_hash}) is not used by this node"
        ));
    };
    let hash = description.hash();
    if !type_hash.is_empty() && type_hash != hash {
        return fail(format!(
            "type {type_name:?} has hash {hash}, not {type_hash}"
        ));
    }
    GetTypeDescriptionResponse {
        successful: true,
        failure_reason: String::new(),
        type_description: TypeDescription::from(description),
        // No source files ship with this binary.
        type_sources: Vec::new(),
        extra_information: vec![KeyValue {
            key: "type_hash".into(),
            value: hash,
        }],
    }
}

/// Serves `~/get_type_description` for the types `node` uses.
///
/// # Errors
/// Zenoh or naming errors.
pub fn serve_type_descriptions(node: &Node) -> Result<ServiceServer<GetTypeDescription>> {
    let registry = node.type_registry();
    node.create_service::<GetTypeDescription, _>("~/get_type_description", move |req, _| {
        describe(&registry, &req.type_name, &req.type_hash)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;
    use crate::ros::msgs::geometry_msgs::PoseStamped;

    #[test]
    fn lookup() {
        let registry = TypeRegistry::default();
        registry.lock().unwrap().insert(
            PoseStamped::TYPE_NAME.into(),
            PoseStamped::type_description(),
        );
        let ok = describe(&registry, PoseStamped::TYPE_NAME, &PoseStamped::type_hash());
        assert!(ok.successful);
        assert_eq!(ok.type_description.referenced_type_descriptions.len(), 5);
        assert!(!describe(&registry, PoseStamped::TYPE_NAME, "RIHS01_00").successful);
        assert!(!describe(&registry, "x/msg/Y", "").successful);
        assert!(describe(&registry, "", &PoseStamped::type_hash()).successful);
    }
}
