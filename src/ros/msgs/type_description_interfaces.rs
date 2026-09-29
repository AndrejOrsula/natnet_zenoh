//! `type_description_interfaces`: the `~/get_type_description` service.

use crate::ros::message::{BoundedString, Service};
use crate::ros::type_description as td;

crate::ros_message! {
    /// A field type (`type_description_interfaces/msg/FieldType`).
    pub struct FieldType("type_description_interfaces/msg/FieldType") {
        /// Type id.
        pub type_id: u8 = 0,
        /// Array size or sequence bound.
        pub capacity: u64,
        /// String bound.
        pub string_capacity: u64,
        /// Nested type name.
        pub nested_type_name: BoundedString<255>,
    }
}

crate::ros_message! {
    /// A field (`type_description_interfaces/msg/Field`).
    pub struct Field("type_description_interfaces/msg/Field") {
        /// Name.
        pub name: String,
        /// Type.
        pub r#type: FieldType,
        /// Default value literal.
        pub default_value: String,
    }
}

crate::ros_message! {
    /// One type (`type_description_interfaces/msg/IndividualTypeDescription`).
    pub struct IndividualTypeDescription("type_description_interfaces/msg/IndividualTypeDescription") {
        /// Fully qualified name.
        pub type_name: BoundedString<255>,
        /// Fields.
        pub fields: Vec<Field>,
    }
}

crate::ros_message! {
    /// A type and its references (`type_description_interfaces/msg/TypeDescription`).
    pub struct TypeDescription("type_description_interfaces/msg/TypeDescription") {
        /// The type.
        pub type_description: IndividualTypeDescription,
        /// Every referenced type.
        pub referenced_type_descriptions: Vec<IndividualTypeDescription>,
    }
}

crate::ros_message! {
    /// A type's source text (`type_description_interfaces/msg/TypeSource`).
    pub struct TypeSource("type_description_interfaces/msg/TypeSource") {
        /// Type name.
        pub type_name: String,
        /// Source encoding (`msg`, `idl`, …).
        pub encoding: String,
        /// Source text.
        pub raw_file_contents: String,
    }
}

crate::ros_message! {
    /// A key/value pair (`type_description_interfaces/msg/KeyValue`).
    pub struct KeyValue("type_description_interfaces/msg/KeyValue") {
        /// Key.
        pub key: String,
        /// Value.
        pub value: String,
    }
}

crate::ros_message! {
    /// Request of [`GetTypeDescription`].
    pub struct GetTypeDescriptionRequest("type_description_interfaces/srv/GetTypeDescription_Request") {
        /// Type name.
        pub type_name: String,
        /// Expected hash.
        pub type_hash: String,
        /// Include source text.
        pub include_type_sources: bool = true,
    }
}

crate::ros_message! {
    /// Response of [`GetTypeDescription`].
    pub struct GetTypeDescriptionResponse("type_description_interfaces/srv/GetTypeDescription_Response") {
        /// Whether the type was found.
        pub successful: bool,
        /// Why not.
        pub failure_reason: String,
        /// The description.
        pub type_description: TypeDescription,
        /// Source texts.
        pub type_sources: Vec<TypeSource>,
        /// Extra information.
        pub extra_information: Vec<KeyValue>,
    }
}

/// `type_description_interfaces/srv/GetTypeDescription`.
#[derive(Debug, Clone, Copy, Default)]
pub struct GetTypeDescription;

impl Service for GetTypeDescription {
    const TYPE_NAME: &'static str = "type_description_interfaces/srv/GetTypeDescription";
    type Request = GetTypeDescriptionRequest;
    type Response = GetTypeDescriptionResponse;
}

fn individual(d: &td::IndividualTypeDescription) -> IndividualTypeDescription {
    IndividualTypeDescription {
        type_name: BoundedString(d.type_name.clone()),
        fields: d
            .fields
            .iter()
            .map(|f| Field {
                name: f.name.clone(),
                r#type: FieldType {
                    type_id: f.field_type.type_id,
                    capacity: f.field_type.capacity,
                    string_capacity: f.field_type.string_capacity,
                    nested_type_name: BoundedString(f.field_type.nested_type_name.clone()),
                },
                default_value: f.default_value.clone(),
            })
            .collect(),
    }
}

impl From<&td::TypeDescription> for TypeDescription {
    fn from(d: &td::TypeDescription) -> Self {
        Self {
            type_description: individual(&d.type_description),
            referenced_type_descriptions: d
                .referenced_type_descriptions
                .iter()
                .map(individual)
                .collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn hashes_match_jazzy() {
        for (hash, expected) in [
            (
                Field::type_hash(),
                "c0b01379cd4226281285ccaf6be46653968f855f7c5e41614ff5d7a854efef7c",
            ),
            (
                FieldType::type_hash(),
                "a70b6dd919645a03a3586f7f821defbc886ea3e531a1d95cc0f380a3973ccaa6",
            ),
            (
                IndividualTypeDescription::type_hash(),
                "55c827d86c3c141bdd318fe6c22e11190e4d3b37c8f4f9751a084aa05ce96560",
            ),
            (
                KeyValue::type_hash(),
                "274fe56bf14f33c7512e34c646a37579ee36779f745f049a9760763e817f0c42",
            ),
            (
                TypeDescription::type_hash(),
                "739f2508c9fa3a6f330913ff5b9d25fb74159a077da71e1087f51a60c12a080b",
            ),
            (
                TypeSource::type_hash(),
                "faeaec7596c04ecf5b6e99ad225e4c7cbb997ad5435f793526fb3984d011aae5",
            ),
            (
                <GetTypeDescription as Service>::type_hash(),
                "69b9c19c1021405984cc60dbbb1edceb147a6538b411d812ba6afabeed962cd5",
            ),
        ] {
            assert_eq!(hash, format!("RIHS01_{expected}"));
        }
    }

    #[test]
    fn converts_descriptions() {
        let d = crate::ros::msgs::geometry_msgs::PoseStamped::type_description();
        let m = TypeDescription::from(&d);
        assert_eq!(
            m.type_description.type_name.0,
            "geometry_msgs/msg/PoseStamped"
        );
        assert_eq!(m.referenced_type_descriptions.len(), 5);
        let back = TypeDescription::from_cdr(&m.to_cdr().unwrap()).unwrap();
        assert_eq!(back, m);
    }
}
