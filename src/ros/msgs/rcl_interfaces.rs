//! `rcl_interfaces`: the `/rosout` log message and the parameter services.

use super::builtin_interfaces::Time;
use crate::ros::message::{BoundedVec, Service};

crate::ros_message! {
    /// A log record (`rcl_interfaces/msg/Log`).
    pub struct Log("rcl_interfaces/msg/Log") {
        /// Time of the record.
        pub stamp: Time,
        /// Severity, one of the associated constants.
        pub level: u8,
        /// Logger name.
        pub name: String,
        /// Message.
        pub msg: String,
        /// Source file.
        pub file: String,
        /// Source function.
        pub function: String,
        /// Source line.
        pub line: u32,
    }
}

impl Log {
    /// Debug.
    pub const DEBUG: u8 = 10;
    /// Info.
    pub const INFO: u8 = 20;
    /// Warning.
    pub const WARN: u8 = 30;
    /// Error.
    pub const ERROR: u8 = 40;
    /// Fatal.
    pub const FATAL: u8 = 50;
}

crate::ros_message! {
    /// A parameter value; `type` selects which value field is populated
    /// (`rcl_interfaces/msg/ParameterValue`).
    pub struct ParameterValue("rcl_interfaces/msg/ParameterValue") {
        /// One of the `ParameterValue::*` type constants.
        pub r#type: u8,
        /// Boolean value.
        pub bool_value: bool,
        /// Integer value.
        pub integer_value: i64,
        /// Floating-point value.
        pub double_value: f64,
        /// String value.
        pub string_value: String,
        /// Byte array value.
        pub byte_array_value: Vec<u8> [BYTE],
        /// Boolean array value.
        pub bool_array_value: Vec<bool>,
        /// Integer array value.
        pub integer_array_value: Vec<i64>,
        /// Floating-point array value.
        pub double_array_value: Vec<f64>,
        /// String array value.
        pub string_array_value: Vec<String>,
    }
}

impl ParameterValue {
    /// Unset.
    pub const NOT_SET: u8 = 0;
    /// `bool`.
    pub const BOOL: u8 = 1;
    /// `int64`.
    pub const INTEGER: u8 = 2;
    /// `float64`.
    pub const DOUBLE: u8 = 3;
    /// `string`.
    pub const STRING: u8 = 4;
    /// `byte[]`.
    pub const BYTE_ARRAY: u8 = 5;
    /// `bool[]`.
    pub const BOOL_ARRAY: u8 = 6;
    /// `int64[]`.
    pub const INTEGER_ARRAY: u8 = 7;
    /// `float64[]`.
    pub const DOUBLE_ARRAY: u8 = 8;
    /// `string[]`.
    pub const STRING_ARRAY: u8 = 9;
}

crate::ros_message! {
    /// A named parameter (`rcl_interfaces/msg/Parameter`).
    pub struct Parameter("rcl_interfaces/msg/Parameter") {
        /// Name.
        pub name: String,
        /// Value.
        pub value: ParameterValue,
    }
}

crate::ros_message! {
    /// A floating-point range (`rcl_interfaces/msg/FloatingPointRange`).
    pub struct FloatingPointRange("rcl_interfaces/msg/FloatingPointRange") {
        /// Lower bound.
        pub from_value: f64,
        /// Upper bound.
        pub to_value: f64,
        /// Step, 0 for continuous.
        pub step: f64,
    }
}

crate::ros_message! {
    /// An integer range (`rcl_interfaces/msg/IntegerRange`).
    pub struct IntegerRange("rcl_interfaces/msg/IntegerRange") {
        /// Lower bound.
        pub from_value: i64,
        /// Upper bound.
        pub to_value: i64,
        /// Step, 0 for any.
        pub step: u64,
    }
}

crate::ros_message! {
    /// A parameter's metadata (`rcl_interfaces/msg/ParameterDescriptor`).
    pub struct ParameterDescriptor("rcl_interfaces/msg/ParameterDescriptor") {
        /// Name.
        pub name: String,
        /// Type.
        pub r#type: u8,
        /// Description.
        pub description: String,
        /// Constraints in prose.
        pub additional_constraints: String,
        /// Whether it can change.
        pub read_only: bool = false,
        /// Whether its type can change.
        pub dynamic_typing: bool = false,
        /// Range of a floating-point parameter.
        pub floating_point_range: BoundedVec<FloatingPointRange, 1>,
        /// Range of an integer parameter.
        pub integer_range: BoundedVec<IntegerRange, 1>,
    }
}

crate::ros_message! {
    /// Parameter changes of a node (`rcl_interfaces/msg/ParameterEvent`).
    pub struct ParameterEvent("rcl_interfaces/msg/ParameterEvent") {
        /// Time of the change.
        pub stamp: Time,
        /// Fully qualified node name.
        pub node: String,
        /// Newly declared parameters.
        pub new_parameters: Vec<Parameter>,
        /// Changed parameters.
        pub changed_parameters: Vec<Parameter>,
        /// Deleted parameters.
        pub deleted_parameters: Vec<Parameter>,
    }
}

crate::ros_message! {
    /// Result of listing parameters (`rcl_interfaces/msg/ListParametersResult`).
    pub struct ListParametersResult("rcl_interfaces/msg/ListParametersResult") {
        /// Matching names.
        pub names: Vec<String>,
        /// Their prefixes.
        pub prefixes: Vec<String>,
    }
}

crate::ros_message! {
    /// Result of setting a parameter (`rcl_interfaces/msg/SetParametersResult`).
    pub struct SetParametersResult("rcl_interfaces/msg/SetParametersResult") {
        /// Whether it was set.
        pub successful: bool,
        /// Why not.
        pub reason: String,
    }
}

macro_rules! service {
    ($(#[$m:meta])* $name:ident, $type_name:literal,
     $req:ident($req_name:literal) { $($rf:ident: $rt:ty),* },
     $res:ident($res_name:literal) { $($sf:ident: $st:ty),* }) => {
        crate::ros_message! {
            #[doc = concat!("Request of [`", stringify!($name), "`].")]
            pub struct $req($req_name) {
                $(#[doc = stringify!($rf)] pub $rf: $rt,)*
            }
        }
        crate::ros_message! {
            #[doc = concat!("Response of [`", stringify!($name), "`].")]
            pub struct $res($res_name) {
                $(#[doc = stringify!($sf)] pub $sf: $st,)*
            }
        }
        $(#[$m])*
        #[derive(Debug, Clone, Copy, Default)]
        pub struct $name;
        impl Service for $name {
            const TYPE_NAME: &'static str = $type_name;
            type Request = $req;
            type Response = $res;
        }
    };
}

service!(
    /// `rcl_interfaces/srv/ListParameters`.
    ListParameters, "rcl_interfaces/srv/ListParameters",
    ListParametersRequest("rcl_interfaces/srv/ListParameters_Request") { prefixes: Vec<String>, depth: u64 },
    ListParametersResponse("rcl_interfaces/srv/ListParameters_Response") { result: ListParametersResult }
);
service!(
    /// `rcl_interfaces/srv/GetParameters`.
    GetParameters, "rcl_interfaces/srv/GetParameters",
    GetParametersRequest("rcl_interfaces/srv/GetParameters_Request") { names: Vec<String> },
    GetParametersResponse("rcl_interfaces/srv/GetParameters_Response") { values: Vec<ParameterValue> }
);
service!(
    /// `rcl_interfaces/srv/GetParameterTypes`.
    GetParameterTypes, "rcl_interfaces/srv/GetParameterTypes",
    GetParameterTypesRequest("rcl_interfaces/srv/GetParameterTypes_Request") { names: Vec<String> },
    GetParameterTypesResponse("rcl_interfaces/srv/GetParameterTypes_Response") { types: Vec<u8> }
);
service!(
    /// `rcl_interfaces/srv/DescribeParameters`.
    DescribeParameters, "rcl_interfaces/srv/DescribeParameters",
    DescribeParametersRequest("rcl_interfaces/srv/DescribeParameters_Request") { names: Vec<String> },
    DescribeParametersResponse("rcl_interfaces/srv/DescribeParameters_Response") { descriptors: Vec<ParameterDescriptor> }
);
service!(
    /// `rcl_interfaces/srv/SetParameters`.
    SetParameters, "rcl_interfaces/srv/SetParameters",
    SetParametersRequest("rcl_interfaces/srv/SetParameters_Request") { parameters: Vec<Parameter> },
    SetParametersResponse("rcl_interfaces/srv/SetParameters_Response") { results: Vec<SetParametersResult> }
);
service!(
    /// `rcl_interfaces/srv/SetParametersAtomically`.
    SetParametersAtomically, "rcl_interfaces/srv/SetParametersAtomically",
    SetParametersAtomicallyRequest("rcl_interfaces/srv/SetParametersAtomically_Request") { parameters: Vec<Parameter> },
    SetParametersAtomicallyResponse("rcl_interfaces/srv/SetParametersAtomically_Response") { result: SetParametersResult }
);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn hash_matches_jazzy() {
        assert_eq!(
            Log::type_hash(),
            "RIHS01_e28ce254ca8abc06abf92773b74602cdbf116ed34fbaf294fb9f81da9f318eac"
        );
    }

    #[test]
    fn parameter_hashes_match_jazzy() {
        use crate::ros::message::Service as _;
        for (hash, expected) in [
            (
                FloatingPointRange::type_hash(),
                "e6af23a23c177fee5f3075c8b1e435162a9b63c863d78c06017460b49684262d",
            ),
            (
                IntegerRange::type_hash(),
                "f7b7fdc0f65f07702e099218e13288c3963bcb9345bde78b560e6cd19800fc5a",
            ),
            (
                ListParametersResult::type_hash(),
                "237ae3428413dcbcfb452b510c42355f3a2b021dc091afa3e18526d57022f1cd",
            ),
            (
                ParameterDescriptor::type_hash(),
                "52175dbfda6c51153101d33d2a9da05743f66f02d5ab2ca9ec4709b46b73d704",
            ),
            (
                ParameterEvent::type_hash(),
                "043e627780fcad87a22d225bc2a037361dba713fca6a6b9f4b869a5aa0393204",
            ),
            (
                Parameter::type_hash(),
                "ddfe6442cffc462317adb5c92536a7b6dd55858c5c3e1e328165a6b73c2831af",
            ),
            (
                ParameterValue::type_hash(),
                "115fc089a387e23c7ecd3525c9189c379109119d6ab82e8dfbde0fdf6a7f9b68",
            ),
            (
                SetParametersResult::type_hash(),
                "cfcc0fb0371ee5159b403960ef4300f8f9d2f1fd6117c8666b7f9654d528a9b1",
            ),
            (
                DescribeParameters::type_hash(),
                "845b484d71eb0673dae682f2e3ba3c4851a65a3dcfb97bddd82c5b57e91e4cff",
            ),
            (
                GetParameters::type_hash(),
                "bf9803d5c74cf989a5de3e0c2e99444599a627c7ff75f97b8c05b01003675cbc",
            ),
            (
                GetParameterTypes::type_hash(),
                "da199c878688b3e530bdfe3ca8f74cb9fa0c303101e980a9e8f260e25e1c80ca",
            ),
            (
                ListParameters::type_hash(),
                "3e6062bfbb27bfb8730d4cef2558221f51a11646d78e7bb30a1e83afac3aad9d",
            ),
            (
                SetParametersAtomically::type_hash(),
                "0e192ef259c07fc3c07a13191d27002222e65e00ccec653ca05e856f79285fcd",
            ),
            (
                SetParameters::type_hash(),
                "56eed9a67e169f9cb6c1f987bc88f868c14a8fc9f743a263bc734c154015d7e0",
            ),
        ] {
            assert_eq!(hash, format!("RIHS01_{expected}"));
        }
    }
}
