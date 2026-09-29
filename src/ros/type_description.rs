//! ROS 2 type descriptions and the `RIHS01` type-description hash (REP 2011).
//!
//! `rmw_zenoh_cpp` puts the type hash in every data key expression and in
//! every liveliness token, so a publisher whose hash differs from the
//! subscriber's is simply never matched. Instead of transcribing hashes, this
//! module computes them from the type description with the exact algorithm of
//! `rosidl_generator_type_description.calculate_type_hash`:
//!
//! 1. build the full description: the type's own fields plus every nested
//!    type it references, transitively, sorted by type name;
//! 2. drop the `default_value` of every field;
//! 3. serialize with Python's `json.dumps(..., separators=(", ", ": "))`
//!    field order (`type_name`, `fields`; `name`, `type`; `type_id`,
//!    `capacity`, `string_capacity`, `nested_type_name`);
//! 4. hash the UTF-8 bytes with SHA-256 and prefix `RIHS01_`.
//!
//! The unit tests pin the result against the hashes shipped in the ROS 2
//! Jazzy `share/<package>/msg/<Type>.json` files.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use sha2::{Digest, Sha256};

/// `type_id` values of `type_description_interfaces/msg/FieldType`.
pub mod field_type {
    /// A nested message type.
    pub const NESTED_TYPE: u8 = 1;
    /// `int8`.
    pub const INT8: u8 = 2;
    /// `uint8`.
    pub const UINT8: u8 = 3;
    /// `int16`.
    pub const INT16: u8 = 4;
    /// `uint16`.
    pub const UINT16: u8 = 5;
    /// `int32`.
    pub const INT32: u8 = 6;
    /// `uint32`.
    pub const UINT32: u8 = 7;
    /// `int64`.
    pub const INT64: u8 = 8;
    /// `uint64`.
    pub const UINT64: u8 = 9;
    /// `float32`.
    pub const FLOAT: u8 = 10;
    /// `float64`.
    pub const DOUBLE: u8 = 11;
    /// `char`.
    pub const CHAR: u8 = 13;
    /// `bool`.
    pub const BOOLEAN: u8 = 15;
    /// `byte`.
    pub const BYTE: u8 = 16;
    /// `string`.
    pub const STRING: u8 = 17;
    /// `wstring`.
    pub const WSTRING: u8 = 18;
    /// `string<=N`.
    pub const BOUNDED_STRING: u8 = 21;
    /// `wstring<=N`.
    pub const BOUNDED_WSTRING: u8 = 22;
    /// Offset added to a base id for a fixed-size array.
    pub const ARRAY_OFFSET: u8 = 48;
    /// Offset added to a base id for a bounded sequence.
    pub const BOUNDED_SEQUENCE_OFFSET: u8 = 96;
    /// Offset added to a base id for an unbounded sequence.
    pub const UNBOUNDED_SEQUENCE_OFFSET: u8 = 144;
}

/// The type of one field.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FieldType {
    /// The `type_id`, including any array/sequence offset.
    pub type_id: u8,
    /// Array size or sequence bound; 0 for scalars and unbounded sequences.
    pub capacity: u64,
    /// Bound of a bounded string; 0 otherwise.
    pub string_capacity: u64,
    /// Fully qualified nested type name, empty for primitives.
    pub nested_type_name: String,
}

impl FieldType {
    /// A scalar primitive.
    #[must_use]
    pub fn primitive(type_id: u8) -> Self {
        Self {
            type_id,
            capacity: 0,
            string_capacity: 0,
            nested_type_name: String::new(),
        }
    }

    /// A bounded string of at most `bound` bytes.
    #[must_use]
    pub fn bounded_string(bound: u64) -> Self {
        Self {
            type_id: field_type::BOUNDED_STRING,
            capacity: 0,
            string_capacity: bound,
            nested_type_name: String::new(),
        }
    }

    /// A nested message type.
    #[must_use]
    pub fn nested(type_name: &str) -> Self {
        Self {
            type_id: field_type::NESTED_TYPE,
            capacity: 0,
            string_capacity: 0,
            nested_type_name: type_name.to_owned(),
        }
    }

    /// This element type as an unbounded sequence.
    #[must_use]
    pub fn unbounded_sequence(mut self) -> Self {
        self.type_id += field_type::UNBOUNDED_SEQUENCE_OFFSET;
        self
    }

    /// This element type as a bounded sequence of at most `bound` elements.
    #[must_use]
    pub fn bounded_sequence(mut self, bound: u64) -> Self {
        self.type_id += field_type::BOUNDED_SEQUENCE_OFFSET;
        self.capacity = bound;
        self
    }

    /// This element type as a fixed-size array of `size` elements.
    #[must_use]
    pub fn array(mut self, size: u64) -> Self {
        self.type_id += field_type::ARRAY_OFFSET;
        self.capacity = size;
        self
    }
}

/// One named field.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Field {
    /// Field name.
    pub name: String,
    /// Field type.
    pub field_type: FieldType,
    /// The IDL default as rosidl spells it (`1.0`, `True`), else empty. It
    /// is served in type descriptions but not hashed.
    pub default_value: String,
}

impl Field {
    /// A field named `name` of type `field_type`.
    #[must_use]
    pub fn new(name: &str, field_type: FieldType) -> Self {
        Self {
            name: name.to_owned(),
            field_type,
            default_value: String::new(),
        }
    }

    /// [`Self::new`] with an IDL default given as Rust source (`1.0`,
    /// `true`), spelled the way rosidl records it (`1.0`, `True`).
    #[must_use]
    pub fn with_default(name: &str, field_type: FieldType, rust_literal: &str) -> Self {
        let default_value = match rust_literal {
            "true" => "True".to_owned(),
            "false" => "False".to_owned(),
            other => other.to_owned(),
        };
        Self {
            default_value,
            ..Self::new(name, field_type)
        }
    }
}

/// The description of one type, without its references
/// (`type_description_interfaces/msg/IndividualTypeDescription`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IndividualTypeDescription {
    /// Fully qualified name, e.g. `geometry_msgs/msg/Pose`.
    pub type_name: String,
    /// Fields in declaration order.
    pub fields: Vec<Field>,
}

/// The set of referenced descriptions, keyed and therefore sorted by name.
pub type TypeSet = BTreeMap<String, IndividualTypeDescription>;

/// A full type description: the type and every type it references.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypeDescription {
    /// The described type.
    pub type_description: IndividualTypeDescription,
    /// Every transitively referenced type, sorted by name.
    pub referenced_type_descriptions: Vec<IndividualTypeDescription>,
}

impl TypeDescription {
    /// Assembles a full description from a type and its referenced set.
    ///
    /// Only types reachable from `root` are retained, matching
    /// `extract_full_type_description`.
    #[must_use]
    pub fn new(root: IndividualTypeDescription, set: &TypeSet) -> Self {
        let mut reachable = std::collections::BTreeSet::new();
        let mut queue: Vec<String> = root
            .fields
            .iter()
            .filter(|f| !f.field_type.nested_type_name.is_empty())
            .map(|f| f.field_type.nested_type_name.clone())
            .collect();
        while let Some(name) = queue.pop() {
            if reachable.insert(name.clone())
                && let Some(td) = set.get(&name)
            {
                queue.extend(
                    td.fields
                        .iter()
                        .filter(|f| !f.field_type.nested_type_name.is_empty())
                        .map(|f| f.field_type.nested_type_name.clone()),
                );
            }
        }
        let referenced_type_descriptions = reachable
            .iter()
            .filter_map(|name| set.get(name).cloned())
            .collect();
        Self {
            type_description: root,
            referenced_type_descriptions,
        }
    }

    /// The hashable JSON representation (no default values).
    #[must_use]
    pub fn hashable_json(&self) -> String {
        let mut out = String::with_capacity(1024);
        out.push_str("{\"type_description\": ");
        write_individual(&mut out, &self.type_description);
        out.push_str(", \"referenced_type_descriptions\": [");
        for (i, td) in self.referenced_type_descriptions.iter().enumerate() {
            if i > 0 {
                out.push_str(", ");
            }
            write_individual(&mut out, td);
        }
        out.push_str("]}");
        out
    }

    /// The `RIHS01_<sha256 hex>` hash string.
    #[must_use]
    pub fn hash(&self) -> String {
        let digest = Sha256::digest(self.hashable_json().as_bytes());
        let mut out = String::with_capacity(7 + 64);
        out.push_str("RIHS01_");
        for byte in digest {
            let _ = write!(out, "{byte:02x}");
        }
        out
    }
}

fn write_json_string(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{8}' => out.push_str("\\b"),
            '\u{c}' => out.push_str("\\f"),
            c if (c as u32) < 0x20 || c == '\u{7f}' || !c.is_ascii() => {
                // Python's ensure_ascii escapes as UTF-16 code units.
                let mut units = [0_u16; 2];
                for unit in c.encode_utf16(&mut units) {
                    let _ = write!(out, "\\u{unit:04x}");
                }
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

fn write_individual(out: &mut String, td: &IndividualTypeDescription) {
    out.push_str("{\"type_name\": ");
    write_json_string(out, &td.type_name);
    out.push_str(", \"fields\": [");
    for (i, f) in td.fields.iter().enumerate() {
        if i > 0 {
            out.push_str(", ");
        }
        out.push_str("{\"name\": ");
        write_json_string(out, &f.name);
        let _ = write!(
            out,
            ", \"type\": {{\"type_id\": {}, \"capacity\": {}, \"string_capacity\": {}, \"nested_type_name\": ",
            f.field_type.type_id, f.field_type.capacity, f.field_type.string_capacity
        );
        write_json_string(out, &f.field_type.nested_type_name);
        out.push_str("}}");
    }
    out.push_str("]}");
}

#[cfg(test)]
mod tests {
    use super::field_type::*;
    use super::*;

    fn time() -> IndividualTypeDescription {
        IndividualTypeDescription {
            type_name: "builtin_interfaces/msg/Time".into(),
            fields: vec![
                Field::new("sec", FieldType::primitive(INT32)),
                Field::new("nanosec", FieldType::primitive(UINT32)),
            ],
        }
    }

    #[test]
    fn time_hash_matches_jazzy() {
        let td = TypeDescription::new(time(), &TypeSet::new());
        assert_eq!(
            td.hash(),
            "RIHS01_b106235e25a4c5ed35098aa0a61a3ee9c9b18d197f398b0e4206cea9acf9c197"
        );
    }

    #[test]
    fn header_hash_matches_jazzy_and_references_are_pruned() {
        let header = IndividualTypeDescription {
            type_name: "std_msgs/msg/Header".into(),
            fields: vec![
                Field::new("stamp", FieldType::nested("builtin_interfaces/msg/Time")),
                Field::new("frame_id", FieldType::primitive(STRING)),
            ],
        };
        let mut set = TypeSet::new();
        set.insert("builtin_interfaces/msg/Time".into(), time());
        // An unreachable entry must not change the hash.
        set.insert(
            "zzz/msg/Unused".into(),
            IndividualTypeDescription {
                type_name: "zzz/msg/Unused".into(),
                fields: vec![],
            },
        );
        let td = TypeDescription::new(header, &set);
        assert_eq!(td.referenced_type_descriptions.len(), 1);
        assert_eq!(
            td.hash(),
            "RIHS01_f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01"
        );
    }

    #[test]
    fn json_escapes_like_python() {
        let mut s = String::new();
        write_json_string(&mut s, "a\"b\\é");
        assert_eq!(s, "\"a\\\"b\\\\\\u00e9\"");
        // json.dumps("\b\f\x7f\x01~")
        let mut s = String::new();
        write_json_string(&mut s, "\u{8}\u{c}\u{7f}\u{1}~");
        assert_eq!(s, r#""\b\f\u007f\u0001~""#);
    }
}
