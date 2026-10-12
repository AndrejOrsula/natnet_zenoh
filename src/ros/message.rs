//! Typed ROS 2 messages and services: CDR encoding plus type descriptions.
//!
//! A message is declared once with [`ros_message!`](crate::ros_message); the
//! macro derives its CDR layout, its [`IndividualTypeDescription`] and the
//! registration of every nested type, so the wire encoding and the `RIHS01`
//! hash come from the same field list and cannot drift apart.

use super::cdr::{self, CdrReader, CdrWriter};
use super::type_description::{
    Field, FieldType, IndividualTypeDescription, TypeDescription, TypeSet, field_type,
};

/// A type that can appear as a field element: a primitive, a string or a
/// nested message.
pub trait RosType: Sized {
    /// Minimum encoded size in bytes, used to bound decoded sequence lengths.
    const MIN_SIZE: usize;

    /// The field type of a scalar field of this type.
    fn element_type() -> FieldType;

    /// Registers this type's description and its references, if nested.
    fn register(_set: &mut TypeSet) {}

    /// Encodes one value.
    ///
    /// # Errors
    /// A [`cdr::CdrError`] when the value is not representable.
    fn write(&self, w: &mut CdrWriter) -> cdr::Result<()>;

    /// Decodes one value.
    ///
    /// # Errors
    /// A [`cdr::CdrError`] for truncated or invalid input.
    fn read(r: &mut CdrReader<'_>) -> cdr::Result<Self>;

    /// Encodes a slice of values (overridden for bytes).
    ///
    /// # Errors
    /// As [`RosType::write`].
    fn write_slice(items: &[Self], w: &mut CdrWriter) -> cdr::Result<()> {
        for item in items {
            item.write(w)?;
        }
        Ok(())
    }

    /// Decodes `n` values (overridden for bytes).
    ///
    /// # Errors
    /// As [`RosType::read`].
    fn read_vec(r: &mut CdrReader<'_>, n: usize) -> cdr::Result<Vec<Self>> {
        // The count comes from the network: grow as elements decode rather
        // than trusting it for one large allocation.
        let mut out = Vec::with_capacity(n.min(1024));
        for _ in 0..n {
            out.push(Self::read(r)?);
        }
        Ok(out)
    }
}

/// A type that can be a message field: a [`RosType`] scalar, an unbounded
/// sequence (`Vec<T>`) or a fixed-size array (`[T; N]`).
pub trait RosField: Sized {
    /// The field type descriptor.
    fn field_type() -> FieldType;
    /// Registers nested types.
    fn register(set: &mut TypeSet);
    /// Encodes the field.
    ///
    /// # Errors
    /// A [`cdr::CdrError`] when the value is not representable.
    fn write_field(&self, w: &mut CdrWriter) -> cdr::Result<()>;
    /// Decodes the field.
    ///
    /// # Errors
    /// A [`cdr::CdrError`] for truncated or invalid input.
    fn read_field(r: &mut CdrReader<'_>) -> cdr::Result<Self>;
    /// The zero value (arrays of any length, unlike [`Default`]).
    fn default_value() -> Self;
}

macro_rules! impl_primitive {
    ($ty:ty, $id:ident, $size:expr, $write:ident, $read:ident) => {
        impl RosType for $ty {
            const MIN_SIZE: usize = $size;
            fn element_type() -> FieldType {
                FieldType::primitive(field_type::$id)
            }
            fn write(&self, w: &mut CdrWriter) -> cdr::Result<()> {
                w.$write(*self);
                Ok(())
            }
            fn read(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
                r.$read()
            }
        }
        impl RosField for $ty {
            fn field_type() -> FieldType {
                <$ty as RosType>::element_type()
            }
            fn register(_set: &mut TypeSet) {}
            fn write_field(&self, w: &mut CdrWriter) -> cdr::Result<()> {
                RosType::write(self, w)
            }
            fn read_field(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
                <$ty as RosType>::read(r)
            }
            fn default_value() -> Self {
                Self::default()
            }
        }
    };
}

impl_primitive!(bool, BOOLEAN, 1, bool, bool);
impl_primitive!(i8, INT8, 1, i8, i8);
impl_primitive!(i16, INT16, 2, i16, i16);
impl_primitive!(u16, UINT16, 2, u16, u16);
impl_primitive!(i32, INT32, 4, i32, i32);
impl_primitive!(u32, UINT32, 4, u32, u32);
impl_primitive!(i64, INT64, 8, i64, i64);
impl_primitive!(u64, UINT64, 8, u64, u64);
impl_primitive!(f32, FLOAT, 4, f32, f32);
impl_primitive!(f64, DOUBLE, 8, f64, f64);

impl RosType for u8 {
    const MIN_SIZE: usize = 1;
    fn element_type() -> FieldType {
        FieldType::primitive(field_type::UINT8)
    }
    fn write(&self, w: &mut CdrWriter) -> cdr::Result<()> {
        w.u8(*self);
        Ok(())
    }
    fn read(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
        r.u8()
    }
    fn write_slice(items: &[Self], w: &mut CdrWriter) -> cdr::Result<()> {
        w.bytes(items);
        Ok(())
    }
    fn read_vec(r: &mut CdrReader<'_>, n: usize) -> cdr::Result<Vec<Self>> {
        Ok(r.bytes(n)?.to_vec())
    }
}

impl RosField for u8 {
    fn field_type() -> FieldType {
        <Self as RosType>::element_type()
    }
    fn register(_set: &mut TypeSet) {}
    fn write_field(&self, w: &mut CdrWriter) -> cdr::Result<()> {
        RosType::write(self, w)
    }
    fn read_field(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
        <Self as RosType>::read(r)
    }
    fn default_value() -> Self {
        0
    }
}

impl RosType for String {
    const MIN_SIZE: usize = 4;
    fn element_type() -> FieldType {
        FieldType::primitive(field_type::STRING)
    }
    fn write(&self, w: &mut CdrWriter) -> cdr::Result<()> {
        w.string(self)
    }
    fn read(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
        r.string()
    }
}

impl RosField for String {
    fn field_type() -> FieldType {
        <Self as RosType>::element_type()
    }
    fn register(_set: &mut TypeSet) {}
    fn write_field(&self, w: &mut CdrWriter) -> cdr::Result<()> {
        RosType::write(self, w)
    }
    fn read_field(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
        r.string()
    }
    fn default_value() -> Self {
        Self::new()
    }
}

impl<T: RosType> RosField for Vec<T> {
    fn field_type() -> FieldType {
        T::element_type().unbounded_sequence()
    }
    fn register(set: &mut TypeSet) {
        T::register(set);
    }
    fn write_field(&self, w: &mut CdrWriter) -> cdr::Result<()> {
        w.seq_len(self.len())?;
        T::write_slice(self, w)
    }
    fn read_field(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
        let n = r.seq_len(T::MIN_SIZE)?;
        T::read_vec(r, n)
    }
    fn default_value() -> Self {
        Self::new()
    }
}

impl<T: RosField + RosType + Copy, const N: usize> RosField for [T; N] {
    fn field_type() -> FieldType {
        T::element_type().array(N as u64)
    }
    fn register(set: &mut TypeSet) {
        <T as RosType>::register(set);
    }
    fn write_field(&self, w: &mut CdrWriter) -> cdr::Result<()> {
        T::write_slice(self, w)
    }
    fn read_field(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
        let mut out = [T::default_value(); N];
        for slot in &mut out {
            *slot = <T as RosType>::read(r)?;
        }
        Ok(out)
    }
    fn default_value() -> Self {
        [T::default_value(); N]
    }
}

/// A bounded sequence `T[<=N]`: CDR-identical to an unbounded one, but a
/// distinct type in the type description (and therefore the hash).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BoundedVec<T, const N: usize>(pub Vec<T>);

impl<T: RosType, const N: usize> RosField for BoundedVec<T, N> {
    fn field_type() -> FieldType {
        T::element_type().bounded_sequence(N as u64)
    }
    fn register(set: &mut TypeSet) {
        T::register(set);
    }
    fn write_field(&self, w: &mut CdrWriter) -> cdr::Result<()> {
        if self.0.len() > N {
            return Err(cdr::CdrError::BoundExceeded {
                len: self.0.len(),
                bound: N,
            });
        }
        self.0.write_field(w)
    }
    fn read_field(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
        let n = r.bounded_seq_len(T::MIN_SIZE, N)?;
        T::read_vec(r, n).map(Self)
    }
    fn default_value() -> Self {
        Self(Vec::new())
    }
}

/// A bounded string `string<=N`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BoundedString<const N: usize>(pub String);

impl<const N: usize> RosType for BoundedString<N> {
    const MIN_SIZE: usize = 4;
    fn element_type() -> FieldType {
        FieldType::bounded_string(N as u64)
    }
    fn write(&self, w: &mut CdrWriter) -> cdr::Result<()> {
        if self.0.len() > N {
            return Err(cdr::CdrError::BoundExceeded {
                len: self.0.len(),
                bound: N,
            });
        }
        w.string(&self.0)
    }
    fn read(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
        let bytes = r.string_bytes()?;
        if bytes.len() > N {
            return Err(cdr::CdrError::BoundExceeded {
                len: bytes.len(),
                bound: N,
            });
        }
        let s = String::from_utf8_lossy(bytes).into_owned();
        if s.len() > N {
            return Err(cdr::CdrError::BoundExceeded {
                len: s.len(),
                bound: N,
            });
        }
        Ok(Self(s))
    }
}

impl<const N: usize> RosField for BoundedString<N> {
    fn field_type() -> FieldType {
        <Self as RosType>::element_type()
    }
    fn register(_set: &mut TypeSet) {}
    fn write_field(&self, w: &mut CdrWriter) -> cdr::Result<()> {
        RosType::write(self, w)
    }
    fn read_field(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
        <Self as RosType>::read(r)
    }
    fn default_value() -> Self {
        Self(String::new())
    }
}

/// A ROS 2 message type.
pub trait Message: RosType + Clone + Default + Send + Sync + 'static {
    /// Fully qualified type name, e.g. `geometry_msgs/msg/PoseStamped`.
    const TYPE_NAME: &'static str;

    /// This type's own description.
    fn individual_description() -> IndividualTypeDescription;

    /// The full description, including every referenced type.
    fn type_description() -> TypeDescription {
        let mut set = TypeSet::new();
        Self::register(&mut set);
        TypeDescription::new(Self::individual_description(), &set)
    }

    /// The `RIHS01_…` type hash.
    fn type_hash() -> String {
        Self::type_description().hash()
    }

    /// The DDS-mangled name used by `rmw_zenoh_cpp` key expressions, e.g.
    /// `geometry_msgs::msg::dds_::PoseStamped_`.
    fn dds_type_name() -> String {
        dds_type_name(Self::TYPE_NAME)
    }

    /// Serializes to an encapsulated CDR payload.
    ///
    /// # Errors
    /// A [`cdr::CdrError`] when a value is not representable.
    fn to_cdr(&self) -> cdr::Result<Vec<u8>> {
        let mut w = CdrWriter::new();
        self.write(&mut w)?;
        Ok(w.into_bytes())
    }

    /// Deserializes from an encapsulated CDR payload.
    ///
    /// # Errors
    /// A [`cdr::CdrError`] for truncated or invalid input.
    fn from_cdr(payload: &[u8]) -> cdr::Result<Self> {
        let mut r = CdrReader::new(payload)?;
        Self::read(&mut r)
    }
}

/// Converts `pkg/msg/Type` into `pkg::msg::dds_::Type_`.
#[must_use]
pub fn dds_type_name(type_name: &str) -> String {
    match type_name.rsplit_once('/') {
        Some((namespace, name)) => format!("{}::dds_::{name}_", namespace.replace('/', "::")),
        None => format!("dds_::{type_name}_"),
    }
}

/// Registers `T`'s own description (once) and then its references.
pub fn register_message<T: Message>(set: &mut TypeSet, reference_fields: fn(&mut TypeSet)) {
    if !set.contains_key(T::TYPE_NAME) {
        set.insert(T::TYPE_NAME.to_owned(), T::individual_description());
        reference_fields(set);
    }
}

/// A ROS 2 service type.
pub trait Service: Send + Sync + 'static {
    /// Fully qualified service name, e.g. `std_srvs/srv/Trigger`.
    const TYPE_NAME: &'static str;
    /// The request message; its type name must be `<TYPE_NAME>_Request`.
    type Request: Message;
    /// The response message; its type name must be `<TYPE_NAME>_Response`.
    type Response: Message;

    /// The full service description, including the generated `_Event` type.
    fn type_description() -> TypeDescription {
        let event_name = format!("{}_Event", Self::TYPE_NAME);
        let mut set = TypeSet::new();
        Self::Request::register(&mut set);
        Self::Response::register(&mut set);
        <super::msgs::service_msgs::ServiceEventInfo as RosType>::register(&mut set);
        set.insert(
            event_name.clone(),
            IndividualTypeDescription {
                type_name: event_name.clone(),
                fields: vec![
                    Field::new(
                        "info",
                        FieldType::nested(super::msgs::service_msgs::ServiceEventInfo::TYPE_NAME),
                    ),
                    Field::new(
                        "request",
                        FieldType::nested(Self::Request::TYPE_NAME).bounded_sequence(1),
                    ),
                    Field::new(
                        "response",
                        FieldType::nested(Self::Response::TYPE_NAME).bounded_sequence(1),
                    ),
                ],
            },
        );
        let root = IndividualTypeDescription {
            type_name: Self::TYPE_NAME.to_owned(),
            fields: vec![
                Field::new(
                    "request_message",
                    FieldType::nested(Self::Request::TYPE_NAME),
                ),
                Field::new(
                    "response_message",
                    FieldType::nested(Self::Response::TYPE_NAME),
                ),
                Field::new("event_message", FieldType::nested(&event_name)),
            ],
        };
        TypeDescription::new(root, &set)
    }

    /// The `RIHS01_…` service type hash.
    fn type_hash() -> String {
        Self::type_description().hash()
    }

    /// The DDS-mangled service name used by `rmw_zenoh_cpp`, e.g.
    /// `std_srvs::srv::dds_::Trigger_`.
    fn dds_type_name() -> String {
        dds_type_name(Self::TYPE_NAME)
    }
}

/// Declares a ROS 2 message struct with its CDR encoding and description.
///
/// ```
/// natnet_zenoh::ros_message! {
///     /// A 2-D point.
///     pub struct Point2("example_msgs/msg/Point2") {
///         /// X.
///         pub x: f64,
///         /// Y.
///         pub y: f64,
///     }
/// }
/// use natnet_zenoh::ros::Message;
/// assert_eq!(Point2::dds_type_name(), "example_msgs::msg::dds_::Point2_");
/// let bytes = Point2 { x: 1.0, y: 2.0 }.to_cdr().unwrap();
/// assert_eq!(Point2::from_cdr(&bytes).unwrap(), Point2 { x: 1.0, y: 2.0 });
/// ```
#[macro_export]
macro_rules! ros_message {
    (
        $(#[$meta:meta])*
        pub struct $name:ident($type_name:literal) {
            $(
                $(#[$fmeta:meta])*
                pub $field:ident : $fty:ty $([$override:ident])? $(= $default:expr)?
            ),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq)]
        pub struct $name {
            $(
                $(#[$fmeta])*
                pub $field: $fty,
            )*
        }

        impl ::core::default::Default for $name {
            fn default() -> Self {
                Self {
                    $(
                        $field: $crate::ros_message!(@default $fty $(, $default)?),
                    )*
                }
            }
        }

        impl $crate::ros::message::RosType for $name {
            const MIN_SIZE: usize = 1;
            fn element_type() -> $crate::ros::type_description::FieldType {
                $crate::ros::type_description::FieldType::nested($type_name)
            }
            fn register(set: &mut $crate::ros::type_description::TypeSet) {
                $crate::ros::message::register_message::<$name>(set, |_set| {
                    $(
                        <$fty as $crate::ros::message::RosField>::register(_set);
                    )*
                });
            }
            #[allow(unused_variables)]
            fn write(
                &self,
                w: &mut $crate::ros::cdr::CdrWriter,
            ) -> $crate::ros::cdr::Result<()> {
                $(
                    $crate::ros::message::RosField::write_field(&self.$field, w)?;
                )*
                Ok(())
            }
            #[allow(unused_variables)]
            fn read(
                r: &mut $crate::ros::cdr::CdrReader<'_>,
            ) -> $crate::ros::cdr::Result<Self> {
                Ok(Self {
                    $(
                        $field: <$fty as $crate::ros::message::RosField>::read_field(r)?,
                    )*
                })
            }
        }

        impl $crate::ros::message::RosField for $name {
            fn field_type() -> $crate::ros::type_description::FieldType {
                <$name as $crate::ros::message::RosType>::element_type()
            }
            fn register(set: &mut $crate::ros::type_description::TypeSet) {
                <$name as $crate::ros::message::RosType>::register(set);
            }
            fn write_field(
                &self,
                w: &mut $crate::ros::cdr::CdrWriter,
            ) -> $crate::ros::cdr::Result<()> {
                $crate::ros::message::RosType::write(self, w)
            }
            fn read_field(
                r: &mut $crate::ros::cdr::CdrReader<'_>,
            ) -> $crate::ros::cdr::Result<Self> {
                <$name as $crate::ros::message::RosType>::read(r)
            }
            fn default_value() -> Self {
                ::core::default::Default::default()
            }
        }

        impl $crate::ros::message::Message for $name {
            const TYPE_NAME: &'static str = $type_name;
            fn individual_description() -> $crate::ros::type_description::IndividualTypeDescription {
                $crate::ros::type_description::IndividualTypeDescription {
                    type_name: $type_name.to_owned(),
                    fields: vec![
                        $(
                            $crate::ros::type_description::Field::with_default(
                                stringify!($field).trim_start_matches("r#"),
                                $crate::ros_message!(@field_type $fty $(, $override)?),
                                $crate::ros_message!(@default_str $($default)?),
                            ),
                        )*
                    ],
                }
            }
        }
    };
    (@default $fty:ty) => {
        <$fty as $crate::ros::message::RosField>::default_value()
    };
    (@default $fty:ty, $default:expr) => {
        $default
    };
    (@default_str) => {
        ""
    };
    (@default_str $default:expr) => {
        stringify!($default)
    };
    (@field_type $fty:ty) => {
        <$fty as $crate::ros::message::RosField>::field_type()
    };
    (@field_type $fty:ty, $override:ident) => {{
        let mut t = <$fty as $crate::ros::message::RosField>::field_type();
        let base = <$fty as $crate::ros::message::RosField>::field_type().type_id % 48;
        t.type_id = t.type_id - base + $crate::ros::type_description::field_type::$override;
        t
    }};
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    thread_local! {
        static ITEM_READS: Cell<usize> = const { Cell::new(0) };
    }

    #[derive(Debug, PartialEq)]
    struct CountedItem;

    impl RosType for CountedItem {
        const MIN_SIZE: usize = 1;

        fn element_type() -> FieldType {
            FieldType::primitive(field_type::UINT8)
        }

        fn write(&self, _w: &mut CdrWriter) -> cdr::Result<()> {
            Ok(())
        }

        fn read(r: &mut CdrReader<'_>) -> cdr::Result<Self> {
            ITEM_READS.with(|reads| reads.set(reads.get() + 1));
            r.u8()?;
            Ok(Self)
        }
    }

    #[test]
    fn dds_names() {
        assert_eq!(
            dds_type_name("geometry_msgs/msg/PoseStamped"),
            "geometry_msgs::msg::dds_::PoseStamped_"
        );
        assert_eq!(
            dds_type_name("std_srvs/srv/Trigger"),
            "std_srvs::srv::dds_::Trigger_"
        );
    }

    #[test]
    fn bounded_sequence_rejects_count_before_reading_items() {
        ITEM_READS.with(|reads| reads.set(0));
        let payload = [0, 1, 0, 0, 2, 0, 0, 0, 10, 11];
        let mut reader = CdrReader::new(&payload).unwrap();

        assert_eq!(
            BoundedVec::<CountedItem, 1>::read_field(&mut reader),
            Err(cdr::CdrError::BoundExceeded { len: 2, bound: 1 })
        );
        ITEM_READS.with(|reads| assert_eq!(reads.get(), 0));
    }

    #[test]
    fn bounded_string_checks_raw_bytes_then_preserves_lossy_bound() {
        let over_bound = [0, 1, 0, 0, 3, 0, 0, 0, 0xff, 0xfe, 0];
        let mut reader = CdrReader::new(&over_bound).unwrap();
        assert_eq!(
            BoundedString::<1>::read(&mut reader),
            Err(cdr::CdrError::BoundExceeded { len: 2, bound: 1 })
        );

        let lossy_expansion = [0, 1, 0, 0, 2, 0, 0, 0, 0xff, 0];
        let mut reader = CdrReader::new(&lossy_expansion).unwrap();
        assert_eq!(
            BoundedString::<1>::read(&mut reader),
            Err(cdr::CdrError::BoundExceeded { len: 3, bound: 1 })
        );

        let terminated = [0, 1, 0, 0, 2, 0, 0, 0, b'a', 0];
        let mut reader = CdrReader::new(&terminated).unwrap();
        assert_eq!(
            BoundedString::<1>::read(&mut reader),
            Ok(BoundedString("a".into()))
        );

        let unterminated = [0, 1, 0, 0, 1, 0, 0, 0, b'a'];
        let mut reader = CdrReader::new(&unterminated).unwrap();
        assert_eq!(
            BoundedString::<1>::read(&mut reader),
            Ok(BoundedString("a".into()))
        );

        let interior_nul = [0, 1, 0, 0, 4, 0, 0, 0, b'a', 0, b'b', 0];
        let mut reader = CdrReader::new(&interior_nul).unwrap();
        assert_eq!(
            BoundedString::<3>::read(&mut reader),
            Ok(BoundedString("a\0b".into()))
        );

        let truncated = [0, 1, 0, 0, 3, 0, 0, 0, b'a'];
        let mut reader = CdrReader::new(&truncated).unwrap();
        assert!(matches!(
            BoundedString::<1>::read(&mut reader),
            Err(cdr::CdrError::ImplausibleLength { .. })
        ));
    }
}
