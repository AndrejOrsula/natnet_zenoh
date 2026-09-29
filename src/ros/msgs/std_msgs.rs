//! `std_msgs`: header, string, colour and multi-arrays.
//!
//! This module defines a message named `String`; plain strings are therefore
//! spelled `::std::string::String` here.

use super::builtin_interfaces::Time;

crate::ros_message! {
    /// Stamp and coordinate frame of a message (`std_msgs/msg/Header`).
    pub struct Header("std_msgs/msg/Header") {
        /// Acquisition time.
        pub stamp: Time,
        /// Coordinate frame the data is expressed in.
        pub frame_id: ::std::string::String,
    }
}

crate::ros_message! {
    /// A single string (`std_msgs/msg/String`).
    pub struct String("std_msgs/msg/String") {
        /// The text.
        pub data: ::std::string::String,
    }
}

crate::ros_message! {
    /// An RGBA colour with components in `0..=1` (`std_msgs/msg/ColorRGBA`).
    pub struct ColorRGBA("std_msgs/msg/ColorRGBA") {
        /// Red.
        pub r: f32,
        /// Green.
        pub g: f32,
        /// Blue.
        pub b: f32,
        /// Alpha.
        pub a: f32,
    }
}

crate::ros_message! {
    /// One dimension of a multi-array layout (`std_msgs/msg/MultiArrayDimension`).
    pub struct MultiArrayDimension("std_msgs/msg/MultiArrayDimension") {
        /// Label of the dimension.
        pub label: ::std::string::String,
        /// Size of the dimension.
        pub size: u32,
        /// Stride of the dimension.
        pub stride: u32,
    }
}

crate::ros_message! {
    /// Layout of a multi-array (`std_msgs/msg/MultiArrayLayout`).
    pub struct MultiArrayLayout("std_msgs/msg/MultiArrayLayout") {
        /// Dimensions, outermost first.
        pub dim: Vec<MultiArrayDimension>,
        /// Padding elements at the front of `data`.
        pub data_offset: u32,
    }
}

crate::ros_message! {
    /// A multi-dimensional `float32` array (`std_msgs/msg/Float32MultiArray`).
    pub struct Float32MultiArray("std_msgs/msg/Float32MultiArray") {
        /// Layout of `data`.
        pub layout: MultiArrayLayout,
        /// Row-major data.
        pub data: Vec<f32>,
    }
}

impl ColorRGBA {
    /// A colour from its components.
    #[must_use]
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }
}

impl Float32MultiArray {
    /// A 2-D row-major array of `rows` × `cols` with the given labels.
    #[must_use]
    pub fn matrix(
        row_label: &str,
        col_label: &str,
        rows: usize,
        cols: usize,
        data: Vec<f32>,
    ) -> Self {
        Self {
            layout: MultiArrayLayout {
                dim: vec![
                    MultiArrayDimension {
                        label: row_label.to_owned(),
                        size: rows as u32,
                        stride: (rows * cols) as u32,
                    },
                    MultiArrayDimension {
                        label: col_label.to_owned(),
                        size: cols as u32,
                        stride: cols as u32,
                    },
                ],
                data_offset: 0,
            },
            data,
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
                Header::type_hash(),
                "f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01",
            ),
            (
                String::type_hash(),
                "df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
            ),
            (
                ColorRGBA::type_hash(),
                "77a7a5b9ae477306097665106e0413ba74440245b1f3d0c6d6405fe5c7813fe8",
            ),
            (
                MultiArrayDimension::type_hash(),
                "5e773a60a4c7fc8a54985f307c7837aa2994252a126c301957a24e31282c9cbe",
            ),
            (
                MultiArrayLayout::type_hash(),
                "4c66e6f78e740ac103a94cf63259f968e48c617e7699e829b63c21a5cb50dac6",
            ),
            (
                Float32MultiArray::type_hash(),
                "0599f6f85b4bfca379873a0b4375a0aca022156bd2d7021275d116ed1fa8bfe0",
            ),
        ] {
            assert_eq!(hash, format!("RIHS01_{expected}"));
        }
    }
}
