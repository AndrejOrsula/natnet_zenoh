//! `sensor_msgs`: point clouds and compressed images.

use super::std_msgs::Header;

crate::ros_message! {
    /// One field of a [`PointCloud2`] point (`sensor_msgs/msg/PointField`).
    pub struct PointField("sensor_msgs/msg/PointField") {
        /// Field name, e.g. `x`.
        pub name: String,
        /// Byte offset within the point.
        pub offset: u32,
        /// Element datatype, one of the associated constants.
        pub datatype: u8,
        /// Number of elements.
        pub count: u32,
    }
}

crate::ros_message! {
    /// A packed point cloud (`sensor_msgs/msg/PointCloud2`).
    pub struct PointCloud2("sensor_msgs/msg/PointCloud2") {
        /// Stamp and frame.
        pub header: Header,
        /// Rows; 1 for an unordered cloud.
        pub height: u32,
        /// Points per row.
        pub width: u32,
        /// Point-field definitions and byte offsets.
        pub fields: Vec<PointField>,
        /// Whether `data` is big-endian.
        pub is_bigendian: bool,
        /// Bytes per point.
        pub point_step: u32,
        /// Bytes per row.
        pub row_step: u32,
        /// Packed points.
        pub data: Vec<u8>,
        /// Whether there are no invalid points.
        pub is_dense: bool,
    }
}

crate::ros_message! {
    /// A compressed image (`sensor_msgs/msg/CompressedImage`).
    pub struct CompressedImage("sensor_msgs/msg/CompressedImage") {
        /// Stamp and frame.
        pub header: Header,
        /// Compression format, e.g. `jpeg`.
        pub format: String,
        /// Compressed bytes.
        pub data: Vec<u8>,
    }
}

impl PointField {
    /// `int8`.
    pub const INT8: u8 = 1;
    /// `uint8`.
    pub const UINT8: u8 = 2;
    /// `int16`.
    pub const INT16: u8 = 3;
    /// `uint16`.
    pub const UINT16: u8 = 4;
    /// `int32`.
    pub const INT32: u8 = 5;
    /// `uint32`.
    pub const UINT32: u8 = 6;
    /// `float32`.
    pub const FLOAT32: u8 = 7;
    /// `float64`.
    pub const FLOAT64: u8 = 8;

    /// A single-element channel.
    #[must_use]
    pub fn new(name: &str, offset: u32, datatype: u8) -> Self {
        Self {
            name: name.to_owned(),
            offset,
            datatype,
            count: 1,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn hashes_match_jazzy() {
        assert_eq!(
            PointField::type_hash(),
            "RIHS01_5c6a4750728c2bcfbbf7037225b20b02d4429634732146b742dee1726637ef01"
        );
        assert_eq!(
            PointCloud2::type_hash(),
            "RIHS01_9198cabf7da3796ae6fe19c4cb3bdd3525492988c70522628af5daa124bae2b5"
        );
        assert_eq!(
            CompressedImage::type_hash(),
            "RIHS01_15640771531571185e2efc8a100baf923961a4d15d5569652e6cb6691e8e371a"
        );
    }
}
