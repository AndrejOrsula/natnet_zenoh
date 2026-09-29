//! `visualization_msgs`: RViz markers.

use super::builtin_interfaces::Duration;
use super::geometry_msgs::{Point, Pose, Vector3};
use super::sensor_msgs::CompressedImage;
use super::std_msgs::{ColorRGBA, Header};

crate::ros_message! {
    /// Texture coordinate (`visualization_msgs/msg/UVCoordinate`).
    pub struct UVCoordinate("visualization_msgs/msg/UVCoordinate") {
        /// U.
        pub u: f32,
        /// V.
        pub v: f32,
    }
}

crate::ros_message! {
    /// An embedded mesh file (`visualization_msgs/msg/MeshFile`).
    pub struct MeshFile("visualization_msgs/msg/MeshFile") {
        /// File name, used for its extension.
        pub filename: String,
        /// File contents.
        pub data: Vec<u8>,
    }
}

crate::ros_message! {
    /// A visualization primitive (`visualization_msgs/msg/Marker`).
    pub struct Marker("visualization_msgs/msg/Marker") {
        /// Stamp and frame.
        pub header: Header,
        /// Namespace; with `id` identifies the marker.
        pub ns: String,
        /// Identifier within the namespace.
        pub id: i32,
        /// Shape, one of the associated shape constants.
        pub r#type: i32,
        /// Action, one of the associated action constants.
        pub action: i32,
        /// Pose of the marker.
        pub pose: Pose,
        /// Scale; semantics depend on the type.
        pub scale: Vector3,
        /// Colour.
        pub color: ColorRGBA,
        /// Lifetime; zero means forever.
        pub lifetime: Duration,
        /// Whether to retransform into the frame every update.
        pub frame_locked: bool,
        /// Points for list and strip types.
        pub points: Vec<Point>,
        /// Per-point colours.
        pub colors: Vec<ColorRGBA>,
        /// Texture resource URI.
        pub texture_resource: String,
        /// Embedded texture.
        pub texture: CompressedImage,
        /// Texture coordinates.
        pub uv_coordinates: Vec<UVCoordinate>,
        /// Text for `TEXT_VIEW_FACING`.
        pub text: String,
        /// Mesh resource URI for `MESH_RESOURCE`.
        pub mesh_resource: String,
        /// Embedded mesh.
        pub mesh_file: MeshFile,
        /// Whether to use the mesh's embedded materials.
        pub mesh_use_embedded_materials: bool,
    }
}

crate::ros_message! {
    /// A batch of markers (`visualization_msgs/msg/MarkerArray`).
    pub struct MarkerArray("visualization_msgs/msg/MarkerArray") {
        /// The markers.
        pub markers: Vec<Marker>,
    }
}

impl Marker {
    /// Arrow.
    pub const ARROW: i32 = 0;
    /// Cube.
    pub const CUBE: i32 = 1;
    /// Sphere.
    pub const SPHERE: i32 = 2;
    /// Cylinder.
    pub const CYLINDER: i32 = 3;
    /// Line strip.
    pub const LINE_STRIP: i32 = 4;
    /// Line list.
    pub const LINE_LIST: i32 = 5;
    /// Cube list.
    pub const CUBE_LIST: i32 = 6;
    /// Sphere list.
    pub const SPHERE_LIST: i32 = 7;
    /// Points.
    pub const POINTS: i32 = 8;
    /// View-facing text.
    pub const TEXT_VIEW_FACING: i32 = 9;
    /// Mesh resource.
    pub const MESH_RESOURCE: i32 = 10;
    /// Triangle list.
    pub const TRIANGLE_LIST: i32 = 11;
    /// Arrow strip.
    pub const ARROW_STRIP: i32 = 12;

    /// Add or modify.
    pub const ADD: i32 = 0;
    /// Modify (deprecated alias of `ADD`).
    pub const MODIFY: i32 = 0;
    /// Delete this marker.
    pub const DELETE: i32 = 2;
    /// Delete every marker in the namespace.
    pub const DELETEALL: i32 = 3;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn hashes_match_jazzy() {
        assert_eq!(
            UVCoordinate::type_hash(),
            "RIHS01_f27f7ed21fe360c6066944f856b801a0c0d1e94e815b6886444b42d90b196a26"
        );
        assert_eq!(
            MeshFile::type_hash(),
            "RIHS01_7710ece15a148fb7c9b546364cfb215bb06098087bd6394fe5b73a493508f8c4"
        );
        assert_eq!(
            Marker::type_hash(),
            "RIHS01_45b13ccf791f225962bf74e746f9644518855d783a6f42ba0cc14fde2b4f3ce0"
        );
        assert_eq!(
            MarkerArray::type_hash(),
            "RIHS01_86cb8800b6fb05b5eff1abd7a56f62a5641d3ae9a1c29e78e67e704f1d067dcf"
        );
    }
}
