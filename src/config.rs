//! The bridge configuration file (TOML). Every field has a default, so an
//! empty file is valid; `natnet_zenoh config` prints the defaults.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::mocap::timing::StampPolicy;
use crate::mocap::transform::UpAxis;
use crate::ros::context::RmwZenohCompat;
use crate::ros::names;
use crate::ros::qos::{Durability, Reliability};
use crate::source::live::LiveConfig;
use crate::source::replay::ReplayConfig;
use crate::source::synthetic::SyntheticConfig;

/// The whole configuration.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    /// Where frames come from.
    pub source: SourceConfig,
    /// The zenoh session shared by the ROS 2 and native outputs.
    pub zenoh: ZenohConfig,
    /// ROS 2 (rmw_zenoh) outputs.
    pub ros: RosConfig,
    /// Native zenoh outputs.
    pub native: NativeConfig,
    /// Frame conventions.
    pub transform: TransformConfig,
    /// Time stamping.
    pub timing: TimingConfig,
    /// Rigid-body outputs.
    pub rigid_bodies: RigidBodyConfig,
    /// Marker outputs.
    pub markers: MarkerConfig,
    /// Skeleton outputs.
    pub skeletons: SkeletonConfig,
    /// Force-plate and analog-device outputs.
    pub analog: AnalogConfig,
    /// Diagnostics.
    pub diagnostics: DiagnosticsConfig,
}

/// Which source runs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SourceKind {
    /// A Motive server.
    #[default]
    Live,
    /// A recorded capture.
    Replay,
    /// The built-in synthetic scene.
    Synthetic,
}

/// Source selection and settings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SourceConfig {
    /// The source in use.
    pub kind: SourceKind,
    /// Capacity of the source-to-bridge event queue. A full queue drops
    /// frames unless the source is lossless; unpaced replay (`rate = 0`)
    /// waits for room.
    pub queue: usize,
    /// Live client settings.
    pub live: LiveConfig,
    /// Replay settings.
    pub replay: ReplayConfig,
    /// Synthetic scene settings.
    pub synthetic: SyntheticConfig,
    /// Stop the bridge when a replay ends.
    pub exit_on_end: bool,
}

impl Default for SourceConfig {
    fn default() -> Self {
        Self {
            kind: SourceKind::Live,
            queue: 64,
            live: LiveConfig::default(),
            replay: ReplayConfig::default(),
            synthetic: SyntheticConfig::default(),
            exit_on_end: true,
        }
    }
}

/// The zenoh session.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ZenohConfig {
    /// A zenoh configuration file; defaults to `ZENOH_SESSION_CONFIG_URI`
    /// or the rmw_zenoh session defaults (a peer that connects to the
    /// router on `tcp/localhost:7447`).
    pub config_file: Option<PathBuf>,
    /// Inline JSON5 configuration, used when no file is given.
    pub config_json5: Option<String>,
    /// `key=value` overrides, as in `ZENOH_CONFIG_OVERRIDE`.
    pub overrides: Vec<String>,
    /// Router check attempts before continuing anyway (0 waits forever;
    /// unset uses `ZENOH_ROUTER_CHECK_ATTEMPTS`, else 1; negative in the
    /// variable skips the check).
    pub router_check_attempts: Option<u64>,
}

/// Largest `source.queue`: a few seconds of frames at the highest rates.
pub const MAX_QUEUE: usize = 65_536;

/// Largest `ros.stream_qos.depth`; it also sizes the transient-local cache.
pub const MAX_DEPTH: usize = 10_000;

/// Largest `rigid_bodies.path_length`; each `nav_msgs/Path` carries them all.
pub const MAX_PATH_LENGTH: usize = 100_000;

/// ROS 2 settings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RosConfig {
    /// Publish ROS 2 topics and serve ROS 2 services.
    pub enabled: bool,
    /// ROS domain id; unset reads `ROS_DOMAIN_ID` (default 0).
    pub domain_id: Option<u32>,
    /// Node name.
    pub node_name: String,
    /// Node namespace.
    pub namespace: String,
    /// Security enclave.
    pub enclave: String,
    /// The `rmw_zenoh_cpp` release of the other nodes: `current` (Jazzy
    /// 0.2.4 and later, and the current Kilted and Rolling releases),
    /// `humble`, `jazzy-0.2.3`, `jazzy-0.2.2` or `jazzy-0.2.0` (0.2.0 and
    /// 0.2.1).
    pub rmw_zenoh: RmwZenohCompat,
    /// The fixed mocap world frame.
    pub world_frame: String,
    /// Prefix of every published child frame.
    pub frame_prefix: String,
    /// Publish `/rosout` log messages.
    pub rosout: bool,
    /// Serve the `rcl_interfaces` parameter services and publish
    /// `/parameter_events`, so `ros2 param` can inspect the configuration and
    /// set the keys that can change at run time.
    pub parameters: bool,
    /// Serve `~/get_type_description` (never with `rmw_zenoh = "humble"`).
    pub type_description_service: bool,
    /// Static transforms published once on `/tf_static`.
    pub static_transforms: Vec<StaticTransform>,
    /// QoS of the high-rate streams.
    pub stream_qos: QosConfig,
}

impl Default for RosConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            domain_id: None,
            node_name: "natnet".into(),
            namespace: "/".into(),
            enclave: "/".into(),
            rmw_zenoh: RmwZenohCompat::Current,
            world_frame: "mocap".into(),
            frame_prefix: String::new(),
            rosout: true,
            parameters: true,
            type_description_service: true,
            static_transforms: Vec::new(),
            stream_qos: QosConfig::default(),
        }
    }
}

/// A fixed transform.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StaticTransform {
    /// Parent frame.
    pub parent: String,
    /// Child frame.
    pub child: String,
    /// Translation, metres.
    #[serde(default)]
    pub translation: [f64; 3],
    /// Rotation as roll, pitch, yaw, radians.
    #[serde(default)]
    pub rpy: [f64; 3],
}

/// QoS of a stream.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct QosConfig {
    /// Reliability.
    pub reliability: Reliability,
    /// Durability.
    pub durability: Durability,
    /// History depth.
    pub depth: usize,
}

impl Default for QosConfig {
    fn default() -> Self {
        Self {
            reliability: Reliability::Reliable,
            durability: Durability::Volatile,
            depth: 10,
        }
    }
}

/// Native zenoh outputs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct NativeConfig {
    /// Publish native zenoh keys.
    pub enabled: bool,
    /// Key prefix.
    pub prefix: String,
    /// Publish each frame as JSON on `<prefix>/frame`.
    pub frame_json: bool,
    /// Publish each rigid body as JSON on `<prefix>/rigid_bodies/<name>`.
    pub rigid_body_json: bool,
    /// Publish each frame re-encoded as a NatNet datagram on
    /// `<prefix>/raw/frame`.
    pub raw: bool,
}

impl Default for NativeConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            prefix: "natnet".into(),
            frame_json: true,
            rigid_body_json: true,
            raw: false,
        }
    }
}

/// Frame conventions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TransformConfig {
    /// Motive's streaming up axis.
    pub up_axis: UpAxis,
    /// Scale applied to positions (Motive streams metres).
    pub scale: f64,
    /// A rigid transform applied after the axis conversion, mapping the
    /// mocap frame into `ros.world_frame`.
    pub offset_translation: [f64; 3],
    /// Rotation of that transform as roll, pitch, yaw, radians.
    pub offset_rpy: [f64; 3],
}

impl Default for TransformConfig {
    fn default() -> Self {
        Self {
            up_axis: UpAxis::Y,
            scale: 1.0,
            offset_translation: [0.0; 3],
            offset_rpy: [0.0; 3],
        }
    }
}

/// Time stamping.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TimingConfig {
    /// Stamp policy.
    pub policy: StampPolicy,
    /// Clock-offset estimation window, seconds.
    pub window_secs: f64,
    /// Publish at most this many frames per second (0: every frame);
    /// every frame is still processed, so velocities stay accurate.
    pub max_rate_hz: f64,
}

impl Default for TimingConfig {
    fn default() -> Self {
        Self {
            policy: StampPolicy::ClockSync,
            window_secs: 10.0,
            max_rate_hz: 0.0,
        }
    }
}

/// Rigid-body outputs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RigidBodyConfig {
    /// Publish `/tf`.
    pub tf: bool,
    /// Publish `~/rigid_bodies/<name>/pose` (`geometry_msgs/PoseStamped`).
    pub pose: bool,
    /// Publish `~/rigid_bodies/<name>/odom` (`nav_msgs/Odometry`).
    pub odometry: bool,
    /// Publish `~/rigid_bodies/<name>/twist` (`geometry_msgs/TwistStamped`,
    /// body frame).
    pub twist: bool,
    /// Publish `~/rigid_bodies/<name>/path` (`nav_msgs/Path`).
    pub path: bool,
    /// Path length, poses; paths are sampled at 10 Hz.
    pub path_length: usize,
    /// Publish all bodies on `~/rigid_bodies/poses` (`geometry_msgs/PoseArray`).
    pub pose_array: bool,
    /// Publish `mocap4r2_msgs/RigidBodies` on `rigid_bodies`.
    pub mocap4r2: bool,
    /// Publish the pose Motive still reports for untracked bodies and
    /// skeleton bones (usually the last tracked one); velocity is never
    /// estimated from it.
    pub publish_untracked: bool,
    /// Only these bodies (names, `*` wildcards); empty for all.
    pub include: Vec<String>,
    /// Never these bodies (names, `*` wildcards).
    pub exclude: Vec<String>,
    /// Output names by Motive name.
    pub rename: BTreeMap<String, String>,
    /// Velocity smoothing factor in (0, 1]; 1 disables smoothing.
    pub velocity_alpha: f64,
    /// Longest gap bridged by velocity estimation, seconds.
    pub velocity_max_gap: f64,
}

impl Default for RigidBodyConfig {
    fn default() -> Self {
        Self {
            tf: true,
            pose: true,
            odometry: true,
            twist: false,
            path: false,
            path_length: 500,
            pose_array: true,
            mocap4r2: false,
            publish_untracked: false,
            include: Vec::new(),
            exclude: Vec::new(),
            rename: BTreeMap::new(),
            velocity_alpha: 0.5,
            velocity_max_gap: 0.1,
        }
    }
}

/// Marker outputs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MarkerConfig {
    /// Publish markers on `~/markers` (`sensor_msgs/PointCloud2`).
    pub point_cloud: bool,
    /// Include unlabeled markers in the cloud.
    pub include_unlabeled: bool,
    /// Publish `mocap4r2_msgs/Markers` on `markers`.
    pub mocap4r2: bool,
    /// Publish `visualization_msgs/MarkerArray` on `~/visualization`.
    pub visualization: bool,
    /// Visualization rate limit, Hz (0: every frame).
    pub visualization_rate_hz: f64,
    /// Axis length of the rigid-body triads, metres.
    pub axis_length: f64,
}

impl Default for MarkerConfig {
    fn default() -> Self {
        Self {
            point_cloud: true,
            include_unlabeled: true,
            mocap4r2: false,
            visualization: true,
            visualization_rate_hz: 30.0,
            axis_length: 0.1,
        }
    }
}

/// Whether skeleton bone poses are global or relative to the parent bone
/// (Motive's "Skeleton Coordinates" streaming setting).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoneCoordinates {
    /// Bone poses in the world frame.
    #[default]
    Global,
    /// Bone poses relative to the parent bone.
    Local,
}

/// Skeleton outputs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SkeletonConfig {
    /// Publish bone frames on `/tf`.
    pub tf: bool,
    /// Publish `~/skeletons/<name>/poses` (`geometry_msgs/PoseArray`, world
    /// frame, bone order).
    pub pose_array: bool,
    /// How Motive streams bone poses.
    pub coordinates: BoneCoordinates,
}

impl Default for SkeletonConfig {
    fn default() -> Self {
        Self {
            tf: true,
            pose_array: true,
            coordinates: BoneCoordinates::Global,
        }
    }
}

/// Force plates and analog devices.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct AnalogConfig {
    /// Publish `~/force_plates/force_plate_<id>/wrench`
    /// (`geometry_msgs/WrenchStamped`).
    pub force_plate_wrench: bool,
    /// Publish `~/force_plates/force_plate_<id>/channels`
    /// (`std_msgs/Float32MultiArray`,
    /// channels × subframes).
    pub force_plate_channels: bool,
    /// Publish `~/devices/<name>/channels` (`std_msgs/Float32MultiArray`).
    pub device_channels: bool,
}

impl Default for AnalogConfig {
    fn default() -> Self {
        Self {
            force_plate_wrench: true,
            force_plate_channels: true,
            device_channels: true,
        }
    }
}

/// Diagnostics.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DiagnosticsConfig {
    /// Publish `/diagnostics`.
    pub enabled: bool,
    /// Period, seconds.
    pub period_secs: f64,
    /// Warn above this share of dropped frames over a period.
    pub drop_warn_ratio: f64,
    /// Warn above this total latency, seconds.
    pub latency_warn_secs: f64,
}

impl Default for DiagnosticsConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            period_secs: 1.0,
            drop_warn_ratio: 0.05,
            latency_warn_secs: 0.05,
        }
    }
}

/// A configuration error.
#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    /// The file could not be read.
    #[error("{path}: {source}")]
    Io {
        /// The file.
        path: PathBuf,
        /// The cause.
        source: std::io::Error,
    },
    /// The file is not valid TOML for this schema.
    #[error("{path}: {source}")]
    Parse {
        /// The file.
        path: PathBuf,
        /// The cause.
        source: Box<toml::de::Error>,
    },
    /// A value is out of range or contradictory.
    #[error("invalid configuration: {0}")]
    Invalid(String),
}

impl Config {
    /// Reads and validates a configuration file.
    ///
    /// # Errors
    /// I/O, parse and validation errors.
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let cfg = Self::read(path)?;
        cfg.validate()?;
        Ok(cfg)
    }

    /// Reads a configuration file without [validating](Self::validate) it,
    /// for callers that change it first.
    ///
    /// # Errors
    /// I/O and parse errors.
    pub fn read(path: &Path) -> Result<Self, ConfigError> {
        let text = std::fs::read_to_string(path).map_err(|source| ConfigError::Io {
            path: path.to_owned(),
            source,
        })?;
        toml::from_str(&text).map_err(|source| ConfigError::Parse {
            path: path.to_owned(),
            source: Box::new(source),
        })
    }

    /// The defaults as TOML.
    #[must_use]
    pub fn default_toml() -> String {
        toml::to_string_pretty(&Self::default()).unwrap_or_default()
    }

    /// Checks the selected source, settings always in use, and enabled ROS
    /// and native sections.
    ///
    /// # Errors
    /// [`ConfigError::Invalid`] naming the first problem.
    pub fn validate(&self) -> Result<(), ConfigError> {
        self.validate_sections(false)
    }

    /// [`Self::validate`] for every section, used or not, so that command
    /// line switches (`--replay`, `--native`, ...) cannot uncover a problem
    /// later. A replay section without a path is skipped unless selected.
    ///
    /// # Errors
    /// [`ConfigError::Invalid`] naming the first problem.
    pub fn validate_all(&self) -> Result<(), ConfigError> {
        self.validate_sections(true)
    }

    fn validate_sections(&self, all: bool) -> Result<(), ConfigError> {
        let bad = |m: String| Err(ConfigError::Invalid(m));
        let kind = self.source.kind;
        if all || kind == SourceKind::Live {
            self.source
                .live
                .validate()
                .map_err(|e| ConfigError::Invalid(format!("source.live: {e}")))?;
        }
        if kind == SourceKind::Replay || (all && !self.source.replay.path.as_os_str().is_empty()) {
            self.source
                .replay
                .validate()
                .map_err(|e| ConfigError::Invalid(format!("source.replay: {e}")))?;
        }
        if all || kind == SourceKind::Synthetic {
            crate::source::synthetic::Scene::new(self.source.synthetic.clone())
                .map_err(|e| ConfigError::Invalid(format!("source.synthetic: {e}")))?;
        }
        // The channel allocates every slot up front.
        if !(1..=MAX_QUEUE).contains(&self.source.queue) {
            return bad(format!("source.queue must be in 1..={MAX_QUEUE}"));
        }
        if all || self.ros.enabled {
            names::validate_node_name(&self.ros.node_name)
                .map_err(|e| ConfigError::Invalid(format!("ros.node_name: {e}")))?;
            names::validate_namespace(&names::normalize_namespace(&self.ros.namespace))
                .map_err(|e| ConfigError::Invalid(format!("ros.namespace: {e}")))?;
            if self.ros.world_frame.is_empty() {
                return bad("ros.world_frame must not be empty".into());
            }
            if !(1..=MAX_DEPTH).contains(&self.ros.stream_qos.depth) {
                return bad(format!("ros.stream_qos.depth must be in 1..={MAX_DEPTH}"));
            }
        }
        if (all || self.native.enabled)
            && (zenoh::key_expr::keyexpr::new(self.native.prefix.as_str()).is_err()
                || self.native.prefix.contains(['*', '$', '#', '?']))
        {
            return bad(
                "native.prefix must be a key expression without wildcards or empty chunks".into(),
            );
        }
        if !(self.transform.scale.is_finite() && self.transform.scale > 0.0) {
            return bad("transform.scale must be positive".into());
        }
        if !(self.timing.window_secs.is_finite() && self.timing.window_secs > 0.0) {
            return bad("timing.window_secs must be positive".into());
        }
        let rate_or_off = |r: f64| r == 0.0 || (r.is_finite() && (1e-3..=1e5).contains(&r));
        if !rate_or_off(self.timing.max_rate_hz) {
            return bad("timing.max_rate_hz must be 0 (off) or in [0.001, 100000]".into());
        }
        if self.timing.window_secs > 1e6 {
            return bad("timing.window_secs must be at most 1e6".into());
        }
        let finite = |v: &[f64]| v.iter().all(|x| x.is_finite());
        if !finite(&self.transform.offset_translation) || !finite(&self.transform.offset_rpy) {
            return bad("transform offsets must be finite".into());
        }
        for t in &self.ros.static_transforms {
            if t.parent.is_empty() || t.child.is_empty() {
                return bad("ros.static_transforms: parent and child must not be empty".into());
            }
            if !finite(&t.translation) || !finite(&t.rpy) {
                return bad(format!(
                    "ros.static_transforms {}: values must be finite",
                    t.child
                ));
            }
        }
        let rb = &self.rigid_bodies;
        if !(rb.velocity_alpha > 0.0 && rb.velocity_alpha <= 1.0) {
            return bad("rigid_bodies.velocity_alpha must be in (0, 1]".into());
        }
        if !(rb.velocity_max_gap.is_finite()
            && rb.velocity_max_gap > 0.0
            && rb.velocity_max_gap <= 1e6)
        {
            return bad("rigid_bodies.velocity_max_gap must be positive".into());
        }
        if rb.path && !(1..=MAX_PATH_LENGTH).contains(&rb.path_length) {
            return bad(format!(
                "rigid_bodies.path_length must be in 1..={MAX_PATH_LENGTH}"
            ));
        }
        for (from, to) in &rb.rename {
            if to.trim().is_empty() {
                return bad(format!("rigid_bodies.rename: empty name for {from:?}"));
            }
        }
        if !rate_or_off(self.markers.visualization_rate_hz) {
            return bad(
                "markers.visualization_rate_hz must be 0 (every frame) or in [0.001, 100000]"
                    .into(),
            );
        }
        if !(self.markers.axis_length.is_finite()
            && self.markers.axis_length > 0.0
            && self.markers.axis_length <= 1e3)
        {
            return bad("markers.axis_length must be in (0, 1000] m".into());
        }
        let d = &self.diagnostics;
        if !(d.drop_warn_ratio.is_finite() && d.drop_warn_ratio >= 0.0) {
            return bad("diagnostics.drop_warn_ratio must be a non-negative number".into());
        }
        if !(d.latency_warn_secs.is_finite() && d.latency_warn_secs >= 0.0) {
            return bad("diagnostics.latency_warn_secs must be a non-negative number".into());
        }
        if !(self.diagnostics.period_secs.is_finite()
            && (1e-3..=1e6).contains(&self.diagnostics.period_secs))
        {
            return bad("diagnostics.period_secs must be in [0.001, 1e6]".into());
        }
        Ok(())
    }
}

/// Matches `name` against a pattern with `*` wildcards.
#[must_use]
pub fn glob_match(pattern: &str, name: &str) -> bool {
    let parts: Vec<&str> = pattern.split('*').collect();
    if parts.len() == 1 {
        return pattern == name;
    }
    let mut rest = name;
    for (i, part) in parts.iter().enumerate() {
        if i == 0 {
            let Some(r) = rest.strip_prefix(part) else {
                return false;
            };
            rest = r;
        } else if i == parts.len() - 1 {
            return rest.ends_with(part);
        } else if let Some(pos) = rest.find(part) {
            rest = rest.get(pos + part.len()..).unwrap_or_default();
        } else {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_round_trip_and_validate() {
        let text = Config::default_toml();
        let back: Config = toml::from_str(&text).unwrap();
        assert_eq!(back, Config::default());
        back.validate().unwrap();
        let empty: Config = toml::from_str("").unwrap();
        assert_eq!(empty, Config::default());
    }

    #[test]
    fn rejects_unknown_keys_and_bad_values() {
        assert!(toml::from_str::<Config>("[ros]\nnode = 'x'").is_err());
        let mut c = Config::default();
        c.ros.node_name = "bad name".into();
        assert!(c.validate().is_err());
        let mut c = Config::default();
        c.rigid_bodies.velocity_alpha = 0.0;
        assert!(c.validate().is_err());
        let c: Config = toml::from_str("[source]\nkind = 'replay'").unwrap();
        assert!(c.validate().is_err(), "replay needs a path");

        // Unused sections only fail `validate_all`.
        let mut c = Config::default();
        c.native.prefix = "natnet/".into();
        c.validate().unwrap();
        assert!(c.validate_all().is_err());
        c.native.prefix = "lab/natnet".into();
        c.validate_all().unwrap();
        let mut c = Config::default();
        c.source.kind = SourceKind::Synthetic;
        c.source.live.command_port = 0;
        c.validate().unwrap();
        assert!(c.validate_all().is_err());
    }

    #[test]
    fn rejects_values_that_would_panic_or_poison_poses() {
        let invalid = |edit: fn(&mut Config)| {
            let mut c = Config::default();
            edit(&mut c);
            c.validate().is_err()
        };
        assert!(invalid(|c| c.timing.max_rate_hz = 1e-20));
        assert!(invalid(|c| c.markers.visualization_rate_hz = f64::INFINITY));
        assert!(invalid(|c| c.diagnostics.period_secs = 1e20));
        assert!(invalid(|c| c.transform.offset_translation[0] = f64::NAN));
        assert!(invalid(|c| c.rigid_bodies.velocity_max_gap = 1e300));
        assert!(invalid(|c| c.markers.axis_length = f64::NAN));
        assert!(invalid(|c| c.diagnostics.drop_warn_ratio = f64::NAN));
        assert!(invalid(|c| c.diagnostics.latency_warn_secs = -1.0));
        assert!(invalid(|c| {
            c.rigid_bodies.path = true;
            c.rigid_bodies.path_length = usize::MAX;
        }));
        assert!(invalid(|c| c.ros.stream_qos.depth = usize::MAX));
        assert!(!invalid(|c| c.timing.max_rate_hz = 0.0));
    }

    #[test]
    fn partial_file() {
        let c: Config = toml::from_str(
            r#"
            [source]
            kind = "synthetic"
            [source.synthetic]
            rigid_bodies = 5
            [ros]
            namespace = "/lab"
            [[ros.static_transforms]]
            parent = "map"
            child = "mocap"
            translation = [1.0, 2.0, 0.0]
            [rigid_bodies.rename]
            "Rover 1" = "rover1"
            "#,
        )
        .unwrap();
        c.validate().unwrap();
        assert_eq!(c.source.synthetic.rigid_bodies, 5);
        assert_eq!(c.source.synthetic.markers_per_body, 4);
        assert_eq!(c.ros.static_transforms[0].rpy, [0.0; 3]);
    }

    #[test]
    fn globs() {
        assert!(glob_match("rover*", "rover_1"));
        assert!(glob_match("*_1", "rover_1"));
        assert!(glob_match("r*v*1", "rover_1"));
        assert!(glob_match("*", ""));
        assert!(!glob_match("rover", "rover_1"));
        assert!(!glob_match("a*b", "ab_c"));
    }
}
