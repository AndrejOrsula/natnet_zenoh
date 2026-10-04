//! Turns decoded NatNet frames into output-ready samples: names resolved
//! from the model, poses converted to the output frame, stamps computed and
//! velocities estimated. Everything here is independent of zenoh and ROS.

use std::collections::{BTreeMap, BTreeSet};
use std::time::{Instant, SystemTime};

use natnet::{FrameOfData, ModelDef, ServerInfo, Version};
use serde::Serialize;

use crate::config::{BoneCoordinates, Config, glob_match};
use crate::mocap::model::Model;
use crate::mocap::sections::{self, split_id};
use crate::mocap::stats::{Sequence, StreamStats};
use crate::mocap::timing::{Stamp, StampPolicy, Stamper};
use crate::mocap::transform::FrameConvention;
use crate::mocap::transform::quat::{self, Quat};
use crate::mocap::velocity::{Velocity, VelocityEstimator};
use crate::ros::names::sanitize_token;

/// Labeled-marker `params` bit: occluded.
pub const MARKER_OCCLUDED: u16 = 0x01;
/// Labeled-marker `params` bit: unlabeled.
pub const MARKER_UNLABELED: u16 = 0x10;

/// A pose in the output frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Pose {
    /// Position, metres.
    pub position: [f64; 3],
    /// Orientation `[x, y, z, w]`, unit.
    pub orientation: Quat,
}

impl Pose {
    /// The identity.
    pub const IDENTITY: Self = Self {
        position: [0.0; 3],
        orientation: quat::IDENTITY,
    };

    /// `self ∘ other`: `other` expressed in `self`'s parent frame.
    #[must_use]
    pub fn compose(&self, other: &Self) -> Self {
        let r = quat::rotate(self.orientation, other.position);
        Self {
            position: [
                self.position[0] + r[0],
                self.position[1] + r[1],
                self.position[2] + r[2],
            ],
            orientation: quat::normalize(quat::mul(self.orientation, other.orientation))
                .unwrap_or(quat::IDENTITY),
        }
    }

    /// The inverse transform.
    #[must_use]
    pub fn inverse(&self) -> Self {
        let q = quat::conj(self.orientation);
        let p = quat::rotate(q, self.position);
        Self {
            position: [-p[0], -p[1], -p[2]],
            orientation: q,
        }
    }
}

/// One rigid body.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BodySample {
    /// Streaming id.
    pub id: i32,
    /// Name in Motive.
    pub motive_name: String,
    /// Output name (a valid ROS name token).
    pub name: String,
    /// TF child frame.
    pub frame_id: String,
    /// Motive tracks the body.
    pub tracked: bool,
    /// The pose, when valid.
    pub pose: Option<Pose>,
    /// Mean marker error, metres.
    pub mean_error: f32,
    /// Estimated velocity.
    pub velocity: Option<Velocity>,
    /// Model marker positions in the output frame (for mocap4r2 and the
    /// native documents; empty when neither is enabled).
    pub markers: Vec<[f64; 3]>,
}

/// One labeled marker.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MarkerSample {
    /// Full marker id.
    pub id: i32,
    /// Owning model id (0 for unlabeled or free markers).
    pub model_id: i32,
    /// Member index within the model.
    pub member_id: i32,
    /// Position, output frame.
    pub position: [f64; 3],
    /// Diameter, metres.
    pub size: f32,
    /// Residual, metres.
    pub residual: f32,
    /// NatNet parameter bits.
    pub params: u16,
}

impl MarkerSample {
    /// Motive labeled the marker.
    #[must_use]
    pub fn labeled(&self) -> bool {
        self.params & MARKER_UNLABELED == 0
    }
}

/// One skeleton bone.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BoneSample {
    /// Bone id within the skeleton.
    pub id: i32,
    /// Bone name.
    pub name: String,
    /// TF child frame.
    pub frame_id: String,
    /// Parent bone id from the model (`None` for a root or an unknown bone).
    pub parent: Option<i32>,
    /// TF parent frame (the parent bone in local mode, else the world).
    pub parent_frame: Option<String>,
    /// Pose relative to `parent_frame` (the world when `None`).
    pub local: Pose,
    /// Pose in the world frame.
    pub global: Pose,
    /// Motive tracks the bone.
    pub tracked: bool,
}

/// One skeleton.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SkeletonSample {
    /// Id.
    pub id: i32,
    /// Output name.
    pub name: String,
    /// Bones in stream order.
    pub bones: Vec<BoneSample>,
}

/// One force plate or analog device.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct AnalogSample {
    /// Id.
    pub id: i32,
    /// Output name.
    pub name: String,
    /// Channel names from the model (may be empty).
    pub channel_names: Vec<String>,
    /// Samples per channel within this frame.
    pub channels: Vec<Vec<f32>>,
}

/// Everything derived from one frame.
#[derive(Debug, Clone, Serialize)]
pub struct Snapshot {
    /// Frame number.
    pub frame_number: i32,
    /// Stamp, seconds since the Unix epoch.
    pub stamp_secs: f64,
    /// Stamp details.
    #[serde(skip)]
    pub stamp: Stamp,
    /// Server latency, seconds.
    pub server_latency: Option<f64>,
    /// Total latency estimate, seconds.
    pub total_latency: Option<f64>,
    /// Rigid bodies passing the include/exclude filters.
    pub bodies: Vec<BodySample>,
    /// Labeled markers.
    pub markers: Vec<MarkerSample>,
    /// Skeletons.
    pub skeletons: Vec<SkeletonSample>,
    /// Force plates.
    pub force_plates: Vec<AnalogSample>,
    /// Analog devices.
    pub devices: Vec<AnalogSample>,
    /// Motive is recording.
    pub recording: bool,
}

#[derive(Debug, Clone)]
struct BodyName {
    motive: String,
    token: String,
    frame_id: String,
    included: bool,
    /// Whether the model describes the body (false for a generic name).
    known: bool,
}

#[derive(Debug, Clone, Default)]
struct Names {
    bodies: BTreeMap<i32, BodyName>,
    skeletons: BTreeMap<i32, SkeletonNames>,
    force_plates: BTreeMap<i32, String>,
    devices: BTreeMap<i32, String>,
}

/// A skeleton's output token and its bones' names, by bone id.
#[derive(Debug, Clone, Default)]
struct SkeletonNames {
    token: String,
    bones: BTreeMap<i32, BoneName>,
}

/// A bone's output name, frame id and parent bone.
#[derive(Debug, Clone)]
struct BoneName {
    name: String,
    frame_id: String,
    parent: Option<i32>,
}

/// `token`, or when taken `token_<id>` (then `token_<id>_2`, ...), recorded
/// as taken.
fn unique(taken: &mut BTreeSet<String>, token: &str, id: i32) -> String {
    let mut candidate = token.to_owned();
    let mut n = 1;
    while taken.contains(&candidate) {
        candidate = if n == 1 {
            format!("{token}_{id}")
        } else {
            format!("{token}_{id}_{n}")
        };
        n += 1;
    }
    taken.insert(candidate.clone());
    candidate
}

/// The frame processor.
#[derive(Debug)]
pub struct Processor {
    convention: FrameConvention,
    offset: Pose,
    frame_prefix: String,
    world_frame: String,
    rename: BTreeMap<String, String>,
    include: Vec<String>,
    exclude: Vec<String>,
    coordinates: BoneCoordinates,
    publish_untracked: bool,
    /// Whether an output uses [`BodySample::markers`].
    body_markers: bool,
    model: Model,
    names: Names,
    stamper: Stamper,
    timing: (StampPolicy, f64),
    velocity: VelocityEstimator,
    /// Stream statistics.
    pub stats: StreamStats,
    unknown: BTreeSet<String>,
    server_info: Option<ServerInfo>,
    model_version: Option<Version>,
}

impl Processor {
    /// A processor for a configuration.
    #[must_use]
    pub fn new(cfg: &Config) -> Self {
        let t = &cfg.transform;
        let [roll, pitch, yaw] = t.offset_rpy;
        let rb = &cfg.rigid_bodies;
        let mut p = Self {
            convention: FrameConvention {
                up_axis: t.up_axis,
                scale: t.scale,
            },
            offset: Pose {
                position: t.offset_translation,
                orientation: quat::from_rpy(roll, pitch, yaw),
            },
            frame_prefix: cfg.ros.frame_prefix.clone(),
            world_frame: cfg.ros.world_frame.clone(),
            rename: rb.rename.clone(),
            include: rb.include.clone(),
            exclude: rb.exclude.clone(),
            coordinates: cfg.skeletons.coordinates,
            publish_untracked: cfg.rigid_bodies.publish_untracked,
            body_markers: cfg.native.enabled || (cfg.ros.enabled && rb.mocap4r2),
            model: Model::default(),
            names: Names::default(),
            stamper: Stamper::new(cfg.timing.policy, 0, cfg.timing.window_secs),
            timing: (cfg.timing.policy, cfg.timing.window_secs),
            velocity: VelocityEstimator::new(rb.velocity_alpha, rb.velocity_max_gap),
            stats: StreamStats::default(),
            unknown: BTreeSet::new(),
            server_info: None,
            model_version: None,
        };
        p.rebuild_names();
        p
    }

    /// Adopts a configuration changed at run time. Velocity history is
    /// dropped only when the frame convention or the estimator changed, and
    /// clock synchronization restarts only when its settings changed.
    pub fn reconfigure(&mut self, cfg: &Config) {
        let fresh = Self::new(cfg);
        if fresh.convention != self.convention
            || fresh.offset != self.offset
            || fresh.velocity.settings() != self.velocity.settings()
        {
            self.velocity = fresh.velocity;
        }
        self.convention = fresh.convention;
        self.offset = fresh.offset;
        self.include = fresh.include;
        self.exclude = fresh.exclude;
        self.coordinates = fresh.coordinates;
        self.publish_untracked = fresh.publish_untracked;
        self.body_markers = fresh.body_markers;
        if (cfg.timing.policy, cfg.timing.window_secs) != self.timing {
            self.timing = fresh.timing;
            let mut stamper = fresh.stamper;
            stamper.continue_from(&self.stamper);
            self.stamper = stamper;
            if let Some(info) = &self.server_info {
                self.stamper.set_frequency(info.high_res_clock_frequency);
            }
        }
        self.rebuild_names();
    }

    /// Adopts a SERVERINFO.
    pub fn set_server_info(&mut self, info: &ServerInfo) {
        self.stamper.set_frequency(info.high_res_clock_frequency);
        self.server_info = Some(info.clone());
    }

    /// The last SERVERINFO.
    #[must_use]
    pub fn server_info(&self) -> Option<&ServerInfo> {
        self.server_info.as_ref()
    }

    /// Adopts a model definition; returns decoding problems worth logging.
    pub fn set_model(&mut self, md: &ModelDef, version: Version) -> Vec<(i32, String)> {
        let model = Model::from_modeldef(md, version);
        // An id that now names another body (or none) must not carry the
        // old body's motion into its first velocity; the model is polled
        // periodically, so unchanged bodies keep theirs.
        for (id, old) in &self.model.rigid_bodies {
            if model
                .rigid_bodies
                .get(id)
                .is_none_or(|new| new.name != old.name)
            {
                self.velocity.reset(*id);
            }
        }
        self.model = model;
        self.model_version = Some(version);
        self.unknown.clear();
        self.rebuild_names();
        self.model.errors.clone()
    }

    /// The decoded model.
    #[must_use]
    pub fn model(&self) -> &Model {
        &self.model
    }

    /// Forgets timing and velocity history (stream restart).
    pub fn reset(&mut self) {
        self.stamper.reset();
        self.velocity.clear();
    }

    /// Takes the assets (`"rigid body 3"`) seen in frames but missing from
    /// the model since the last call; non-empty means the model should be
    /// refreshed.
    pub fn take_unknown(&mut self) -> BTreeSet<String> {
        std::mem::take(&mut self.unknown)
    }

    fn included(&self, motive: &str, token: &str) -> bool {
        let matches = |p: &String| glob_match(p, motive) || glob_match(p, token);
        (self.include.is_empty() || self.include.iter().any(matches))
            && !self.exclude.iter().any(matches)
    }

    fn rebuild_names(&mut self) {
        let mut names = Names::default();
        // Body frames must not shadow the world frame.
        let mut taken: BTreeSet<String> = self
            .world_frame
            .strip_prefix(self.frame_prefix.as_str())
            .map(str::to_owned)
            .into_iter()
            .collect();
        for (&id, rb) in &self.model.rigid_bodies {
            let out = self
                .rename
                .get(&rb.name)
                .cloned()
                .unwrap_or_else(|| rb.name.clone());
            let token = unique(&mut taken, &sanitize_token(&out), id);
            names.bodies.insert(
                id,
                BodyName {
                    motive: rb.name.clone(),
                    frame_id: format!("{}{token}", self.frame_prefix),
                    included: self.included(&rb.name, &token),
                    token,
                    known: true,
                },
            );
        }
        let mut taken = BTreeSet::new();
        for (&id, sk) in &self.model.skeletons {
            let token = unique(&mut taken, &sanitize_token(&sk.name), id);
            let mut bone_names = BTreeSet::new();
            let bones = sk
                .bones
                .iter()
                .map(|b| {
                    let name = unique(&mut bone_names, &sanitize_token(&b.name), b.id);
                    let bone = BoneName {
                        frame_id: format!("{}{token}_{name}", self.frame_prefix),
                        name,
                        // A parent missing from the model stays set: its
                        // children then have no pose in local mode rather
                        // than a relative pose passed off as a world one.
                        parent: (b.parent_id > 0).then_some(b.parent_id),
                    };
                    (b.id, bone)
                })
                .collect();
            names.skeletons.insert(id, SkeletonNames { token, bones });
        }
        for &id in self.model.force_plates.keys() {
            names.force_plates.insert(id, format!("force_plate_{id}"));
        }
        let mut taken = BTreeSet::new();
        for (&id, d) in &self.model.devices {
            names
                .devices
                .insert(id, unique(&mut taken, &sanitize_token(&d.name), id));
        }
        self.names = names;
    }

    /// The body's names, cached; a generic name for a body the model does
    /// not describe.
    fn body_name(&mut self, id: i32) -> &BodyName {
        // An unknown body is reported on every frame, not just the first,
        // so a model request that went unanswered is retried.
        if let Some(known) = self.names.bodies.get(&id).map(|n| n.known) {
            if !known {
                self.unknown.insert(format!("rigid body {id}"));
            }
            return &self.names.bodies[&id];
        }
        self.unknown.insert(format!("rigid body {id}"));
        let motive = format!("rigid_body_{id}");
        let token = motive.clone();
        let n = BodyName {
            included: self.included(&motive, &token),
            frame_id: format!("{}{token}", self.frame_prefix),
            motive,
            token,
            known: false,
        };
        self.names.bodies.entry(id).or_insert(n)
    }

    /// Converts a native position to the world frame.
    fn world_point(&self, p: [f32; 3]) -> [f64; 3] {
        let c = self.convention.position(p);
        let r = quat::rotate(self.offset.orientation, c);
        [
            r[0] + self.offset.position[0],
            r[1] + self.offset.position[1],
            r[2] + self.offset.position[2],
        ]
    }

    /// Converts a native pose to the world frame.
    fn world_pose(&self, p: [f32; 3], q: [f32; 4]) -> Option<Pose> {
        let local = self.local_pose(p, q)?;
        Some(self.offset.compose(&local))
    }

    /// Converts a native relative pose (no world offset).
    fn local_pose(&self, p: [f32; 3], q: [f32; 4]) -> Option<Pose> {
        let position = self.convention.position(p);
        if !position.iter().all(|v| v.is_finite()) {
            return None;
        }
        Some(Pose {
            position,
            orientation: self.convention.orientation(q)?,
        })
    }

    /// Processes one frame; `None` for a late (reordered) frame, which would
    /// move outputs back in time.
    pub fn process(
        &mut self,
        frame: &FrameOfData,
        received: SystemTime,
        received_mono: Instant,
    ) -> Option<Snapshot> {
        match self.stats.record_frame(frame.frame_number, received_mono) {
            Sequence::Late => return None,
            Sequence::Restart => self.reset(),
            Sequence::First | Sequence::Next | Sequence::Duplicate => {}
        }
        let stamp = self.stamper.stamp(&frame.timing, received);
        self.stats.server_latency = stamp.server_latency;
        self.stats.total_latency = stamp.total_latency;
        self.stats.clock_offset = stamp.clock_offset;
        let t_vel = self
            .stamper
            .mid_exposure_secs(&frame.timing)
            .unwrap_or_else(|| stamp.time.as_nanos() as f64 * 1e-9);

        let mut bodies = Vec::with_capacity(frame.rigid_bodies.len());
        let mut tracked = 0;
        for rb in &frame.rigid_bodies {
            let is_tracked = rb.tracking_valid();
            tracked += usize::from(is_tracked);
            let name = self.body_name(rb.id);
            if !name.included {
                continue;
            }
            let name = name.clone();
            // Motive keeps reporting a (usually the last) pose for untracked
            // bodies; it is only used on request.
            let pose = if is_tracked || self.publish_untracked {
                self.world_pose(rb.position, rb.orientation)
            } else {
                None
            };
            let velocity = if let Some(p) = pose.filter(|_| is_tracked) {
                self.velocity
                    .update(rb.id, t_vel, p.position, p.orientation)
            } else {
                self.velocity.reset(rb.id);
                None
            };
            let markers = match (pose, self.model.rigid_bodies.get(&rb.id)) {
                (Some(p), Some(desc)) if self.body_markers => desc
                    .markers
                    .iter()
                    .map(|m| {
                        let o = self.convention.position(m.offset);
                        let w = p.compose(&Pose {
                            position: o,
                            orientation: quat::IDENTITY,
                        });
                        w.position
                    })
                    .collect(),
                _ => Vec::new(),
            };
            bodies.push(BodySample {
                id: rb.id,
                motive_name: name.motive,
                name: name.token,
                frame_id: name.frame_id,
                tracked: is_tracked,
                pose,
                mean_error: (f64::from(rb.mean_marker_error) * self.convention.scale) as f32,
                velocity,
                markers,
            });
        }
        self.stats.rigid_bodies = frame.rigid_bodies.len();
        self.stats.tracked_rigid_bodies = tracked;

        // Non-finite positions would break the dense point cloud and every
        // marker consumer downstream.
        let markers: Vec<MarkerSample> = frame
            .labeled_markers
            .iter()
            .filter(|m| m.position.iter().all(|v| v.is_finite()))
            .map(|m| {
                let (model_id, member_id) = split_id(m.id);
                MarkerSample {
                    id: m.id,
                    model_id,
                    member_id,
                    position: self.world_point(m.position),
                    size: (f64::from(m.size) * self.convention.scale) as f32,
                    residual: (f64::from(m.residual) * self.convention.scale) as f32,
                    params: m.params,
                }
            })
            .collect();
        self.stats.labeled_markers = markers.len();

        let skeletons = self.skeletons(frame);
        let force_plates = self.analog(&frame.force_plates, true);
        let devices = self.analog(&frame.devices, false);

        Some(Snapshot {
            frame_number: frame.frame_number,
            stamp_secs: stamp.time.as_nanos() as f64 * 1e-9,
            server_latency: stamp.server_latency,
            total_latency: stamp.total_latency,
            stamp,
            bodies,
            markers,
            skeletons,
            force_plates,
            devices,
            recording: frame.params & 0x01 != 0,
        })
    }

    fn skeletons(&mut self, frame: &FrameOfData) -> Vec<SkeletonSample> {
        let decoded = match sections::skeletons(&frame.skeletons) {
            Ok(s) => s,
            Err(e) => {
                // Counted per frame; logged loudly at powers of two only.
                self.stats.decode_errors += 1;
                if self.stats.decode_errors.is_power_of_two() {
                    tracing::warn!("skeleton section undecodable, dropped: {e}");
                } else {
                    tracing::debug!("skeleton section: {e}");
                }
                return Vec::new();
            }
        };
        let mut out = Vec::with_capacity(decoded.len());
        for sk in decoded {
            let desc = self.model.skeletons.get(&sk.id);
            if desc.is_none() {
                self.unknown.insert(format!("skeleton {}", sk.id));
            }
            let fallback;
            let names = if let Some(n) = self.names.skeletons.get(&sk.id) {
                n
            } else {
                fallback = SkeletonNames {
                    token: format!("skeleton_{}", sk.id),
                    bones: BTreeMap::new(),
                };
                &fallback
            };
            let name = names.token.clone();
            let mut globals: BTreeMap<i32, Pose> = BTreeMap::new();
            let mut bones = Vec::with_capacity(sk.bones.len());
            for bone in &sk.bones {
                let tracked = bone.tracking_valid();
                // Like rigid bodies: Motive keeps sending a stale pose for
                // bones it lost, used only on request.
                if !tracked && !self.publish_untracked {
                    continue;
                }
                let (_, bone_id) = split_id(bone.id);
                let (bone_name, frame_id, parent) = if let Some(b) = names.bones.get(&bone_id) {
                    (b.name.clone(), b.frame_id.clone(), b.parent)
                } else if self.coordinates == BoneCoordinates::Local {
                    // Without the model, a local pose has no known parent.
                    continue;
                } else {
                    let n = format!("bone_{bone_id}");
                    (n.clone(), format!("{}{name}_{n}", self.frame_prefix), None)
                };
                let (local, global, parent_frame) = match self.coordinates {
                    BoneCoordinates::Global => {
                        let Some(g) = self.world_pose(bone.position, bone.orientation) else {
                            continue;
                        };
                        (g, g, None)
                    }
                    BoneCoordinates::Local => {
                        if let Some((pid, parent_global)) =
                            parent.and_then(|p| globals.get(&p).map(|g| (p, *g)))
                        {
                            let Some(l) = self.local_pose(bone.position, bone.orientation) else {
                                continue;
                            };
                            let parent_frame = names.bones.get(&pid).map(|b| b.frame_id.clone());
                            (l, parent_global.compose(&l), parent_frame)
                        } else if parent.is_some() {
                            // The parent has no pose this frame: the local
                            // pose must not be mistaken for a world pose.
                            continue;
                        } else {
                            let Some(g) = self.world_pose(bone.position, bone.orientation) else {
                                continue;
                            };
                            (g, g, None)
                        }
                    }
                };
                globals.insert(bone_id, global);
                bones.push(BoneSample {
                    id: bone_id,
                    name: bone_name,
                    frame_id,
                    parent,
                    parent_frame,
                    local,
                    global,
                    tracked,
                });
            }
            out.push(SkeletonSample {
                id: sk.id,
                name,
                bones,
            });
        }
        out
    }

    fn analog(&mut self, section: &natnet::RawSection, plates: bool) -> Vec<AnalogSample> {
        let decoded = match sections::analog(section) {
            Ok(a) => a,
            Err(e) => {
                // Counted per frame; logged loudly at powers of two only.
                self.stats.decode_errors += 1;
                if self.stats.decode_errors.is_power_of_two() {
                    tracing::warn!("analog section undecodable, dropped: {e}");
                } else {
                    tracing::debug!("analog section: {e}");
                }
                return Vec::new();
            }
        };
        decoded
            .into_iter()
            .map(|a| {
                let (name, channel_names) = if plates {
                    (
                        self.names
                            .force_plates
                            .get(&a.id)
                            .cloned()
                            .unwrap_or_else(|| format!("force_plate_{}", a.id)),
                        self.model
                            .force_plates
                            .get(&a.id)
                            .map(|d| d.channel_names.clone())
                            .unwrap_or_default(),
                    )
                } else {
                    (
                        self.names
                            .devices
                            .get(&a.id)
                            .cloned()
                            .unwrap_or_else(|| format!("device_{}", a.id)),
                        self.model
                            .devices
                            .get(&a.id)
                            .map(|d| d.channel_names.clone())
                            .unwrap_or_default(),
                    )
                };
                AnalogSample {
                    id: a.id,
                    name,
                    channel_names,
                    channels: a.channels,
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::synthetic::{Scene, SyntheticConfig};

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-5)
    }

    fn setup(cfg: &Config) -> (Processor, Scene) {
        let scene = Scene::new(SyntheticConfig::default()).unwrap();
        let mut p = Processor::new(cfg);
        p.set_server_info(&scene.server_info(false, 0, [0; 4]));
        let problems = p.set_model(&scene.model_def(), scene.version());
        assert!(problems.is_empty(), "{problems:?}");
        (p, scene)
    }

    #[test]
    fn bodies_markers_and_names() {
        let mut cfg = Config::default();
        cfg.rigid_bodies.mocap4r2 = true;
        cfg.rigid_bodies
            .rename
            .insert("body_2".into(), "rover 2".into());
        cfg.rigid_bodies.exclude.push("body_3".into());
        cfg.ros.frame_prefix = "mocap/".into();
        let (mut p, scene) = setup(&cfg);
        let s = p
            .process(&scene.frame(10), SystemTime::now(), Instant::now())
            .unwrap();
        assert_eq!(s.bodies.len(), 2);
        assert_eq!(s.bodies[1].name, "rover_2");
        assert_eq!(s.bodies[1].motive_name, "body_2");
        assert_eq!(s.bodies[1].frame_id, "mocap/rover_2");
        // Y-up to Z-up: native y (height) becomes z.
        let native = scene.frame(10).rigid_bodies[0].position;
        let pose = s.bodies[0].pose.unwrap();
        assert!(close(
            pose.position,
            [
                f64::from(native[0]),
                -f64::from(native[2]),
                f64::from(native[1])
            ]
        ));
        assert!(p.take_unknown().is_empty());
        assert_eq!(s.markers.len(), 13);
        assert_eq!((s.markers[0].model_id, s.markers[0].member_id), (1, 1));
        assert!(!s.markers[12].labeled());
        // Model marker offsets land on the streamed marker positions.
        assert!(
            !s.bodies[0].markers.is_empty(),
            "body 0 has no model markers"
        );
        for (m, streamed) in s.bodies[0].markers.iter().zip(&s.markers) {
            assert!(
                close(*m, streamed.position),
                "{m:?} vs {:?}",
                streamed.position
            );
        }
        assert_eq!(s.skeletons[0].bones.len(), 3);
        assert_eq!(s.skeletons[0].bones[1].frame_id, "mocap/actor_Spine");
        assert_eq!(s.force_plates[0].channel_names.len(), 6);
        assert_eq!(s.devices[0].name, "synthetic_daq");
        assert_eq!(p.stats.tracked_rigid_bodies, 3);
    }

    #[test]
    fn velocity_matches_circular_motion() {
        let (mut p, scene) = setup(&Config::default());
        let mut last = None;
        for n in 0..60 {
            last = p
                .process(&scene.frame(n), SystemTime::now(), Instant::now())
                .unwrap()
                .bodies[0]
                .velocity;
        }
        let v = last.unwrap();
        // radius 1 m, period 10 s: speed 2π/10 m/s, yaw rate 2π/10 rad/s.
        let speed = v.linear_world.iter().map(|c| c * c).sum::<f64>().sqrt();
        assert!(
            (speed - std::f64::consts::TAU / 10.0).abs() < 1e-2,
            "{speed}"
        );
        assert!(
            (v.angular_world[2].abs() - std::f64::consts::TAU / 10.0).abs() < 1e-2,
            "{v:?}"
        );
        // Heading along the tangent: body-frame velocity is along one axis only.
        assert!(
            v.linear_body[1].abs() < 1e-2 && v.linear_body[2].abs() < 1e-2,
            "{v:?}"
        );
    }

    #[test]
    fn body_markers_only_when_an_output_uses_them() {
        let (mut p, scene) = setup(&Config::default());
        let s = p
            .process(&scene.frame(10), SystemTime::now(), Instant::now())
            .unwrap();
        assert!(s.bodies[0].pose.is_some());
        assert!(s.bodies.iter().all(|b| b.markers.is_empty()));
        let mut cfg = Config::default();
        cfg.native.enabled = true;
        p.reconfigure(&cfg);
        let s = p
            .process(&scene.frame(11), SystemTime::now(), Instant::now())
            .unwrap();
        assert!(
            !s.bodies[0].markers.is_empty(),
            "body 0 has no model markers"
        );
    }

    #[test]
    fn unknown_bodies_and_untracked() {
        let scene = Scene::new(SyntheticConfig {
            dropout_period: 2,
            ..SyntheticConfig::default()
        })
        .unwrap();
        let mut p = Processor::new(&Config::default());
        let s = p
            .process(&scene.frame(1), SystemTime::now(), Instant::now())
            .unwrap();
        assert_eq!(s.bodies[0].name, "rigid_body_1");
        assert!(s.bodies[0].pose.is_none() && !s.bodies[0].tracked);
        assert_eq!(p.take_unknown().len(), 4, "three bodies and the skeleton");
        let s = p
            .process(&scene.frame(2), SystemTime::now(), Instant::now())
            .unwrap();
        assert!(s.bodies[0].pose.is_some());
        // Still unknown on later frames, so an unanswered request is retried.
        assert_eq!(p.take_unknown().len(), 4);
    }

    #[test]
    fn local_bones_compose_to_global() {
        let mut cfg = Config::default();
        cfg.skeletons.coordinates = BoneCoordinates::Local;
        let (mut p, scene) = setup(&cfg);
        let s = p
            .process(&scene.frame(3), SystemTime::now(), Instant::now())
            .unwrap();
        let bones = &s.skeletons[0].bones;
        assert_eq!(bones[0].parent_frame, None);
        assert_eq!(bones[1].parent_frame.as_deref(), Some("actor_Hip"));
        let g = bones[0].global.compose(&bones[1].local);
        assert!(close(g.position, bones[1].global.position));
        assert!(
            g.position[2] > bones[0].global.position[2],
            "spine above hip"
        );
    }

    #[test]
    fn local_bones_need_the_model() {
        // Without the model no bone's parent is known, so local poses must
        // not be published as world poses.
        let mut cfg = Config::default();
        cfg.skeletons.coordinates = BoneCoordinates::Local;
        let scene = Scene::new(SyntheticConfig::default()).unwrap();
        let mut p = Processor::new(&cfg);
        p.set_server_info(&scene.server_info(false, 0, [0; 4]));
        let s = p
            .process(&scene.frame(3), SystemTime::now(), Instant::now())
            .unwrap();
        assert!(s.skeletons[0].bones.is_empty(), "{:?}", s.skeletons);
    }

    #[test]
    fn names_stay_unique() {
        let mut taken = BTreeSet::new();
        let got: Vec<String> = [(1, "x"), (2, "x_3"), (3, "x"), (4, "x")]
            .into_iter()
            .map(|(id, t)| unique(&mut taken, t, id))
            .collect();
        assert_eq!(got, ["x", "x_3", "x_3_2", "x_4"]);
        // No body frame shadows the world frame.
        let mut cfg = Config::default();
        cfg.rigid_bodies
            .rename
            .insert("body_1".into(), "mocap".into());
        let (mut p, scene) = setup(&cfg);
        let s = p
            .process(&scene.frame(1), SystemTime::now(), Instant::now())
            .unwrap();
        assert_eq!(s.bodies[0].frame_id, "mocap_1");
    }

    #[test]
    fn world_offset_and_pose_algebra() {
        let mut cfg = Config::default();
        cfg.transform.offset_translation = [1.0, 2.0, 3.0];
        cfg.transform.offset_rpy = [0.0, 0.0, std::f64::consts::FRAC_PI_2];
        let p = Processor::new(&cfg);
        // Native (1, 0, 0) is ROS (1, 0, 0); yawed 90° it is (0, 1, 0).
        assert!(close(p.world_point([1.0, 0.0, 0.0]), [1.0, 3.0, 3.0]));
        let a = Pose {
            position: [1.0, 2.0, 3.0],
            orientation: quat::from_rpy(0.1, 0.2, 0.3),
        };
        let id = a.compose(&a.inverse());
        assert!(close(id.position, [0.0; 3]));
        assert!((id.orientation[3].abs() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn restart_resets_history() {
        let (mut p, scene) = setup(&Config::default());
        for n in 5000..5010 {
            p.process(&scene.frame(n), SystemTime::now(), Instant::now())
                .unwrap();
        }
        let s = p
            .process(&scene.frame(0), SystemTime::now(), Instant::now())
            .unwrap();
        assert_eq!(p.stats.resets, 1);
        assert!(s.bodies[0].velocity.is_none(), "history cleared");
    }
}
