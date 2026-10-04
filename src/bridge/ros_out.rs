//! ROS 2 outputs over rmw_zenoh.

use std::collections::{BTreeMap, VecDeque};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use tracing::{debug, warn};

use super::Shared;
use super::snapshot::{AnalogSample, BodySample, Pose, Snapshot};
use crate::config::{Config, QosConfig};
use crate::mocap::transform::quat;
use crate::ros::introspection::serve_type_descriptions;
use crate::ros::msgs::builtin_interfaces::{Duration as RosDuration, Time};
use crate::ros::msgs::diagnostic_msgs::{DiagnosticArray, DiagnosticStatus};
use crate::ros::msgs::geometry_msgs::{
    Point, Pose as RosPose, PoseArray, PoseStamped, PoseWithCovariance, Quaternion, Transform,
    TransformStamped, Twist, TwistStamped, TwistWithCovariance, Vector3, Wrench, WrenchStamped,
};
use crate::ros::msgs::mocap4r2_msgs;
use crate::ros::msgs::nav_msgs::{Odometry, Path};
use crate::ros::msgs::rcl_interfaces::Parameter;
use crate::ros::msgs::sensor_msgs::{PointCloud2, PointField};
use crate::ros::msgs::std_msgs::{self, ColorRGBA, Float32MultiArray, Header};
use crate::ros::msgs::std_srvs::{Trigger, TriggerResponse};
use crate::ros::msgs::tf2_msgs::TFMessage;
use crate::ros::msgs::type_description_interfaces::GetTypeDescription;
use crate::ros::msgs::visualization_msgs::{Marker, MarkerArray};
use crate::ros::parameters::{Param, ParameterServer, SetHandler, flatten_json, json_from_value};
use crate::ros::{Message, Node, Publisher, QosProfile, ServiceServer};
use crate::source::SourceCommand;

/// Path sampling and publishing period; a full path per frame would cost
/// megabytes per second per body.
const PATH_INTERVAL: Duration = Duration::from_millis(100);

/// Position variance floor for odometry covariance, m².
const MIN_POSITION_VARIANCE: f64 = 1e-8;
/// Orientation variance reported in odometry, rad².
const ORIENTATION_VARIANCE: f64 = 1e-6;
/// Pose variance reported in odometry for a body Motive lost (published
/// with `rigid_bodies.publish_untracked`): its last pose is stale.
const UNTRACKED_VARIANCE: f64 = 1e6;
/// Linear and angular velocity variances reported in odometry.
const TWIST_VARIANCE: [f64; 2] = [1e-4, 1e-3];

fn point(p: [f64; 3]) -> Point {
    Point {
        x: p[0],
        y: p[1],
        z: p[2],
    }
}

fn vector(v: [f64; 3]) -> Vector3 {
    Vector3 {
        x: v[0],
        y: v[1],
        z: v[2],
    }
}

fn quaternion(q: [f64; 4]) -> Quaternion {
    Quaternion {
        x: q[0],
        y: q[1],
        z: q[2],
        w: q[3],
    }
}

fn ros_pose(p: &Pose) -> RosPose {
    RosPose {
        position: point(p.position),
        orientation: quaternion(p.orientation),
    }
}

fn transform(p: &Pose) -> Transform {
    Transform {
        translation: vector(p.position),
        rotation: quaternion(p.orientation),
    }
}

fn header(stamp: &Time, frame_id: &str) -> Header {
    Header {
        stamp: stamp.clone(),
        frame_id: frame_id.to_owned(),
    }
}

/// The QoS of a configured stream.
#[must_use]
pub fn stream_qos(q: &QosConfig) -> QosProfile {
    QosProfile::keep_last(q.depth)
        .with_reliability(q.reliability)
        .with_durability(q.durability)
}

/// `/tf` QoS as tf2_ros publishes it.
fn tf_qos() -> QosProfile {
    QosProfile::keep_last(100)
}

/// `/tf_static` QoS as tf2_ros publishes it.
fn tf_static_qos() -> QosProfile {
    QosProfile::transient_local(1)
}

/// A publisher that serializes only when someone listens, or when its
/// transient-local cache must stay current for late joiners.
struct Out<M: Message> {
    publisher: Publisher<M>,
    latched: bool,
}

impl<M: Message> Out<M> {
    fn new(node: &Node, topic: &str, qos: QosProfile) -> crate::ros::context::Result<Self> {
        Ok(Self {
            publisher: node.create_publisher(topic, qos)?,
            latched: qos.durability == crate::ros::qos::Durability::TransientLocal,
        })
    }

    fn wanted(&self) -> bool {
        self.latched || self.publisher.has_subscribers()
    }

    fn publish(&self, m: &M) {
        if let Err(e) = self.publisher.publish(m) {
            debug!(
                "publish on {} failed: {e}",
                self.publisher.raw().topic_name()
            );
        }
    }

    /// Publishes `make()` when matched.
    fn publish_with(&self, make: impl FnOnce() -> M) {
        if self.wanted() {
            self.publish(&make());
        }
    }
}

#[derive(Default)]
struct BodyOut {
    pose: Option<Out<PoseStamped>>,
    odom: Option<Out<Odometry>>,
    twist: Option<Out<TwistStamped>>,
    path: Option<Out<Path>>,
    path_poses: VecDeque<(Time, Pose)>,
    last_path: Option<Instant>,
}

/// Configuration keys that can be set at run time through ROS parameters.
/// Every other key needs the configuration file and a restart.
pub const WRITABLE_PARAMETERS: &[&str] = &[
    "transform.up_axis",
    "transform.scale",
    "transform.offset_translation",
    "transform.offset_rpy",
    "timing.policy",
    "timing.window_secs",
    "timing.max_rate_hz",
    "rigid_bodies.include",
    "rigid_bodies.exclude",
    "rigid_bodies.publish_untracked",
    "rigid_bodies.velocity_alpha",
    "rigid_bodies.velocity_max_gap",
    "markers.include_unlabeled",
    "markers.visualization_rate_hz",
    "markers.axis_length",
    "skeletons.coordinates",
    "diagnostics.drop_warn_ratio",
    "diagnostics.latency_warn_secs",
];

/// The configuration as ROS parameters (`ros.world_frame`, ...), plus
/// `use_sim_time`. Only [`WRITABLE_PARAMETERS`] can be set. The `zenoh`
/// section is left out: inline zenoh configuration can carry credentials.
#[must_use]
pub fn config_parameters(cfg: &Config) -> BTreeMap<String, Param> {
    let mut value = serde_json::to_value(cfg).unwrap_or_default();
    if let Some(map) = value.as_object_mut() {
        map.remove("zenoh");
    }
    let mut params: BTreeMap<String, Param> = flatten_json("", &value)
        .into_iter()
        .map(|(name, value)| {
            let read_only = !WRITABLE_PARAMETERS.contains(&name.as_str());
            let description = if read_only {
                format!("Configuration key {name} (read-only; set it in the configuration file)")
            } else {
                format!("Configuration key {name} (can be set at run time)")
            };
            (
                name,
                Param {
                    value,
                    description,
                    read_only,
                },
            )
        })
        .collect();
    params.insert(
        "use_sim_time".into(),
        Param {
            value: crate::ros::parameters::value_from_json(&serde_json::Value::Bool(false))
                .unwrap_or_default(),
            description: "Stamps use the wall clock; simulated time is not supported".into(),
            read_only: true,
        },
    );
    params
}

/// A handler that applies parameter changes to a copy of `cfg`, validates
/// the result and sends the new configuration to `updates`; the bridge
/// adopts it between frames.
#[must_use]
pub fn config_setter(cfg: &Config, updates: Sender<Config>) -> SetHandler {
    let current = std::sync::Mutex::new(cfg.clone());
    Arc::new(move |changes: &[Parameter]| {
        let mut current = current
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let mut json = serde_json::to_value(&*current).map_err(|e| e.to_string())?;
        for p in changes {
            let pointer = format!("/{}", p.name.replace('.', "/"));
            let slot = json
                .pointer_mut(&pointer)
                .ok_or_else(|| format!("{} is not a configuration key", p.name))?;
            *slot = json_from_value(&p.value)
                .ok_or_else(|| format!("{}: unsupported value", p.name))?;
        }
        let new: Config =
            serde_json::from_value(json).map_err(|e| format!("invalid value: {e}"))?;
        new.validate().map_err(|e| e.to_string())?;
        updates
            .send(new.clone())
            .map_err(|_| "the bridge has stopped".to_owned())?;
        *current = new;
        Ok(())
    })
}

/// The ROS 2 side of the bridge.
pub struct RosOutputs {
    node: Node,
    cfg: Config,
    qos: QosProfile,
    tf: Option<Out<TFMessage>>,
    _tf_static: Option<Out<TFMessage>>,
    bodies: BTreeMap<String, BodyOut>,
    pose_array: Option<Out<PoseArray>>,
    mocap_bodies: Option<Out<mocap4r2_msgs::RigidBodies>>,
    mocap_markers: Option<Out<mocap4r2_msgs::Markers>>,
    cloud: Option<Out<PointCloud2>>,
    viz: Option<Out<MarkerArray>>,
    last_viz: Option<Instant>,
    viz_reset: bool,
    prune: bool,
    skeletons: BTreeMap<String, Out<PoseArray>>,
    wrenches: BTreeMap<String, Out<WrenchStamped>>,
    plate_channels: BTreeMap<String, Out<Float32MultiArray>>,
    device_channels: BTreeMap<String, Out<Float32MultiArray>>,
    diagnostics: Option<Out<DiagnosticArray>>,
    server_info: Out<std_msgs::String>,
    model: Out<std_msgs::String>,
    _services: Vec<ServiceServer<Trigger>>,
    _parameters: Option<ParameterServer>,
    _type_descriptions: Option<ServiceServer<GetTypeDescription>>,
}

impl std::fmt::Debug for RosOutputs {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RosOutputs")
            .field("node", &self.node.fully_qualified_name())
            .field("bodies", &self.bodies.keys().collect::<Vec<_>>())
            .finish_non_exhaustive()
    }
}

impl RosOutputs {
    /// Declares the fixed publishers and services.
    ///
    /// # Errors
    /// Zenoh or naming errors.
    pub fn new(
        node: Node,
        cfg: &Config,
        shared: &Arc<Shared>,
        commands: &Sender<SourceCommand>,
        on_set: Option<SetHandler>,
    ) -> crate::ros::context::Result<Self> {
        let qos = stream_qos(&cfg.ros.stream_qos);
        let any_tf = cfg.rigid_bodies.tf || cfg.skeletons.tf;
        let tf = any_tf
            .then(|| Out::new(&node, "/tf", tf_qos()))
            .transpose()?;
        let tf_static = if cfg.ros.static_transforms.is_empty() {
            None
        } else {
            let out = Out::<TFMessage>::new(&node, "/tf_static", tf_static_qos())?;
            let now = Time::now();
            out.publish(&TFMessage {
                transforms: cfg
                    .ros
                    .static_transforms
                    .iter()
                    .map(|t| {
                        let [r, p, y] = t.rpy;
                        TransformStamped {
                            header: header(&now, &t.parent),
                            child_frame_id: t.child.clone(),
                            transform: transform(&Pose {
                                position: t.translation,
                                orientation: quat::from_rpy(r, p, y),
                            }),
                        }
                    })
                    .collect(),
            });
            Some(out)
        };
        let rb = &cfg.rigid_bodies;
        let pose_array = rb
            .pose_array
            .then(|| Out::new(&node, "~/rigid_bodies/poses", qos))
            .transpose()?;
        let mocap_bodies = rb
            .mocap4r2
            .then(|| Out::new(&node, "rigid_bodies", qos))
            .transpose()?;
        let mocap_markers = cfg
            .markers
            .mocap4r2
            .then(|| Out::new(&node, "markers", qos))
            .transpose()?;
        let cloud = cfg
            .markers
            .point_cloud
            .then(|| Out::new(&node, "~/markers", qos))
            .transpose()?;
        let viz = cfg
            .markers
            .visualization
            .then(|| Out::new(&node, "~/visualization", QosProfile::keep_last(5)))
            .transpose()?;
        let diagnostics = cfg
            .diagnostics
            .enabled
            .then(|| Out::new(&node, "/diagnostics", QosProfile::keep_last(10)))
            .transpose()?;
        let server_info = Out::new(&node, "~/server_info", QosProfile::transient_local(1))?;
        let model = Out::new(&node, "~/model", QosProfile::transient_local(1))?;
        let services = Self::services(&node, shared, commands)?;
        let parameters = cfg
            .ros
            .parameters
            .then(|| ParameterServer::new(&node, config_parameters(cfg), on_set))
            .transpose()?;
        // The registry is shared, so types of publishers created later
        // (per rigid body, per skeleton) are described too.
        // Humble has no type_description_interfaces to call it with.
        let type_descriptions = (cfg.ros.type_description_service
            && cfg.ros.rmw_zenoh.type_hashes())
        .then(|| serve_type_descriptions(&node))
        .transpose()?;
        Ok(Self {
            node,
            cfg: cfg.clone(),
            qos,
            tf,
            _tf_static: tf_static,
            bodies: BTreeMap::new(),
            pose_array,
            mocap_bodies,
            mocap_markers,
            cloud,
            viz,
            last_viz: None,
            viz_reset: true,
            prune: false,
            skeletons: BTreeMap::new(),
            wrenches: BTreeMap::new(),
            plate_channels: BTreeMap::new(),
            device_channels: BTreeMap::new(),
            diagnostics,
            server_info,
            model,
            _services: services,
            _parameters: parameters,
            _type_descriptions: type_descriptions,
        })
    }

    fn services(
        node: &Node,
        shared: &Arc<Shared>,
        commands: &Sender<SourceCommand>,
    ) -> crate::ros::context::Result<Vec<ServiceServer<Trigger>>> {
        let mut services = Vec::new();
        let tx = std::sync::Mutex::new(commands.clone());
        services.push(
            node.create_service::<Trigger, _>("~/refresh_model", move |_, _| {
                let ok = tx
                    .lock()
                    .is_ok_and(|tx| tx.send(SourceCommand::RefreshModel).is_ok());
                TriggerResponse {
                    success: ok,
                    message: if ok {
                        "model definitions requested".into()
                    } else {
                        "the source has stopped".into()
                    },
                }
            })?,
        );
        let s = Arc::clone(shared);
        services.push(
            node.create_service::<Trigger, _>("~/get_server_info", move |_, _| {
                let info = s.snapshot_json("server_info");
                TriggerResponse {
                    success: info != "null",
                    message: info,
                }
            })?,
        );
        let s = Arc::clone(shared);
        services.push(
            node.create_service::<Trigger, _>("~/get_model", move |_, _| {
                let model = s.snapshot_json("model");
                TriggerResponse {
                    success: model != "null",
                    message: model,
                }
            })?,
        );
        let s = Arc::clone(shared);
        services.push(
            node.create_service::<Trigger, _>("~/get_stats", move |_, _| TriggerResponse {
                success: true,
                message: s.snapshot_json("stats"),
            })?,
        );
        let s = Arc::clone(shared);
        services.push(
            node.create_service::<Trigger, _>("~/reset_clock_sync", move |_, _| {
                s.reset_requested.store(true, Ordering::Relaxed);
                TriggerResponse {
                    success: true,
                    message: "clock synchronization and velocity history reset".into(),
                }
            })?,
        );
        Ok(services)
    }

    /// The node.
    #[must_use]
    pub fn node(&self) -> &Node {
        &self.node
    }

    /// Publishes the server description (transient local).
    pub fn publish_server_info(&self, json: &str) {
        self.server_info.publish(&std_msgs::String {
            data: json.to_owned(),
        });
    }

    /// Publishes the model description (transient local) and clears stale
    /// visualization.
    pub fn publish_model(&mut self, json: &str) {
        self.model.publish(&std_msgs::String {
            data: json.to_owned(),
        });
        self.viz_reset = true;
        self.prune = true;
    }

    /// Withdraws per-asset outputs of assets missing from `snap`, after a
    /// model change renamed or removed them.
    fn prune_outputs(&mut self, snap: &Snapshot) {
        let names = |it: &mut dyn Iterator<Item = &String>| -> std::collections::BTreeSet<String> {
            it.cloned().collect()
        };
        let bodies = names(&mut snap.bodies.iter().map(|b| &b.name));
        let skeletons = names(&mut snap.skeletons.iter().map(|s| &s.name));
        let plates = names(&mut snap.force_plates.iter().map(|a| &a.name));
        let devices = names(&mut snap.devices.iter().map(|a| &a.name));
        self.bodies.retain(|n, _| bodies.contains(n));
        self.skeletons.retain(|n, _| skeletons.contains(n));
        self.wrenches.retain(|n, _| plates.contains(n));
        self.plate_channels.retain(|n, _| plates.contains(n));
        self.device_channels.retain(|n, _| devices.contains(n));
    }

    /// Publishes diagnostics.
    pub fn publish_diagnostics(&self, status: Vec<DiagnosticStatus>) {
        if let Some(d) = &self.diagnostics {
            d.publish(&DiagnosticArray {
                header: header(&Time::now(), ""),
                status,
            });
        }
    }

    fn body_out(&mut self, name: &str) -> &mut BodyOut {
        if !self.bodies.contains_key(name) {
            let rb = &self.cfg.rigid_bodies;
            let make = |suffix: &str| format!("~/rigid_bodies/{name}/{suffix}");
            let mut out = BodyOut::default();
            let node = &self.node;
            let qos = self.qos;
            let log = |r: crate::ros::context::Result<()>| {
                if let Err(e) = r {
                    warn!("cannot create publishers for rigid body {name}: {e}");
                }
            };
            log((|| {
                if rb.pose {
                    out.pose = Some(Out::new(node, &make("pose"), qos)?);
                }
                if rb.odometry {
                    out.odom = Some(Out::new(node, &make("odom"), qos)?);
                }
                if rb.twist {
                    out.twist = Some(Out::new(node, &make("twist"), qos)?);
                }
                if rb.path {
                    out.path = Some(Out::new(node, &make("path"), QosProfile::keep_last(1))?);
                }
                Ok(())
            })());
            self.bodies.insert(name.to_owned(), out);
        }
        self.bodies.get_mut(name).unwrap_or_else(|| unreachable!())
    }

    /// Publishes everything derived from one frame.
    pub fn publish_frame(&mut self, snap: &Snapshot) {
        if std::mem::take(&mut self.prune) {
            self.prune_outputs(snap);
        }
        let stamp = snap.stamp.time.clone();
        let world = self.cfg.ros.world_frame.clone();
        self.publish_tf(snap, &stamp, &world);
        for body in snap.bodies.iter().filter(|b| b.pose.is_some()) {
            self.publish_body(body, &stamp, &world);
        }
        if let Some(out) = &self.pose_array {
            out.publish_with(|| PoseArray {
                header: header(&stamp, &world),
                poses: snap
                    .bodies
                    .iter()
                    .filter_map(|b| b.pose.as_ref().map(ros_pose))
                    .collect(),
            });
        }
        if let Some(out) = &self.mocap_bodies {
            out.publish_with(|| mocap4r2_msgs::RigidBodies {
                header: header(&stamp, &world),
                frame_number: snap.frame_number as u32,
                rigidbodies: snap
                    .bodies
                    .iter()
                    .filter_map(|b| {
                        let pose = b.pose.as_ref()?;
                        Some(mocap4r2_msgs::RigidBody {
                            rigid_body_name: b.name.clone(),
                            markers: b
                                .markers
                                .iter()
                                .enumerate()
                                .map(|(i, m)| mocap4r2_msgs::Marker {
                                    id_type: mocap4r2_msgs::Marker::USE_INDEX,
                                    marker_index: i as i32,
                                    marker_name: String::new(),
                                    translation: point(*m),
                                })
                                .collect(),
                            pose: ros_pose(pose),
                        })
                    })
                    .collect(),
            });
        }
        if let Some(out) = &self.mocap_markers {
            out.publish_with(|| mocap4r2_msgs::Markers {
                header: header(&stamp, &world),
                frame_number: snap.frame_number as u32,
                markers: snap
                    .markers
                    .iter()
                    .filter(|m| self.cfg.markers.include_unlabeled || m.labeled())
                    .map(|m| mocap4r2_msgs::Marker {
                        id_type: mocap4r2_msgs::Marker::USE_INDEX,
                        marker_index: m.id,
                        marker_name: String::new(),
                        translation: point(m.position),
                    })
                    .collect(),
            });
        }
        if let Some(out) = &self.cloud {
            out.publish_with(|| self.cloud_message(snap, &stamp, &world));
        }
        self.publish_visualization(snap, &stamp, &world);
        self.publish_skeletons(snap, &stamp, &world);
        self.publish_analog(snap, &stamp);
    }

    fn publish_tf(&self, snap: &Snapshot, stamp: &Time, world: &str) {
        let Some(tf) = self.tf.as_ref().filter(|tf| tf.wanted()) else {
            return;
        };
        let mut transforms = Vec::new();
        if self.cfg.rigid_bodies.tf {
            transforms.extend(snap.bodies.iter().filter_map(|b| {
                Some(TransformStamped {
                    header: header(stamp, world),
                    child_frame_id: b.frame_id.clone(),
                    transform: transform(b.pose.as_ref()?),
                })
            }));
        }
        if self.cfg.skeletons.tf {
            for sk in &snap.skeletons {
                transforms.extend(sk.bones.iter().map(|b| TransformStamped {
                    header: header(stamp, b.parent_frame.as_deref().unwrap_or(world)),
                    child_frame_id: b.frame_id.clone(),
                    transform: transform(&b.local),
                }));
            }
        }
        if !transforms.is_empty() {
            tf.publish(&TFMessage { transforms });
        }
    }

    fn publish_body(&mut self, body: &BodySample, stamp: &Time, world: &str) {
        let path_length = self.cfg.rigid_bodies.path_length;
        let out = self.body_out(&body.name);
        let Some(pose) = body.pose.as_ref() else {
            return;
        };
        let stamped = || PoseStamped {
            header: header(stamp, world),
            pose: ros_pose(pose),
        };
        if let Some(o) = &out.pose {
            o.publish_with(stamped);
        }
        if let Some(o) = &out.odom {
            o.publish_with(|| {
                let sp = f64::from(body.mean_error)
                    .powi(2)
                    .max(MIN_POSITION_VARIANCE);
                let v = body.velocity.unwrap_or_default();
                Odometry {
                    header: header(stamp, world),
                    child_frame_id: body.frame_id.clone(),
                    pose: PoseWithCovariance {
                        pose: ros_pose(pose),
                        covariance: if body.tracked {
                            diagonal([
                                sp,
                                sp,
                                sp,
                                ORIENTATION_VARIANCE,
                                ORIENTATION_VARIANCE,
                                ORIENTATION_VARIANCE,
                            ])
                        } else {
                            diagonal([UNTRACKED_VARIANCE; 6])
                        },
                    },
                    twist: TwistWithCovariance {
                        twist: Twist {
                            linear: vector(v.linear_body),
                            angular: vector(v.angular_body),
                        },
                        covariance: if body.velocity.is_some() {
                            let [l, a] = TWIST_VARIANCE;
                            diagonal([l, l, l, a, a, a])
                        } else {
                            // Unknown: ROS convention marks it with -1.
                            let mut c = [0.0; 36];
                            c[0] = -1.0;
                            c
                        },
                    },
                }
            });
        }
        if let (Some(o), Some(v)) = (&out.twist, body.velocity) {
            o.publish_with(|| TwistStamped {
                header: header(stamp, &body.frame_id),
                twist: Twist {
                    linear: vector(v.linear_body),
                    angular: vector(v.angular_body),
                },
            });
        }
        if let Some(o) = &out.path {
            let now = Instant::now();
            if out
                .last_path
                .is_none_or(|t| now.duration_since(t) >= PATH_INTERVAL)
            {
                out.last_path = Some(now);
                out.path_poses.push_back((stamp.clone(), *pose));
                while out.path_poses.len() > path_length {
                    out.path_poses.pop_front();
                }
                o.publish_with(|| Path {
                    header: header(stamp, world),
                    poses: out
                        .path_poses
                        .iter()
                        .map(|(t, p)| PoseStamped {
                            header: header(t, world),
                            pose: ros_pose(p),
                        })
                        .collect(),
                });
            }
        }
    }

    /// Clears accumulated paths (stream restart).
    pub fn reset(&mut self) {
        for b in self.bodies.values_mut() {
            b.path_poses.clear();
            b.last_path = None;
        }
        self.viz_reset = true;
    }

    /// Adopts a configuration changed through parameters: outputs of assets
    /// no longer published are withdrawn, and paths and visualization
    /// restart when the frame convention changed.
    pub fn reconfigure(&mut self, cfg: &Config) {
        let convention_changed = cfg.transform != self.cfg.transform;
        self.cfg = cfg.clone();
        self.prune = true;
        if convention_changed {
            self.reset();
        }
    }

    fn cloud_message(&self, snap: &Snapshot, stamp: &Time, world: &str) -> PointCloud2 {
        const STEP: u32 = 28;
        let markers: Vec<_> = snap
            .markers
            .iter()
            .filter(|m| self.cfg.markers.include_unlabeled || m.labeled())
            .collect();
        let mut data = Vec::with_capacity(markers.len() * STEP as usize);
        for m in &markers {
            for c in m.position {
                data.extend_from_slice(&(c as f32).to_le_bytes());
            }
            data.extend_from_slice(&m.id.to_le_bytes());
            data.extend_from_slice(&m.size.to_le_bytes());
            data.extend_from_slice(&m.residual.to_le_bytes());
            data.extend_from_slice(&u32::from(m.params).to_le_bytes());
        }
        PointCloud2 {
            header: header(stamp, world),
            height: 1,
            width: markers.len() as u32,
            fields: vec![
                PointField::new("x", 0, PointField::FLOAT32),
                PointField::new("y", 4, PointField::FLOAT32),
                PointField::new("z", 8, PointField::FLOAT32),
                PointField::new("id", 12, PointField::INT32),
                PointField::new("size", 16, PointField::FLOAT32),
                PointField::new("residual", 20, PointField::FLOAT32),
                PointField::new("params", 24, PointField::UINT32),
            ],
            is_bigendian: false,
            point_step: STEP,
            row_step: STEP * markers.len() as u32,
            data,
            is_dense: true,
        }
    }

    fn publish_visualization(&mut self, snap: &Snapshot, stamp: &Time, world: &str) {
        let Some(out) = &self.viz else { return };
        let rate = self.cfg.markers.visualization_rate_hz;
        if rate > 0.0
            && self
                .last_viz
                .is_some_and(|t| t.elapsed() < Duration::from_secs_f64(1.0 / rate))
        {
            return;
        }
        if !out.wanted() {
            return;
        }
        self.last_viz = Some(Instant::now());
        let mut markers = Vec::new();
        if std::mem::take(&mut self.viz_reset) {
            markers.push(Marker {
                header: header(stamp, world),
                action: Marker::DELETEALL,
                ..Marker::default()
            });
        }
        // Markers outlive two refresh periods, so slow updates do not flicker.
        let refresh_hz = [
            self.cfg.markers.visualization_rate_hz,
            self.cfg.timing.max_rate_hz,
        ]
        .into_iter()
        .filter(|r| *r > 0.0)
        .fold(f64::INFINITY, f64::min);
        let lifetime_secs = (2.0 / refresh_hz).clamp(0.5, 60.0);
        let lifetime = RosDuration {
            sec: lifetime_secs.trunc() as i32,
            nanosec: (lifetime_secs.fract() * 1e9) as u32,
        };
        let base = |ns: &str, id: i32, kind: i32| Marker {
            header: header(stamp, world),
            ns: ns.to_owned(),
            id,
            r#type: kind,
            action: Marker::ADD,
            pose: RosPose::IDENTITY,
            lifetime: lifetime.clone(),
            ..Marker::default()
        };
        // All markers: labeled cyan, unlabeled grey, occluded red.
        let mut cloud = base("markers", 0, Marker::SPHERE_LIST);
        cloud.scale = vector([0.015; 3]);
        cloud.color = ColorRGBA::new(1.0, 1.0, 1.0, 1.0);
        for m in &snap.markers {
            cloud.points.push(point(m.position));
            cloud
                .colors
                .push(if m.params & super::snapshot::MARKER_OCCLUDED != 0 {
                    ColorRGBA::new(1.0, 0.2, 0.2, 1.0)
                } else if m.labeled() {
                    ColorRGBA::new(0.1, 0.9, 0.9, 1.0)
                } else {
                    ColorRGBA::new(0.6, 0.6, 0.6, 1.0)
                });
        }
        if !cloud.points.is_empty() {
            markers.push(cloud);
        }
        let len = self.cfg.markers.axis_length;
        for b in &snap.bodies {
            let Some(p) = b.pose else { continue };
            let mut axes = base("axes", b.id, Marker::LINE_LIST);
            axes.scale = vector([len * 0.08, 0.0, 0.0]);
            for (axis, color) in [
                ([len, 0.0, 0.0], ColorRGBA::new(1.0, 0.0, 0.0, 1.0)),
                ([0.0, len, 0.0], ColorRGBA::new(0.0, 1.0, 0.0, 1.0)),
                ([0.0, 0.0, len], ColorRGBA::new(0.0, 0.0, 1.0, 1.0)),
            ] {
                let tip = p.compose(&Pose {
                    position: axis,
                    orientation: quat::IDENTITY,
                });
                axes.points.push(point(p.position));
                axes.points.push(point(tip.position));
                axes.colors.push(color.clone());
                axes.colors.push(color);
            }
            markers.push(axes);
            let mut label = base("labels", b.id, Marker::TEXT_VIEW_FACING);
            label.pose.position = point([p.position[0], p.position[1], p.position[2] + len * 1.5]);
            label.scale.z = len * 0.6;
            label.color = ColorRGBA::new(1.0, 1.0, 1.0, 0.9);
            label.text.clone_from(&b.name);
            markers.push(label);
        }
        for sk in &snap.skeletons {
            let mut bones = base("skeletons", sk.id, Marker::LINE_LIST);
            bones.scale = vector([0.02, 0.0, 0.0]);
            bones.color = ColorRGBA::new(1.0, 0.8, 0.2, 1.0);
            // A segment from every bone to its parent, from the model.
            let by_id: BTreeMap<i32, &Pose> = sk.bones.iter().map(|b| (b.id, &b.global)).collect();
            let parents = sk.bones.iter().filter_map(|b| {
                let parent = by_id.get(&b.parent?)?;
                Some((parent.position, b.global.position))
            });
            for (a, b) in parents {
                bones.points.push(point(a));
                bones.points.push(point(b));
            }
            if !bones.points.is_empty() {
                markers.push(bones);
            }
        }
        out.publish(&MarkerArray { markers });
    }

    fn publish_skeletons(&mut self, snap: &Snapshot, stamp: &Time, world: &str) {
        if !self.cfg.skeletons.pose_array {
            return;
        }
        for sk in &snap.skeletons {
            if !self.skeletons.contains_key(&sk.name) {
                match Out::new(
                    &self.node,
                    &format!("~/skeletons/{}/poses", sk.name),
                    self.qos,
                ) {
                    Ok(o) => {
                        self.skeletons.insert(sk.name.clone(), o);
                    }
                    Err(e) => {
                        warn!("skeleton {}: {e}", sk.name);
                        continue;
                    }
                }
            }
            if let Some(o) = self.skeletons.get(&sk.name) {
                o.publish_with(|| PoseArray {
                    header: header(stamp, world),
                    poses: sk.bones.iter().map(|b| ros_pose(&b.global)).collect(),
                });
            }
        }
    }

    fn publish_analog(&mut self, snap: &Snapshot, stamp: &Time) {
        let a = &self.cfg.analog;
        for plate in &snap.force_plates {
            if a.force_plate_wrench
                && let Some(o) = lazy(
                    &mut self.wrenches,
                    &self.node,
                    &plate.name,
                    || format!("~/force_plates/{}/wrench", plate.name),
                    self.qos,
                )
            {
                o.publish_with(|| WrenchStamped {
                    header: header(
                        stamp,
                        &format!("{}{}", self.cfg.ros.frame_prefix, plate.name),
                    ),
                    wrench: wrench(plate),
                });
            }
            if a.force_plate_channels
                && let Some(o) = lazy(
                    &mut self.plate_channels,
                    &self.node,
                    &plate.name,
                    || format!("~/force_plates/{}/channels", plate.name),
                    self.qos,
                )
            {
                o.publish_with(|| channels(plate));
            }
        }
        if a.device_channels {
            for device in &snap.devices {
                if let Some(o) = lazy(
                    &mut self.device_channels,
                    &self.node,
                    &device.name,
                    || format!("~/devices/{}/channels", device.name),
                    self.qos,
                ) {
                    o.publish_with(|| channels(device));
                }
            }
        }
    }
}

fn lazy<'a, M: Message>(
    map: &'a mut BTreeMap<String, Out<M>>,
    node: &Node,
    key: &str,
    topic: impl FnOnce() -> String,
    qos: QosProfile,
) -> Option<&'a Out<M>> {
    if !map.contains_key(key) {
        let topic = topic();
        match Out::new(node, &topic, qos) {
            Ok(o) => {
                map.insert(key.to_owned(), o);
            }
            Err(e) => {
                warn!("{topic}: {e}");
                return None;
            }
        }
    }
    map.get(key)
}

fn diagonal(d: [f64; 6]) -> [f64; 36] {
    let mut c = [0.0; 36];
    for (i, v) in d.into_iter().enumerate() {
        c[i * 7] = v;
    }
    c
}

/// The wrench of a force plate: channels named `Fx … Mz` (any case), else
/// the first six channels in that order; the last subframe sample.
#[must_use]
pub fn wrench(plate: &AnalogSample) -> Wrench {
    let last = |i: usize| {
        plate
            .channels
            .get(i)
            .and_then(|c| c.last())
            .map_or(0.0, |v| f64::from(*v))
    };
    let find = |name: &str, fallback: usize| {
        plate
            .channel_names
            .iter()
            .position(|n| n.eq_ignore_ascii_case(name))
            .unwrap_or(fallback)
    };
    let c = |name: &str, fallback: usize| last(find(name, fallback));
    Wrench {
        force: Vector3 {
            x: c("Fx", 0),
            y: c("Fy", 1),
            z: c("Fz", 2),
        },
        torque: Vector3 {
            x: c("Mx", 3),
            y: c("My", 4),
            z: c("Mz", 5),
        },
    }
}

/// Channels × subframes as a row-major matrix.
#[must_use]
pub fn channels(a: &AnalogSample) -> Float32MultiArray {
    let cols = a.channels.iter().map(Vec::len).max().unwrap_or(0);
    let mut data = Vec::with_capacity(a.channels.len() * cols);
    for ch in &a.channels {
        data.extend_from_slice(ch);
        // Ragged channels are padded with NaN to keep the matrix shape.
        data.extend(std::iter::repeat_n(f32::NAN, cols - ch.len()));
    }
    Float32MultiArray::matrix("channels", "subframes", a.channels.len(), cols, data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_parameters_skip_zenoh() {
        let mut cfg = Config::default();
        cfg.zenoh.config_json5 = Some("{secret: 1}".into());
        let params = config_parameters(&cfg);
        assert!(params.keys().all(|k| !k.starts_with("zenoh")));
        assert_eq!(params["ros.world_frame"].value.string_value, "mocap");
        assert_eq!(params["rigid_bodies.path_length"].value.integer_value, 500);
        assert!(params.contains_key("use_sim_time"));
        assert!(params["ros.world_frame"].read_only);
        // Every writable key exists, so none is misspelled.
        for name in WRITABLE_PARAMETERS {
            assert!(!params[*name].read_only, "{name}");
        }
    }

    #[test]
    fn setter_validates_and_forwards() {
        let param = |name: &str, value: serde_json::Value| Parameter {
            name: name.into(),
            value: crate::ros::parameters::value_from_json(&value).unwrap(),
        };
        let (tx, rx) = std::sync::mpsc::channel();
        let set = config_setter(&Config::default(), tx);
        set(&[
            param("timing.max_rate_hz", serde_json::json!(30.5)),
            param("transform.up_axis", serde_json::json!("z")),
        ])
        .unwrap();
        let cfg = rx.try_recv().unwrap();
        assert!((cfg.timing.max_rate_hz - 30.5).abs() < f64::EPSILON);
        assert_eq!(cfg.transform.up_axis, crate::mocap::transform::UpAxis::Z);
        // Later changes build on the earlier ones.
        set(&[param("markers.axis_length", serde_json::json!(0.2))]).unwrap();
        let cfg = rx.try_recv().unwrap();
        assert!((cfg.timing.max_rate_hz - 30.5).abs() < f64::EPSILON);
        assert!(
            set(&[param("transform.up_axis", serde_json::json!("x"))])
                .unwrap_err()
                .contains("invalid value")
        );
        assert!(set(&[param("timing.max_rate_hz", serde_json::json!(-1.0))]).is_err());
        assert!(rx.try_recv().is_err());
    }

    #[test]
    fn wrench_by_name_and_position() {
        let plate = AnalogSample {
            id: 1,
            name: "fp".into(),
            channel_names: ["Mz", "Fx", "Fy", "Fz", "Mx", "My"]
                .map(String::from)
                .to_vec(),
            channels: (0..6).map(|i| vec![0.0, i as f32]).collect(),
        };
        let w = wrench(&plate);
        assert_eq!((w.force.x, w.force.z, w.torque.z), (1.0, 3.0, 0.0));
        let unnamed = AnalogSample {
            channel_names: vec![],
            ..plate
        };
        assert_eq!(wrench(&unnamed).force.x, 0.0);
        assert_eq!(wrench(&unnamed).torque.z, 5.0);
    }

    #[test]
    fn ragged_channels() {
        let a = AnalogSample {
            id: 1,
            name: "d".into(),
            channel_names: vec![],
            channels: vec![vec![1.0, 2.0], vec![3.0]],
        };
        let m = channels(&a);
        assert_eq!(m.layout.dim[0].size, 2);
        assert_eq!(m.layout.dim[1].size, 2);
        assert!(m.data[3].is_nan());
    }
}
