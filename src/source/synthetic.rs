//! A synthetic motion-capture scene: rigid bodies circling with markers, an
//! optional skeleton, force plate and analog device. Used by the synthetic
//! source, the simulated NatNet server and tests.

use std::f64::consts::TAU;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant, SystemTime};

use natnet::{
    ConnectionInfo, DataSet, Decoder, FrameOfData, FrameTiming, LabeledMarker, MarkerSet,
    MarkerSetDef, Message, ModelDef, RawSection, RigidBody, ServerInfo, Version,
};
use serde::{Deserialize, Serialize};

use super::capture::MAX_RECORD;
use super::{ConnectionState, SourceCommand, SourceContext, SourceEvent};
use crate::mocap::model::{
    self, DeviceDesc, ForcePlateDesc, MarkerDesc, RigidBodyDesc, SkeletonDesc,
};
use crate::mocap::sections::{self, AnalogFrame, Skeleton};
use crate::mocap::transform::quat;

/// Synthetic scene settings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SyntheticConfig {
    /// Frame rate, Hz.
    pub rate_hz: f64,
    /// Number of rigid bodies.
    pub rigid_bodies: usize,
    /// Markers per rigid body.
    pub markers_per_body: usize,
    /// Include a three-bone skeleton.
    pub skeleton: bool,
    /// Include a force plate.
    pub force_plate: bool,
    /// Include a two-channel analog device.
    pub device: bool,
    /// Report every body untracked for one frame out of this many (0: never).
    pub dropout_period: u32,
    /// Protocol version to emit.
    pub protocol_version: String,
    /// Camera clock frequency, Hz.
    pub clock_frequency: u64,
    /// Circle radius, metres.
    pub radius: f64,
    /// Seconds per revolution.
    pub period_secs: f64,
}

impl Default for SyntheticConfig {
    fn default() -> Self {
        Self {
            rate_hz: 120.0,
            rigid_bodies: 3,
            markers_per_body: 4,
            skeleton: true,
            force_plate: true,
            device: true,
            dropout_period: 0,
            protocol_version: "4.1".to_owned(),
            clock_frequency: 10_000_000,
            radius: 1.0,
            period_secs: 10.0,
        }
    }
}

/// Parses `major.minor[.build[.revision]]`.
///
/// # Errors
/// A message for a malformed or unsupported version.
pub fn parse_version(s: &str) -> Result<Version, String> {
    let parts: Vec<u8> = s
        .split('.')
        .map(|p| {
            p.trim()
                .parse::<u8>()
                .map_err(|_| format!("invalid version {s:?}"))
        })
        .collect::<Result<_, _>>()?;
    if parts.len() < 2 || parts.len() > 4 {
        return Err(format!(
            "invalid version {s:?}: expected major.minor[.build[.revision]]"
        ));
    }
    let v = Version::new(
        parts[0],
        parts[1],
        parts.get(2).copied().unwrap_or(0),
        parts.get(3).copied().unwrap_or(0),
    );
    v.validate()
        .map_err(|e| format!("unsupported NatNet version {v}: {e}"))?;
    Ok(v)
}

/// The synthetic scene.
#[derive(Debug, Clone)]
pub struct Scene {
    cfg: SyntheticConfig,
    version: Version,
}

const SKELETON_ID: i32 = 1;
const FORCE_PLATE_ID: i32 = 1;
const DEVICE_ID: i32 = 2;

impl Scene {
    /// A scene from settings.
    ///
    /// # Errors
    /// A message for an invalid protocol version or scene geometry.
    pub fn new(cfg: SyntheticConfig) -> Result<Self, String> {
        let version = parse_version(&cfg.protocol_version)?;
        if !(cfg.radius.is_finite() && cfg.period_secs.is_finite() && cfg.period_secs > 0.0) {
            return Err(
                "synthetic radius and period_secs must be finite, period_secs positive".into(),
            );
        }
        if !(cfg.rate_hz > 0.0 && cfg.rate_hz <= 10_000.0) {
            return Err("synthetic rate_hz must be in (0, 10000]".into());
        }
        // Frames and the model must each fit one datagram; the bounds keep
        // the size check below cheap.
        if cfg.rigid_bodies > 1_000 || cfg.markers_per_body > 1_000 {
            return Err("synthetic rigid_bodies and markers_per_body must be at most 1000".into());
        }
        let scene = Self { cfg, version };
        // Checked encoders refuse payloads over 65,535 bytes; report that as
        // a size so the datagram-limit check below names it.
        let size = |encoded: natnet::Result<Vec<u8>>| match encoded {
            Ok(bytes) => Ok(bytes.len()),
            Err(natnet::Error::LengthOverflow {
                field: "message payload size",
                length,
                ..
            }) => Ok(length.saturating_add(4)),
            Err(e) => Err(e.to_string()),
        };
        let frame = size(scene.frame_datagram(0))?;
        let model = size(scene.model_def().to_datagram(version))?;
        let largest = frame.max(model);
        if largest > MAX_RECORD {
            return Err(format!(
                "a synthetic frame or model takes {largest} bytes, over the {MAX_RECORD}-byte datagram limit; use fewer rigid bodies or markers"
            ));
        }
        Ok(scene)
    }

    /// The protocol version emitted.
    #[must_use]
    pub fn version(&self) -> Version {
        self.version
    }

    /// The settings.
    #[must_use]
    pub fn config(&self) -> &SyntheticConfig {
        &self.cfg
    }

    /// The SERVERINFO a server of this scene reports.
    #[must_use]
    pub fn server_info(&self, multicast: bool, data_port: u16, group: [u8; 4]) -> ServerInfo {
        ServerInfo {
            application_name: "natnet_zenoh synthetic".to_owned(),
            server_version: Version::new(3, 1, 0, 0),
            protocol_version: self.version,
            high_res_clock_frequency: self.cfg.clock_frequency,
            connection: ConnectionInfo {
                data_port,
                multicast,
                multicast_group: group,
            },
        }
    }

    fn body_name(i: usize) -> String {
        format!("body_{}", i + 1)
    }

    fn marker_offset(&self, j: usize) -> [f64; 3] {
        let n = self.cfg.markers_per_body.max(1) as f64;
        let a = TAU * j as f64 / n;
        // Native Motive frame: Y up; markers on a horizontal ring, alternating height.
        [
            0.08 * a.cos(),
            if j.is_multiple_of(2) { 0.02 } else { 0.0 },
            0.08 * a.sin(),
        ]
    }

    /// The model definition.
    #[must_use]
    pub fn model_def(&self) -> ModelDef {
        let mut datasets = Vec::new();
        for i in 0..self.cfg.rigid_bodies {
            let name = Self::body_name(i);
            datasets.push(DataSet::MarkerSet(MarkerSetDef {
                name: name.clone(),
                marker_names: (0..self.cfg.markers_per_body)
                    .map(|j| format!("{name}_marker_{}", j + 1))
                    .collect(),
            }));
        }
        for i in 0..self.cfg.rigid_bodies {
            let name = Self::body_name(i);
            let desc = RigidBodyDesc {
                name: name.clone(),
                id: i as i32 + 1,
                parent_id: -1,
                offset: [0.0; 3],
                markers: (0..self.cfg.markers_per_body)
                    .map(|j| MarkerDesc {
                        name: format!("{name}_marker_{}", j + 1),
                        label: 0,
                        offset: self.marker_offset(j).map(|v| v as f32),
                    })
                    .collect(),
            };
            datasets.push(model::encode::rigid_body(&desc, self.version));
        }
        if self.cfg.skeleton {
            let bone = |name: &str, id: i32, parent_id: i32, offset: [f32; 3]| RigidBodyDesc {
                name: name.to_owned(),
                id,
                parent_id,
                offset,
                markers: Vec::new(),
            };
            let sk = SkeletonDesc {
                name: "actor".to_owned(),
                id: SKELETON_ID,
                bones: vec![
                    bone("Hip", 1, 0, [0.0, 0.9, 0.0]),
                    bone("Spine", 2, 1, [0.0, 0.25, 0.0]),
                    bone("Head", 3, 2, [0.0, 0.4, 0.0]),
                ],
            };
            datasets.push(model::encode::skeleton(&sk, self.version));
        }
        if self.cfg.force_plate {
            datasets.push(model::encode::force_plate(&ForcePlateDesc {
                id: FORCE_PLATE_ID,
                serial: "SYN-FP-1".to_owned(),
                width: 0.6,
                length: 0.4,
                origin: [0.0; 3],
                calibration: vec![0.0; 144],
                corners: [
                    [-0.3, 0.0, -0.2],
                    [0.3, 0.0, -0.2],
                    [0.3, 0.0, 0.2],
                    [-0.3, 0.0, 0.2],
                ],
                plate_type: 1,
                channel_data_type: 0,
                channel_names: ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
                    .map(String::from)
                    .to_vec(),
            }));
        }
        if self.cfg.device {
            datasets.push(model::encode::device(&DeviceDesc {
                id: DEVICE_ID,
                name: "synthetic_daq".to_owned(),
                serial: "SYN-DAQ-1".to_owned(),
                device_type: 1,
                channel_data_type: 0,
                channel_names: vec!["ai0".to_owned(), "ai1".to_owned()],
            }));
        }
        ModelDef { datasets }
    }

    /// The pose of body `i` at time `t`, native frame.
    #[allow(clippy::many_single_char_names)]
    fn body_pose(&self, i: usize, t: f64) -> ([f64; 3], [f64; 4]) {
        let n = self.cfg.rigid_bodies.max(1) as f64;
        let phase = TAU * i as f64 / n;
        let w = TAU / self.cfg.period_secs.max(1e-3);
        let a = w * t + phase;
        let r = self.cfg.radius;
        // Circle in the horizontal XZ plane, heading along the tangent.
        let position = [r * a.cos(), 0.2 + 0.1 * i as f64, r * a.sin()];
        // Yaw about the up (Y) axis.
        let orientation = quat::from_axis_angle([0.0, 1.0, 0.0], -a - std::f64::consts::FRAC_PI_2);
        (position, orientation)
    }

    /// The frame at index `frame_number` (time `frame_number / rate`).
    #[must_use]
    #[allow(clippy::many_single_char_names)]
    pub fn frame(&self, frame_number: i32) -> FrameOfData {
        let rate = self.cfg.rate_hz.max(1e-3);
        let t = f64::from(frame_number) / rate;
        let dropout = self.cfg.dropout_period > 0
            && (frame_number as u32) % self.cfg.dropout_period == self.cfg.dropout_period - 1;

        let mut rigid_bodies = Vec::new();
        let mut marker_sets = Vec::new();
        let mut labeled_markers = Vec::new();
        for i in 0..self.cfg.rigid_bodies {
            let id = i as i32 + 1;
            let (p, q) = self.body_pose(i, t);
            rigid_bodies.push(RigidBody {
                id,
                position: p.map(|v| v as f32),
                orientation: q.map(|v| v as f32),
                mean_marker_error: 0.0002,
                params: u16::from(!dropout),
            });
            let mut set = Vec::new();
            for j in 0..self.cfg.markers_per_body {
                let o = quat::rotate(q, self.marker_offset(j));
                let m = [p[0] + o[0], p[1] + o[1], p[2] + o[2]].map(|v| v as f32);
                set.push(m);
                labeled_markers.push(LabeledMarker {
                    id: (id << 16) | (j as i32 + 1),
                    position: m,
                    size: 0.014,
                    params: 0,
                    residual: 0.0001,
                });
            }
            marker_sets.push(MarkerSet {
                name: Self::body_name(i),
                markers: set,
            });
        }
        // An unlabeled stray marker.
        labeled_markers.push(LabeledMarker {
            id: 1000,
            position: [0.0, 0.01, 0.0],
            size: 0.01,
            params: 0x10,
            residual: 0.0,
        });

        let skeletons = if self.cfg.skeleton {
            let sway = 0.2 * (TAU * t / 3.0).sin();
            let bone = |b: i32, offset: [f32; 3], angle: f64| RigidBody {
                id: (SKELETON_ID << 16) | b,
                position: offset,
                orientation: quat::from_axis_angle([0.0, 0.0, 1.0], angle).map(|v| v as f32),
                mean_marker_error: 0.0,
                params: 1,
            };
            sections::encode::skeletons(&[Skeleton {
                id: SKELETON_ID,
                bones: vec![
                    bone(1, [0.0, 0.9, 0.0], 0.0),
                    bone(2, [0.0, 0.25, 0.0], sway),
                    bone(3, [0.0, 0.4, 0.0], -sway),
                ],
            }])
        } else {
            empty()
        };
        let samples = 8;
        let force_plates = if self.cfg.force_plate {
            let fz = 700.0 + 50.0 * (TAU * t).sin();
            let channel = |v: f64| (0..samples).map(|_| v as f32).collect::<Vec<_>>();
            sections::encode::analog(&[AnalogFrame {
                id: FORCE_PLATE_ID,
                channels: vec![
                    channel(5.0),
                    channel(-3.0),
                    channel(fz),
                    channel(1.0),
                    channel(2.0),
                    channel(0.5),
                ],
            }])
        } else {
            empty()
        };
        let devices = if self.cfg.device {
            sections::encode::analog(&[AnalogFrame {
                id: DEVICE_ID,
                channels: vec![vec![(TAU * t).sin() as f32], vec![(TAU * t).cos() as f32]],
            }])
        } else {
            empty()
        };

        let freq = self.cfg.clock_frequency as f64;
        let mid = (t * freq).round() as u64;
        FrameOfData {
            frame_number,
            marker_sets,
            other_markers: Vec::new(),
            rigid_bodies,
            skeletons,
            assets: empty(),
            labeled_markers,
            force_plates,
            devices,
            timing: FrameTiming {
                timecode: 0,
                timecode_subframe: 0,
                timestamp: t,
                camera_mid_exposure: mid,
                camera_data_received: mid + (0.002 * freq) as u64,
                transmit: mid + (0.004 * freq) as u64,
                precision_timestamp_secs: 0,
                precision_timestamp_fractional_secs: 0,
            },
            params: 0,
        }
    }

    /// The frame encoded as a datagram for this scene's version.
    ///
    /// # Errors
    /// The `natnet` encoder error.
    pub fn frame_datagram(&self, frame_number: i32) -> natnet::Result<Vec<u8>> {
        self.frame(frame_number).to_datagram(self.version)
    }
}

fn empty() -> RawSection {
    RawSection {
        count: 0,
        payload: Vec::new(),
    }
}

/// A pause longer than this re-anchors a [`Pacer`] instead of bursting the
/// frames it missed.
const MAX_LAG: Duration = Duration::from_secs(1);

/// Schedules frames at a fixed rate. The schedule counts in `u64`, so it
/// never wraps; the frame numbers wrap within NatNet's non-negative `i32`
/// range. After a stall (a suspended host, a slow consumer) it carries on
/// from the present instead of sending the missed frames at once.
#[derive(Debug)]
pub(crate) struct Pacer {
    period: Duration,
    anchor: Instant,
    anchor_tick: u64,
    tick: u64,
}

impl Pacer {
    pub(crate) fn new(rate_hz: f64) -> Self {
        Self {
            period: Duration::from_secs_f64(1.0 / rate_hz.max(1e-3)),
            anchor: Instant::now(),
            anchor_tick: 0,
            tick: 0,
        }
    }

    /// The next frame number and when it is due.
    pub(crate) fn next(&mut self) -> (i32, Instant) {
        let mut due = self.due();
        let now = Instant::now();
        if now.saturating_duration_since(due) > MAX_LAG {
            self.anchor = now;
            self.anchor_tick = self.tick;
            due = now;
        }
        let frame_number = (self.tick % (1_u64 << 31)) as i32;
        self.tick += 1;
        (frame_number, due)
    }

    fn due(&self) -> Instant {
        let ticks = (self.tick - self.anchor_tick) as f64;
        self.anchor
            .checked_add(self.period.mul_f64(ticks))
            .unwrap_or(self.anchor)
    }
}

/// Runs the synthetic source: frames go through the real encoder and decoder
/// so the bridge sees exactly what a live server would produce.
pub fn run(scene: &Scene, ctx: &SourceContext, commands: &Receiver<SourceCommand>) {
    let version = scene.version();
    let decoder = match Decoder::new(version) {
        Ok(d) => d,
        Err(e) => {
            ctx.send(SourceEvent::DecodeError(e.to_string()));
            return;
        }
    };
    if !ctx.send(SourceEvent::ServerInfo(scene.server_info(false, 0, [0; 4]))) {
        return;
    }
    let send_model = |ctx: &SourceContext| {
        let md = scene.model_def();
        // Round-trip through the wire format as well.
        let md = match md
            .to_datagram(version)
            .and_then(|d| decoder.parse(&d))
            .map(natnet::DecodedMessage::into_message)
        {
            Ok(Message::ModelDef(m)) => m,
            _ => md,
        };
        ctx.send(SourceEvent::ModelDef(md, version))
    };
    if !send_model(ctx) {
        return;
    }
    ctx.send(SourceEvent::Connection(ConnectionState::Streaming));
    let mut pacer = Pacer::new(scene.config().rate_hz);
    while !ctx.stopped() {
        let (n, due) = pacer.next();
        // Sleep in slices so stop requests stay responsive at low rates.
        loop {
            let now = Instant::now();
            if due <= now || ctx.stopped() {
                break;
            }
            std::thread::sleep((due - now).min(Duration::from_millis(100)));
        }
        if ctx.stopped() {
            return;
        }
        while let Ok(cmd) = commands.try_recv() {
            match cmd {
                SourceCommand::RefreshModel => {
                    if !send_model(ctx) {
                        return;
                    }
                }
            }
        }
        let frame = match scene
            .frame_datagram(n)
            .and_then(|d| decoder.parse(&d))
            .map(natnet::DecodedMessage::into_message)
        {
            Ok(Message::FrameOfData(f)) => f,
            Ok(_) => continue,
            Err(e) => {
                ctx.send(SourceEvent::DecodeError(e.to_string()));
                continue;
            }
        };
        if !ctx.send_frame(frame, SystemTime::now(), Instant::now()) {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocap::model::Model;

    #[test]
    fn pacer_wraps_frame_numbers_and_recovers_from_stalls() {
        let mut pacer = Pacer::new(100.0);
        let (n0, due0) = pacer.next();
        let (n1, due1) = pacer.next();
        assert_eq!((n0, n1), (0, 1));
        assert_eq!(due1 - due0, Duration::from_millis(10));
        // Frame numbers wrap within the non-negative i32 range while the
        // schedule keeps counting.
        pacer.tick = (1 << 31) - 1;
        pacer.anchor_tick = pacer.tick;
        pacer.anchor = Instant::now();
        let (last, due_last) = pacer.next();
        let (wrapped, due_wrapped) = pacer.next();
        assert_eq!((last, wrapped), (i32::MAX, 0));
        assert_eq!(due_wrapped - due_last, Duration::from_millis(10));
        // A stall re-anchors on the present instead of bursting.
        pacer.anchor = Instant::now().checked_sub(Duration::from_secs(5)).unwrap();
        let (_, due) = pacer.next();
        assert!(due.elapsed() < Duration::from_millis(100));
        let (_, next) = pacer.next();
        assert_eq!(next - due, Duration::from_millis(10));
    }

    #[test]
    fn rejects_scenes_that_do_not_fit_a_datagram() {
        let err = Scene::new(SyntheticConfig {
            rigid_bodies: 500,
            ..SyntheticConfig::default()
        })
        .unwrap_err();
        assert!(err.contains("datagram limit"), "{err}");
        assert!(
            Scene::new(SyntheticConfig {
                markers_per_body: 100_000,
                ..SyntheticConfig::default()
            })
            .is_err()
        );
    }

    #[test]
    fn scene_round_trips_through_natnet() {
        for v in ["4.1", "4.0", "3.1"] {
            let scene = Scene::new(SyntheticConfig {
                protocol_version: v.into(),
                ..SyntheticConfig::default()
            })
            .unwrap();
            let version = scene.version();
            let decoder = Decoder::new(version).unwrap();
            let Message::FrameOfData(f) = decoder
                .parse(&scene.frame_datagram(5).unwrap())
                .unwrap()
                .into_message()
            else {
                panic!("not a frame");
            };
            assert_eq!(f.rigid_bodies.len(), 3);
            assert_eq!(f.labeled_markers.len(), 13);
            assert_eq!(sections::skeletons(&f.skeletons).unwrap()[0].bones.len(), 3);
            assert_eq!(
                sections::analog(&f.force_plates).unwrap()[0].channels.len(),
                6
            );

            let md = scene.model_def();
            let Message::ModelDef(md) = decoder
                .parse(&md.to_datagram(version).unwrap())
                .unwrap()
                .into_message()
            else {
                panic!("not a modeldef");
            };
            let model = Model::from_modeldef(&md, version);
            assert!(model.errors.is_empty(), "{v}: {:?}", model.errors);
            assert_eq!(model.rigid_bodies.len(), 3);
            assert_eq!(model.skeletons[&1].bones.len(), 3);
            assert_eq!(model.force_plates.len(), 1);
            assert_eq!(model.devices.len(), 1);
        }
    }

    #[test]
    fn versions() {
        assert_eq!(parse_version("4.1").unwrap(), Version::V4_1);
        assert_eq!(parse_version("3.0.2").unwrap(), Version::new(3, 0, 2, 0));
        assert!(parse_version("5.0").is_err());
        assert!(parse_version("4").is_err());
        assert!(parse_version("x.y").is_err());
    }

    #[test]
    fn dropouts() {
        let scene = Scene::new(SyntheticConfig {
            dropout_period: 4,
            ..SyntheticConfig::default()
        })
        .unwrap();
        assert!(scene.frame(2).rigid_bodies[0].tracking_valid());
        assert!(!scene.frame(3).rigid_bodies[0].tracking_valid());
    }
}
