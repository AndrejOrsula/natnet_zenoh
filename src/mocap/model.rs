//! A typed view of a MODELDEF reply: names, hierarchy and metadata of every
//! tracked asset, keyed by streaming id.
//!
//! `natnet` decodes marker sets and rigid bodies and keeps the other
//! datasets (skeletons, force plates, devices, cameras, assets) and the
//! rigid-body marker tails opaque; this module decodes those bodies with the
//! grammar of the NatNet SDK. A dataset that fails to decode is recorded in
//! [`Model::errors`] instead of failing the whole model.

use std::collections::{BTreeMap, BTreeSet};

use natnet::modeldef::{TYPE_ASSET, TYPE_CAMERA, TYPE_DEVICE, TYPE_FORCE_PLATE, TYPE_SKELETON};
use natnet::{AssetDef, DataSet, ForcePlateDef, MarkerSetDef, ModelDef, RigidBodyDef, Version};
use serde::Serialize;

use super::reader::{ReadError, Reader, Writer};

/// A marker of a rigid-body definition.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MarkerDesc {
    /// Marker name (NatNet ≥ 4.0; empty before).
    pub name: String,
    /// Active-marker label, 0 for passive markers.
    pub label: i32,
    /// Position relative to the rigid body, native frame, metres.
    pub offset: [f32; 3],
}

/// A rigid-body (or skeleton bone, or asset body) definition.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RigidBodyDesc {
    /// Name.
    pub name: String,
    /// Streaming id.
    pub id: i32,
    /// Parent id, −1 for none.
    pub parent_id: i32,
    /// Offset from the parent, native frame, metres.
    pub offset: [f32; 3],
    /// Markers.
    pub markers: Vec<MarkerDesc>,
}

/// A skeleton definition.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SkeletonDesc {
    /// Name.
    pub name: String,
    /// Skeleton id.
    pub id: i32,
    /// Bones; bone ids are local to the skeleton.
    pub bones: Vec<RigidBodyDesc>,
}

/// A force-plate definition.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ForcePlateDesc {
    /// Id.
    pub id: i32,
    /// Serial number.
    pub serial: String,
    /// Width, metres.
    pub width: f32,
    /// Length, metres.
    pub length: f32,
    /// Electrical-centre offset.
    pub origin: [f32; 3],
    /// 12×12 row-major calibration matrix.
    pub calibration: Vec<f32>,
    /// Plate corners, four `[x, y, z]`, native frame.
    pub corners: [[f32; 3]; 4],
    /// Plate type.
    pub plate_type: i32,
    /// Channel data type.
    pub channel_data_type: i32,
    /// Channel names (typically `Fx Fy Fz Mx My Mz Cx Cy`).
    pub channel_names: Vec<String>,
}

/// A generic analog device definition.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DeviceDesc {
    /// Id.
    pub id: i32,
    /// Name.
    pub name: String,
    /// Serial number.
    pub serial: String,
    /// Device type.
    pub device_type: i32,
    /// Channel data type.
    pub channel_data_type: i32,
    /// Channel names.
    pub channel_names: Vec<String>,
}

/// A camera definition.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CameraDesc {
    /// Name.
    pub name: String,
    /// Position, native frame, metres.
    pub position: [f32; 3],
    /// Orientation `[x, y, z, w]`, native frame.
    pub orientation: [f32; 4],
}

/// A marker of an asset definition.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct AssetMarkerDesc {
    /// Name.
    pub name: String,
    /// Id.
    pub id: i32,
    /// Position.
    pub position: [f32; 3],
    /// Size.
    pub size: f32,
    /// Parameters.
    pub params: u16,
}

/// An asset definition (NatNet 4.1).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct AssetDesc {
    /// Name.
    pub name: String,
    /// Asset type.
    pub asset_type: i32,
    /// Id.
    pub id: i32,
    /// Rigid bodies.
    pub rigid_bodies: Vec<RigidBodyDesc>,
    /// Markers.
    pub markers: Vec<AssetMarkerDesc>,
}

/// A decoded model definition.
#[derive(Debug, Clone, PartialEq, Default, Serialize)]
pub struct Model {
    /// Marker sets in wire order.
    #[serde(serialize_with = "marker_sets_json")]
    pub marker_sets: Vec<MarkerSetDef>,
    /// Rigid bodies by streaming id.
    pub rigid_bodies: BTreeMap<i32, RigidBodyDesc>,
    /// Skeletons by id.
    pub skeletons: BTreeMap<i32, SkeletonDesc>,
    /// Force plates by id.
    pub force_plates: BTreeMap<i32, ForcePlateDesc>,
    /// Devices by id.
    pub devices: BTreeMap<i32, DeviceDesc>,
    /// Cameras in wire order.
    pub cameras: Vec<CameraDesc>,
    /// Assets by id.
    pub assets: BTreeMap<i32, AssetDesc>,
    /// Datasets that could not be decoded: `(type id, error)`.
    pub errors: Vec<(i32, String)>,
}

/// Serializes marker sets in this crate's own shape, so the native and ROS
/// model documents do not follow `natnet`'s serde representation.
fn marker_sets_json<S: serde::Serializer>(
    sets: &[MarkerSetDef],
    serializer: S,
) -> Result<S::Ok, S::Error> {
    #[derive(Serialize)]
    struct MarkerSet<'a> {
        name: &'a str,
        marker_names: &'a [String],
    }
    serializer.collect_seq(sets.iter().map(|set| MarkerSet {
        name: &set.name,
        marker_names: &set.marker_names,
    }))
}

fn marker_names(version: Version) -> bool {
    version.major >= 4
}

fn rigid_body_tail(r: &mut Reader<'_>, version: Version) -> Result<Vec<MarkerDesc>, ReadError> {
    let n = r.count(if marker_names(version) { 17 } else { 16 })?;
    let offsets = (0..n).map(|_| r.vec3()).collect::<Result<Vec<_>, _>>()?;
    let labels = (0..n).map(|_| r.i32()).collect::<Result<Vec<_>, _>>()?;
    let names = if marker_names(version) {
        (0..n).map(|_| r.cstr()).collect::<Result<Vec<_>, _>>()?
    } else {
        vec![String::new(); n]
    };
    Ok(offsets
        .into_iter()
        .zip(labels)
        .zip(names)
        .map(|((offset, label), name)| MarkerDesc {
            name,
            label,
            offset,
        })
        .collect())
}

fn rigid_body_from_def(def: &RigidBodyDef, version: Version) -> (RigidBodyDesc, Option<ReadError>) {
    let mut r = Reader::new(&def.tail);
    let (markers, error) = match rigid_body_tail(&mut r, version) {
        Ok(m) => (m, None),
        Err(e) => (Vec::new(), Some(e)),
    };
    (
        RigidBodyDesc {
            name: def.name.clone(),
            id: def.id,
            parent_id: def.parent_id,
            offset: def.offset,
            markers,
        },
        error,
    )
}

/// Skeleton bones or asset rigid bodies; `natnet` has already validated
/// their marker descriptions.
fn rigid_bodies_from_defs(
    defs: &[RigidBodyDef],
    version: Version,
) -> Result<Vec<RigidBodyDesc>, ReadError> {
    defs.iter()
        .map(|def| match rigid_body_from_def(def, version) {
            (desc, None) => Ok(desc),
            (_, Some(e)) => Err(e),
        })
        .collect()
}

fn force_plate_from_def(def: &ForcePlateDef) -> ForcePlateDesc {
    ForcePlateDesc {
        id: def.id,
        serial: def.serial_number.clone(),
        width: def.width,
        length: def.length,
        origin: def.origin,
        calibration: def.calibration_matrix.iter().flatten().copied().collect(),
        corners: def.corners,
        plate_type: def.plate_type,
        channel_data_type: def.channel_data_type,
        channel_names: def.channel_names.clone(),
    }
}

fn asset_from_def(def: &AssetDef, version: Version) -> Result<AssetDesc, ReadError> {
    Ok(AssetDesc {
        name: def.name.clone(),
        asset_type: def.asset_type,
        id: def.id,
        rigid_bodies: rigid_bodies_from_defs(&def.rigid_bodies, version)?,
        markers: def
            .markers
            .iter()
            .map(|m| AssetMarkerDesc {
                name: m.name.clone(),
                id: m.id,
                position: m.position,
                size: m.size,
                params: m.params,
            })
            .collect(),
    })
}

fn rigid_body(r: &mut Reader<'_>, version: Version) -> Result<RigidBodyDesc, ReadError> {
    Ok(RigidBodyDesc {
        name: r.cstr()?,
        id: r.i32()?,
        parent_id: r.i32()?,
        offset: r.vec3()?,
        markers: rigid_body_tail(r, version)?,
    })
}

fn skeleton(r: &mut Reader<'_>, version: Version) -> Result<SkeletonDesc, ReadError> {
    let name = r.cstr()?;
    let id = r.i32()?;
    let n = r.count(25)?;
    let bones = (0..n)
        .map(|_| rigid_body(r, version))
        .collect::<Result<_, _>>()?;
    Ok(SkeletonDesc { name, id, bones })
}

fn names(r: &mut Reader<'_>) -> Result<Vec<String>, ReadError> {
    let n = r.count(1)?;
    (0..n).map(|_| r.cstr()).collect()
}

fn force_plate(r: &mut Reader<'_>) -> Result<ForcePlateDesc, ReadError> {
    let id = r.i32()?;
    let serial = r.cstr()?;
    let width = r.f32()?;
    let length = r.f32()?;
    let origin = r.vec3()?;
    let calibration = (0..144).map(|_| r.f32()).collect::<Result<_, _>>()?;
    let mut corners = [[0.0_f32; 3]; 4];
    for c in &mut corners {
        *c = r.vec3()?;
    }
    Ok(ForcePlateDesc {
        id,
        serial,
        width,
        length,
        origin,
        calibration,
        corners,
        plate_type: r.i32()?,
        channel_data_type: r.i32()?,
        channel_names: names(r)?,
    })
}

fn device(r: &mut Reader<'_>) -> Result<DeviceDesc, ReadError> {
    Ok(DeviceDesc {
        id: r.i32()?,
        name: r.cstr()?,
        serial: r.cstr()?,
        device_type: r.i32()?,
        channel_data_type: r.i32()?,
        channel_names: names(r)?,
    })
}

fn camera(r: &mut Reader<'_>) -> Result<CameraDesc, ReadError> {
    Ok(CameraDesc {
        name: r.cstr()?,
        position: r.vec3()?,
        orientation: r.quat()?,
    })
}

fn asset(r: &mut Reader<'_>, version: Version) -> Result<AssetDesc, ReadError> {
    let name = r.cstr()?;
    let asset_type = r.i32()?;
    let id = r.i32()?;
    let n = r.count(25)?;
    let rigid_bodies = (0..n)
        .map(|_| rigid_body(r, version))
        .collect::<Result<_, _>>()?;
    let n = r.count(23)?;
    let markers = (0..n)
        .map(|_| {
            Ok(AssetMarkerDesc {
                name: r.cstr()?,
                id: r.i32()?,
                position: r.vec3()?,
                size: r.f32()?,
                params: r.u16()?,
            })
        })
        .collect::<Result<_, ReadError>>()?;
    Ok(AssetDesc {
        name,
        asset_type,
        id,
        rigid_bodies,
        markers,
    })
}

fn insert_first<K: Ord, V>(map: &mut BTreeMap<K, V>, key: K, value: V) -> bool {
    match map.entry(key) {
        std::collections::btree_map::Entry::Vacant(entry) => {
            entry.insert(value);
            true
        }
        std::collections::btree_map::Entry::Occupied(_) => false,
    }
}

fn unique_bones(
    bones: Vec<RigidBodyDesc>,
    skeleton_id: i32,
    type_id: i32,
    errors: &mut Vec<(i32, String)>,
) -> Vec<RigidBodyDesc> {
    let mut seen = BTreeSet::new();
    bones
        .into_iter()
        .filter(|bone| {
            if seen.insert(bone.id) {
                true
            } else {
                errors.push((
                    type_id,
                    format!("duplicate bone id {} in skeleton {skeleton_id}", bone.id),
                ));
                false
            }
        })
        .collect()
}

impl Model {
    /// Decodes a model definition received under protocol `version`.
    #[must_use]
    pub fn from_modeldef(modeldef: &ModelDef, version: Version) -> Self {
        let mut model = Self::default();
        for dataset in &modeldef.datasets {
            let type_id = dataset.type_id();
            let result: Result<(), ReadError> = match dataset {
                DataSet::MarkerSet(m) => {
                    model.marker_sets.push(m.clone());
                    Ok(())
                }
                DataSet::RigidBody(def) => {
                    let (desc, error) = rigid_body_from_def(def, version);
                    let id = desc.id;
                    if !insert_first(&mut model.rigid_bodies, id, desc) {
                        model
                            .errors
                            .push((type_id, format!("duplicate rigid body id {id}")));
                    }
                    error.map_or(Ok(()), Err)
                }
                DataSet::Skeleton(def) => {
                    rigid_bodies_from_defs(&def.rigid_bodies, version).map(|bones| {
                        let bones = unique_bones(bones, def.id, type_id, &mut model.errors);
                        let id = def.id;
                        if !insert_first(
                            &mut model.skeletons,
                            id,
                            SkeletonDesc {
                                name: def.name.clone(),
                                id,
                                bones,
                            },
                        ) {
                            model
                                .errors
                                .push((type_id, format!("duplicate skeleton id {id}")));
                        }
                    })
                }
                DataSet::ForcePlate(def) => {
                    let id = def.id;
                    if !insert_first(&mut model.force_plates, id, force_plate_from_def(def)) {
                        model
                            .errors
                            .push((type_id, format!("duplicate force plate id {id}")));
                    }
                    Ok(())
                }
                DataSet::Device(def) => {
                    let id = def.id;
                    if !insert_first(
                        &mut model.devices,
                        id,
                        DeviceDesc {
                            id,
                            name: def.name.clone(),
                            serial: def.serial_number.clone(),
                            device_type: def.device_type,
                            channel_data_type: def.channel_data_type,
                            channel_names: def.channel_names.clone(),
                        },
                    ) {
                        model
                            .errors
                            .push((type_id, format!("duplicate device id {id}")));
                    }
                    Ok(())
                }
                DataSet::Camera(def) => {
                    model.cameras.push(CameraDesc {
                        name: def.name.clone(),
                        position: def.position,
                        orientation: def.orientation,
                    });
                    Ok(())
                }
                DataSet::Asset(def) => asset_from_def(def, version).map(|a| {
                    let id = a.id;
                    if !insert_first(&mut model.assets, id, a) {
                        model
                            .errors
                            .push((type_id, format!("duplicate asset id {id}")));
                    }
                }),
                // `natnet` keeps a description it could not represent exactly
                // (and any dataset a generator builds by hand) as raw bytes.
                DataSet::Other { type_id, body } => {
                    let mut r = Reader::new(body);
                    match *type_id {
                        TYPE_SKELETON => skeleton(&mut r, version).map(|mut s| {
                            s.bones = unique_bones(s.bones, s.id, *type_id, &mut model.errors);
                            let id = s.id;
                            if !insert_first(&mut model.skeletons, id, s) {
                                model
                                    .errors
                                    .push((*type_id, format!("duplicate skeleton id {id}")));
                            }
                        }),
                        TYPE_FORCE_PLATE => force_plate(&mut r).map(|f| {
                            let id = f.id;
                            if !insert_first(&mut model.force_plates, id, f) {
                                model
                                    .errors
                                    .push((*type_id, format!("duplicate force plate id {id}")));
                            }
                        }),
                        TYPE_DEVICE => device(&mut r).map(|d| {
                            let id = d.id;
                            if !insert_first(&mut model.devices, id, d) {
                                model
                                    .errors
                                    .push((*type_id, format!("duplicate device id {id}")));
                            }
                        }),
                        TYPE_CAMERA => camera(&mut r).map(|c| model.cameras.push(c)),
                        TYPE_ASSET => asset(&mut r, version).map(|a| {
                            let id = a.id;
                            if !insert_first(&mut model.assets, id, a) {
                                model
                                    .errors
                                    .push((*type_id, format!("duplicate asset id {id}")));
                            }
                        }),
                        _ => Ok(()),
                    }
                }
                _ => Ok(()),
            };
            if let Err(e) = result {
                tracing::warn!("cannot decode MODELDEF dataset of type {type_id}: {e}");
                model.errors.push((type_id, e.to_string()));
            }
        }
        model
    }

    /// The name of a rigid body.
    #[must_use]
    pub fn rigid_body_name(&self, id: i32) -> Option<&str> {
        self.rigid_bodies.get(&id).map(|b| b.name.as_str())
    }

    /// Total number of described assets of every kind.
    #[must_use]
    pub fn len(&self) -> usize {
        self.marker_sets.len()
            + self.rigid_bodies.len()
            + self.skeletons.len()
            + self.force_plates.len()
            + self.devices.len()
            + self.cameras.len()
            + self.assets.len()
    }

    /// Whether the model describes nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// Encoders of the dataset descriptions, for synthetic sources and tests.
pub mod encode {
    use natnet::{DataSet, DeviceDef, ForcePlateDef, RigidBodyDef, SkeletonDef, Version};

    use super::{DeviceDesc, ForcePlateDesc};
    use super::{MarkerDesc, RigidBodyDesc, SkeletonDesc, Writer, marker_names};

    fn tail(w: &mut Writer, markers: &[MarkerDesc], version: Version) {
        w.count(markers.len());
        for m in markers {
            w.f32s(&m.offset);
        }
        for m in markers {
            w.i32(m.label);
        }
        if marker_names(version) {
            for m in markers {
                w.cstr(&m.name);
            }
        }
    }

    fn rigid_body_def(desc: &RigidBodyDesc, version: Version) -> RigidBodyDef {
        let mut w = Writer::default();
        tail(&mut w, &desc.markers, version);
        RigidBodyDef {
            name: desc.name.clone(),
            id: desc.id,
            parent_id: desc.parent_id,
            offset: desc.offset,
            tail: w.bytes,
        }
    }

    /// A rigid-body dataset.
    #[must_use]
    pub fn rigid_body(desc: &RigidBodyDesc, version: Version) -> DataSet {
        DataSet::RigidBody(rigid_body_def(desc, version))
    }

    /// A skeleton dataset.
    #[must_use]
    pub fn skeleton(desc: &SkeletonDesc, version: Version) -> DataSet {
        DataSet::Skeleton(SkeletonDef {
            name: desc.name.clone(),
            id: desc.id,
            rigid_bodies: desc
                .bones
                .iter()
                .map(|bone| rigid_body_def(bone, version))
                .collect(),
        })
    }

    /// A force-plate dataset. Calibration values past 144 are dropped and
    /// missing ones are zero.
    #[must_use]
    pub fn force_plate(desc: &ForcePlateDesc) -> DataSet {
        let mut calibration_matrix = [[0.0; 12]; 12];
        for (cell, value) in calibration_matrix
            .as_flattened_mut()
            .iter_mut()
            .zip(&desc.calibration)
        {
            *cell = *value;
        }
        DataSet::ForcePlate(Box::new(ForcePlateDef {
            id: desc.id,
            serial_number: desc.serial.clone(),
            width: desc.width,
            length: desc.length,
            origin: desc.origin,
            calibration_matrix,
            corners: desc.corners,
            plate_type: desc.plate_type,
            channel_data_type: desc.channel_data_type,
            channel_names: desc.channel_names.clone(),
        }))
    }

    /// A device dataset.
    #[must_use]
    pub fn device(desc: &DeviceDesc) -> DataSet {
        DataSet::Device(DeviceDef {
            id: desc.id,
            name: desc.name.clone(),
            serial_number: desc.serial.clone(),
            device_type: desc.device_type,
            channel_data_type: desc.channel_data_type,
            channel_names: desc.channel_names.clone(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body(name: &str, id: i32) -> RigidBodyDesc {
        RigidBodyDesc {
            name: name.into(),
            id,
            parent_id: -1,
            offset: [0.0; 3],
            markers: vec![
                MarkerDesc {
                    name: format!("{name}_1"),
                    label: 0,
                    offset: [0.1, 0.0, 0.0],
                },
                MarkerDesc {
                    name: format!("{name}_2"),
                    label: 5,
                    offset: [0.0, 0.1, 0.0],
                },
            ],
        }
    }

    #[test]
    fn decodes_all_dataset_kinds() {
        for version in [
            Version::V4_1,
            Version::new(4, 0, 0, 0),
            Version::new(3, 1, 0, 0),
        ] {
            let mut rb = body("rover", 1);
            if version.major < 4 {
                for m in &mut rb.markers {
                    m.name.clear();
                }
            }
            let sk = SkeletonDesc {
                name: "actor".into(),
                id: 2,
                bones: vec![
                    RigidBodyDesc {
                        parent_id: 0,
                        ..body("hip", 1)
                    },
                    RigidBodyDesc {
                        parent_id: 1,
                        ..body("spine", 2)
                    },
                ],
            };
            let mut sk_expected = sk.clone();
            if version.major < 4 {
                for b in &mut sk_expected.bones {
                    for m in &mut b.markers {
                        m.name.clear();
                    }
                }
            }
            let fp = ForcePlateDesc {
                id: 1,
                serial: "FP-1".into(),
                width: 0.6,
                length: 0.4,
                origin: [0.0; 3],
                calibration: vec![0.0; 144],
                corners: [[0.0; 3], [0.6, 0.0, 0.0], [0.6, 0.0, 0.4], [0.0, 0.0, 0.4]],
                plate_type: 1,
                channel_data_type: 0,
                channel_names: ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
                    .map(String::from)
                    .to_vec(),
            };
            let dev = DeviceDesc {
                id: 9,
                name: "NI-DAQ".into(),
                serial: "123".into(),
                device_type: 1,
                channel_data_type: 0,
                channel_names: vec!["ai0".into()],
            };
            let md = ModelDef {
                datasets: vec![
                    DataSet::MarkerSet(MarkerSetDef {
                        name: "all".into(),
                        marker_names: vec!["a".into()],
                    }),
                    encode::rigid_body(&rb, version),
                    encode::skeleton(&sk, version),
                    encode::force_plate(&fp),
                    encode::device(&dev),
                ],
            };
            let model = Model::from_modeldef(&md, version);
            assert!(model.errors.is_empty(), "{:?}", model.errors);
            assert_eq!(model.rigid_bodies[&1], rb);
            assert_eq!(model.skeletons[&2], sk_expected);
            assert_eq!(model.force_plates[&1], fp);
            assert_eq!(model.devices[&9], dev);
            assert_eq!(model.marker_sets.len(), 1);
            assert_eq!(model.rigid_body_name(1), Some("rover"));
            assert_eq!(model.len(), 5);

            // Through the wire, `natnet` returns typed datasets; the model
            // is the same.
            let natnet::Message::ModelDef(parsed) = natnet::Decoder::new(version)
                .unwrap()
                .parse(&md.to_datagram(version).unwrap())
                .unwrap()
                .into_message()
            else {
                panic!("not a MODELDEF");
            };
            assert!(matches!(parsed.datasets[2], DataSet::Skeleton(_)));
            assert!(matches!(parsed.datasets[3], DataSet::ForcePlate(_)));
            assert!(matches!(parsed.datasets[4], DataSet::Device(_)));
            assert_eq!(Model::from_modeldef(&parsed, version), model);
        }
    }

    #[test]
    fn decodes_typed_cameras_and_assets() {
        let DataSet::RigidBody(rb) = encode::rigid_body(&body("tool", 3), Version::V4_1) else {
            panic!("not a rigid body");
        };
        let md = ModelDef {
            datasets: vec![
                DataSet::Camera(natnet::CameraDef {
                    name: "cam_1".into(),
                    position: [1.0, 2.0, 3.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                }),
                DataSet::Asset(AssetDef {
                    name: "mat".into(),
                    asset_type: 1,
                    id: 4,
                    rigid_bodies: vec![rb],
                    markers: vec![natnet::MarkerDef {
                        name: "m1".into(),
                        id: 7,
                        position: [0.1, 0.2, 0.3],
                        size: 0.01,
                        params: 1,
                    }],
                }),
            ],
        };
        let model = Model::from_modeldef(&md, Version::V4_1);
        assert!(model.errors.is_empty(), "{:?}", model.errors);
        assert_eq!(model.cameras.len(), 1);
        assert_eq!(model.cameras[0].name, "cam_1");
        assert_eq!(model.cameras[0].position, [1.0, 2.0, 3.0]);
        let asset = &model.assets[&4];
        assert_eq!(asset.rigid_bodies, [body("tool", 3)]);
        assert_eq!(asset.markers.len(), 1);
        assert_eq!((asset.markers[0].id, asset.markers[0].params), (7, 1));
    }

    #[test]
    fn bad_dataset_is_recorded_not_fatal() {
        let md = ModelDef {
            datasets: vec![DataSet::Other {
                type_id: TYPE_SKELETON,
                body: vec![b'x', 0, 1],
            }],
        };
        let model = Model::from_modeldef(&md, Version::V4_1);
        assert_eq!(model.errors.len(), 1);
        assert!(model.skeletons.is_empty());
    }

    #[test]
    fn duplicate_ids_keep_first_definition_and_report_errors() {
        let force_plate = |id, serial: &str| ForcePlateDesc {
            id,
            serial: serial.into(),
            width: 0.6,
            length: 0.4,
            origin: [0.0; 3],
            calibration: vec![0.0; 144],
            corners: [[0.0; 3]; 4],
            plate_type: 1,
            channel_data_type: 0,
            channel_names: vec![],
        };
        let device = |id, name: &str| DeviceDesc {
            id,
            name: name.into(),
            serial: String::new(),
            device_type: 1,
            channel_data_type: 0,
            channel_names: vec![],
        };
        let asset = |id, name: &str| AssetDef {
            name: name.into(),
            asset_type: 1,
            id,
            rigid_bodies: vec![],
            markers: vec![],
        };
        let skeleton = |name: &str| SkeletonDesc {
            name: name.into(),
            id: 7,
            bones: vec![body("Hip", 1), body("DuplicateHip", 1)],
        };
        let mut bone_duplicates = skeleton("bone duplicates");
        bone_duplicates.id = 8;
        let version = Version::V4_1;
        let md = ModelDef {
            datasets: vec![
                encode::rigid_body(&body("first body", 7), version),
                encode::rigid_body(&body("later body", 7), version),
                // The same numeric ID is valid across independent categories.
                encode::skeleton(
                    &SkeletonDesc {
                        bones: vec![],
                        ..skeleton("first skeleton")
                    },
                    version,
                ),
                encode::skeleton(
                    &SkeletonDesc {
                        bones: vec![],
                        ..skeleton("later skeleton")
                    },
                    version,
                ),
                encode::force_plate(&force_plate(7, "first plate")),
                encode::force_plate(&force_plate(7, "later plate")),
                encode::device(&device(7, "first device")),
                encode::device(&device(7, "later device")),
                DataSet::Asset(asset(7, "first asset")),
                DataSet::Asset(asset(7, "later asset")),
                encode::skeleton(&bone_duplicates, version),
            ],
        };

        let model = Model::from_modeldef(&md, version);
        assert_eq!(model.rigid_bodies[&7].name, "first body");
        assert_eq!(model.skeletons[&7].name, "first skeleton");
        assert_eq!(model.force_plates[&7].serial, "first plate");
        assert_eq!(model.devices[&7].name, "first device");
        assert_eq!(model.assets[&7].name, "first asset");
        assert_eq!(model.skeletons[&7].bones.len(), 0);
        assert_eq!(
            model
                .errors
                .iter()
                .map(|(_, error)| error.as_str())
                .collect::<Vec<_>>(),
            [
                "duplicate rigid body id 7",
                "duplicate skeleton id 7",
                "duplicate force plate id 7",
                "duplicate device id 7",
                "duplicate asset id 7",
                "duplicate bone id 1 in skeleton 8",
            ]
        );
        let bones = model
            .skeletons
            .values()
            .find(|s| s.name == "bone duplicates")
            .unwrap();
        assert_eq!(
            bones
                .bones
                .iter()
                .map(|b| b.name.as_str())
                .collect::<Vec<_>>(),
            ["Hip"]
        );
    }
}
