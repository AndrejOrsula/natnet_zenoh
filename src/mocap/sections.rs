//! Typed decoding of the frame sections `natnet` keeps as raw bytes:
//! skeletons, assets, force plates and devices.
//!
//! The section bodies share one grammar across NatNet 3.x, 4.0 and 4.1 (4.1
//! only adds an outer size, which `natnet` already strips):
//!
//! * skeleton: `id, nBones, bones[id, pos, quat, error, params]`;
//! * asset: skeleton layout followed by `nMarkers, markers[id, pos, size,
//!   params, residual]`;
//! * force plate / device: `id, nChannels, channels[nSamples, f32 …]`.

use natnet::{LabeledMarker, RawSection, RigidBody};

use super::reader::{ReadError, Reader};

/// A tracked skeleton.
#[derive(Debug, Clone, PartialEq)]
pub struct Skeleton {
    /// Skeleton id.
    pub id: i32,
    /// Bones; each id is `(skeleton id << 16) | bone id`.
    pub bones: Vec<RigidBody>,
}

/// A tracked asset (NatNet 4.1 trained markersets and similar).
#[derive(Debug, Clone, PartialEq)]
pub struct Asset {
    /// Asset id.
    pub id: i32,
    /// Asset rigid bodies.
    pub rigid_bodies: Vec<RigidBody>,
    /// Asset markers.
    pub markers: Vec<LabeledMarker>,
}

/// One analog device frame: a force plate or a generic device.
#[derive(Debug, Clone, PartialEq)]
pub struct AnalogFrame {
    /// Device id.
    pub id: i32,
    /// Per channel, the samples received in this mocap frame (analog
    /// devices usually sample faster than the cameras).
    pub channels: Vec<Vec<f32>>,
}

impl AnalogFrame {
    /// The most recent sample of every channel (NaN for an empty channel).
    #[must_use]
    pub fn latest(&self) -> Vec<f32> {
        self.channels
            .iter()
            .map(|c| c.last().copied().unwrap_or(f32::NAN))
            .collect()
    }
}

/// Splits a combined id into `(model id, member id)`: the upper and lower
/// 16 bits used for skeleton bones and labeled markers.
#[must_use]
pub const fn split_id(id: i32) -> (i32, i32) {
    (id >> 16, id & 0xffff)
}

fn rigid_body(r: &mut Reader<'_>) -> Result<RigidBody, ReadError> {
    Ok(RigidBody {
        id: r.i32()?,
        position: r.vec3()?,
        orientation: r.quat()?,
        mean_marker_error: r.f32()?,
        params: r.u16()?,
    })
}

fn labeled_marker(r: &mut Reader<'_>) -> Result<LabeledMarker, ReadError> {
    Ok(LabeledMarker {
        id: r.i32()?,
        position: r.vec3()?,
        size: r.f32()?,
        params: r.u16()?,
        residual: r.f32()?,
    })
}

fn items<T>(
    section: &RawSection,
    mut item: impl FnMut(&mut Reader<'_>) -> Result<T, ReadError>,
) -> Result<Vec<T>, ReadError> {
    let n = usize::try_from(section.count).map_err(|_| ReadError::Count {
        count: section.count,
        offset: 0,
    })?;
    let mut r = Reader::new(&section.payload);
    let mut out = Vec::with_capacity(n.min(section.payload.len() / 8 + 1));
    for _ in 0..n {
        out.push(item(&mut r)?);
    }
    Ok(out)
}

/// Decodes the skeleton section.
///
/// # Errors
/// A [`ReadError`] for a malformed body.
pub fn skeletons(section: &RawSection) -> Result<Vec<Skeleton>, ReadError> {
    items(section, |r| {
        let id = r.i32()?;
        let n = r.count(38)?;
        let bones = (0..n).map(|_| rigid_body(r)).collect::<Result<_, _>>()?;
        Ok(Skeleton { id, bones })
    })
}

/// Decodes the asset section.
///
/// # Errors
/// A [`ReadError`] for a malformed body.
pub fn assets(section: &RawSection) -> Result<Vec<Asset>, ReadError> {
    items(section, |r| {
        let id = r.i32()?;
        let n = r.count(38)?;
        let rigid_bodies = (0..n).map(|_| rigid_body(r)).collect::<Result<_, _>>()?;
        let n = r.count(26)?;
        let markers = (0..n)
            .map(|_| labeled_marker(r))
            .collect::<Result<_, _>>()?;
        Ok(Asset {
            id,
            rigid_bodies,
            markers,
        })
    })
}

/// Decodes a force-plate or device section.
///
/// # Errors
/// A [`ReadError`] for a malformed body.
pub fn analog(section: &RawSection) -> Result<Vec<AnalogFrame>, ReadError> {
    items(section, |r| {
        let id = r.i32()?;
        let n = r.count(4)?;
        let mut channels = Vec::with_capacity(n);
        for _ in 0..n {
            let samples = r.count(4)?;
            channels.push((0..samples).map(|_| r.f32()).collect::<Result<_, _>>()?);
        }
        Ok(AnalogFrame { id, channels })
    })
}

/// Encoders for synthetic sources and tests.
pub mod encode {
    use natnet::{LabeledMarker, RawSection, RigidBody};

    use super::super::reader::Writer;
    use super::{AnalogFrame, Asset, Skeleton};

    fn rigid_body(w: &mut Writer, b: &RigidBody) {
        w.i32(b.id)
            .f32s(&b.position)
            .f32s(&b.orientation)
            .f32(b.mean_marker_error)
            .u16(b.params);
    }

    fn marker(w: &mut Writer, m: &LabeledMarker) {
        w.i32(m.id)
            .f32s(&m.position)
            .f32(m.size)
            .u16(m.params)
            .f32(m.residual);
    }

    fn section(count: usize, w: Writer) -> RawSection {
        RawSection {
            count: i32::try_from(count).unwrap_or(i32::MAX),
            payload: w.bytes,
        }
    }

    /// Encodes a skeleton section.
    #[must_use]
    pub fn skeletons(items: &[Skeleton]) -> RawSection {
        let mut w = Writer::default();
        for s in items {
            w.i32(s.id).count(s.bones.len());
            for b in &s.bones {
                rigid_body(&mut w, b);
            }
        }
        section(items.len(), w)
    }

    /// Encodes an asset section.
    #[must_use]
    pub fn assets(items: &[Asset]) -> RawSection {
        let mut w = Writer::default();
        for a in items {
            w.i32(a.id).count(a.rigid_bodies.len());
            for b in &a.rigid_bodies {
                rigid_body(&mut w, b);
            }
            w.count(a.markers.len());
            for m in &a.markers {
                marker(&mut w, m);
            }
        }
        section(items.len(), w)
    }

    /// Encodes a force-plate or device section.
    #[must_use]
    pub fn analog(items: &[AnalogFrame]) -> RawSection {
        let mut w = Writer::default();
        for d in items {
            w.i32(d.id).count(d.channels.len());
            for c in &d.channels {
                w.count(c.len()).f32s(c);
            }
        }
        section(items.len(), w)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body(id: i32) -> RigidBody {
        RigidBody {
            id,
            position: [1.0, 2.0, 3.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            mean_marker_error: 0.001,
            params: 1,
        }
    }

    #[test]
    fn round_trips() {
        let sk = vec![Skeleton {
            id: 2,
            bones: vec![body((2 << 16) | 1), body((2 << 16) | 2)],
        }];
        assert_eq!(skeletons(&encode::skeletons(&sk)).unwrap(), sk);

        let a = vec![Asset {
            id: 7,
            rigid_bodies: vec![body(1)],
            markers: vec![LabeledMarker {
                id: 3,
                position: [0.1, 0.2, 0.3],
                size: 0.01,
                params: 4,
                residual: 0.0002,
            }],
        }];
        assert_eq!(assets(&encode::assets(&a)).unwrap(), a);

        let d = vec![AnalogFrame {
            id: 1,
            channels: vec![vec![1.0, 2.0], vec![], vec![5.0]],
        }];
        assert_eq!(analog(&encode::analog(&d)).unwrap(), d);
        assert!(d[0].latest()[1].is_nan());
        assert_eq!(d[0].latest()[2], 5.0);
    }

    #[test]
    fn malformed_is_an_error() {
        let mut s = encode::skeletons(&[Skeleton {
            id: 1,
            bones: vec![body(1)],
        }]);
        s.payload.truncate(10);
        assert!(skeletons(&s).is_err());
        s.count = -1;
        assert!(skeletons(&s).is_err());
        assert_eq!(split_id((3 << 16) | 9), (3, 9));
    }
}
