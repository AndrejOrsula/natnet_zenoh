//! Frame conventions and the small amount of quaternion algebra the bridge
//! needs.
//!
//! Motive streams in its own right-handed frame whose up axis is set in the
//! streaming panel (Y by default). ROS uses Z up (REP 103). Converting Y-up
//! to Z-up is a +90° rotation about X: positions map `(x, y, z) ↦ (x, −z, y)`
//! and orientations are conjugated by the same rotation, which maps the
//! quaternion's vector part the same way and keeps its scalar part, so
//! a body at rest in Motive stays at rest with its axes expressed in ROS.

use serde::{Deserialize, Serialize};

/// The up axis Motive streams with.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UpAxis {
    /// Y up (Motive's default): converted to ROS Z up.
    #[default]
    Y,
    /// Z up: already the ROS convention, passed through.
    Z,
}

/// Converts native NatNet poses into the output frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameConvention {
    /// The up axis of the incoming stream.
    pub up_axis: UpAxis,
    /// Factor applied to positions (1.0 for metres).
    pub scale: f64,
}

impl Default for FrameConvention {
    fn default() -> Self {
        Self {
            up_axis: UpAxis::Y,
            scale: 1.0,
        }
    }
}

impl FrameConvention {
    /// Converts a position.
    #[must_use]
    pub fn position(&self, p: [f32; 3]) -> [f64; 3] {
        let [x, y, z] = p.map(|v| f64::from(v) * self.scale);
        match self.up_axis {
            UpAxis::Y => [x, -z, y],
            UpAxis::Z => [x, y, z],
        }
    }

    /// Converts a free vector (a direction or velocity: no scaling).
    #[must_use]
    pub fn vector(&self, v: [f32; 3]) -> [f64; 3] {
        let [x, y, z] = v.map(f64::from);
        match self.up_axis {
            UpAxis::Y => [x, -z, y],
            UpAxis::Z => [x, y, z],
        }
    }

    /// Converts and normalizes an orientation `[x, y, z, w]`; `None` when
    /// the quaternion is zero or not finite.
    #[must_use]
    #[allow(clippy::many_single_char_names)]
    pub fn orientation(&self, q: [f32; 4]) -> Option<[f64; 4]> {
        let [x, y, z, w] = q.map(f64::from);
        let converted = match self.up_axis {
            UpAxis::Y => [x, -z, y, w],
            UpAxis::Z => [x, y, z, w],
        };
        quat::normalize(converted)
    }
}

/// Quaternions as `[x, y, z, w]` arrays.
pub mod quat {
    /// A quaternion `[x, y, z, w]`.
    pub type Quat = [f64; 4];

    /// The identity rotation.
    pub const IDENTITY: Quat = [0.0, 0.0, 0.0, 1.0];

    /// Normalizes; `None` for a zero or non-finite quaternion.
    #[must_use]
    pub fn normalize(q: Quat) -> Option<Quat> {
        let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !n.is_finite() || n < 1e-9 {
            return None;
        }
        Some(q.map(|v| v / n))
    }

    /// The Hamilton product `a ⊗ b`.
    #[must_use]
    pub fn mul(a: Quat, b: Quat) -> Quat {
        let [ax, ay, az, aw] = a;
        let [bx, by, bz, bw] = b;
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    }

    /// The conjugate (the inverse of a unit quaternion).
    #[must_use]
    pub fn conj(q: Quat) -> Quat {
        [-q[0], -q[1], -q[2], q[3]]
    }

    /// Rotates vector `v` by unit quaternion `q`.
    #[must_use]
    pub fn rotate(q: Quat, v: [f64; 3]) -> [f64; 3] {
        let r = mul(mul(q, [v[0], v[1], v[2], 0.0]), conj(q));
        [r[0], r[1], r[2]]
    }

    /// The rotation vector (axis × angle) of unit quaternion `q`, taking
    /// the shorter of the two equivalent rotations.
    #[must_use]
    pub fn log(q: Quat) -> [f64; 3] {
        let q = if q[3] < 0.0 { q.map(|v| -v) } else { q };
        let s = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2]).sqrt();
        if s < 1e-12 {
            // Small-angle limit: angle ≈ 2 s, axis·angle ≈ 2 v.
            return [2.0 * q[0], 2.0 * q[1], 2.0 * q[2]];
        }
        let angle = 2.0 * s.atan2(q[3]);
        [q[0] / s * angle, q[1] / s * angle, q[2] / s * angle]
    }

    /// The quaternion of a rotation of `angle` radians about unit `axis`.
    #[must_use]
    pub fn from_axis_angle(axis: [f64; 3], angle: f64) -> Quat {
        let (s, c) = (angle / 2.0).sin_cos();
        [axis[0] * s, axis[1] * s, axis[2] * s, c]
    }

    /// Roll, pitch, yaw (XYZ fixed axes) to a quaternion.
    #[must_use]
    pub fn from_rpy(roll: f64, pitch: f64, yaw: f64) -> Quat {
        let qx = from_axis_angle([1.0, 0.0, 0.0], roll);
        let qy = from_axis_angle([0.0, 1.0, 0.0], pitch);
        let qz = from_axis_angle([0.0, 0.0, 1.0], yaw);
        mul(qz, mul(qy, qx))
    }
}

#[cfg(test)]
mod tests {
    use super::quat::*;
    use super::*;

    fn close(a: &[f64], b: &[f64]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-6)
    }

    #[test]
    fn y_up_to_z_up() {
        let c = FrameConvention::default();
        // Motive "up" (+Y) becomes ROS up (+Z); Motive +Z becomes ROS −Y.
        assert!(close(&c.position([0.0, 1.0, 0.0]), &[0.0, 0.0, 1.0]));
        assert!(close(&c.position([0.0, 0.0, 1.0]), &[0.0, -1.0, 0.0]));
        assert!(close(&c.position([1.0, 0.0, 0.0]), &[1.0, 0.0, 0.0]));

        // A yaw about Motive's up axis becomes a yaw about ROS Z.
        let angle = 0.7_f64;
        let motive = from_axis_angle([0.0, 1.0, 0.0], angle).map(|v| v as f32);
        let ros = c.orientation(motive).unwrap();
        assert!(close(&ros, &from_axis_angle([0.0, 0.0, 1.0], angle)));

        // Conjugation commutes with rotating vectors.
        let q = normalize([0.1, 0.5, -0.3, 0.8]).unwrap();
        let v = [0.3_f32, -1.2, 2.0];
        let rotated_native = rotate(q, v.map(f64::from)).map(|x| x as f32);
        let lhs = c.position(rotated_native);
        let rhs = rotate(c.orientation(q.map(|x| x as f32)).unwrap(), c.position(v));
        assert!(lhs.iter().zip(rhs).all(|(a, b)| (a - b).abs() < 1e-5));
    }

    #[test]
    fn z_up_passthrough_and_invalid() {
        let c = FrameConvention {
            up_axis: UpAxis::Z,
            scale: 0.001,
        };
        assert!(close(
            &c.position([1000.0, 2000.0, 3000.0]),
            &[1.0, 2.0, 3.0]
        ));
        assert!(c.orientation([0.0; 4]).is_none());
        assert!(c.orientation([f32::NAN, 0.0, 0.0, 1.0]).is_none());
    }

    #[test]
    fn quaternion_algebra() {
        let q = from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2);
        assert!(close(&rotate(q, [1.0, 0.0, 0.0]), &[0.0, 1.0, 0.0]));
        assert!(close(&mul(q, conj(q)), &IDENTITY));
        assert!(close(&log(q), &[0.0, 0.0, std::f64::consts::FRAC_PI_2]));
        // log picks the short way round.
        assert!(close(
            &log(q.map(|v| -v)),
            &[0.0, 0.0, std::f64::consts::FRAC_PI_2]
        ));
        assert!(close(
            &from_rpy(0.0, 0.0, 0.3),
            &from_axis_angle([0.0, 0.0, 1.0], 0.3)
        ));
    }
}
