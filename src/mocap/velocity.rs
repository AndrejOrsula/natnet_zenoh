//! Finite-difference velocity estimation for rigid bodies.

use std::collections::HashMap;

use super::transform::quat::{self, Quat};

/// A velocity estimate.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize)]
pub struct Velocity {
    /// Linear velocity in the world frame, m/s.
    pub linear_world: [f64; 3],
    /// Angular velocity in the world frame, rad/s.
    pub angular_world: [f64; 3],
    /// Linear velocity in the body frame, m/s.
    pub linear_body: [f64; 3],
    /// Angular velocity in the body frame, rad/s.
    pub angular_body: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
struct Last {
    t: f64,
    position: [f64; 3],
    orientation: Quat,
    velocity: Option<Velocity>,
}

/// Per-body velocity estimator with exponential smoothing.
#[derive(Debug, Clone)]
pub struct VelocityEstimator {
    /// Smoothing factor in `(0, 1]`: 1 disables smoothing.
    alpha: f64,
    /// Longest gap between samples that is still differenced, seconds.
    max_gap: f64,
    last: HashMap<i32, Last>,
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn lerp(prev: [f64; 3], new: [f64; 3], alpha: f64) -> [f64; 3] {
    [
        prev[0] + alpha * (new[0] - prev[0]),
        prev[1] + alpha * (new[1] - prev[1]),
        prev[2] + alpha * (new[2] - prev[2]),
    ]
}

impl VelocityEstimator {
    /// An estimator with smoothing `alpha` and maximum gap `max_gap` seconds.
    #[must_use]
    pub fn new(alpha: f64, max_gap: f64) -> Self {
        Self {
            alpha: alpha.clamp(1e-3, 1.0),
            max_gap,
            last: HashMap::new(),
        }
    }

    /// The smoothing factor and the longest differenced gap.
    #[must_use]
    pub fn settings(&self) -> (f64, f64) {
        (self.alpha, self.max_gap)
    }

    /// Forgets the history of body `id` (after a tracking loss).
    pub fn reset(&mut self, id: i32) {
        self.last.remove(&id);
    }

    /// Forgets every body.
    pub fn clear(&mut self) {
        self.last.clear();
    }

    /// Adds a pose of body `id` at time `t` (seconds, any monotonic
    /// origin) and returns the velocity once two samples are available.
    pub fn update(
        &mut self,
        id: i32,
        t: f64,
        position: [f64; 3],
        orientation: Quat,
    ) -> Option<Velocity> {
        let prev = self.last.get(&id).copied();
        let mut entry = Last {
            t,
            position,
            orientation,
            velocity: None,
        };
        let velocity = prev.and_then(|p| {
            let dt = t - p.t;
            if !(dt > 1e-6 && dt <= self.max_gap) {
                return None;
            }
            let linear_world = sub(position, p.position).map(|v| v / dt);
            // q = q_prev ⊗ Δ_body = Δ_world ⊗ q_prev
            let delta_world = quat::mul(orientation, quat::conj(p.orientation));
            let delta_body = quat::mul(quat::conj(p.orientation), orientation);
            let angular_world = quat::log(delta_world).map(|v| v / dt);
            let angular_body = quat::log(delta_body).map(|v| v / dt);
            let raw = Velocity {
                linear_world,
                angular_world,
                linear_body: quat::rotate(quat::conj(orientation), linear_world),
                angular_body,
            };
            Some(match p.velocity {
                Some(pv) => Velocity {
                    linear_world: lerp(pv.linear_world, raw.linear_world, self.alpha),
                    angular_world: lerp(pv.angular_world, raw.angular_world, self.alpha),
                    linear_body: lerp(pv.linear_body, raw.linear_body, self.alpha),
                    angular_body: lerp(pv.angular_body, raw.angular_body, self.alpha),
                },
                None => raw,
            })
        });
        // Keep the old sample if this one repeats its timestamp; a time that
        // goes backwards (replay loop, server restart) restarts the history.
        if prev.is_some_and(|p| t == p.t) {
            return None;
        }
        entry.velocity = velocity;
        self.last.insert(id, entry);
        velocity
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constant_motion() {
        let mut v = VelocityEstimator::new(1.0, 0.5);
        let w = 0.5; // rad/s about Z
        let mut out = None;
        for i in 0..5 {
            let t = f64::from(i) * 0.01;
            let q = quat::from_axis_angle([0.0, 0.0, 1.0], w * t);
            out = v.update(1, t, [t * 2.0, 0.0, 0.0], q);
        }
        let out = out.unwrap();
        assert!((out.linear_world[0] - 2.0).abs() < 1e-9);
        assert!((out.angular_world[2] - w).abs() < 1e-9);
        assert!((out.angular_body[2] - w).abs() < 1e-9);
        // Body frame: yawed by 0.02 rad, forward speed mostly along body x.
        let yaw: f64 = w * 0.04;
        assert!((out.linear_body[0] - 2.0 * yaw.cos()).abs() < 1e-9);
        assert!((out.linear_body[1] + 2.0 * yaw.sin()).abs() < 1e-9);
    }

    #[test]
    fn gaps_and_repeats_reset() {
        let mut v = VelocityEstimator::new(0.5, 0.1);
        assert!(v.update(1, 0.0, [0.0; 3], quat::IDENTITY).is_none());
        assert!(
            v.update(1, 1.0, [1.0, 0.0, 0.0], quat::IDENTITY).is_none(),
            "gap too long"
        );
        assert!(v.update(1, 1.05, [1.1, 0.0, 0.0], quat::IDENTITY).is_some());
        assert!(
            v.update(1, 1.05, [9.0, 0.0, 0.0], quat::IDENTITY).is_none(),
            "repeat"
        );
        let s = v.update(1, 1.1, [1.2, 0.0, 0.0], quat::IDENTITY).unwrap();
        assert!(
            (s.linear_world[0] - 2.0).abs() < 1e-9,
            "repeat did not corrupt history"
        );
        v.reset(1);
        assert!(v.update(1, 1.15, [1.3, 0.0, 0.0], quat::IDENTITY).is_none());
        // Time going backwards restarts the history instead of stalling it.
        assert!(v.update(1, 0.0, [0.0; 3], quat::IDENTITY).is_none());
        assert!(
            v.update(1, 0.01, [0.01, 0.0, 0.0], quat::IDENTITY)
                .is_some()
        );
    }
}
