//! ROS 2 quality-of-service profiles and their `rmw_zenoh_cpp` encoding.
//!
//! `rmw_zenoh_cpp` embeds the (adapted) QoS of every publisher and
//! subscription in its liveliness token as
//! `<rel>:<dur>:<hist>,<depth>:<dl_s>,<dl_ns>:<ls_s>,<ls_ns>:<liv>,<lease_s>,<lease_ns>`,
//! where each component is left empty when it equals the rmw_zenoh default
//! (reliable, volatile, keep last 42, infinite durations, automatic
//! liveliness; Jazzy's 0.2.0 to 0.2.2 defaulted to transient local instead).
//! Tools such as `ros2 topic info -v` and QoS-compatibility checks read it
//! back from there.

use std::fmt;

/// Reliability policy (`rmw_qos_reliability_policy_e`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Reliability {
    /// Use the implementation default.
    SystemDefault = 0,
    /// Retransmit until delivered.
    Reliable = 1,
    /// Deliver at most once, may drop.
    BestEffort = 2,
}

/// Durability policy (`rmw_qos_durability_policy_e`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Durability {
    /// Use the implementation default.
    SystemDefault = 0,
    /// Late-joining subscribers receive the last `depth` samples.
    TransientLocal = 1,
    /// Only samples published after subscribing are delivered.
    Volatile = 2,
}

/// History policy (`rmw_qos_history_policy_e`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum History {
    /// Use the implementation default.
    SystemDefault = 0,
    /// Keep the last `depth` samples.
    KeepLast = 1,
    /// Keep every sample.
    KeepAll = 2,
}

/// Liveliness policy (`rmw_qos_liveliness_policy_e`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Liveliness {
    /// Use the implementation default.
    SystemDefault = 0,
    /// Asserted by the middleware.
    Automatic = 1,
    /// Asserted by the publisher.
    ManualByTopic = 3,
}

/// An `rmw_time_t`: unsigned seconds and nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RmwDuration {
    /// Seconds.
    pub sec: u64,
    /// Nanoseconds.
    pub nsec: u64,
}

impl RmwDuration {
    /// `RMW_DURATION_INFINITE`.
    pub const INFINITE: Self = Self {
        sec: 9_223_372_036,
        nsec: 854_775_807,
    };
    /// `RMW_DURATION_UNSPECIFIED`, the "use default" value.
    pub const UNSPECIFIED: Self = Self { sec: 0, nsec: 0 };
}

/// A QoS profile (`rmw_qos_profile_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct QosProfile {
    /// History policy.
    pub history: History,
    /// History depth, for [`History::KeepLast`].
    pub depth: usize,
    /// Reliability policy.
    pub reliability: Reliability,
    /// Durability policy.
    pub durability: Durability,
    /// Deadline.
    pub deadline: RmwDuration,
    /// Lifespan.
    pub lifespan: RmwDuration,
    /// Liveliness policy.
    pub liveliness: Liveliness,
    /// Liveliness lease duration.
    pub liveliness_lease_duration: RmwDuration,
}

impl Default for QosProfile {
    /// `rmw_qos_profile_default`: reliable, volatile, keep last 10.
    fn default() -> Self {
        Self::keep_last(10)
    }
}

impl QosProfile {
    /// The `rmw_zenoh_cpp` defaults, which replace system-default values
    /// and are the reference of the key expression encoding.
    pub const RMW_ZENOH_DEFAULT: Self = Self {
        history: History::KeepLast,
        depth: 42,
        reliability: Reliability::Reliable,
        durability: Durability::Volatile,
        deadline: RmwDuration::INFINITE,
        lifespan: RmwDuration::INFINITE,
        liveliness: Liveliness::Automatic,
        liveliness_lease_duration: RmwDuration::INFINITE,
    };

    /// The defaults of Jazzy's `rmw_zenoh_cpp` 0.2.0 to 0.2.2, which were
    /// transient local.
    pub const RMW_ZENOH_DEFAULT_BEFORE_0_2_3: Self = Self {
        durability: Durability::TransientLocal,
        ..Self::RMW_ZENOH_DEFAULT
    };

    /// Reliable, volatile, keep last `depth` (rclcpp's `QoS(depth)`).
    #[must_use]
    pub const fn keep_last(depth: usize) -> Self {
        Self {
            history: History::KeepLast,
            depth,
            reliability: Reliability::Reliable,
            durability: Durability::Volatile,
            deadline: RmwDuration::UNSPECIFIED,
            lifespan: RmwDuration::UNSPECIFIED,
            liveliness: Liveliness::SystemDefault,
            liveliness_lease_duration: RmwDuration::UNSPECIFIED,
        }
    }

    /// `rmw_qos_profile_sensor_data`: best effort, volatile, keep last 5.
    #[must_use]
    pub const fn sensor_data() -> Self {
        Self {
            reliability: Reliability::BestEffort,
            ..Self::keep_last(5)
        }
    }

    /// Reliable, transient local, keep last `depth`; the profile of latched
    /// topics such as `/tf_static` (depth 1) and `/robot_description`.
    #[must_use]
    pub const fn transient_local(depth: usize) -> Self {
        Self {
            durability: Durability::TransientLocal,
            ..Self::keep_last(depth)
        }
    }

    /// `rmw_qos_profile_services_default`: reliable, volatile, keep last 10.
    #[must_use]
    pub const fn services_default() -> Self {
        Self::keep_last(10)
    }

    /// Returns this profile with the given reliability.
    #[must_use]
    pub const fn with_reliability(mut self, reliability: Reliability) -> Self {
        self.reliability = reliability;
        self
    }

    /// Returns this profile with the given durability.
    #[must_use]
    pub const fn with_durability(mut self, durability: Durability) -> Self {
        self.durability = durability;
        self
    }

    /// Returns this profile with keep-last history of the given depth.
    #[must_use]
    pub const fn with_depth(mut self, depth: usize) -> Self {
        self.history = History::KeepLast;
        self.depth = depth;
        self
    }

    /// Replaces system-default values with `defaults` (see
    /// [`QosProfile::RMW_ZENOH_DEFAULT`]), like `rmw_zenoh_cpp`'s
    /// `QoS::best_available_qos` does before creating an entity.
    #[must_use]
    pub fn adapted(mut self, defaults: &Self) -> Self {
        let d = defaults;
        if self.history == History::SystemDefault {
            self.history = d.history;
        }
        if self.depth == 0 {
            self.depth = d.depth;
        }
        if self.reliability == Reliability::SystemDefault {
            self.reliability = d.reliability;
        }
        if self.durability == Durability::SystemDefault {
            self.durability = d.durability;
        }
        if self.deadline == RmwDuration::UNSPECIFIED {
            self.deadline = d.deadline;
        }
        if self.lifespan == RmwDuration::UNSPECIFIED {
            self.lifespan = d.lifespan;
        }
        if self.liveliness_lease_duration == RmwDuration::UNSPECIFIED {
            self.liveliness_lease_duration = d.liveliness_lease_duration;
        }
        if self.liveliness == Liveliness::SystemDefault {
            self.liveliness = d.liveliness;
        }
        self
    }

    /// The liveliness-token encoding (`qos_to_keyexpr`) against the
    /// release's `defaults`. Apply [`QosProfile::adapted`] first to match
    /// what `rmw_zenoh_cpp` emits.
    #[must_use]
    pub fn to_keyexpr(&self, defaults: &Self) -> String {
        fn part<T: PartialEq + fmt::Display + Copy>(out: &mut String, v: T, d: T) {
            if v != d {
                out.push_str(&v.to_string());
            }
        }
        let d = defaults;
        let mut s = String::with_capacity(32);
        part(&mut s, self.reliability as u8, d.reliability as u8);
        s.push(':');
        part(&mut s, self.durability as u8, d.durability as u8);
        s.push(':');
        part(&mut s, self.history as u8, d.history as u8);
        s.push(',');
        part(&mut s, self.depth, d.depth);
        s.push(':');
        part(&mut s, self.deadline.sec, d.deadline.sec);
        s.push(',');
        part(&mut s, self.deadline.nsec, d.deadline.nsec);
        s.push(':');
        part(&mut s, self.lifespan.sec, d.lifespan.sec);
        s.push(',');
        part(&mut s, self.lifespan.nsec, d.lifespan.nsec);
        s.push(':');
        part(&mut s, self.liveliness as u8, d.liveliness as u8);
        s.push(',');
        part(
            &mut s,
            self.liveliness_lease_duration.sec,
            d.liveliness_lease_duration.sec,
        );
        s.push(',');
        part(
            &mut s,
            self.liveliness_lease_duration.nsec,
            d.liveliness_lease_duration.nsec,
        );
        s
    }

    /// Parses the liveliness-token encoding (`keyexpr_to_qos`) against the
    /// release's `defaults`.
    #[must_use]
    pub fn from_keyexpr(s: &str, defaults: &Self) -> Option<Self> {
        fn num<T: std::str::FromStr>(s: &str, default: T) -> Option<T> {
            if s.is_empty() {
                Some(default)
            } else {
                s.parse().ok()
            }
        }
        let d = defaults;
        let parts: Vec<&str> = s.split(':').collect();
        if parts.len() < 6 {
            return None;
        }
        let hist: Vec<&str> = parts[2].split(',').collect();
        let deadline: Vec<&str> = parts[3].split(',').collect();
        let lifespan: Vec<&str> = parts[4].split(',').collect();
        let liv: Vec<&str> = parts[5].split(',').collect();
        if hist.len() < 2 || deadline.len() < 2 || lifespan.len() < 2 || liv.len() < 3 {
            return None;
        }
        let reliability = match num(parts[0], d.reliability as u8)? {
            0 => Reliability::SystemDefault,
            1 => Reliability::Reliable,
            2 => Reliability::BestEffort,
            _ => return None,
        };
        let durability = match num(parts[1], d.durability as u8)? {
            0 => Durability::SystemDefault,
            1 => Durability::TransientLocal,
            2 => Durability::Volatile,
            _ => return None,
        };
        let history = match num(hist[0], d.history as u8)? {
            0 => History::SystemDefault,
            1 => History::KeepLast,
            2 => History::KeepAll,
            _ => return None,
        };
        let liveliness = match num(liv[0], d.liveliness as u8)? {
            0 => Liveliness::SystemDefault,
            1 => Liveliness::Automatic,
            3 => Liveliness::ManualByTopic,
            _ => return None,
        };
        Some(Self {
            history,
            depth: num(hist[1], d.depth)?,
            reliability,
            durability,
            deadline: RmwDuration {
                sec: num(deadline[0], d.deadline.sec)?,
                nsec: num(deadline[1], d.deadline.nsec)?,
            },
            lifespan: RmwDuration {
                sec: num(lifespan[0], d.lifespan.sec)?,
                nsec: num(lifespan[1], d.lifespan.nsec)?,
            },
            liveliness,
            liveliness_lease_duration: RmwDuration {
                sec: num(liv[1], d.liveliness_lease_duration.sec)?,
                nsec: num(liv[2], d.liveliness_lease_duration.nsec)?,
            },
        })
    }

    /// The zenoh congestion control `rmw_zenoh_cpp` uses for publishers of
    /// this (adapted) profile: block only for reliable keep-all.
    #[must_use]
    pub fn congestion_control(&self) -> zenoh::qos::CongestionControl {
        if self.reliability == Reliability::Reliable && self.history == History::KeepAll {
            zenoh::qos::CongestionControl::Block
        } else {
            zenoh::qos::CongestionControl::Drop
        }
    }

    /// The zenoh reliability `rmw_zenoh_cpp` uses for publishers of this
    /// (adapted) profile.
    #[must_use]
    pub fn zenoh_reliability(&self) -> zenoh::qos::Reliability {
        if self.reliability == Reliability::BestEffort {
            zenoh::qos::Reliability::BestEffort
        } else {
            zenoh::qos::Reliability::Reliable
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const D: &QosProfile = &QosProfile::RMW_ZENOH_DEFAULT;
    const OLD: &QosProfile = &QosProfile::RMW_ZENOH_DEFAULT_BEFORE_0_2_3;

    #[test]
    fn rmw_zenoh_default_encodes_empty() {
        assert_eq!(D.to_keyexpr(D), "::,:,:,:,,");
        assert_eq!(OLD.to_keyexpr(OLD), "::,:,:,:,,");
    }

    #[test]
    fn rclcpp_default_encoding() {
        // rclcpp::QoS(10): reliable, volatile, keep last 10.
        let encode = |q: QosProfile, d| q.adapted(d).to_keyexpr(d);
        assert_eq!(encode(QosProfile::keep_last(10), D), "::,10:,:,:,,");
        assert_eq!(encode(QosProfile::sensor_data(), D), "2::,5:,:,:,,");
        assert_eq!(encode(QosProfile::transient_local(1), D), ":1:,1:,:,:,,");
        // Jazzy 0.2.0 to 0.2.2 left transient local implicit instead.
        assert_eq!(encode(QosProfile::keep_last(10), OLD), ":2:,10:,:,:,,");
        assert_eq!(encode(QosProfile::sensor_data(), OLD), "2:2:,5:,:,:,,");
        assert_eq!(encode(QosProfile::transient_local(1), OLD), "::,1:,:,:,,");
        // A system-default durability takes the release's default.
        let system = QosProfile::keep_last(1).with_durability(Durability::SystemDefault);
        assert_eq!(system.adapted(D).durability, Durability::Volatile);
        assert_eq!(system.adapted(OLD).durability, Durability::TransientLocal);
    }

    #[test]
    fn keyexpr_round_trip() {
        for d in [D, OLD] {
            for q in [
                QosProfile::keep_last(10),
                QosProfile::sensor_data(),
                QosProfile::transient_local(1),
                QosProfile {
                    history: History::KeepAll,
                    deadline: RmwDuration { sec: 1, nsec: 5 },
                    liveliness: Liveliness::ManualByTopic,
                    ..QosProfile::keep_last(3)
                },
            ] {
                let q = q.adapted(d);
                assert_eq!(QosProfile::from_keyexpr(&q.to_keyexpr(d), d), Some(q));
            }
        }
        assert_eq!(QosProfile::from_keyexpr("garbage", D), None);
        assert_eq!(QosProfile::from_keyexpr("9::,:,:,:,,", D), None);
    }
}
