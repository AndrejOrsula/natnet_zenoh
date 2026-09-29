//! `builtin_interfaces`: time and duration.

use std::time::{Duration as StdDuration, SystemTime, UNIX_EPOCH};

crate::ros_message! {
    /// A point in time (`builtin_interfaces/msg/Time`).
    pub struct Time("builtin_interfaces/msg/Time") {
        /// Seconds since the epoch.
        pub sec: i32,
        /// Nanoseconds within the second, `0..1_000_000_000`.
        pub nanosec: u32,
    }
}

crate::ros_message! {
    /// A signed time span (`builtin_interfaces/msg/Duration`).
    pub struct Duration("builtin_interfaces/msg/Duration") {
        /// Seconds.
        pub sec: i32,
        /// Nanoseconds within the second, `0..1_000_000_000`.
        pub nanosec: u32,
    }
}

impl Time {
    /// Converts nanoseconds since the Unix epoch, saturating at the `i32`
    /// seconds range.
    #[must_use]
    pub fn from_nanos(nanos: i64) -> Self {
        let sec = nanos.div_euclid(1_000_000_000);
        let nanosec = nanos.rem_euclid(1_000_000_000) as u32;
        match i32::try_from(sec) {
            Ok(sec) => Self { sec, nanosec },
            Err(_) if sec < 0 => Self {
                sec: i32::MIN,
                nanosec: 0,
            },
            Err(_) => Self {
                sec: i32::MAX,
                nanosec: 999_999_999,
            },
        }
    }

    /// Converts floating-point seconds since the Unix epoch.
    #[must_use]
    pub fn from_secs_f64(secs: f64) -> Self {
        Self::from_nanos((secs * 1e9).round() as i64)
    }

    /// Nanoseconds since the Unix epoch.
    #[must_use]
    pub fn as_nanos(&self) -> i64 {
        i64::from(self.sec) * 1_000_000_000 + i64::from(self.nanosec)
    }

    /// The current system time.
    #[must_use]
    pub fn now() -> Self {
        Self::from_system_time(SystemTime::now())
    }

    /// Converts a [`SystemTime`].
    #[must_use]
    pub fn from_system_time(t: SystemTime) -> Self {
        match t.duration_since(UNIX_EPOCH) {
            Ok(d) => Self::from_nanos(i64::try_from(d.as_nanos()).unwrap_or(i64::MAX)),
            Err(e) => Self::from_nanos(-i64::try_from(e.duration().as_nanos()).unwrap_or(i64::MAX)),
        }
    }
}

impl Duration {
    /// Converts a non-negative [`std::time::Duration`], saturating.
    #[must_use]
    pub fn from_std(d: StdDuration) -> Self {
        Self {
            sec: i32::try_from(d.as_secs()).unwrap_or(i32::MAX),
            nanosec: d.subsec_nanos(),
        }
    }

    /// Converts floating-point seconds (may be negative).
    #[must_use]
    pub fn from_secs_f64(secs: f64) -> Self {
        let t = Time::from_secs_f64(secs);
        Self {
            sec: t.sec,
            nanosec: t.nanosec,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::Message;

    #[test]
    fn hashes_match_jazzy() {
        assert_eq!(
            Time::type_hash(),
            "RIHS01_b106235e25a4c5ed35098aa0a61a3ee9c9b18d197f398b0e4206cea9acf9c197"
        );
        assert_eq!(
            Duration::type_hash(),
            "RIHS01_e8d009f659816f758b75334ee1a9ca5b5c0b859843261f14c7f937349599d93b"
        );
    }

    #[test]
    fn time_conversions() {
        let t = Time::from_nanos(-1);
        assert_eq!((t.sec, t.nanosec), (-1, 999_999_999));
        assert_eq!(t.as_nanos(), -1);
        let t = Time::from_secs_f64(1.5);
        assert_eq!((t.sec, t.nanosec), (1, 500_000_000));
        assert_eq!(Time::from_nanos(i64::MAX).sec, i32::MAX);
    }
}
