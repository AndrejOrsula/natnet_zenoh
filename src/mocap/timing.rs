//! Message timestamps.
//!
//! NatNet frames carry camera-clock ticks (mid-exposure, data received,
//! transmit) on the server's high-resolution clock, which is not the ROS
//! clock. Three policies map a frame to a ROS time:
//!
//! * `receive`: host time when the datagram arrived. Simple, but includes
//!   the whole pipeline and network latency and its jitter.
//! * `latency_compensated`: receive time minus the server-reported
//!   pipeline latency `(transmit − mid_exposure) / frequency`. Removes the
//!   camera and solver latency; network latency and jitter remain.
//! * `clock_sync` (default): estimates the offset between the server clock
//!   and the host clock as the running minimum of `receive − transmit` over a
//!   sliding window (the minimum-delay envelope: the least-delayed datagram
//!   bounds the offset most tightly), then stamps `mid_exposure + offset`.
//!   The result is the mid-exposure instant on the host clock, free of
//!   network jitter; it degrades gracefully to `latency_compensated` until
//!   the window has samples.

use std::collections::VecDeque;
use std::time::SystemTime;

use natnet::FrameTiming;
use serde::{Deserialize, Serialize};

use crate::ros::msgs::builtin_interfaces::Time;

/// How frames are stamped.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StampPolicy {
    /// Host time at reception.
    Receive,
    /// Reception time minus the server-reported latency.
    LatencyCompensated,
    /// Mid-exposure time mapped through an estimated clock offset.
    #[default]
    ClockSync,
}

fn secs_since_epoch(t: SystemTime) -> f64 {
    match t.duration_since(SystemTime::UNIX_EPOCH) {
        Ok(d) => d.as_secs_f64(),
        Err(e) => -e.duration().as_secs_f64(),
    }
}

/// A sliding-window minimum of `(time, value)` samples.
#[derive(Debug, Clone)]
struct SlidingMin {
    window: f64,
    samples: VecDeque<(f64, f64)>,
}

impl SlidingMin {
    fn push(&mut self, t: f64, v: f64) {
        while self.samples.back().is_some_and(|&(_, b)| b >= v) {
            self.samples.pop_back();
        }
        self.samples.push_back((t, v));
        while self
            .samples
            .front()
            .is_some_and(|&(ft, _)| t - ft > self.window)
        {
            self.samples.pop_front();
        }
    }

    fn min(&self) -> Option<f64> {
        self.samples.front().map(|&(_, v)| v)
    }

    fn clear(&mut self) {
        self.samples.clear();
    }
}

/// Turns frame timing into ROS stamps.
#[derive(Debug, Clone)]
pub struct Stamper {
    policy: StampPolicy,
    frequency: u64,
    offset: SlidingMin,
    last_transmit: Option<u64>,
    /// The clock offset applied to the last frame, and that frame's
    /// mid-exposure (server clock, seconds).
    applied: Option<(f64, f64)>,
    /// The last stamp, nanoseconds.
    last_sync: Option<i64>,
}

/// How fast a lowered clock-offset estimate is taken in: at most this
/// fraction of the server time elapsed between frames, so clock-synchronized
/// stamps keep advancing at 95 % of real time or more while they converge.
const SLEW: f64 = 0.05;

/// The outcome of stamping one frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Stamp {
    /// The ROS time for the frame's messages.
    pub time: Time,
    /// Server-reported latency from mid-exposure to transmit, seconds.
    pub server_latency: Option<f64>,
    /// Current clock-offset estimate (host − server), seconds.
    pub clock_offset: Option<f64>,
    /// Estimated age of the data at reception (receive − mid-exposure on
    /// the host clock), seconds; needs a clock-offset estimate.
    pub total_latency: Option<f64>,
}

impl Stamper {
    /// A stamper for a server clock of `frequency` ticks per second; the
    /// clock-sync window is `window_secs` long.
    #[must_use]
    pub fn new(policy: StampPolicy, frequency: u64, window_secs: f64) -> Self {
        Self {
            policy,
            frequency,
            offset: SlidingMin {
                window: window_secs.max(0.0),
                samples: VecDeque::new(),
            },
            last_transmit: None,
            applied: None,
            last_sync: None,
        }
    }

    /// Keeps stamps from going backwards across a switch from `previous`,
    /// a stamper with other settings.
    pub fn continue_from(&mut self, previous: &Self) {
        self.last_sync = previous.last_sync;
    }

    /// Updates the server clock frequency (after a new SERVERINFO).
    pub fn set_frequency(&mut self, frequency: u64) {
        if frequency != self.frequency {
            self.frequency = frequency;
            self.reset();
        }
    }

    /// Forgets the clock-offset estimate.
    pub fn reset(&mut self) {
        self.offset.clear();
        self.last_transmit = None;
        self.applied = None;
        self.last_sync = None;
    }

    /// The offset to apply to a frame of mid-exposure `mid`: a raised
    /// estimate at once, a lowered one gradually, a change of a second or
    /// more (a clock step) at once.
    fn slew(&mut self, mid: f64, target: f64) -> f64 {
        let applied = match self.applied {
            Some((a, last_mid)) if target < a && a - target < 1.0 => {
                (a - SLEW * (mid - last_mid).max(0.0)).max(target)
            }
            _ => target,
        };
        self.applied = Some((applied, mid));
        applied
    }

    fn ticks(&self, t: u64) -> Option<f64> {
        (self.frequency > 0 && t > 0).then(|| t as f64 / self.frequency as f64)
    }

    /// The server-clock time of the frame's mid-exposure, seconds, when the
    /// frame carries camera ticks.
    #[must_use]
    pub fn mid_exposure_secs(&self, timing: &FrameTiming) -> Option<f64> {
        self.ticks(timing.camera_mid_exposure)
    }

    /// Stamps one frame received at `received`.
    pub fn stamp(&mut self, timing: &FrameTiming, received: SystemTime) -> Stamp {
        let recv = secs_since_epoch(received);
        let mid = self.ticks(timing.camera_mid_exposure);
        let transmit = self.ticks(timing.transmit);
        let server_latency = match (mid, transmit) {
            (Some(m), Some(t)) if t >= m && t - m < 10.0 => Some(t - m),
            _ => None,
        };

        if let (Some(tx), Some(last)) = (Some(timing.transmit), self.last_transmit)
            && tx < last
        {
            // The server clock went backwards (Motive restarted).
            self.offset.clear();
            self.applied = None;
        }
        if timing.transmit > 0 {
            self.last_transmit = Some(timing.transmit);
        }
        if let Some(t) = transmit {
            self.offset.push(recv, recv - t);
        }
        let clock_offset = self.offset.min();
        let total_latency = match (mid, clock_offset) {
            (Some(m), Some(o)) => Some(recv - (m + o)),
            _ => None,
        };

        let compensated = recv - server_latency.unwrap_or(0.0);
        let secs = match self.policy {
            StampPolicy::Receive => recv,
            StampPolicy::LatencyCompensated => compensated,
            StampPolicy::ClockSync => match (mid, clock_offset) {
                (Some(m), Some(o)) => m + self.slew(m, o),
                _ => compensated,
            },
        };
        let mut time = Time::from_secs_f64(secs);
        // Stamps must never go backwards, or tf2 drops the frames as old
        // data: slewing covers clock synchronization, this the frames
        // without camera ticks and a switch of settings. A jump of a second
        // or more is a clock change and is taken as is.
        let nanos = time.as_nanos();
        if self.policy == StampPolicy::ClockSync
            && let Some(last) = self.last_sync
            && nanos <= last
            && last - nanos < 1_000_000_000
        {
            time = Time::from_nanos(last + 1);
        }
        self.last_sync = Some(time.as_nanos());
        Stamp {
            time,
            server_latency,
            clock_offset,
            total_latency,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    const F: u64 = 10_000_000;

    fn timing(mid: f64, transmit: f64) -> FrameTiming {
        FrameTiming {
            timecode: 0,
            timecode_subframe: 0,
            timestamp: mid,
            camera_mid_exposure: (mid * F as f64) as u64,
            camera_data_received: (mid * F as f64) as u64,
            transmit: (transmit * F as f64) as u64,
            precision_timestamp_secs: 0,
            precision_timestamp_fractional_secs: 0,
        }
    }

    fn at(secs: f64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_secs_f64(secs)
    }

    #[test]
    fn policies() {
        // Server clock = host clock − 1000 s; pipeline 5 ms; network 1-3 ms.
        let host0 = 1_700_000_000.0;
        let mut receive = Stamper::new(StampPolicy::Receive, F, 10.0);
        let mut comp = Stamper::new(StampPolicy::LatencyCompensated, F, 10.0);
        let mut sync = Stamper::new(StampPolicy::ClockSync, F, 10.0);
        let mut last = None;
        for i in 0..100 {
            let net = [0.003, 0.001, 0.002, 0.003][i % 4];
            let mid = 10.0 + i as f64 * 0.01;
            let t = timing(mid, mid + 0.005);
            let recv = at(host0 + mid - 1000.0 + 0.005 + net);
            let r = receive.stamp(&t, recv);
            let c = comp.stamp(&t, recv);
            let s = sync.stamp(&t, recv);
            let truth = host0 + mid - 1000.0;
            assert!((r.time.as_nanos() as f64 / 1e9 - (truth + 0.005 + net)).abs() < 1e-6);
            assert!((c.time.as_nanos() as f64 / 1e9 - (truth + net)).abs() < 1e-6);
            assert!((c.server_latency.unwrap() - 0.005).abs() < 1e-6);
            last = Some((s, truth));
        }
        // Once it has taken in the 1 ms datagram (gradually), clock sync is
        // within 1 ms and jitter-free.
        let (s, truth) = last.unwrap();
        assert!((s.time.as_nanos() as f64 / 1e9 - (truth + 0.001)).abs() < 1e-6);
        assert!((s.total_latency.unwrap() - 0.007).abs() < 1e-6);
    }

    #[test]
    fn falls_back_without_ticks_and_resets_on_restart() {
        let mut s = Stamper::new(StampPolicy::ClockSync, 0, 10.0);
        let st = s.stamp(&timing(1.0, 1.0), at(100.0));
        assert_eq!(st.time, Time::from_secs_f64(100.0));
        assert!(st.clock_offset.is_none());

        let mut s = Stamper::new(StampPolicy::ClockSync, F, 10.0);
        s.stamp(&timing(50.0, 50.0), at(100.0));
        assert!(
            (s.stamp(&timing(50.1, 50.1), at(100.1))
                .clock_offset
                .unwrap()
                - 50.0)
                .abs()
                < 1e-6
        );
        // Restart: ticks go back to ~0; the stale offset must not survive.
        let st = s.stamp(&timing(1.0, 1.0), at(101.0));
        assert!((st.clock_offset.unwrap() - 100.0).abs() < 1e-6);
    }

    #[test]
    fn clock_sync_stamps_slew_instead_of_going_backwards() {
        // Frames 5 ms apart; the first datagram was delayed 20 ms, the
        // others only 1 ms, which lowers the offset estimate by 19 ms.
        let mut s = Stamper::new(StampPolicy::ClockSync, F, 10.0);
        let mut last = s.stamp(&timing(10.0, 10.0), at(100.02)).time.as_nanos();
        for i in 1..=200 {
            let mid = 10.0 + 0.005 * f64::from(i);
            let now = s.stamp(&timing(mid, mid), at(90.001 + mid)).time.as_nanos();
            // Always forward, at 95 % of the frame spacing or more (less
            // the 100 ns tick truncation).
            assert!(now - last >= 4_749_800, "frame {i}: {}", now - last);
            last = now;
        }
        // After a second the stamps follow the new estimate.
        assert!((last as f64 / 1e9 - (90.001 + 11.0)).abs() < 1e-6);
    }

    #[test]
    fn a_switch_of_settings_does_not_go_backwards() {
        let mut receive = Stamper::new(StampPolicy::Receive, F, 10.0);
        let before = receive.stamp(&timing(10.0, 10.0), at(100.02)).time;
        let mut sync = Stamper::new(StampPolicy::ClockSync, F, 10.0);
        sync.continue_from(&receive);
        let after = sync.stamp(&timing(10.005, 10.005), at(100.006)).time;
        assert!(after.as_nanos() > before.as_nanos());
    }
}
