//! Stream statistics reported through diagnostics.

use std::time::Instant;

use serde::Serialize;

/// Counters and rates of the incoming NatNet stream.
#[derive(Debug, Clone, Serialize, Default)]
pub struct StreamStats {
    /// Frames received.
    pub frames: u64,
    /// Frames missing according to the frame-number sequence.
    pub dropped_frames: u64,
    /// Datagrams that failed to decode.
    pub decode_errors: u64,
    /// Frames whose frame number went far backwards (server restart,
    /// replay loop).
    pub resets: u64,
    /// Frames that arrived after a newer one (network reordering); they are
    /// discarded and stay counted in `dropped_frames`.
    pub late_frames: u64,
    /// Frames that repeated the newest frame number (a paused server).
    pub duplicate_frames: u64,
    /// Smoothed frame rate, Hz.
    pub frame_rate: f64,
    /// Last server-reported latency, seconds.
    pub server_latency: Option<f64>,
    /// Last total latency estimate, seconds.
    pub total_latency: Option<f64>,
    /// Current clock offset estimate, seconds.
    pub clock_offset: Option<f64>,
    /// Rigid bodies in the last frame.
    pub rigid_bodies: usize,
    /// Tracked rigid bodies in the last frame.
    pub tracked_rigid_bodies: usize,
    /// Labeled markers in the last frame.
    pub labeled_markers: usize,
    /// Newest frame number.
    pub last_frame_number: Option<i32>,
    #[serde(skip)]
    last_frame_at: Option<Instant>,
    #[serde(skip)]
    late_run: u32,
}

/// How a frame number relates to the stream so far.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sequence {
    /// The first frame.
    First,
    /// Newer than every frame so far, possibly after a gap.
    Next,
    /// Repeats the newest frame number.
    Duplicate,
    /// Older than the newest frame: reordered in the network.
    Late,
    /// The stream restarted (far backwards, or backwards for too long).
    Restart,
}

/// Frames further behind the newest one than this mean a restart rather
/// than reordering. Reordering over Wi-Fi reaches a hundred-odd frames.
pub const REORDER_WINDOW: u32 = 1000;

/// A forward jump beyond this many frames (minutes of stream) is a
/// restart or a take switch, not loss.
pub const MAX_GAP: u32 = 100_000;

/// This many late frames in a row mean a restart (a replay looping a take
/// shorter than [`REORDER_WINDOW`]). Reordering produces short runs.
pub const MAX_LATE_RUN: u32 = 60;

impl StreamStats {
    /// Records a frame received at `now` and classifies it.
    pub fn record_frame(&mut self, frame_number: i32, now: Instant) -> Sequence {
        self.frames += 1;
        let sequence = match self.last_frame_number {
            None => Sequence::First,
            Some(last) => {
                // Frame numbers wrap as u32 bit patterns.
                let delta = (frame_number as u32).wrapping_sub(last as u32);
                let behind = (last as u32).wrapping_sub(frame_number as u32);
                if delta == 0 {
                    self.duplicate_frames += 1;
                    Sequence::Duplicate
                } else if delta <= MAX_GAP {
                    self.dropped_frames += u64::from(delta - 1);
                    Sequence::Next
                } else if delta <= u32::MAX / 2 {
                    self.resets += 1;
                    Sequence::Restart
                } else if behind <= REORDER_WINDOW && self.late_run < MAX_LATE_RUN {
                    self.late_frames += 1;
                    self.late_run += 1;
                    // Arrival times of late frames say nothing about the rate.
                    return Sequence::Late;
                } else {
                    self.resets += 1;
                    Sequence::Restart
                }
            }
        };
        self.late_run = 0;
        self.last_frame_number = Some(frame_number);
        if let Some(prev) = self.last_frame_at {
            let dt = now.duration_since(prev).as_secs_f64();
            if dt > 0.0 {
                let rate = 1.0 / dt;
                self.frame_rate = if self.frame_rate == 0.0 {
                    rate
                } else {
                    0.95 * self.frame_rate + 0.05 * rate
                };
            }
        }
        self.last_frame_at = Some(now);
        sequence
    }

    /// Frames that advanced the stream: neither late nor repeated.
    #[must_use]
    pub fn new_frames(&self) -> u64 {
        self.frames - self.late_frames - self.duplicate_frames
    }

    /// Seconds since the last frame.
    #[must_use]
    pub fn since_last_frame(&self, now: Instant) -> Option<f64> {
        self.last_frame_at
            .map(|t| now.duration_since(t).as_secs_f64())
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    #[test]
    fn drops_resets_and_rate() {
        let mut s = StreamStats::default();
        let t0 = Instant::now();
        assert_eq!(s.record_frame(10, t0), Sequence::First);
        s.record_frame(11, t0 + Duration::from_millis(10));
        assert_eq!(
            s.record_frame(14, t0 + Duration::from_millis(20)),
            Sequence::Next
        );
        assert_eq!(s.dropped_frames, 2);
        assert_eq!(
            s.record_frame(3, t0 + Duration::from_millis(30)),
            Sequence::Late
        );
        assert_eq!(
            s.record_frame(14, t0 + Duration::from_millis(30)),
            Sequence::Duplicate
        );
        assert_eq!(
            s.record_frame(-5000, t0 + Duration::from_millis(30)),
            Sequence::Restart
        );
        assert_eq!((s.resets, s.late_frames, s.duplicate_frames), (1, 1, 1));
        assert!((s.frame_rate - 100.0).abs() < 1e-6);
        assert_eq!(s.frames, 6);
        assert_eq!(s.new_frames(), 4);
    }

    #[test]
    fn frame_numbers_wrap() {
        let mut s = StreamStats::default();
        let t0 = Instant::now();
        s.record_frame(i32::MAX - 1, t0);
        assert_eq!(s.record_frame(i32::MIN + 1, t0), Sequence::Next);
        assert_eq!(s.dropped_frames, 2);
        assert_eq!(s.record_frame(i32::MAX, t0), Sequence::Late);
        // A jump far ahead restarts rather than counting billions of drops.
        assert_eq!(s.record_frame(1_000_000, t0), Sequence::Restart);
        assert_eq!(s.dropped_frames, 2);
    }

    #[test]
    fn long_late_runs_restart() {
        let mut s = StreamStats::default();
        let t0 = Instant::now();
        s.record_frame(500, t0);
        // A replay looping a 100-frame take: late at first, then a restart.
        let seq: Vec<Sequence> = (400..=(400 + MAX_LATE_RUN as i32))
            .map(|n| s.record_frame(n, t0))
            .collect();
        assert!(
            seq[..MAX_LATE_RUN as usize]
                .iter()
                .all(|q| *q == Sequence::Late)
        );
        assert_eq!(seq[MAX_LATE_RUN as usize], Sequence::Restart);
        assert_eq!(
            s.record_frame(400 + MAX_LATE_RUN as i32 + 1, t0),
            Sequence::Next
        );
    }
}
