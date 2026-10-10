//! NatNet data sources: a live Motive server, a recorded capture, or a
//! synthetic generator. Each runs on its own thread and delivers
//! [`SourceEvent`]s over a bounded channel.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, SyncSender, TrySendError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime};

use natnet::{FrameOfData, ModelDef, ServerInfo, Version};

pub mod capture;
pub mod dispatch;
pub mod live;
pub mod replay;
pub mod server;
pub mod synthetic;

pub use natnet::NAT_KEEPALIVE;

/// Connection state changes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionState {
    /// Frames are arriving.
    Streaming,
    /// No frame arrived within the timeout.
    Lost,
    /// A replay reached its end.
    Finished,
}

/// Something that happened at the source.
#[derive(Debug, Clone)]
pub enum SourceEvent {
    /// A SERVERINFO reply (or the configured equivalent for passive and
    /// replay sources).
    ServerInfo(ServerInfo),
    /// A model definition, with the protocol version it was decoded with.
    ModelDef(ModelDef, Version),
    /// A frame of data.
    Frame {
        /// The decoded frame.
        frame: Box<FrameOfData>,
        /// Wall-clock reception time.
        received: SystemTime,
        /// Monotonic reception time.
        received_mono: Instant,
    },
    /// A NAT_MESSAGESTRING text from the server.
    ServerMessage(String),
    /// A datagram that failed to decode, or another error the source hit: a
    /// capture it cannot read, a decoder it cannot set up, or a multicast
    /// port it cannot bind or join.
    DecodeError(String),
    /// A connection state change.
    Connection(ConnectionState),
}

/// Requests from the bridge to the source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceCommand {
    /// Request the model definitions again.
    RefreshModel,
}

/// Shared plumbing handed to a source thread.
#[derive(Debug, Clone)]
pub struct SourceContext {
    events: SyncSender<SourceEvent>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
    dropped_reports: Arc<AtomicU64>,
    lossless: Arc<AtomicBool>,
}

impl SourceContext {
    /// Makes [`Self::send_frame`] and [`Self::report`] wait for room instead
    /// of dropping, and [`SourceHandle::next_event`] skip no frame (for
    /// unpaced replays, where the producer is faster than any consumer).
    pub fn set_lossless(&self, lossless: bool) {
        self.lossless.store(lossless, Ordering::Relaxed);
    }

    /// Whether the source should stop.
    #[must_use]
    pub fn stopped(&self) -> bool {
        self.stop.load(Ordering::Relaxed)
    }

    /// Delivers a control event, waiting for room; returns false once the
    /// receiver is gone.
    pub fn send(&self, event: SourceEvent) -> bool {
        self.events.send(event).is_ok()
    }

    /// Delivers a diagnostic event (a decode error) without blocking unless
    /// lossless: a full queue drops it and counts it, since a stream of bad
    /// datagrams must not stall the receiver. Returns false once the
    /// receiver is gone.
    pub fn report(&self, event: SourceEvent) -> bool {
        if self.lossless.load(Ordering::Relaxed) {
            return self.events.send(event).is_ok();
        }
        match self.events.try_send(event) {
            Ok(()) => true,
            Err(TrySendError::Full(_)) => {
                self.dropped_reports.fetch_add(1, Ordering::Relaxed);
                true
            }
            Err(TrySendError::Disconnected(_)) => false,
        }
    }

    /// Delivers a frame without blocking; a full queue drops the frame
    /// (the consumer is behind) and counts it. Returns false once the
    /// receiver is gone.
    pub fn send_frame(
        &self,
        frame: Box<FrameOfData>,
        received: SystemTime,
        received_mono: Instant,
    ) -> bool {
        let event = SourceEvent::Frame {
            frame,
            received,
            received_mono,
        };
        if self.lossless.load(Ordering::Relaxed) {
            return self.events.send(event).is_ok();
        }
        match self.events.try_send(event) {
            Ok(()) => true,
            Err(TrySendError::Full(_)) => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
                true
            }
            Err(TrySendError::Disconnected(_)) => false,
        }
    }
}

/// A running source.
#[derive(Debug)]
pub struct SourceHandle {
    /// Events from the source. Read directly, the queue skips no frame, and
    /// only frames that found it full count in [`Self::dropped_frames`];
    /// [`Self::next_event`] also skips stale frames. Use one or the other:
    /// an event `next_event` took from the queue while looking past a frame
    /// waits in the handle.
    pub events: Receiver<SourceEvent>,
    commands: Sender<SourceCommand>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
    dropped_reports: Arc<AtomicU64>,
    lossless: Arc<AtomicBool>,
    /// An event taken from the queue while looking past a frame.
    pending: Option<SourceEvent>,
    /// When each of the last `calls_kept` calls to `next_event` began and
    /// returned, oldest first.
    calls: VecDeque<(Instant, Instant)>,
    /// The queue's capacity plus two. A frame queued as it is received has
    /// at most `capacity` events ahead of it, `pending` included, so one of
    /// the next `capacity + 1` calls takes it while the calls around its
    /// receipt are still remembered.
    calls_kept: usize,
    /// Whether calls were forgotten to keep `calls` within `calls_kept`.
    forgot_calls: bool,
    thread: Option<JoinHandle<()>>,
}

impl SourceHandle {
    /// Spawns `run` on a named thread with a queue of `capacity` events.
    ///
    /// # Errors
    /// The OS error when the thread cannot be spawned.
    pub fn spawn<F>(name: &str, capacity: usize, run: F) -> std::io::Result<Self>
    where
        F: FnOnce(SourceContext, Receiver<SourceCommand>) + Send + 'static,
    {
        let capacity = capacity.max(1);
        let (events_tx, events) = mpsc::sync_channel(capacity);
        let (commands, commands_rx) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let dropped = Arc::new(AtomicU64::new(0));
        let dropped_reports = Arc::new(AtomicU64::new(0));
        let lossless = Arc::new(AtomicBool::new(false));
        let ctx = SourceContext {
            events: events_tx,
            stop: Arc::clone(&stop),
            dropped: Arc::clone(&dropped),
            dropped_reports: Arc::clone(&dropped_reports),
            lossless: Arc::clone(&lossless),
        };
        let thread = std::thread::Builder::new()
            .name(name.to_owned())
            .spawn(move || run(ctx, commands_rx))?;
        Ok(Self {
            events,
            commands,
            stop,
            dropped,
            dropped_reports,
            lossless,
            pending: None,
            calls: VecDeque::new(),
            calls_kept: capacity.saturating_add(2),
            forgot_calls: false,
            thread: Some(thread),
        })
    }

    /// The next event, waiting up to `wait`. A queued frame that was
    /// received while no call was in progress is stale when a newer frame
    /// had also been received by the time this call began: it is skipped
    /// and counted as dropped, so a consumer that fell behind catches up on
    /// the newest frame instead of working through the backlog. A queued
    /// frame received during a call, such as one of two that arrive
    /// microseconds apart while the consumer waits, is never skipped: this
    /// call or a later one returns it, however old it is by then. The
    /// handle remembers its last `capacity + 2` calls; once it has
    /// forgotten older ones, a frame received before the oldest it
    /// remembers is not skipped either, since it may have been received
    /// during a forgotten call. Other events keep their order, and lossless
    /// sources get every frame.
    ///
    /// # Errors
    /// [`RecvTimeoutError::Timeout`] when nothing arrived in time, and
    /// [`RecvTimeoutError::Disconnected`] once the source is gone.
    pub fn next_event(&mut self, wait: Duration) -> Result<SourceEvent, RecvTimeoutError> {
        self.next_event_at(wait, Instant::now)
    }

    /// [`Self::next_event`], reading the time from `now` when the call
    /// begins and when it returns.
    fn next_event_at(
        &mut self,
        wait: Duration,
        mut now: impl FnMut() -> Instant,
    ) -> Result<SourceEvent, RecvTimeoutError> {
        let began = now();
        let next = self.take(began, wait);
        if self.calls.len() >= self.calls_kept {
            self.calls.pop_front();
            self.forgot_calls = true;
        }
        self.calls.push_back((began, now()));
        next
    }

    /// The next event for a call that began at `began`.
    fn take(&mut self, began: Instant, wait: Duration) -> Result<SourceEvent, RecvTimeoutError> {
        let mut event = match self.pending.take() {
            Some(e) => e,
            None => self.events.recv_timeout(wait)?,
        };
        if self.lossless.load(Ordering::Relaxed) {
            return Ok(event);
        }
        while let SourceEvent::Frame { received_mono, .. } = &event {
            if *received_mono > began || self.maybe_received_during_a_call(*received_mono) {
                break;
            }
            match self.events.try_recv() {
                Ok(next @ SourceEvent::Frame { received_mono, .. }) if received_mono <= began => {
                    self.dropped.fetch_add(1, Ordering::Relaxed);
                    event = next;
                }
                Ok(other) => {
                    self.pending = Some(other);
                    break;
                }
                Err(_) => break,
            }
        }
        Ok(event)
    }

    /// Whether `at` falls within one of the remembered calls, or may fall
    /// within a forgotten one: before the oldest remembered call, once
    /// older calls were forgotten.
    fn maybe_received_during_a_call(&self, at: Instant) -> bool {
        // The calls follow one another, so their ends are in order.
        let i = self.calls.partition_point(|&(_, returned)| returned < at);
        match self.calls.get(i) {
            Some(&(began, _)) if began < at => true,
            Some(_) => i == 0 && self.forgot_calls,
            None => false,
        }
    }

    /// Sends a command to the source.
    pub fn command(&self, command: SourceCommand) {
        let _ = self.commands.send(command);
    }

    /// A sender for commands, usable from other threads.
    #[must_use]
    pub fn commander(&self) -> Sender<SourceCommand> {
        self.commands.clone()
    }

    /// Frames dropped because the consumer fell behind.
    #[must_use]
    pub fn dropped_frames(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Decode errors dropped on a full queue since the last call.
    #[must_use]
    pub fn take_dropped_reports(&self) -> u64 {
        self.dropped_reports.swap(0, Ordering::Relaxed)
    }

    /// Asks the source to stop and waits for its thread. Pending events are
    /// discarded so a source blocked on a full queue can observe the stop.
    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            while !t.is_finished() {
                while self.events.try_recv().is_ok() {}
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
            let _ = t.join();
        }
    }
}

impl Drop for SourceHandle {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::synthetic::{Scene, SyntheticConfig};

    /// Numbers of the frames and a marker for each server info that
    /// `next_event` hands out, until the queue is empty.
    fn drain(h: &mut SourceHandle) -> Vec<i32> {
        let mut seen = Vec::new();
        while let Ok(event) = h.next_event(Duration::from_millis(200)) {
            match event {
                SourceEvent::Frame { frame, .. } => seen.push(frame.frame_number),
                SourceEvent::ServerInfo(_) => seen.push(-1),
                _ => {}
            }
        }
        seen
    }

    fn backlog(lossless: bool) -> SourceHandle {
        let (queued, done) = mpsc::channel();
        let h = SourceHandle::spawn("backlog", 16, move |ctx, _| {
            ctx.set_lossless(lossless);
            let scene = Scene::new(SyntheticConfig::default()).unwrap();
            let now = (SystemTime::now(), Instant::now());
            for n in [1, 2] {
                ctx.send_frame(Box::new(scene.frame(n)), now.0, now.1);
            }
            ctx.send(SourceEvent::ServerInfo(scene.server_info(false, 0, [0; 4])));
            for n in [3, 4, 5] {
                ctx.send_frame(Box::new(scene.frame(n)), now.0, now.1);
            }
            queued.send(()).unwrap();
        })
        .unwrap();
        done.recv().unwrap();
        h
    }

    #[test]
    fn a_backlog_of_frames_is_skipped_to_the_newest() {
        let mut h = backlog(false);
        // The server info stays between the frames it separates.
        assert_eq!(drain(&mut h), [2, -1, 5]);
        assert_eq!(h.dropped_frames(), 3);
    }

    #[test]
    fn lossless_sources_get_every_frame() {
        let mut h = backlog(true);
        assert_eq!(drain(&mut h), [1, 2, -1, 3, 4, 5]);
        assert_eq!(h.dropped_frames(), 0);
    }

    /// Queues frames 1, 2 and 3: the first `before` stamped when they are
    /// sent, the others an hour ahead. A stamp in the future stands for a
    /// frame that arrived while the consumer waited.
    fn stamped(before: usize) -> SourceHandle {
        let (queued, done) = mpsc::channel();
        let h = SourceHandle::spawn("stamped", 16, move |ctx, _| {
            let scene = Scene::new(SyntheticConfig::default()).unwrap();
            let now = Instant::now();
            let later = now + Duration::from_secs(3600);
            for (i, n) in [1, 2, 3].into_iter().enumerate() {
                let at = if i < before { now } else { later };
                ctx.send_frame(Box::new(scene.frame(n)), SystemTime::now(), at);
            }
            queued.send(()).unwrap();
        })
        .unwrap();
        done.recv().unwrap();
        h
    }

    #[test]
    fn frames_received_during_the_call_are_all_delivered() {
        let mut h = stamped(0);
        assert_eq!(drain(&mut h), [1, 2, 3]);
        assert_eq!(h.dropped_frames(), 0);
    }

    #[test]
    fn only_frames_queued_before_the_call_are_skipped() {
        let mut h = stamped(2);
        assert_eq!(drain(&mut h), [2, 3]);
        assert_eq!(h.dropped_frames(), 1);
    }

    /// Queues frames 1, 2, … stamped as received the given numbers of
    /// milliseconds after `t0`.
    fn received_at(t0: Instant, ms: &'static [u64]) -> SourceHandle {
        let (queued, done) = mpsc::channel();
        let h = SourceHandle::spawn("received-at", 16, move |ctx, _| {
            let scene = Scene::new(SyntheticConfig::default()).unwrap();
            for (n, &ms) in (1..).zip(ms) {
                let at = t0 + Duration::from_millis(ms);
                ctx.send_frame(Box::new(scene.frame(n)), SystemTime::now(), at);
            }
            queued.send(()).unwrap();
        })
        .unwrap();
        done.recv().unwrap();
        h
    }

    /// Numbers of the frames handed out by one call per span, each call
    /// beginning and returning the span's numbers of milliseconds after
    /// `t0`.
    fn calls_at(h: &mut SourceHandle, t0: Instant, spans: &[(u64, u64)]) -> Vec<i32> {
        let mut seen = Vec::new();
        for &(began, returned) in spans {
            let mut clock = [began, returned]
                .into_iter()
                .map(|ms| t0 + Duration::from_millis(ms));
            let next = h.next_event_at(Duration::from_millis(50), || clock.next().unwrap());
            if let Ok(SourceEvent::Frame { frame, .. }) = next {
                seen.push(frame.frame_number);
            }
        }
        seen
    }

    #[test]
    fn frames_received_during_a_call_are_not_skipped_later() {
        // Frames 1 to 3 are received during the first call, which returns
        // frame 1. They are all old by the time of the next calls.
        let t0 = Instant::now();
        let mut h = received_at(t0, &[1000, 1000, 1000]);
        let spans = [
            (0, 2000),
            (10_000, 10_000),
            (11_000, 11_000),
            (12_000, 12_000),
        ];
        assert_eq!(calls_at(&mut h, t0, &spans), [1, 2, 3]);
        assert_eq!(h.dropped_frames(), 0);
    }

    #[test]
    fn frames_received_between_calls_are_still_skipped() {
        // As above, then frames 4 and 5 are received after the first call
        // returned: once 2 and 3 are delivered, 4 is skipped for 5.
        let t0 = Instant::now();
        let mut h = received_at(t0, &[1000, 1000, 1000, 5000, 5000]);
        let spans = [
            (0, 2000),
            (10_000, 10_000),
            (11_000, 11_000),
            (12_000, 12_000),
            (13_000, 13_000),
        ];
        assert_eq!(calls_at(&mut h, t0, &spans), [1, 2, 3, 5]);
        assert_eq!(h.dropped_frames(), 1);
    }

    #[test]
    fn frames_received_during_a_forgotten_call_are_not_skipped() {
        // With a queue of 2 the handle remembers 4 calls. Frames 2 and 3 are
        // received during the first call but queued only after four more,
        // when the first is forgotten. Frames 4 and 5 are received after
        // the seventh call returned: 4 is skipped for 5.
        let t0 = Instant::now();
        let (go, batches) = mpsc::channel::<&'static [(i32, u64)]>();
        let (queued, done) = mpsc::channel();
        let mut h = SourceHandle::spawn("late", 2, move |ctx, _| {
            let scene = Scene::new(SyntheticConfig::default()).unwrap();
            for batch in batches.iter().take(3) {
                for &(n, ms) in batch {
                    let at = t0 + Duration::from_millis(ms);
                    ctx.send_frame(Box::new(scene.frame(n)), SystemTime::now(), at);
                }
                queued.send(()).unwrap();
            }
        })
        .unwrap();
        let send = |batch: &'static [(i32, u64)]| {
            go.send(batch).unwrap();
            done.recv().unwrap();
        };
        send(&[(1, 1000)]);
        let spans = [
            (0, 2000),
            (10_000, 10_000),
            (11_000, 11_000),
            (12_000, 12_000),
            (13_000, 13_000),
        ];
        let mut seen = calls_at(&mut h, t0, &spans);
        send(&[(2, 1500), (3, 1600)]);
        seen.extend(calls_at(&mut h, t0, &[(14_000, 14_000), (15_000, 15_000)]));
        send(&[(4, 16_000), (5, 16_000)]);
        seen.extend(calls_at(&mut h, t0, &[(17_000, 17_000)]));
        assert_eq!(seen, [1, 2, 3, 5]);
        assert_eq!(h.dropped_frames(), 1);
    }
}
