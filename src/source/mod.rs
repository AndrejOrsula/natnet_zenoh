//! NatNet data sources: a live Motive server, a recorded capture, or a
//! synthetic generator. Each runs on its own thread and delivers
//! [`SourceEvent`]s over a bounded channel.

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
    /// Makes [`Self::send_frame`] wait for room instead of dropping (for
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
    /// Events from the source.
    pub events: Receiver<SourceEvent>,
    commands: Sender<SourceCommand>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
    dropped_reports: Arc<AtomicU64>,
    lossless: Arc<AtomicBool>,
    /// An event taken from the queue while looking past a frame.
    pending: Option<SourceEvent>,
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
        let (events_tx, events) = mpsc::sync_channel(capacity.max(1));
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
            thread: Some(thread),
        })
    }

    /// The next event, waiting up to `wait`. A frame with another frame
    /// queued behind it is stale: it is skipped and counted as dropped, so a
    /// consumer that fell behind catches up on the newest frame instead of
    /// working through the backlog. Other events keep their order, and
    /// lossless sources get every frame.
    ///
    /// # Errors
    /// [`RecvTimeoutError::Timeout`] when nothing arrived in time, and
    /// [`RecvTimeoutError::Disconnected`] once the source is gone.
    pub fn next_event(&mut self, wait: Duration) -> Result<SourceEvent, RecvTimeoutError> {
        let mut event = match self.pending.take() {
            Some(e) => e,
            None => self.events.recv_timeout(wait)?,
        };
        if self.lossless.load(Ordering::Relaxed) {
            return Ok(event);
        }
        while matches!(event, SourceEvent::Frame { .. }) {
            match self.events.try_recv() {
                Ok(next @ SourceEvent::Frame { .. }) => {
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
}
