//! Forwards `tracing` events to `/rosout` (`rcl_interfaces/msg/Log`), so
//! `ros2 topic echo /rosout` and rqt_console show the bridge's log.
//!
//! The layer never publishes on the logging thread: events are queued to a
//! worker that owns the publisher, which avoids re-entering zenoh (whose own
//! `tracing` events are ignored) and never blocks the caller; a full queue
//! drops the event. The worker, and with it the `/rosout` publisher, lives
//! as long as the [`RosoutWorker`] returned by [`RosoutHandle::attach`].

use std::fmt::Write as _;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;
use std::time::Duration;

use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_subscriber::layer::{Context as LayerContext, Layer};

use super::msgs::builtin_interfaces::Time;
use super::msgs::rcl_interfaces::Log;
use super::{Node, QosProfile};

/// `/rosout` QoS as rcl declares it: reliable, transient local, depth 1000,
/// lifespan 10 s.
#[must_use]
pub fn rosout_qos() -> QosProfile {
    let mut q = QosProfile::transient_local(1000);
    q.lifespan = super::qos::RmwDuration { sec: 10, nsec: 0 };
    q
}

fn level(l: Level) -> u8 {
    match l {
        Level::ERROR => Log::ERROR,
        Level::WARN => Log::WARN,
        Level::INFO => Log::INFO,
        Level::DEBUG | Level::TRACE => Log::DEBUG,
    }
}

/// A `tracing` layer feeding `/rosout`.
#[derive(Debug, Clone)]
pub struct RosoutLayer {
    tx: SyncSender<Log>,
    min_level: Level,
    targets: Arc<[String]>,
    name: Arc<Mutex<String>>,
}

/// The receiving end; attach it to a node to start publishing.
#[derive(Debug)]
pub struct RosoutHandle {
    rx: Arc<Mutex<Option<Receiver<Log>>>>,
    name: Arc<Mutex<String>>,
}

/// Publishes queued events to `/rosout` until dropped; dropping it stops
/// forwarding, withdraws the publisher and lets the handle attach again.
#[derive(Debug)]
#[must_use = "dropping the worker stops publishing to /rosout"]
pub struct RosoutWorker {
    stop: Arc<AtomicBool>,
    name: Arc<Mutex<String>>,
    thread: Option<JoinHandle<()>>,
}

impl Drop for RosoutWorker {
    fn drop(&mut self) {
        self.name
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clear();
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl RosoutLayer {
    /// A layer forwarding events at `min_level` or above whose target starts
    /// with one of `targets`.
    #[must_use]
    pub fn new(min_level: Level, targets: &[&str]) -> (Self, RosoutHandle) {
        let (tx, rx) = mpsc::sync_channel(1024);
        let name = Arc::new(Mutex::new(String::new()));
        (
            Self {
                tx,
                min_level,
                targets: targets.iter().map(|t| (*t).to_owned()).collect(),
                name: Arc::clone(&name),
            },
            RosoutHandle {
                rx: Arc::new(Mutex::new(Some(rx))),
                name,
            },
        )
    }
}

struct MessageVisitor(String);

impl Visit for MessageVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            let _ = write!(self.0, "{value:?}");
        } else {
            let sep = if self.0.is_empty() { "" } else { " " };
            let _ = write!(self.0, "{sep}{}={value:?}", field.name());
        }
    }

    fn record_str(&mut self, field: &Field, value: &str) {
        if field.name() == "message" {
            self.0.push_str(value);
        } else {
            self.record_debug(field, &value);
        }
    }
}

impl<S: Subscriber> Layer<S> for RosoutLayer {
    fn on_event(&self, event: &Event<'_>, _ctx: LayerContext<'_, S>) {
        let meta = event.metadata();
        if *meta.level() > self.min_level
            || !self
                .targets
                .iter()
                .any(|t| meta.target().starts_with(t.as_str()))
        {
            return;
        }
        let name = self
            .name
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone();
        if name.is_empty() {
            // Not attached yet.
            return;
        }
        let mut visitor = MessageVisitor(String::new());
        event.record(&mut visitor);
        let _ = self.tx.try_send(Log {
            stamp: Time::now(),
            level: level(*meta.level()),
            name,
            msg: visitor.0,
            file: meta.file().unwrap_or_default().to_owned(),
            function: meta.module_path().unwrap_or_default().to_owned(),
            line: meta.line().unwrap_or_default(),
        });
    }
}

impl RosoutHandle {
    /// Starts publishing to `/rosout` on behalf of `node` until the returned
    /// worker is dropped; `None` while another worker is attached. The
    /// logger name is the node's fully qualified name in rcl's dotted form.
    ///
    /// # Errors
    /// Zenoh or naming errors creating the publisher.
    pub fn attach(&mut self, node: &Node) -> super::context::Result<Option<RosoutWorker>> {
        let mut slot = self.rx.lock().unwrap_or_else(PoisonError::into_inner);
        if slot.is_none() {
            return Ok(None);
        }
        let publisher = node.create_publisher::<Log>("/rosout", rosout_qos())?;
        let Some(rx) = slot.take() else {
            return Ok(None);
        };
        drop(slot);
        let logger = node
            .fully_qualified_name()
            .trim_start_matches('/')
            .replace('/', ".");
        *self.name.lock().unwrap_or_else(PoisonError::into_inner) = logger;
        let stop = Arc::new(AtomicBool::new(false));
        let (flag, home) = (Arc::clone(&stop), Arc::clone(&self.rx));
        // The receiver travels through a slot the thread empties, so that a
        // failed spawn leaves it here to be handed back.
        let handoff = Arc::new(Mutex::new(Some(rx)));
        let passed = Arc::clone(&handoff);
        let thread = std::thread::Builder::new()
            .name("rosout".into())
            .spawn(move || {
                let Some(rx) = passed.lock().unwrap_or_else(PoisonError::into_inner).take() else {
                    return;
                };
                drop(passed);
                while !flag.load(Ordering::Relaxed) {
                    match rx.recv_timeout(Duration::from_millis(100)) {
                        Ok(log) => {
                            let _ = publisher.publish(&log);
                        }
                        Err(RecvTimeoutError::Timeout) => {}
                        Err(RecvTimeoutError::Disconnected) => break,
                    }
                }
                // Events queued meanwhile are dropped; hand the receiver
                // back for the next attach.
                while rx.try_recv().is_ok() {}
                *home.lock().unwrap_or_else(PoisonError::into_inner) = Some(rx);
            });
        let thread = match thread {
            Ok(t) => t,
            Err(e) => {
                self.name
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .clear();
                *self.rx.lock().unwrap_or_else(PoisonError::into_inner) = handoff
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .take();
                return Err(super::RosError::Zenoh(e.to_string()));
            }
        };
        Ok(Some(RosoutWorker {
            stop,
            name: Arc::clone(&self.name),
            thread: Some(thread),
        }))
    }
}
