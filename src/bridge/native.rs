//! Native zenoh outputs for non-ROS consumers.
//!
//! | Key | Content |
//! | --- | --- |
//! | `<prefix>/frame` | every frame as JSON |
//! | `<prefix>/rigid_bodies/<name>` | each rigid body as JSON |
//! | `<prefix>/raw/frame` | each frame re-encoded as a NatNet datagram |
//! | `<prefix>/server_info`, `<prefix>/model`, `<prefix>/stats` | JSON, put on change and served to `get` |
//! | `<prefix>/refresh_model` | `get` requests the model definitions again |
//! | `<prefix>/alive` | a liveliness token while the outputs exist |
//!
//! Every JSON sample and reply carries the encoding
//! `application/json;natnet_zenoh.v1` ([`FORMAT_SCHEMA`]). Within a format
//! version, documents only gain fields; renaming, removing or retyping a
//! field bumps the version. `tests/golden/native/` holds a reference of each
//! document. A raw frame carries `application/octet-stream;natnet.<version>`
//! (see [`raw_frame_encoding`]), the NatNet protocol version it is encoded
//! in.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::sync::mpsc::Sender;

use natnet::{FrameOfData, Version};
use tracing::{debug, warn};
use zenoh::Session;
use zenoh::Wait;
use zenoh::bytes::Encoding;
use zenoh::liveliness::LivelinessToken;
use zenoh::pubsub::Publisher;
use zenoh::query::Queryable;

use super::Shared;
use super::snapshot::{BodySample, Snapshot};
use crate::config::NativeConfig;
use crate::ros::node::Matching;
use crate::source::SourceCommand;

/// The native format version, the schema of every JSON sample and reply.
pub const FORMAT_SCHEMA: &str = "natnet_zenoh.v1";

/// The encoding of native JSON documents: `application/json;natnet_zenoh.v1`.
#[must_use]
pub fn json_encoding() -> Encoding {
    Encoding::APPLICATION_JSON.with_schema(FORMAT_SCHEMA)
}

/// The encoding of a raw frame in NatNet protocol `version`:
/// `application/octet-stream;natnet.4.1.0.0` for NatNet 4.1.
#[must_use]
pub fn raw_frame_encoding(version: Version) -> Encoding {
    Encoding::APPLICATION_OCTET_STREAM.with_schema(format!("natnet.{version}"))
}

/// A frame's native JSON document (`<prefix>/frame`).
///
/// # Errors
/// A serialization error.
pub fn frame_json(snap: &Snapshot) -> serde_json::Result<Vec<u8>> {
    serde_json::to_vec(snap)
}

/// A rigid body's native JSON document (`<prefix>/rigid_bodies/<name>`).
///
/// # Errors
/// A serialization error.
pub fn rigid_body_json(snap: &Snapshot, body: &BodySample) -> serde_json::Result<Vec<u8>> {
    serde_json::to_vec(&BodyDocument {
        frame_number: snap.frame_number,
        stamp_secs: snap.stamp_secs,
        body,
    })
}

/// A publisher and whether anyone subscribes to it.
struct Watched {
    // Dropped before the publisher, whose undeclaration removes it.
    matching: Matching,
    publisher: Publisher<'static>,
}

impl Watched {
    fn new(publisher: Publisher<'static>) -> Self {
        Self {
            matching: Matching::watch(&publisher),
            publisher,
        }
    }

    /// Puts `make()` when someone subscribes.
    fn put_with<T: Into<zenoh::bytes::ZBytes>>(
        &self,
        what: &str,
        make: impl FnOnce() -> Option<T>,
    ) {
        self.put_encoded(what, None, make);
    }

    /// Puts `make()` when someone subscribes, with `encoding` in place of the
    /// publisher's.
    fn put_encoded<T: Into<zenoh::bytes::ZBytes>>(
        &self,
        what: &str,
        encoding: Option<&Encoding>,
        make: impl FnOnce() -> Option<T>,
    ) {
        if !self.matching.get() {
            return;
        }
        let Some(payload) = make() else {
            return;
        };
        let put = self.publisher.put(payload);
        let put = match encoding {
            Some(encoding) => put.encoding(encoding.clone()),
            None => put,
        };
        if let Err(e) = put.wait() {
            debug!("{what} put failed: {e}");
        }
    }
}

/// A rigid body's native document.
#[derive(serde::Serialize)]
struct BodyDocument<'a> {
    frame_number: i32,
    stamp_secs: f64,
    body: &'a BodySample,
}

/// The native zenoh side of the bridge.
pub struct NativeOutputs {
    session: Session,
    cfg: NativeConfig,
    frame: Option<Watched>,
    raw: Option<Watched>,
    /// The raw-frame encoding for the last protocol version seen.
    raw_encoding: Option<(Version, Encoding)>,
    bodies: BTreeMap<String, Watched>,
    prune: bool,
    documents: BTreeMap<&'static str, Publisher<'static>>,
    _queryables: Vec<Queryable<()>>,
    _alive: LivelinessToken,
}

impl std::fmt::Debug for NativeOutputs {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NativeOutputs")
            .field("prefix", &self.cfg.prefix)
            .finish_non_exhaustive()
    }
}

fn zerr(e: impl std::fmt::Display) -> crate::ros::RosError {
    crate::ros::RosError::Zenoh(e.to_string())
}

impl NativeOutputs {
    /// Declares the publishers and queryables.
    ///
    /// # Errors
    /// Zenoh errors.
    pub fn new(
        session: &Session,
        cfg: &NativeConfig,
        shared: &Arc<Shared>,
        commands: &Sender<SourceCommand>,
    ) -> crate::ros::context::Result<Self> {
        let p = &cfg.prefix;
        let json = |key: String| {
            session
                .declare_publisher(key)
                .encoding(json_encoding())
                .wait()
                .map_err(zerr)
        };
        let frame = cfg
            .frame_json
            .then(|| json(format!("{p}/frame")).map(Watched::new))
            .transpose()?;
        let raw = cfg
            .raw
            .then(|| {
                session
                    .declare_publisher(format!("{p}/raw/frame"))
                    .encoding(Encoding::APPLICATION_OCTET_STREAM)
                    .wait()
                    .map(Watched::new)
                    .map_err(zerr)
            })
            .transpose()?;
        let mut documents = BTreeMap::new();
        let mut queryables = Vec::new();
        for name in ["server_info", "model", "stats"] {
            let key = format!("{p}/{name}");
            documents.insert(name, json(key.clone())?);
            let s = Arc::clone(shared);
            let reply_key = key.clone();
            queryables.push(
                session
                    .declare_queryable(key)
                    .callback(move |q| {
                        let body = s.snapshot_json(name);
                        if let Err(e) = q
                            .reply(reply_key.clone(), body)
                            .encoding(json_encoding())
                            .wait()
                        {
                            debug!("reply on {reply_key} failed: {e}");
                        }
                    })
                    .wait()
                    .map_err(zerr)?,
            );
        }
        let key = format!("{p}/refresh_model");
        let tx = std::sync::Mutex::new(commands.clone());
        let reply_key = key.clone();
        queryables.push(
            session
                .declare_queryable(key)
                .callback(move |q| {
                    let ok = tx
                        .lock()
                        .is_ok_and(|tx| tx.send(SourceCommand::RefreshModel).is_ok());
                    let body = if ok { "requested" } else { "source stopped" };
                    if let Err(e) = q
                        .reply(reply_key.clone(), body)
                        .encoding(Encoding::TEXT_PLAIN.with_schema(FORMAT_SCHEMA))
                        .wait()
                    {
                        debug!("reply on {reply_key} failed: {e}");
                    }
                })
                .wait()
                .map_err(zerr)?,
        );
        // Declared last, so the token announces outputs that are all ready.
        let alive = session
            .liveliness()
            .declare_token(format!("{p}/alive"))
            .wait()
            .map_err(zerr)?;
        Ok(Self {
            session: session.clone(),
            cfg: cfg.clone(),
            frame,
            raw,
            raw_encoding: None,
            bodies: BTreeMap::new(),
            prune: false,
            documents,
            _queryables: queryables,
            _alive: alive,
        })
    }

    /// Notes a model change: rigid-body keys of bodies missing from the next
    /// frame are withdrawn.
    pub fn model_changed(&mut self) {
        self.prune = true;
    }

    /// Puts a JSON document (`server_info`, `model`, `stats`).
    pub fn put_document(&self, name: &str, json: &str) {
        if let Some(p) = self.documents.get(name)
            && let Err(e) = p.put(json.to_owned()).wait()
        {
            debug!("put {name} failed: {e}");
        }
    }

    /// Publishes one frame.
    pub fn publish_frame(
        &mut self,
        snap: &Snapshot,
        frame: &FrameOfData,
        version: Option<Version>,
    ) {
        if std::mem::take(&mut self.prune) {
            self.bodies
                .retain(|n, _| snap.bodies.iter().any(|b| &b.name == n));
        }
        if let Some(p) = &self.frame {
            p.put_with("frame", || {
                frame_json(snap)
                    .inspect_err(|e| warn!("frame JSON: {e}"))
                    .ok()
            });
        }
        if let (Some(p), Some(v)) = (&self.raw, version) {
            if self
                .raw_encoding
                .as_ref()
                .is_none_or(|(seen, _)| *seen != v)
            {
                self.raw_encoding = Some((v, raw_frame_encoding(v)));
            }
            let encoding = self.raw_encoding.as_ref().map(|(_, e)| e);
            p.put_encoded("raw frame", encoding, || {
                frame
                    .to_datagram(v)
                    .inspect_err(|e| debug!("frame re-encoding failed: {e}"))
                    .ok()
            });
        }
        if self.cfg.rigid_body_json {
            for b in &snap.bodies {
                if !self.bodies.contains_key(&b.name) {
                    let key = format!("{}/rigid_bodies/{}", self.cfg.prefix, b.name);
                    match self
                        .session
                        .declare_publisher(key)
                        .encoding(json_encoding())
                        .wait()
                    {
                        Ok(p) => {
                            self.bodies.insert(b.name.clone(), Watched::new(p));
                        }
                        Err(e) => {
                            warn!("rigid body {}: {e}", b.name);
                            continue;
                        }
                    }
                }
                if let Some(p) = self.bodies.get(&b.name) {
                    p.put_with("rigid body", || rigid_body_json(snap, b).ok());
                }
            }
        }
    }
}
