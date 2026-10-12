//! The zenoh session shared by every node, and its configuration.

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::Duration;

use zenoh::{Session, Wait};

use super::attachment::AttachmentFormat;
use super::liveliness::{ADMIN_SPACE, Entity};
use super::names::NameError;
use super::node::Node;
use super::qos::QosProfile;

/// Errors of the ROS 2 layer.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum RosError {
    /// A zenoh operation failed.
    #[error("zenoh: {0}")]
    Zenoh(String),
    /// The session configuration is invalid.
    #[error("invalid zenoh configuration: {0}")]
    Config(String),
    /// An invalid node, namespace, topic or service name.
    #[error(transparent)]
    Name(#[from] NameError),
    /// A message could not be encoded or decoded.
    #[error("CDR: {0}")]
    Cdr(#[from] super::cdr::CdrError),
    /// A service call received no reply in time.
    #[error("service call timed out")]
    Timeout,
    /// A service server replied with an error.
    #[error("service replied with an error: {0}")]
    ServiceError(String),
    /// The process-wide ROS entity id space is exhausted.
    #[error("ROS entity id space exhausted")]
    EntityIdExhausted,
}

impl RosError {
    pub(crate) fn zenoh(e: impl std::fmt::Display) -> Self {
        Self::Zenoh(e.to_string())
    }
}

/// Result alias of the ROS 2 layer.
pub type Result<T> = std::result::Result<T, RosError>;

static NEXT_ENTITY_ID: AtomicU64 = AtomicU64::new(0);

fn allocate_entity_id(next: &AtomicU64) -> Result<u64> {
    loop {
        let id = next.load(Ordering::Relaxed);
        let Some(successor) = id.checked_add(1) else {
            return Err(RosError::EntityIdExhausted);
        };
        if next
            .compare_exchange_weak(id, successor, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            return Ok(id);
        }
    }
}

/// Where the zenoh session configuration comes from.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum SessionConfig {
    /// The `rmw_zenoh_cpp` default session configuration: a peer that
    /// connects to a router on `tcp/localhost:7447`, listens on a random
    /// local port and discovers other peers through router gossip.
    #[default]
    RmwZenohDefault,
    /// A zenoh JSON5 configuration file.
    File(PathBuf),
    /// Inline zenoh JSON5 configuration.
    Json5(String),
}

/// The `rmw_zenoh_cpp` release the other ROS 2 nodes run. Jazzy changed the
/// protocol three times after its first release, and Humble has no type
/// hashes; Kilted and Rolling speak the current protocol.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, Default, serde::Serialize, serde::Deserialize,
)]
pub enum RmwZenohCompat {
    /// `rmw_zenoh_cpp` 0.2.4 and later on Jazzy, and the current Kilted
    /// and Rolling releases: bare-field attachments, volatile as the default
    /// durability, and transient-local topics through zenoh-ext's
    /// AdvancedPublisher and AdvancedSubscriber with sample-miss recovery.
    #[default]
    #[serde(rename = "current")]
    Current,
    /// Humble 0.1.2 and later: the current protocol, but Humble has no type
    /// hashes, so key expressions and liveliness tokens carry
    /// [`TYPE_HASH_NOT_SUPPORTED`] in their place. Humble 0.1.0 and 0.1.1,
    /// with named-field attachments, are not supported.
    #[serde(rename = "humble")]
    Humble,
    /// Jazzy 0.2.3: named-field attachments, volatile as the default
    /// durability, advanced pub/sub without sample-miss recovery.
    #[serde(rename = "jazzy-0.2.3")]
    Jazzy023,
    /// Jazzy 0.2.2: like 0.2.3, with transient local as the default
    /// durability.
    #[serde(rename = "jazzy-0.2.2")]
    Jazzy022,
    /// Jazzy 0.2.0 and 0.2.1: named-field attachments, transient local as
    /// the default durability, and transient-local history served by a
    /// queryable on `<zid>/<key>`.
    #[serde(rename = "jazzy-0.2.0")]
    Jazzy020,
}

/// What Humble's `rmw_zenoh_cpp` writes where later releases put the RIHS01
/// type hash.
pub const TYPE_HASH_NOT_SUPPORTED: &str = "TypeHashNotSupported";

impl RmwZenohCompat {
    /// The type hash as this release writes it in key expressions and
    /// liveliness tokens.
    #[must_use]
    pub fn type_hash_key(self, rihs01: String) -> String {
        if self.type_hashes() {
            rihs01
        } else {
            TYPE_HASH_NOT_SUPPORTED.to_owned()
        }
    }

    /// Whether this release's distribution has type hashes and the
    /// `type_description_interfaces` package (Humble has neither).
    #[must_use]
    pub fn type_hashes(self) -> bool {
        self != Self::Humble
    }

    /// The attachment format this release writes.
    #[must_use]
    pub fn attachment_format(self) -> AttachmentFormat {
        match self {
            Self::Current | Self::Humble => AttachmentFormat::Current,
            Self::Jazzy023 | Self::Jazzy022 | Self::Jazzy020 => AttachmentFormat::Legacy,
        }
    }

    /// The QoS defaults this release adapts profiles to and encodes
    /// liveliness tokens against.
    #[must_use]
    pub fn qos_defaults(self) -> &'static QosProfile {
        match self {
            Self::Current | Self::Humble | Self::Jazzy023 => &QosProfile::RMW_ZENOH_DEFAULT,
            Self::Jazzy022 | Self::Jazzy020 => &QosProfile::RMW_ZENOH_DEFAULT_BEFORE_0_2_3,
        }
    }

    /// Whether transient-local topics use zenoh-ext's advanced pub/sub.
    #[must_use]
    pub fn advanced_pubsub(self) -> bool {
        self != Self::Jazzy020
    }

    /// Whether reliable transient-local topics add sample-miss detection
    /// (publisher heartbeats) and recovery (subscribers).
    #[must_use]
    pub fn sample_miss_recovery(self) -> bool {
        matches!(self, Self::Current | Self::Humble)
    }
}

/// Options of a [`Context`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ContextOptions {
    /// ROS domain id (`ROS_DOMAIN_ID`).
    pub domain_id: u32,
    /// Security enclave, `/` by default.
    pub enclave: String,
    /// Base session configuration.
    pub config: SessionConfig,
    /// `key=json5` overrides applied on top of the base configuration,
    /// e.g. `("connect/endpoints", "[\"tcp/10.0.0.1:7447\"]")`.
    pub overrides: Vec<(String, String)>,
    /// How many polls to make before warning, with one-second spacing
    /// between failed polls; `None` skips the check, `Some(0)` waits forever.
    pub router_check_attempts: Option<u64>,
    /// The `rmw_zenoh_cpp` protocol revision to speak.
    pub compat: RmwZenohCompat,
}

impl Default for ContextOptions {
    fn default() -> Self {
        Self {
            domain_id: 0,
            enclave: "/".to_owned(),
            config: SessionConfig::RmwZenohDefault,
            overrides: Vec::new(),
            router_check_attempts: Some(1),
            compat: RmwZenohCompat::Current,
        }
    }
}

impl ContextOptions {
    /// Options from the environment, like `rmw_zenoh_cpp`:
    ///
    /// * `ROS_DOMAIN_ID`: domain id (default 0);
    /// * `ZENOH_SESSION_CONFIG_URI`: path of a session configuration file;
    /// * `ZENOH_ROUTER_CHECK_ATTEMPTS`: router check attempts (default 1,
    ///   0 waits forever, negative skips the check);
    /// * `ZENOH_CONFIG_OVERRIDE`: `key=value;key=value` JSON5 overrides.
    ///
    /// # Errors
    /// [`RosError::Config`] when a variable is malformed.
    pub fn from_env() -> Result<Self> {
        let mut opts = Self::default();
        if let Ok(v) = std::env::var("ROS_DOMAIN_ID")
            && !v.trim().is_empty()
        {
            opts.domain_id = v
                .trim()
                .parse()
                .map_err(|_| RosError::Config(format!("ROS_DOMAIN_ID={v:?} is not a number")))?;
        }
        if let Ok(v) = std::env::var("ZENOH_SESSION_CONFIG_URI")
            && !v.is_empty()
        {
            opts.config = SessionConfig::File(PathBuf::from(v));
        }
        if let Ok(v) = std::env::var("ZENOH_ROUTER_CHECK_ATTEMPTS")
            && !v.trim().is_empty()
        {
            let n: i64 = v.trim().parse().map_err(|_| {
                RosError::Config(format!("ZENOH_ROUTER_CHECK_ATTEMPTS={v:?} is not a number"))
            })?;
            opts.router_check_attempts = u64::try_from(n).ok();
        }
        if let Ok(v) = std::env::var("ZENOH_CONFIG_OVERRIDE") {
            opts.overrides.extend(parse_overrides(&v)?);
        }
        Ok(opts)
    }
}

/// Parses `key=value;key=value` overrides.
///
/// # Errors
/// [`RosError::Config`] for an entry without `=`.
pub fn parse_overrides(s: &str) -> Result<Vec<(String, String)>> {
    s.split(';')
        .map(str::trim)
        .filter(|e| !e.is_empty())
        .map(|entry| {
            entry
                .split_once('=')
                .map(|(k, v)| (k.trim().to_owned(), v.trim().to_owned()))
                .ok_or_else(|| RosError::Config(format!("override {entry:?} is not key=value")))
        })
        .collect()
}

/// The `rmw_zenoh_cpp` default session settings that differ from zenoh's.
const RMW_ZENOH_DEFAULTS: &[(&str, &str)] = &[
    ("mode", r#""peer""#),
    ("connect/endpoints", r#"["tcp/localhost:7447"]"#),
    ("connect/timeout_ms", "{ router: -1, peer: -1, client: 0 }"),
    (
        "connect/exit_on_failure",
        "{ router: false, peer: false, client: true }",
    ),
    ("listen/endpoints", r#"["tcp/localhost:0"]"#),
    ("scouting/multicast/enabled", "false"),
    ("scouting/gossip/enabled", "true"),
    ("scouting/gossip/multihop", "false"),
    (
        "scouting/gossip/autoconnect",
        r#"{ router: [], peer: ["router", "peer"] }"#,
    ),
    (
        "timestamping/enabled",
        "{ router: true, peer: true, client: true }",
    ),
    ("queries_default_timeout", "10000"),
    ("transport/unicast/max_links", "1"),
];

/// Builds the zenoh configuration for `options`.
///
/// # Errors
/// [`RosError::Config`] when the file or an override is invalid.
pub fn build_config(options: &ContextOptions) -> Result<zenoh::Config> {
    let mut config = match &options.config {
        SessionConfig::RmwZenohDefault => {
            let mut c = zenoh::Config::default();
            for (k, v) in RMW_ZENOH_DEFAULTS {
                c.insert_json5(k, v)
                    .map_err(|e| RosError::Config(format!("{k}: {e}")))?;
            }
            c
        }
        SessionConfig::File(path) => zenoh::Config::from_file(path)
            .map_err(|e| RosError::Config(format!("{}: {e}", path.display())))?,
        SessionConfig::Json5(s) => {
            zenoh::Config::from_json5(s).map_err(|e| RosError::Config(e.to_string()))?
        }
    };
    for (k, v) in &options.overrides {
        config
            .insert_json5(k, v)
            .map_err(|e| RosError::Config(format!("{k}={v}: {e}")))?;
    }
    Ok(config)
}

pub(crate) struct ContextInner {
    pub(crate) session: Session,
    pub(crate) domain_id: u32,
    pub(crate) enclave: String,
    pub(crate) zid: String,
    pub(crate) compat: RmwZenohCompat,
}

/// A ROS 2 context: one zenoh session plus the domain and enclave.
///
/// Cheap to clone; the session closes when the last clone and every entity
/// created from it are dropped.
#[derive(Clone)]
pub struct Context {
    pub(crate) inner: Arc<ContextInner>,
}

impl std::fmt::Debug for Context {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Context")
            .field("zid", &self.inner.zid)
            .field("domain_id", &self.inner.domain_id)
            .finish_non_exhaustive()
    }
}

impl Context {
    /// Opens a session.
    ///
    /// # Errors
    /// [`RosError`] when the configuration is invalid or the session cannot
    /// be opened.
    pub fn new(options: &ContextOptions) -> Result<Self> {
        let config = build_config(options)?;
        let session = zenoh::open(config).wait().map_err(RosError::zenoh)?;
        Ok(Self::from_session(session, options))
    }

    /// Wraps an existing session.
    #[must_use]
    pub fn from_session(session: Session, options: &ContextOptions) -> Self {
        let zid = session.zid().to_string();
        let ctx = Self {
            inner: Arc::new(ContextInner {
                session,
                domain_id: options.domain_id,
                enclave: options.enclave.clone(),
                zid,
                compat: options.compat,
            }),
        };
        if let Some(attempts) = options.router_check_attempts {
            ctx.check_router(attempts);
        }
        ctx
    }

    /// Polls for a router up to `attempts` times, spacing failed polls one
    /// second apart (forever for 0); returns whether one was found and warns
    /// otherwise.
    pub fn check_router(&self, attempts: u64) -> bool {
        self.wait_for_router(attempts, &AtomicBool::new(false))
    }

    /// [`Self::check_router`] that also gives up once `stop` is set, so a
    /// shutdown request is not ignored while no router is reachable.
    pub fn wait_for_router(&self, attempts: u64, stop: &AtomicBool) -> bool {
        let mut tried = 0_u64;
        loop {
            if self
                .inner
                .session
                .info()
                .routers_zid()
                .wait()
                .next()
                .is_some()
            {
                return true;
            }
            tried += 1;
            if attempts != 0 && tried >= attempts {
                tracing::warn!(
                    "unable to connect to a zenoh router; ROS 2 nodes using rmw_zenoh_cpp will not \
                     see this bridge unless a router (`ros2 run rmw_zenoh_cpp rmw_zenohd`) is \
                     reachable or peers are configured"
                );
                return false;
            }
            for _ in 0..10 {
                if stop.load(Ordering::Relaxed) {
                    return false;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }

    /// The zenoh session.
    #[must_use]
    pub fn session(&self) -> &Session {
        &self.inner.session
    }

    /// The ROS domain id.
    #[must_use]
    pub fn domain_id(&self) -> u32 {
        self.inner.domain_id
    }

    /// The session's zenoh id as it appears in liveliness tokens.
    #[must_use]
    pub fn zid(&self) -> &str {
        &self.inner.zid
    }

    /// The `rmw_zenoh_cpp` protocol revision this context speaks.
    #[must_use]
    pub fn compat(&self) -> RmwZenohCompat {
        self.inner.compat
    }

    pub(crate) fn next_entity_id() -> Result<u64> {
        allocate_entity_id(&NEXT_ENTITY_ID)
    }

    /// Creates a node.
    ///
    /// # Errors
    /// [`RosError`] for an invalid name or namespace, a zenoh failure, or
    /// exhaustion of the process-wide entity id space.
    pub fn create_node(&self, name: &str, namespace: &str) -> Result<Node> {
        Node::new(self.clone(), name, namespace)
    }

    /// Queries the current ROS 2 graph of this domain: every node, topic
    /// endpoint and service endpoint announced by any `rmw_zenoh_cpp`
    /// participant, including this session.
    ///
    /// # Errors
    /// [`RosError::Zenoh`] when the liveliness query fails.
    pub fn graph(&self, timeout: Duration) -> Result<Vec<Entity>> {
        let replies = self
            .inner
            .session
            .liveliness()
            .get(format!("{ADMIN_SPACE}/{}/**", self.inner.domain_id))
            .timeout(timeout)
            .wait()
            .map_err(RosError::zenoh)?;
        let mut out = Vec::new();
        while let Ok(reply) = replies.recv() {
            if let Ok(sample) = reply.result()
                && let Some(entity) =
                    Entity::parse(sample.key_expr().as_str(), self.inner.compat.qos_defaults())
            {
                out.push(entity);
            }
        }
        out.sort_by(|a, b| (&a.zid, a.id).cmp(&(&b.zid, b.id)));
        out.dedup();
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_is_accepted() {
        build_config(&ContextOptions::default()).unwrap();
    }

    #[test]
    fn overrides() {
        let o =
            parse_overrides("mode=\"client\"; connect/endpoints=[\"tcp/1.2.3.4:7447\"]").unwrap();
        assert_eq!(o.len(), 2);
        assert_eq!(o[0], ("mode".to_owned(), "\"client\"".to_owned()));
        let opts = ContextOptions {
            overrides: o,
            ..ContextOptions::default()
        };
        build_config(&opts).unwrap();
        assert!(parse_overrides("novalue").is_err());
        let bad = ContextOptions {
            overrides: vec![("no/such/key".into(), "1".into())],
            ..ContextOptions::default()
        };
        assert!(build_config(&bad).is_err());
    }
}
