//! Nodes and their publishers, subscriptions, service servers and clients.

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::marker::PhantomData;
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use zenoh::Wait;
use zenoh::handlers::DefaultHandler;
use zenoh::liveliness::LivelinessToken;
use zenoh::matching::{MatchingListener, MatchingListenerBuilder};
use zenoh::query::{ConsolidationMode, QueryTarget, Queryable, ReplyKeyExpr};
use zenoh::sample::{Sample, SampleKind};
use zenoh_ext::{
    AdvancedPublisher, AdvancedPublisherBuilderExt, AdvancedSubscriber,
    AdvancedSubscriberBuilderExt, CacheConfig, HistoryConfig, MissDetectionConfig, RecoveryConfig,
};

use super::attachment::{Attachment, AttachmentFormat};
use super::cdr::CdrWriter;
use super::context::{Context, Result, RosError};
use super::gid::Gid;
use super::liveliness::{ADMIN_SPACE, Entity, EntityKind, NodeInfo, TopicInfo, mangle};
use super::message::{Message, Service};
use super::msgs::builtin_interfaces::Time;
use super::names;
use super::qos::{Durability, QosProfile};
use super::type_description::TypeDescription;

/// The types a node uses, by name, as served by `~/get_type_description`.
pub type TypeRegistry = Arc<Mutex<BTreeMap<String, TypeDescription>>>;

/// A ROS 2 node.
///
/// Dropping the node withdraws its liveliness token; entities created from
/// it stay alive until dropped themselves.
pub struct Node {
    ctx: Context,
    info: NodeInfo,
    nid: u64,
    gid: Gid,
    types: TypeRegistry,
    _token: LivelinessToken,
}

impl std::fmt::Debug for Node {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Node")
            .field("name", &self.info.name)
            .field("namespace", &self.info.namespace)
            .field("nid", &self.nid)
            .finish_non_exhaustive()
    }
}

fn declare_token(ctx: &Context, entity: &Entity) -> Result<LivelinessToken> {
    ctx.session()
        .liveliness()
        .declare_token(entity.liveliness_keyexpr(ctx.compat().qos_defaults()))
        .wait()
        .map_err(RosError::zenoh)
}

impl Node {
    pub(crate) fn new(ctx: Context, name: &str, namespace: &str) -> Result<Self> {
        names::validate_node_name(name)?;
        let namespace = names::normalize_namespace(namespace);
        names::validate_namespace(&namespace)?;
        let nid = Context::next_entity_id()?;
        let info = NodeInfo {
            domain_id: ctx.domain_id(),
            enclave: ctx.inner.enclave.clone(),
            namespace,
            name: name.to_owned(),
        };
        let entity = Entity {
            zid: ctx.zid().to_owned(),
            nid,
            id: nid,
            kind: EntityKind::Node,
            node: info.clone(),
            topic: None,
        };
        let token = declare_token(&ctx, &entity)?;
        Ok(Self {
            gid: entity.gid(ctx.compat().qos_defaults()),
            ctx,
            info,
            nid,
            types: TypeRegistry::default(),
            _token: token,
        })
    }

    /// The node name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.info.name
    }

    /// The node namespace.
    #[must_use]
    pub fn namespace(&self) -> &str {
        &self.info.namespace
    }

    /// The fully qualified node name, e.g. `/mocap/natnet`.
    #[must_use]
    pub fn fully_qualified_name(&self) -> String {
        if self.info.namespace == "/" {
            format!("/{}", self.info.name)
        } else {
            format!("{}/{}", self.info.namespace, self.info.name)
        }
    }

    /// The node's GID.
    #[must_use]
    pub fn gid(&self) -> Gid {
        self.gid
    }

    /// The owning context.
    #[must_use]
    pub fn context(&self) -> &Context {
        &self.ctx
    }

    /// The types this node's entities use.
    #[must_use]
    pub fn type_registry(&self) -> TypeRegistry {
        Arc::clone(&self.types)
    }

    fn register_type(&self, name: &str, description: impl FnOnce() -> TypeDescription) {
        let mut types = self.types.lock().unwrap_or_else(PoisonError::into_inner);
        if !types.contains_key(name) {
            types.insert(name.to_owned(), description());
        }
    }

    /// Expands a topic or service name relative to this node.
    ///
    /// # Errors
    /// [`RosError::Name`] for an invalid name.
    pub fn expand(&self, name: &str) -> Result<String> {
        Ok(names::expand_topic_name(
            name,
            &self.info.name,
            &self.info.namespace,
        )?)
    }

    fn entity(&self, kind: EntityKind, topic: TopicInfo) -> Result<Entity> {
        Ok(Entity {
            zid: self.ctx.zid().to_owned(),
            nid: self.nid,
            id: Context::next_entity_id()?,
            kind,
            node: self.info.clone(),
            topic: Some(topic),
        })
    }

    /// Creates a publisher.
    ///
    /// # Errors
    /// [`RosError`] for an invalid topic name, a zenoh failure, or
    /// exhaustion of the process-wide entity id space.
    pub fn create_publisher<M: Message>(
        &self,
        topic: &str,
        qos: QosProfile,
    ) -> Result<Publisher<M>> {
        let qos = qos.adapted(self.ctx.compat().qos_defaults());
        let entity = self.entity(
            EntityKind::Publisher,
            TopicInfo {
                name: self.expand(topic)?,
                type_name: M::dds_type_name(),
                type_hash: self.ctx.compat().type_hash_key(M::type_hash()),
                qos,
            },
        )?;
        let raw = RawPublisher::new(&self.ctx, entity)?;
        self.register_type(M::TYPE_NAME, M::type_description);
        Ok(Publisher {
            raw,
            last_len: AtomicUsize::new(0),
            _message: PhantomData,
        })
    }

    /// Creates a subscription whose callback runs on a zenoh thread for
    /// every received message.
    ///
    /// # Errors
    /// [`RosError`] for an invalid topic name, a zenoh failure, or
    /// exhaustion of the process-wide entity id space.
    pub fn create_subscription<M, F>(
        &self,
        topic: &str,
        qos: QosProfile,
        callback: F,
    ) -> Result<Subscription<M>>
    where
        M: Message,
        F: Fn(M, &MessageInfo) + Send + Sync + 'static,
    {
        let qos = qos.adapted(self.ctx.compat().qos_defaults());
        let entity = self.entity(
            EntityKind::Subscription,
            TopicInfo {
                name: self.expand(topic)?,
                type_name: M::dds_type_name(),
                type_hash: self.ctx.compat().type_hash_key(M::type_hash()),
                qos,
            },
        )?;
        let sub = Subscription::new(&self.ctx, entity, callback)?;
        self.register_type(M::TYPE_NAME, M::type_description);
        Ok(sub)
    }

    /// Creates a service server whose handler runs on a zenoh thread for
    /// every request.
    ///
    /// # Errors
    /// [`RosError`] for an invalid service name, a zenoh failure, or
    /// exhaustion of the process-wide entity id space.
    pub fn create_service<S, F>(&self, name: &str, handler: F) -> Result<ServiceServer<S>>
    where
        S: Service,
        F: Fn(S::Request, &RequestInfo) -> S::Response + Send + Sync + 'static,
    {
        let entity = self.entity(
            EntityKind::Service,
            TopicInfo {
                name: self.expand(name)?,
                type_name: S::dds_type_name(),
                type_hash: self.ctx.compat().type_hash_key(S::type_hash()),
                qos: QosProfile::services_default().adapted(self.ctx.compat().qos_defaults()),
            },
        )?;
        let server = ServiceServer::new(&self.ctx, entity, handler)?;
        self.register_type(S::TYPE_NAME, S::type_description);
        Ok(server)
    }

    /// Creates a service client.
    ///
    /// # Errors
    /// [`RosError`] for an invalid service name, a zenoh failure, or
    /// exhaustion of the process-wide entity id space.
    pub fn create_client<S: Service>(&self, name: &str) -> Result<Client<S>> {
        let entity = self.entity(
            EntityKind::Client,
            TopicInfo {
                name: self.expand(name)?,
                type_name: S::dds_type_name(),
                type_hash: self.ctx.compat().type_hash_key(S::type_hash()),
                qos: QosProfile::services_default().adapted(self.ctx.compat().qos_defaults()),
            },
        )?;
        let token = declare_token(&self.ctx, &entity)?;
        self.register_type(S::TYPE_NAME, S::type_description);
        Ok(Client {
            ctx: self.ctx.clone(),
            key: entity.topic_keyexpr().unwrap_or_default(),
            gid: entity.gid(self.ctx.compat().qos_defaults()),
            entity,
            seq: AtomicI64::new(1),
            format: self.ctx.compat().attachment_format(),
            _token: token,
            _service: PhantomData,
        })
    }
}

type Cache = Arc<Mutex<VecDeque<(Vec<u8>, Vec<u8>)>>>;

/// Whether a zenoh publisher has matching subscribers, kept current by
/// zenoh's matching listener, so checking it per message is a load instead
/// of a routing-table query. If the listener cannot be declared it always
/// answers true.
pub struct Matching {
    matched: Arc<AtomicBool>,
    _listener: Option<MatchingListener<()>>,
}

impl std::fmt::Debug for Matching {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Matching")
            .field("matched", &self.get())
            .finish_non_exhaustive()
    }
}

impl Matching {
    /// Watches `publisher`; zenoh reports the current status right away.
    #[must_use]
    pub fn watch(publisher: &zenoh::pubsub::Publisher<'_>) -> Self {
        Self::listen(publisher.matching_listener())
    }

    fn listen(builder: MatchingListenerBuilder<'_, DefaultHandler>) -> Self {
        let matched = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&matched);
        let listener = builder
            .callback(move |status| flag.store(status.matching(), Ordering::Relaxed))
            .wait();
        match listener {
            Ok(l) => Self {
                matched,
                _listener: Some(l),
            },
            Err(e) => {
                tracing::debug!("no matching listener ({e}); publishing unconditionally");
                matched.store(true, Ordering::Relaxed);
                Self {
                    matched,
                    _listener: None,
                }
            }
        }
    }

    /// Whether a subscriber currently matches; true if listener setup failed.
    #[must_use]
    pub fn get(&self) -> bool {
        self.matched.load(Ordering::Relaxed)
    }
}

/// The zenoh side of a publisher: plain for volatile topics, zenoh-ext's
/// AdvancedPublisher for transient-local ones, as `rmw_zenoh_cpp` does. (Its
/// volatile publishers are advanced publishers without any option, which
/// behave exactly like plain ones.)
enum ZenohPublisher {
    Plain(zenoh::pubsub::Publisher<'static>),
    Advanced(AdvancedPublisher<'static>),
}

impl ZenohPublisher {
    fn put(&self, payload: Vec<u8>, attachment: Vec<u8>) -> zenoh::Result<()> {
        match self {
            Self::Plain(p) => p.put(payload).attachment(attachment).wait(),
            Self::Advanced(p) => p.put(payload).attachment(attachment).wait(),
        }
    }

    fn matching(&self) -> Matching {
        match self {
            Self::Plain(p) => Matching::watch(p),
            Self::Advanced(p) => Matching::listen(p.matching_listener()),
        }
    }
}

/// Heartbeat period of reliable transient-local publishers, which lets
/// subscribers notice and recover a lost last sample (`rmw_zenoh_cpp`'s
/// `SAMPLE_MISS_DETECTION_HEARTBEAT_PERIOD`).
const HEARTBEAT_PERIOD: Duration = Duration::from_millis(500);

/// A publisher of pre-serialized CDR payloads; the untyped core of
/// [`Publisher`].
pub struct RawPublisher {
    entity: Entity,
    gid: Gid,
    key: String,
    // Declared before `publisher`, so it is dropped first: undeclaring
    // the publisher also removes its matching listeners.
    matching: Matching,
    publisher: ZenohPublisher,
    seq: AtomicI64,
    format: AttachmentFormat,
    /// Jazzy 0.2.0 transient-local history, served by `_cache_queryable`.
    cache: Option<(Cache, usize)>,
    _cache_queryable: Option<Queryable<()>>,
    _token: LivelinessToken,
}

impl std::fmt::Debug for RawPublisher {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RawPublisher")
            .field("key", &self.key)
            .field("gid", &self.gid)
            .finish_non_exhaustive()
    }
}

impl RawPublisher {
    fn new(ctx: &Context, entity: Entity) -> Result<Self> {
        let topic = entity.topic.clone().unwrap_or_else(unreachable_topic);
        let key = topic.keyexpr(ctx.domain_id());
        let transient_local = topic.qos.durability == Durability::TransientLocal;
        let advanced = transient_local && ctx.compat().advanced_pubsub();
        let depth = topic.qos.depth.max(1);
        let builder = ctx
            .session()
            .declare_publisher(key.clone())
            .congestion_control(topic.qos.congestion_control())
            .reliability(topic.qos.zenoh_reliability());
        let publisher = if advanced {
            // Detectable through liveliness and caching the last `depth`
            // samples for late joiners; since Jazzy 0.2.4, reliable ones
            // also announce their last sequence number so subscribers can
            // recover a lost one.
            let mut builder = builder
                .advanced()
                .publisher_detection()
                .cache(CacheConfig::default().max_samples(depth));
            if ctx.compat().sample_miss_recovery()
                && topic.qos.zenoh_reliability() == zenoh::qos::Reliability::Reliable
            {
                builder = builder.sample_miss_detection(
                    MissDetectionConfig::default().sporadic_heartbeat(HEARTBEAT_PERIOD),
                );
            }
            ZenohPublisher::Advanced(builder.wait().map_err(RosError::zenoh)?)
        } else {
            ZenohPublisher::Plain(builder.wait().map_err(RosError::zenoh)?)
        };

        // Jazzy 0.2.0 transient local: answer late joiners' queries on
        // `<zid>/<key>` with the last `depth` samples, like zenoh-ext's
        // PublicationCache.
        let (cache, cache_queryable) = if transient_local && !advanced {
            let cache: Cache = Arc::new(Mutex::new(VecDeque::new()));
            let reply_cache = Arc::clone(&cache);
            let reply_key = key.clone();
            let queryable = ctx
                .session()
                .declare_queryable(format!("{}/{key}", ctx.zid()))
                .complete(true)
                .callback(move |query| {
                    let samples: Vec<(Vec<u8>, Vec<u8>)> = reply_cache
                        .lock()
                        .unwrap_or_else(PoisonError::into_inner)
                        .iter()
                        .cloned()
                        .collect();
                    for (payload, attachment) in samples {
                        if let Err(e) = query
                            .reply(reply_key.clone(), payload)
                            .attachment(attachment)
                            .wait()
                        {
                            tracing::debug!("transient-local reply on {reply_key} failed: {e}");
                        }
                    }
                })
                .wait()
                .map_err(RosError::zenoh)?;
            (Some((cache, depth)), Some(queryable))
        } else {
            (None, None)
        };

        let token = declare_token(ctx, &entity)?;
        Ok(Self {
            gid: entity.gid(ctx.compat().qos_defaults()),
            entity,
            key,
            matching: publisher.matching(),
            publisher,
            seq: AtomicI64::new(1),
            format: ctx.compat().attachment_format(),
            cache,
            _cache_queryable: cache_queryable,
            _token: token,
        })
    }

    /// Publishes an encapsulated CDR payload.
    ///
    /// # Errors
    /// [`RosError::Zenoh`] when the put fails.
    pub fn publish_cdr(&self, payload: Vec<u8>) -> Result<()> {
        let seq = self.seq.fetch_add(1, Ordering::Relaxed);
        let attachment = Attachment::now(seq, self.gid).encode_as(self.format);
        if let Some((cache, depth)) = &self.cache {
            let mut cache = cache.lock().unwrap_or_else(PoisonError::into_inner);
            while cache.len() >= *depth {
                cache.pop_front();
            }
            cache.push_back((payload.clone(), attachment.clone()));
        }
        self.publisher
            .put(payload, attachment)
            .map_err(RosError::zenoh)
    }

    /// Whether any subscriber currently matches this publisher's key; true
    /// if listener setup failed.
    #[must_use]
    pub fn has_subscribers(&self) -> bool {
        self.matching.get()
    }

    /// The graph entity.
    #[must_use]
    pub fn entity(&self) -> &Entity {
        &self.entity
    }

    /// The publisher's GID.
    #[must_use]
    pub fn gid(&self) -> Gid {
        self.gid
    }

    /// The data key expression.
    #[must_use]
    pub fn keyexpr(&self) -> &str {
        &self.key
    }

    /// The fully qualified topic name.
    #[must_use]
    pub fn topic_name(&self) -> &str {
        self.entity.topic.as_ref().map_or("", |t| t.name.as_str())
    }
}

#[cold]
fn unreachable_topic() -> TopicInfo {
    // Every constructor passes a topic; this keeps the code panic-free.
    TopicInfo {
        name: "/".to_owned(),
        type_name: String::new(),
        type_hash: String::new(),
        qos: QosProfile::default(),
    }
}

/// A typed publisher.
pub struct Publisher<M> {
    raw: RawPublisher,
    /// The size of the last message, to size the next buffer in one go.
    last_len: AtomicUsize,
    _message: PhantomData<fn(&M)>,
}

impl<M> std::fmt::Debug for Publisher<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.raw.fmt(f)
    }
}

impl<M: Message> Publisher<M> {
    /// Serializes and publishes a message.
    ///
    /// # Errors
    /// [`RosError`] when encoding or the put fails.
    pub fn publish(&self, message: &M) -> Result<()> {
        // Messages on a topic are usually about the same size from one
        // publication to the next.
        let mut w = CdrWriter::with_capacity(self.last_len.load(Ordering::Relaxed));
        message.write(&mut w)?;
        self.last_len.store(w.body_len(), Ordering::Relaxed);
        self.raw.publish_cdr(w.into_bytes())
    }

    /// The untyped publisher.
    #[must_use]
    pub fn raw(&self) -> &RawPublisher {
        &self.raw
    }

    /// Whether any subscriber currently matches.
    #[must_use]
    pub fn has_subscribers(&self) -> bool {
        self.raw.has_subscribers()
    }
}

/// Metadata of a received message.
#[derive(Debug, Clone, PartialEq)]
pub struct MessageInfo {
    /// The publisher's attachment, when present and valid.
    pub attachment: Option<Attachment>,
    /// Reception time.
    pub received: Time,
}

/// The zenoh side of a subscription, mirroring [`ZenohPublisher`].
#[allow(dead_code, reason = "held only to keep the subscriber declared")]
enum ZenohSubscriber {
    Plain(zenoh::pubsub::Subscriber<()>),
    Advanced(AdvancedSubscriber<()>),
}

/// A typed subscription. Dropping it undeclares the subscriber; callbacks
/// already dispatched may finish afterward.
pub struct Subscription<M> {
    entity: Entity,
    alive: Arc<AtomicBool>,
    _subscriber: ZenohSubscriber,
    _publisher_watch: Option<zenoh::pubsub::Subscriber<()>>,
    _token: LivelinessToken,
    _message: PhantomData<fn(M)>,
}

impl<M> Drop for Subscription<M> {
    fn drop(&mut self) {
        // Suppress history replies that have not passed the alive check.
        // An already-dispatched callback may still complete after this drop.
        self.alive.store(false, Ordering::Relaxed);
    }
}

impl<M> std::fmt::Debug for Subscription<M> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Subscription")
            .field("entity", &self.entity)
            .finish_non_exhaustive()
    }
}

struct SubscriptionHandler<M, F> {
    callback: F,
    topic: String,
    alive: Arc<AtomicBool>,
    /// Jazzy 0.2.0 transient-local only: recent sequence numbers per
    /// publisher, since the live subscriber and the cache query can deliver
    /// the same message, and cached history may arrive after newer live
    /// messages. (zenoh-ext's AdvancedSubscriber does this itself.)
    seen: Option<Mutex<HashMap<Gid, Seen>>>,
    _message: PhantomData<fn(M)>,
}

/// Maximum sequence numbers remembered per publisher by transient-local
/// subscriptions.
const DEDUP_WINDOW: usize = 4096;

/// How long a departed publisher's sequence numbers are kept, so that a
/// cache reply still in flight when it left is recognized as a duplicate.
const DEPARTED_GRACE: Duration = Duration::from_secs(30);

/// What a transient-local subscription remembers of one publisher.
#[derive(Debug, Default)]
struct Seen {
    seqs: BTreeSet<i64>,
    /// When its liveliness token went away.
    departed: Option<Instant>,
}

/// Records a publisher's liveliness change at `now`: a departure starts
/// its grace period, a (re)appearance cancels it, and publishers departed
/// for longer than [`DEPARTED_GRACE`] are forgotten.
fn publisher_liveliness(seen: &mut HashMap<Gid, Seen>, gid: Gid, alive: bool, now: Instant) {
    if alive {
        if let Some(s) = seen.get_mut(&gid) {
            s.departed = None;
        }
    } else {
        seen.entry(gid).or_default().departed = Some(now);
    }
    seen.retain(|_, s| {
        s.departed
            .is_none_or(|t| now.saturating_duration_since(t) < DEPARTED_GRACE)
    });
}

/// Records `seq`; false when it was seen already or is older than the
/// remembered window.
fn first_delivery(seen: &mut BTreeSet<i64>, seq: i64) -> bool {
    if seen.len() >= DEDUP_WINDOW && seen.first().is_some_and(|&oldest| seq <= oldest) {
        return false;
    }
    if !seen.insert(seq) {
        return false;
    }
    if seen.len() > DEDUP_WINDOW {
        seen.pop_first();
    }
    true
}

impl<M: Message, F: Fn(M, &MessageInfo)> SubscriptionHandler<M, F> {
    fn handle(&self, sample: &Sample) {
        if !self.alive.load(Ordering::Relaxed) {
            return;
        }
        let attachment = sample
            .attachment()
            .and_then(|a| Attachment::decode(&a.to_bytes()).ok());
        if let (Some(a), Some(seen)) = (&attachment, &self.seen) {
            let mut seen = seen.lock().unwrap_or_else(PoisonError::into_inner);
            if !first_delivery(
                &mut seen.entry(a.source_gid).or_default().seqs,
                a.sequence_number,
            ) {
                return;
            }
        }
        match M::from_cdr(&sample.payload().to_bytes()) {
            Ok(message) => (self.callback)(
                message,
                &MessageInfo {
                    attachment,
                    received: Time::now(),
                },
            ),
            Err(e) => tracing::warn!("dropping undecodable message on {}: {e}", self.topic),
        }
    }
}

impl<M: Message> Subscription<M> {
    fn new<F>(ctx: &Context, entity: Entity, callback: F) -> Result<Self>
    where
        F: Fn(M, &MessageInfo) + Send + Sync + 'static,
    {
        let topic = entity.topic.clone().unwrap_or_else(unreachable_topic);
        let key = topic.keyexpr(ctx.domain_id());
        let transient_local = topic.qos.durability == Durability::TransientLocal;
        let advanced = transient_local && ctx.compat().advanced_pubsub();
        let alive = Arc::new(AtomicBool::new(true));
        let handler = Arc::new(SubscriptionHandler {
            callback,
            topic: topic.name.clone(),
            alive: Arc::clone(&alive),
            seen: (transient_local && !advanced).then(|| Mutex::new(HashMap::new())),
            _message: PhantomData,
        });
        let live = Arc::clone(&handler);
        let builder = ctx
            .session()
            .declare_subscriber(key.clone())
            .callback(move |sample| live.handle(&sample));
        let subscriber = if advanced {
            // `rmw_zenoh_cpp`'s transient-local subscription: detectable,
            // fetching up to `depth` samples of history from every current
            // and late-joining publisher, with queries that never time out;
            // since Jazzy 0.2.4 also recovering lost samples of reliable
            // ones.
            let builder = builder
                .advanced()
                .subscriber_detection()
                .query_timeout(Duration::from_millis(u64::MAX))
                .history(
                    HistoryConfig::default()
                        .detect_late_publishers()
                        .max_samples(topic.qos.depth.max(1)),
                );
            let builder = if ctx.compat().sample_miss_recovery()
                && topic.qos.zenoh_reliability() == zenoh::qos::Reliability::Reliable
            {
                builder.recovery(RecoveryConfig::default().heartbeat())
            } else {
                builder
            };
            ZenohSubscriber::Advanced(builder.wait().map_err(RosError::zenoh)?)
        } else {
            ZenohSubscriber::Plain(builder.wait().map_err(RosError::zenoh)?)
        };

        let mut publisher_watch = None;
        if transient_local && !advanced {
            // Watch for publishers appearing later, then fetch what every
            // current one has cached; in this order none falls in between.
            let watch_key = format!(
                "{ADMIN_SPACE}/{}/*/*/*/{}/*/*/*/{}/{}/{}/*",
                ctx.domain_id(),
                EntityKind::Publisher.as_str(),
                mangle(&topic.name),
                topic.type_name,
                topic.type_hash
            );
            let watch_ctx = ctx.clone();
            let watch_handler = Arc::clone(&handler);
            let watch_data_key = key.clone();
            publisher_watch = Some(
                ctx.session()
                    .liveliness()
                    .declare_subscriber(watch_key)
                    .callback(move |sample| {
                        let token = sample.key_expr().as_str();
                        let alive = sample.kind() == SampleKind::Put;
                        if let Some(seen) = &watch_handler.seen {
                            publisher_liveliness(
                                &mut seen.lock().unwrap_or_else(PoisonError::into_inner),
                                Gid::from_liveliness_keyexpr(token),
                                alive,
                                Instant::now(),
                            );
                        }
                        if !alive {
                            return;
                        }
                        let qos_defaults = watch_ctx.compat().qos_defaults();
                        let Some(publisher) = Entity::parse(token, qos_defaults) else {
                            return;
                        };
                        let selector = format!("{}/{watch_data_key}", publisher.zid);
                        if let Err(e) = query_cache(&watch_ctx, &selector, &watch_handler) {
                            tracing::debug!("transient-local query {selector} failed: {e}");
                        }
                    })
                    .wait()
                    .map_err(RosError::zenoh)?,
            );
            query_cache(ctx, &format!("*/{key}"), &handler)?;
        }

        let token = declare_token(ctx, &entity)?;
        Ok(Self {
            entity,
            alive,
            _subscriber: subscriber,
            _publisher_watch: publisher_watch,
            _token: token,
            _message: PhantomData,
        })
    }

    /// The graph entity.
    #[must_use]
    pub fn entity(&self) -> &Entity {
        &self.entity
    }
}

fn query_cache<M, F>(
    ctx: &Context,
    selector: &str,
    handler: &Arc<SubscriptionHandler<M, F>>,
) -> Result<()>
where
    M: Message,
    F: Fn(M, &MessageInfo) + Send + Sync + 'static,
{
    let handler = Arc::clone(handler);
    ctx.session()
        .get(selector.to_owned())
        .accept_replies(ReplyKeyExpr::Any)
        .target(QueryTarget::All)
        .consolidation(ConsolidationMode::None)
        // Like Jazzy 0.2.0, wait for every cache however slow.
        .timeout(Duration::from_millis(u64::MAX))
        .callback(move |reply| {
            if let Ok(sample) = reply.result() {
                handler.handle(sample);
            }
        })
        .wait()
        .map_err(RosError::zenoh)
}

/// Answers a request with an error, so the client does not wait for a reply
/// that never comes.
fn reply_err(query: &zenoh::query::Query, service: &str, reason: &str) {
    if let Err(e) = query.reply_err(reason.to_owned()).wait() {
        tracing::warn!("cannot reply on {service}: {e}");
    }
}

/// Metadata of a service request.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RequestInfo {
    /// The client's attachment (sequence number, timestamp and GID).
    pub attachment: Attachment,
}

/// A service server. Dropping it undeclares the queryable.
pub struct ServiceServer<S> {
    entity: Entity,
    _queryable: Queryable<()>,
    _token: LivelinessToken,
    _service: PhantomData<fn(S)>,
}

impl<S> std::fmt::Debug for ServiceServer<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ServiceServer")
            .field("entity", &self.entity)
            .finish_non_exhaustive()
    }
}

impl<S: Service> ServiceServer<S> {
    fn new<F>(ctx: &Context, entity: Entity, handler: F) -> Result<Self>
    where
        F: Fn(S::Request, &RequestInfo) -> S::Response + Send + Sync + 'static,
    {
        let key = entity.topic_keyexpr().unwrap_or_default();
        let reply_key = key.clone();
        let name = entity
            .topic
            .as_ref()
            .map(|t| t.name.clone())
            .unwrap_or_default();
        let queryable = ctx
            .session()
            .declare_queryable(key)
            .complete(true)
            .callback(move |query| {
                let Some((attachment, format)) = query
                    .attachment()
                    .and_then(|a| Attachment::decode_format(&a.to_bytes()).ok())
                else {
                    tracing::warn!("refusing request on {name} without a valid attachment");
                    reply_err(&query, &name, "missing or invalid attachment");
                    return;
                };
                let payload = query
                    .payload()
                    .map(|p| p.to_bytes().into_owned())
                    .unwrap_or_default();
                let request = match S::Request::from_cdr(&payload) {
                    Ok(r) => r,
                    Err(e) => {
                        tracing::warn!("refusing undecodable request on {name}: {e}");
                        reply_err(&query, &name, &e.to_string());
                        return;
                    }
                };
                let response = handler(request, &RequestInfo { attachment });
                let bytes = match response.to_cdr() {
                    Ok(b) => b,
                    Err(e) => {
                        // Answer anyway: the client would otherwise wait
                        // for a reply that never comes.
                        tracing::warn!("cannot encode response on {name}: {e}");
                        reply_err(&query, &name, &e.to_string());
                        return;
                    }
                };
                // Answer in the client's attachment format.
                let reply_attachment =
                    Attachment::now(attachment.sequence_number, attachment.source_gid);
                if let Err(e) = query
                    .reply(reply_key.clone(), bytes)
                    .attachment(reply_attachment.encode_as(format))
                    .wait()
                {
                    tracing::warn!("cannot reply on {name}: {e}");
                }
            })
            .wait()
            .map_err(RosError::zenoh)?;
        let token = declare_token(ctx, &entity)?;
        Ok(Self {
            entity,
            _queryable: queryable,
            _token: token,
            _service: PhantomData,
        })
    }

    /// The graph entity.
    #[must_use]
    pub fn entity(&self) -> &Entity {
        &self.entity
    }
}

/// A service client.
pub struct Client<S> {
    ctx: Context,
    entity: Entity,
    key: String,
    gid: Gid,
    seq: AtomicI64,
    format: AttachmentFormat,
    _token: LivelinessToken,
    _service: PhantomData<fn(S)>,
}

impl<S> std::fmt::Debug for Client<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Client")
            .field("entity", &self.entity)
            .finish_non_exhaustive()
    }
}

impl<S: Service> Client<S> {
    /// Calls the service and waits up to `timeout` for the response.
    ///
    /// # Errors
    /// [`RosError::Timeout`] when no server answers in time, which is at
    /// once when no server exists (see [`Client::service_is_ready`]);
    /// [`RosError::ServiceError`] when the server refuses the request; or
    /// another [`RosError`] for encoding and transport failures.
    pub fn call(&self, request: &S::Request, timeout: Duration) -> Result<S::Response> {
        let seq = self.seq.fetch_add(1, Ordering::Relaxed);
        let replies = self
            .ctx
            .session()
            .get(self.key.clone())
            .payload(request.to_cdr()?)
            .attachment(Attachment::now(seq, self.gid).encode_as(self.format))
            .target(QueryTarget::AllComplete)
            .consolidation(ConsolidationMode::None)
            .timeout(timeout)
            .wait()
            .map_err(RosError::zenoh)?;
        // The first reply wins; the channel closes empty on timeout.
        let Ok(reply) = replies.recv() else {
            return Err(RosError::Timeout);
        };
        match reply.result() {
            Ok(sample) => Ok(S::Response::from_cdr(&sample.payload().to_bytes())?),
            Err(e) => {
                let message = String::from_utf8_lossy(&e.payload().to_bytes()).into_owned();
                // zenoh answers a query that outlives its timeout with this
                // error reply, even when a server exists.
                if message == "Timeout" {
                    Err(RosError::Timeout)
                } else {
                    Err(RosError::ServiceError(message))
                }
            }
        }
    }

    /// Whether at least one server of this service is in the graph.
    ///
    /// # Errors
    /// [`RosError::Zenoh`] when the liveliness query fails.
    pub fn service_is_ready(&self, timeout: Duration) -> Result<bool> {
        let Some(topic) = &self.entity.topic else {
            return Ok(false);
        };
        let key = format!(
            "{ADMIN_SPACE}/{}/*/*/*/{}/*/*/*/{}/{}/{}/*",
            self.ctx.domain_id(),
            EntityKind::Service.as_str(),
            mangle(&topic.name),
            topic.type_name,
            topic.type_hash
        );
        let replies = self
            .ctx
            .session()
            .liveliness()
            .get(key)
            .timeout(timeout)
            .wait()
            .map_err(RosError::zenoh)?;
        while let Ok(reply) = replies.recv() {
            if reply.result().is_ok() {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// The graph entity.
    #[must_use]
    pub fn entity(&self) -> &Entity {
        &self.entity
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duplicates_are_dropped_but_late_history_is_kept() {
        let mut seen = BTreeSet::new();
        // Live message 10 first, then cached history 8..=10.
        assert!(first_delivery(&mut seen, 10));
        assert!(first_delivery(&mut seen, 8));
        assert!(first_delivery(&mut seen, 9));
        assert!(!first_delivery(&mut seen, 10));
        assert!(first_delivery(&mut seen, 11));
        // The window stays bounded; anything older than it is dropped.
        for seq in 100..100 + DEDUP_WINDOW as i64 {
            assert!(first_delivery(&mut seen, seq));
        }
        assert_eq!(seen.len(), DEDUP_WINDOW);
        assert!(!first_delivery(&mut seen, 50));
    }

    #[test]
    fn departed_publishers_are_remembered_for_a_grace_period() {
        let (a, b) = (Gid([1; 16]), Gid([2; 16]));
        let t0 = Instant::now();
        let mut seen: HashMap<Gid, Seen> = HashMap::new();
        assert!(first_delivery(&mut seen.entry(a).or_default().seqs, 7));
        assert!(first_delivery(&mut seen.entry(b).or_default().seqs, 1));
        publisher_liveliness(&mut seen, a, false, t0);
        // A cache reply still in flight when `a` left is a duplicate.
        assert!(!first_delivery(&mut seen.entry(a).or_default().seqs, 7));
        // A reappearance (a session that reconnected) cancels the departure.
        publisher_liveliness(&mut seen, a, true, t0 + DEPARTED_GRACE);
        assert!(seen.contains_key(&a));
        publisher_liveliness(&mut seen, b, false, t0);
        publisher_liveliness(&mut seen, a, true, t0 + DEPARTED_GRACE);
        assert!(!seen.contains_key(&b), "forgotten after the grace period");
        assert!(seen.contains_key(&a));
    }
}
