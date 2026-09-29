//! `rmw_zenoh_cpp` graph entities and their liveliness tokens.
//!
//! Each node, publisher, subscription, service and client declares a zenoh
//! liveliness token under the `@ros2_lv` admin space. The ROS 2 graph
//! (`ros2 node list`, `ros2 topic list`, matching of publishers and
//! subscriptions, service availability) is built entirely from these tokens:
//!
//! ```text
//! @ros2_lv/<domain>/<zid>/<nid>/<id>/<kind>/<enclave>/<namespace>/<node>
//!     [/<topic>/<type>/<type hash>/<qos>]
//! ```
//!
//! with every `/` inside a component replaced by `%`.

use super::gid::Gid;
use super::qos::QosProfile;

/// The liveliness admin space prefix.
pub const ADMIN_SPACE: &str = "@ros2_lv";

/// Replacement for `/` inside a key-expression component.
const SLASH_REPLACEMENT: char = '%';

/// The kind of a graph entity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EntityKind {
    /// A node (`NN`).
    Node,
    /// A publisher (`MP`).
    Publisher,
    /// A subscription (`MS`).
    Subscription,
    /// A service server (`SS`).
    Service,
    /// A service client (`SC`).
    Client,
}

impl EntityKind {
    /// The two-letter token component.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Node => "NN",
            Self::Publisher => "MP",
            Self::Subscription => "MS",
            Self::Service => "SS",
            Self::Client => "SC",
        }
    }

    fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "NN" => Self::Node,
            "MP" => Self::Publisher,
            "MS" => Self::Subscription,
            "SS" => Self::Service,
            "SC" => Self::Client,
            _ => return None,
        })
    }
}

/// Identity of a node.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeInfo {
    /// ROS domain id.
    pub domain_id: u32,
    /// Security enclave, `/` by default.
    pub enclave: String,
    /// Absolute namespace, `/` for the root.
    pub namespace: String,
    /// Node name.
    pub name: String,
}

/// Topic or service of a non-node entity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicInfo {
    /// Fully qualified name, e.g. `/tf`.
    pub name: String,
    /// DDS-mangled type name, e.g. `tf2_msgs::msg::dds_::TFMessage_`.
    pub type_name: String,
    /// `RIHS01_…` type hash.
    pub type_hash: String,
    /// Adapted QoS profile.
    pub qos: QosProfile,
}

impl TopicInfo {
    /// The data key expression: `<domain>/<name>/<type>/<hash>` with the
    /// name's leading and trailing slashes stripped.
    #[must_use]
    pub fn keyexpr(&self, domain_id: u32) -> String {
        format!(
            "{domain_id}/{}/{}/{}",
            self.name.trim_start_matches('/').trim_end_matches('/'),
            self.type_name,
            self.type_hash
        )
    }
}

/// A graph entity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Entity {
    /// Session zenoh id, as printed by zenoh.
    pub zid: String,
    /// Id of the owning node.
    pub nid: u64,
    /// Id of this entity (equal to `nid` for a node).
    pub id: u64,
    /// Entity kind.
    pub kind: EntityKind,
    /// Owning node.
    pub node: NodeInfo,
    /// Topic or service, for everything but nodes.
    pub topic: Option<TopicInfo>,
}

/// Replaces `/` with `%`.
#[must_use]
pub fn mangle(s: &str) -> String {
    s.replace('/', "%")
}

/// Replaces `%` with `/`.
#[must_use]
pub fn demangle(s: &str) -> String {
    s.replace(SLASH_REPLACEMENT, "/")
}

impl Entity {
    /// The liveliness token key expression, with the QoS encoded against
    /// the release's `qos_defaults` ([`RmwZenohCompat::qos_defaults`]).
    ///
    /// [`RmwZenohCompat::qos_defaults`]: super::context::RmwZenohCompat::qos_defaults
    #[must_use]
    pub fn liveliness_keyexpr(&self, qos_defaults: &QosProfile) -> String {
        let mut parts = vec![
            ADMIN_SPACE.to_owned(),
            self.node.domain_id.to_string(),
            self.zid.clone(),
            self.nid.to_string(),
            self.id.to_string(),
            self.kind.as_str().to_owned(),
            mangle(&self.node.enclave),
            mangle(&self.node.namespace),
            mangle(&self.node.name),
        ];
        if let Some(t) = &self.topic {
            parts.push(mangle(&t.name));
            parts.push(mangle(&t.type_name));
            parts.push(mangle(&t.type_hash));
            parts.push(t.qos.to_keyexpr(qos_defaults));
        }
        parts.join("/")
    }

    /// The entity GID, the hash of its liveliness token key expression.
    #[must_use]
    pub fn gid(&self, qos_defaults: &QosProfile) -> Gid {
        Gid::from_liveliness_keyexpr(&self.liveliness_keyexpr(qos_defaults))
    }

    /// The data key expression, for non-node entities.
    #[must_use]
    pub fn topic_keyexpr(&self) -> Option<String> {
        self.topic.as_ref().map(|t| t.keyexpr(self.node.domain_id))
    }

    /// Parses a liveliness token key expression, reading the QoS against
    /// the release's `qos_defaults`.
    #[must_use]
    pub fn parse(keyexpr: &str, qos_defaults: &QosProfile) -> Option<Self> {
        let parts: Vec<&str> = keyexpr.split('/').collect();
        if parts.len() < 9 || parts[0] != ADMIN_SPACE {
            return None;
        }
        let kind = EntityKind::parse(parts[5])?;
        let node = NodeInfo {
            domain_id: parts[1].parse().ok()?,
            enclave: demangle(parts[6]),
            namespace: demangle(parts[7]),
            name: demangle(parts[8]),
        };
        let topic = if kind == EntityKind::Node {
            None
        } else {
            if parts.len() < 13 {
                return None;
            }
            Some(TopicInfo {
                name: demangle(parts[9]),
                type_name: demangle(parts[10]),
                type_hash: demangle(parts[11]),
                qos: QosProfile::from_keyexpr(parts[12], qos_defaults)?,
            })
        };
        Some(Self {
            zid: parts[2].to_owned(),
            nid: parts[3].parse().ok()?,
            id: parts[4].parse().ok()?,
            kind,
            node,
            topic,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const D: &QosProfile = &QosProfile::RMW_ZENOH_DEFAULT;

    fn node() -> NodeInfo {
        NodeInfo {
            domain_id: 0,
            enclave: "/".into(),
            namespace: "/".into(),
            name: "natnet".into(),
        }
    }

    #[test]
    fn node_token() {
        let e = Entity {
            zid: "a1b2".into(),
            nid: 0,
            id: 0,
            kind: EntityKind::Node,
            node: node(),
            topic: None,
        };
        assert_eq!(e.liveliness_keyexpr(D), "@ros2_lv/0/a1b2/0/0/NN/%/%/natnet");
        assert_eq!(Entity::parse(&e.liveliness_keyexpr(D), D), Some(e));
    }

    #[test]
    fn publisher_token_and_key() {
        let e = Entity {
            zid: "a1b2".into(),
            nid: 0,
            id: 3,
            kind: EntityKind::Publisher,
            node: NodeInfo {
                namespace: "/mocap".into(),
                ..node()
            },
            topic: Some(TopicInfo {
                name: "/mocap/rigid_bodies/rover".into(),
                type_name: "geometry_msgs::msg::dds_::PoseStamped_".into(),
                type_hash: "RIHS01_10f3".into(),
                qos: QosProfile::sensor_data().adapted(D),
            }),
        };
        assert_eq!(
            e.liveliness_keyexpr(D),
            "@ros2_lv/0/a1b2/0/3/MP/%/%mocap/natnet/%mocap%rigid_bodies%rover/geometry_msgs::msg::dds_::PoseStamped_/RIHS01_10f3/2::,5:,:,:,,"
        );
        assert_eq!(
            e.topic_keyexpr().unwrap(),
            "0/mocap/rigid_bodies/rover/geometry_msgs::msg::dds_::PoseStamped_/RIHS01_10f3"
        );
        assert_eq!(Entity::parse(&e.liveliness_keyexpr(D), D), Some(e));
    }
}
