//! ROS 2 graph names: validation, expansion and sanitization.
//!
//! Mirrors `rmw_validate_*` and `rcl_expand_topic_name` for the subset of
//! syntax the bridge needs: absolute, relative and private (`~`) names, and
//! the `{node}` / `{ns}` / `{namespace}` substitutions.

/// Why a name is invalid.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum NameError {
    /// The name is empty.
    #[error("name is empty")]
    Empty,
    /// The name contains a character other than `[A-Za-z0-9_/~{}]`.
    #[error("name {name:?} contains invalid character {ch:?}")]
    InvalidCharacter {
        /// The name.
        name: String,
        /// The offending character.
        ch: char,
    },
    /// A token starts with a digit.
    #[error("name {0:?} has a token starting with a digit")]
    TokenStartsWithDigit(String),
    /// The name contains `//` or ends with `/`.
    #[error("name {0:?} has an empty token")]
    EmptyToken(String),
    /// A namespace is not absolute.
    #[error("namespace {0:?} must start with '/'")]
    NotAbsolute(String),
    /// `~` is used anywhere but at the start.
    #[error("name {0:?} uses '~' other than as the leading token")]
    MisplacedTilde(String),
    /// An unknown `{…}` substitution.
    #[error("name {0:?} has an unknown or unterminated substitution")]
    BadSubstitution(String),
    /// The name exceeds the 255-byte limit.
    #[error("name {0:?} is longer than 255 bytes")]
    TooLong(String),
}

const MAX_NAME_LEN: usize = 255;

fn validate_tokens(name: &str, body: &str) -> Result<(), NameError> {
    if body.is_empty() {
        return Ok(());
    }
    for token in body.split('/') {
        let Some(first) = token.chars().next() else {
            return Err(NameError::EmptyToken(name.to_owned()));
        };
        if first.is_ascii_digit() {
            return Err(NameError::TokenStartsWithDigit(name.to_owned()));
        }
        if let Some(ch) = token
            .chars()
            .find(|c| !(c.is_ascii_alphanumeric() || *c == '_'))
        {
            return Err(NameError::InvalidCharacter {
                name: name.to_owned(),
                ch,
            });
        }
    }
    Ok(())
}

/// Validates a node name: `[A-Za-z_][A-Za-z0-9_]*`.
///
/// # Errors
/// A [`NameError`] describing the first violation.
pub fn validate_node_name(name: &str) -> Result<(), NameError> {
    if name.is_empty() {
        return Err(NameError::Empty);
    }
    if name.len() > MAX_NAME_LEN {
        return Err(NameError::TooLong(name.to_owned()));
    }
    if name.contains('/') {
        return Err(NameError::InvalidCharacter {
            name: name.to_owned(),
            ch: '/',
        });
    }
    validate_tokens(name, name)
}

/// Validates an absolute namespace (`/` or `/a/b`).
///
/// # Errors
/// A [`NameError`] describing the first violation.
pub fn validate_namespace(ns: &str) -> Result<(), NameError> {
    if ns.is_empty() {
        return Err(NameError::Empty);
    }
    let Some(body) = ns.strip_prefix('/') else {
        return Err(NameError::NotAbsolute(ns.to_owned()));
    };
    if ns.len() > MAX_NAME_LEN {
        return Err(NameError::TooLong(ns.to_owned()));
    }
    validate_tokens(ns, body)
}

/// Validates a fully qualified topic or service name (`/a/b`).
///
/// # Errors
/// A [`NameError`] describing the first violation.
pub fn validate_full_topic_name(name: &str) -> Result<(), NameError> {
    if name.is_empty() {
        return Err(NameError::Empty);
    }
    let Some(body) = name.strip_prefix('/') else {
        return Err(NameError::NotAbsolute(name.to_owned()));
    };
    if body.is_empty() {
        return Err(NameError::EmptyToken(name.to_owned()));
    }
    if name.len() > MAX_NAME_LEN {
        return Err(NameError::TooLong(name.to_owned()));
    }
    validate_tokens(name, body)
}

/// Normalizes a namespace: empty becomes `/`, a missing leading `/` is
/// added and a trailing `/` removed.
#[must_use]
pub fn normalize_namespace(ns: &str) -> String {
    let trimmed = ns.trim_matches('/');
    format!("/{trimmed}")
}

/// Expands a topic or service name relative to a node, like
/// `rcl_expand_topic_name`, and validates the result.
///
/// `~` expands to the node's fully qualified name, relative names are
/// prefixed with the namespace and `{node}`, `{ns}` and `{namespace}` are
/// substituted.
///
/// # Errors
/// A [`NameError`] if the input or the expansion is invalid.
pub fn expand_topic_name(
    name: &str,
    node_name: &str,
    namespace: &str,
) -> Result<String, NameError> {
    if name.is_empty() {
        return Err(NameError::Empty);
    }
    if name.contains("//") || (name.len() > 1 && name.ends_with('/')) {
        return Err(NameError::EmptyToken(name.to_owned()));
    }
    if name.char_indices().any(|(i, c)| c == '~' && i != 0) {
        return Err(NameError::MisplacedTilde(name.to_owned()));
    }
    let ns = normalize_namespace(namespace);
    let node_fqn = if ns == "/" {
        format!("/{node_name}")
    } else {
        format!("{ns}/{node_name}")
    };

    let mut expanded = if name == "~" {
        node_fqn
    } else if let Some(rest) = name.strip_prefix("~/") {
        format!("{node_fqn}/{rest}")
    } else if name.starts_with('~') {
        return Err(NameError::MisplacedTilde(name.to_owned()));
    } else {
        name.to_owned()
    };

    // Substitutions.
    let mut out = String::with_capacity(expanded.len());
    let mut rest = expanded.as_str();
    while let Some((before, after)) = rest.split_once('{') {
        out.push_str(before);
        let Some((key, tail)) = after.split_once('}') else {
            return Err(NameError::BadSubstitution(name.to_owned()));
        };
        match key {
            "node" => out.push_str(node_name),
            "ns" | "namespace" => out.push_str(&ns),
            _ => return Err(NameError::BadSubstitution(name.to_owned())),
        }
        rest = tail;
    }
    out.push_str(rest);
    expanded = out;

    if !expanded.starts_with('/') {
        expanded = if ns == "/" {
            format!("/{expanded}")
        } else {
            format!("{ns}/{expanded}")
        };
    }
    // A `{ns}` substitution of the root namespace can leave a `//`.
    while expanded.contains("//") {
        expanded = expanded.replace("//", "/");
    }
    validate_full_topic_name(&expanded)?;
    Ok(expanded)
}

/// Turns an arbitrary label (a Motive asset name) into a valid ROS name
/// token: invalid characters become `_`, leading underscores are dropped
/// (they would hide the topics in ROS tools), a leading digit gets an `n`
/// prefix and a label with no alphanumeric character becomes `unnamed`.
#[must_use]
pub fn sanitize_token(label: &str) -> String {
    let mapped: String = label
        .trim()
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect();
    let mut out = mapped.trim_start_matches('_').to_owned();
    if out.is_empty() {
        return "unnamed".to_owned();
    }
    if out.starts_with(|c: char| c.is_ascii_digit()) {
        out.insert(0, 'n');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expansion() {
        assert_eq!(expand_topic_name("/tf", "n", "/").unwrap(), "/tf");
        assert_eq!(expand_topic_name("pose", "n", "/").unwrap(), "/pose");
        assert_eq!(
            expand_topic_name("pose", "n", "/mocap").unwrap(),
            "/mocap/pose"
        );
        assert_eq!(expand_topic_name("~", "n", "/mocap").unwrap(), "/mocap/n");
        assert_eq!(expand_topic_name("~/srv", "n", "/").unwrap(), "/n/srv");
        assert_eq!(
            expand_topic_name("~/srv", "n", "mocap/").unwrap(),
            "/mocap/n/srv"
        );
        assert_eq!(expand_topic_name("{node}/x", "n", "/a").unwrap(), "/a/n/x");
        assert_eq!(expand_topic_name("{ns}/x", "n", "/").unwrap(), "/x");
        // Like rcl, `{ns}` is the absolute namespace.
        assert_eq!(expand_topic_name("{ns}/x", "n", "/a").unwrap(), "/a/x");
        assert!(expand_topic_name("a/~", "n", "/").is_err());
        assert!(expand_topic_name("{bogus}", "n", "/").is_err());
        assert!(expand_topic_name("1abc", "n", "/").is_err());
        assert!(expand_topic_name("a//b", "n", "/").is_err());
        assert!(expand_topic_name("a/", "n", "/").is_err());
        assert!(expand_topic_name("a-b", "n", "/").is_err());
    }

    #[test]
    fn validation() {
        assert!(validate_node_name("natnet_zenoh").is_ok());
        assert!(validate_node_name("9x").is_err());
        assert!(validate_node_name("a/b").is_err());
        assert!(validate_namespace("/").is_ok());
        assert!(validate_namespace("/a/b").is_ok());
        assert!(validate_namespace("a").is_err());
        assert!(validate_namespace("/a/").is_err());
        assert_eq!(normalize_namespace(""), "/");
        assert_eq!(normalize_namespace("a/b/"), "/a/b");
    }

    #[test]
    fn sanitize() {
        assert_eq!(sanitize_token("Rover 1"), "Rover_1");
        assert_eq!(sanitize_token("3dof-arm"), "n3dof_arm");
        assert_eq!(sanitize_token("  "), "unnamed");
        assert_eq!(sanitize_token("Ärm"), "rm");
        assert_eq!(sanitize_token("_hidden"), "hidden");
        assert_eq!(sanitize_token("--"), "unnamed");
    }
}
