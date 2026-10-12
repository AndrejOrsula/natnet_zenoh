//! Entity GIDs: the XXH3-128 hash of an entity's liveliness key expression.

use std::fmt;

/// A 16-byte entity GID (`RMW_GID_STORAGE_SIZE`).
#[derive(Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Gid(pub [u8; 16]);

impl Gid {
    /// The GID `rmw_zenoh_cpp` derives from a liveliness key expression:
    /// XXH3-128 serialized as a little-endian `u128` (low 64 bits then high 64 bits).
    #[must_use]
    pub fn from_liveliness_keyexpr(keyexpr: &str) -> Self {
        let h = xxhash_rust::xxh3::xxh3_128(keyexpr.as_bytes());
        Self(h.to_le_bytes())
    }
}

impl fmt::Debug for Gid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Gid({self})")
    }
}

impl fmt::Display for Gid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The `ros2 topic info -v` format: dot-separated bytes.
        for (i, b) in self.0.iter().enumerate() {
            if i > 0 {
                f.write_str(".")?;
            }
            write!(f, "{b:02x}")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xxh3_128_layout() {
        // XXH3-128 of the empty input is 99aa06d3014798d86001c324468d497f
        // (high 64 bits first in the canonical representation).
        let g = Gid::from_liveliness_keyexpr("");
        assert_eq!(
            g.0,
            [
                0x7f, 0x49, 0x8d, 0x46, 0x24, 0xc3, 0x01, 0x60, 0xd8, 0x98, 0x47, 0x01, 0xd3, 0x06,
                0xaa, 0x99
            ]
        );
    }
}
