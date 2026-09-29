//! The per-sample attachment of `rmw_zenoh_cpp`.
//!
//! Every publication, service request and service reply carries a
//! `zenoh-ext`-serialized attachment. Since `rmw_zenoh_cpp` 0.2.4 (and in
//! every Humble, Kilted and Rolling release) it is three bare fields:
//!
//! ```text
//! i64 LE sequence_number
//! i64 LE source_timestamp (Unix nanoseconds)
//! varint(16) [u8; 16] source_gid
//! ```
//!
//! Jazzy 0.2.0 to 0.2.3 prefixed each field with its name:
//!
//! ```text
//! varint(15) "sequence_number"  i64 LE
//! varint(16) "source_timestamp" i64 LE
//! varint(10) "source_gid"       varint(16) [u8; 16]
//! ```
//!
//! A subscriber that cannot parse the format it expects throws inside a
//! zenoh callback, which aborts the whole ROS 2 process, so the encoder
//! writes exactly one format and the decoder accepts both.

use super::gid::Gid;

/// Encoded size of a current attachment.
pub const ATTACHMENT_LEN: usize = 33;

/// Encoded size of a legacy (Jazzy 0.2.0 to 0.2.3) attachment.
pub const LEGACY_ATTACHMENT_LEN: usize = 77;

/// The wire format of an attachment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AttachmentFormat {
    /// Bare fields, `rmw_zenoh_cpp` 0.2.4 and later.
    #[default]
    Current,
    /// Named fields, Jazzy `rmw_zenoh_cpp` 0.2.0 to 0.2.3.
    Legacy,
}

/// The decoded attachment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Attachment {
    /// Per-entity sequence number, starting at 1.
    pub sequence_number: i64,
    /// Source time in nanoseconds since the Unix epoch.
    pub source_timestamp: i64,
    /// GID of the publishing entity (or, in a service reply, of the client).
    pub source_gid: Gid,
}

/// Why an attachment could not be decoded.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum AttachmentError {
    /// Input ended early.
    #[error("attachment truncated")]
    Truncated,
    /// A field name did not match.
    #[error("attachment field {expected:?} not found")]
    UnexpectedField {
        /// The field name that was expected.
        expected: &'static str,
    },
    /// The GID was not 16 bytes.
    #[error("attachment gid has length {0}, expected 16")]
    GidLength(u64),
    /// Bytes were left over after the GID.
    #[error("attachment has {0} trailing bytes")]
    Trailing(usize),
}

fn put_str(out: &mut Vec<u8>, s: &str) {
    // All field names are shorter than 128 bytes: a one-byte LEB128 length.
    out.push(s.len() as u8);
    out.extend_from_slice(s.as_bytes());
}

struct Reader<'a>(&'a [u8]);

impl Reader<'_> {
    fn take(&mut self, n: usize) -> Result<&[u8], AttachmentError> {
        if self.0.len() < n {
            return Err(AttachmentError::Truncated);
        }
        let (head, tail) = self.0.split_at(n);
        self.0 = tail;
        Ok(head)
    }

    fn varint(&mut self) -> Result<u64, AttachmentError> {
        let mut value = 0_u64;
        for shift in (0..64).step_by(7) {
            let byte = self.take(1)?[0];
            value |= u64::from(byte & 0x7f) << shift;
            if byte & 0x80 == 0 {
                return Ok(value);
            }
        }
        Err(AttachmentError::Truncated)
    }

    fn name(&mut self, expected: &'static str) -> Result<(), AttachmentError> {
        let len = self.varint()?;
        let bytes = self.take(usize::try_from(len).map_err(|_| AttachmentError::Truncated)?)?;
        if bytes == expected.as_bytes() {
            Ok(())
        } else {
            Err(AttachmentError::UnexpectedField { expected })
        }
    }

    fn i64(&mut self) -> Result<i64, AttachmentError> {
        let bytes = self.take(8)?;
        let mut buf = [0_u8; 8];
        buf.copy_from_slice(bytes);
        Ok(i64::from_le_bytes(buf))
    }
}

impl Attachment {
    /// An attachment stamped with the current system time.
    #[must_use]
    pub fn now(sequence_number: i64, source_gid: Gid) -> Self {
        Self {
            sequence_number,
            source_timestamp: super::msgs::builtin_interfaces::Time::now().as_nanos(),
            source_gid,
        }
    }

    /// Encodes in the current wire format.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        self.encode_as(AttachmentFormat::Current)
    }

    /// Encodes in the given wire format.
    #[must_use]
    pub fn encode_as(&self, format: AttachmentFormat) -> Vec<u8> {
        match format {
            AttachmentFormat::Current => {
                let mut out = Vec::with_capacity(ATTACHMENT_LEN);
                out.extend_from_slice(&self.sequence_number.to_le_bytes());
                out.extend_from_slice(&self.source_timestamp.to_le_bytes());
                out.push(16);
                out.extend_from_slice(&self.source_gid.0);
                out
            }
            AttachmentFormat::Legacy => {
                let mut out = Vec::with_capacity(LEGACY_ATTACHMENT_LEN);
                put_str(&mut out, "sequence_number");
                out.extend_from_slice(&self.sequence_number.to_le_bytes());
                put_str(&mut out, "source_timestamp");
                out.extend_from_slice(&self.source_timestamp.to_le_bytes());
                put_str(&mut out, "source_gid");
                out.push(16);
                out.extend_from_slice(&self.source_gid.0);
                out
            }
        }
    }

    /// Decodes either wire format.
    ///
    /// # Errors
    /// An [`AttachmentError`] when the bytes are not an rmw_zenoh attachment.
    pub fn decode(bytes: &[u8]) -> Result<Self, AttachmentError> {
        Self::decode_format(bytes).map(|(a, _)| a)
    }

    /// Decodes either wire format and reports which one it was, so a reply
    /// can answer in kind.
    ///
    /// # Errors
    /// An [`AttachmentError`] when the bytes are not an rmw_zenoh attachment.
    pub fn decode_format(bytes: &[u8]) -> Result<(Self, AttachmentFormat), AttachmentError> {
        // A legacy attachment opens with the length and name of its first
        // field; a current one with a sequence number, which is never
        // `15` followed by `"sequence_number"` in practice.
        let legacy = bytes.len() == LEGACY_ATTACHMENT_LEN
            && bytes.first() == Some(&15)
            && bytes.get(1..16) == Some(b"sequence_number".as_slice());
        let mut r = Reader(bytes);
        if legacy {
            r.name("sequence_number")?;
        }
        let sequence_number = r.i64()?;
        if legacy {
            r.name("source_timestamp")?;
        }
        let source_timestamp = r.i64()?;
        if legacy {
            r.name("source_gid")?;
        }
        let len = r.varint()?;
        if len != 16 {
            return Err(AttachmentError::GidLength(len));
        }
        let mut gid = [0_u8; 16];
        gid.copy_from_slice(r.take(16)?);
        if !r.0.is_empty() {
            return Err(AttachmentError::Trailing(r.0.len()));
        }
        let format = if legacy {
            AttachmentFormat::Legacy
        } else {
            AttachmentFormat::Current
        };
        Ok((
            Self {
                sequence_number,
                source_timestamp,
                source_gid: Gid(gid),
            },
            format,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Attachment {
        Attachment {
            sequence_number: 1,
            source_timestamp: 0x0102_0304_0506_0708,
            source_gid: Gid([0xab; 16]),
        }
    }

    #[test]
    fn current_layout_and_round_trip() {
        let a = sample();
        let bytes = a.encode();
        assert_eq!(bytes.len(), ATTACHMENT_LEN);
        assert_eq!(&bytes[0..8], &1_i64.to_le_bytes());
        assert_eq!(&bytes[8..16], &0x0102_0304_0506_0708_i64.to_le_bytes());
        assert_eq!(bytes[16], 16);
        assert_eq!(&bytes[17..], &[0xab; 16]);
        assert_eq!(
            Attachment::decode_format(&bytes),
            Ok((a, AttachmentFormat::Current))
        );
    }

    #[test]
    fn legacy_layout_and_round_trip() {
        let a = sample();
        let bytes = a.encode_as(AttachmentFormat::Legacy);
        assert_eq!(bytes.len(), LEGACY_ATTACHMENT_LEN);
        assert_eq!(bytes[0], 15);
        assert_eq!(&bytes[1..16], b"sequence_number");
        assert_eq!(&bytes[16..24], &1_i64.to_le_bytes());
        assert_eq!(bytes[24], 16);
        assert_eq!(&bytes[25..41], b"source_timestamp");
        assert_eq!(bytes[49], 10);
        assert_eq!(&bytes[50..60], b"source_gid");
        assert_eq!(bytes[60], 16);
        assert_eq!(
            Attachment::decode_format(&bytes),
            Ok((a, AttachmentFormat::Legacy))
        );
    }

    #[test]
    fn rejects_malformed() {
        let bytes = Attachment::now(3, Gid::default()).encode();
        assert_eq!(
            Attachment::decode(&bytes[..20]),
            Err(AttachmentError::Truncated)
        );
        let mut long = bytes.clone();
        long.push(0);
        assert_eq!(Attachment::decode(&long), Err(AttachmentError::Trailing(1)));
        let mut bad = bytes;
        bad[16] = 15;
        assert_eq!(
            Attachment::decode(&bad),
            Err(AttachmentError::GidLength(15))
        );
        let legacy = Attachment::now(3, Gid::default()).encode_as(AttachmentFormat::Legacy);
        let mut bad = legacy.clone();
        bad[20 + 5] = b'x';
        assert!(matches!(
            Attachment::decode(&bad),
            Err(AttachmentError::UnexpectedField { .. })
        ));
        let mut bad = legacy;
        bad[60] = 15;
        assert_eq!(
            Attachment::decode(&bad),
            Err(AttachmentError::GidLength(15))
        );
    }
}
