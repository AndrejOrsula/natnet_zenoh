//! OMG CDR (XCDR version 1, "DDS_CDR") encoding as used by ROS 2 middlewares.
//!
//! `rmw_zenoh_cpp` serializes every message with Fast CDR in `DDS_CDR` mode:
//! a four-byte encapsulation header (`00 01 00 00` for little endian) followed
//! by the body, where every primitive is aligned to its own size **relative to
//! the start of the body** (the header does not count). Strings are a `u32`
//! length that includes a terminating NUL, the bytes, then the NUL. Sequences
//! are a `u32` element count followed by the elements; fixed arrays carry no
//! count.
//!
//! [`CdrWriter`] always emits little endian. [`CdrReader`] accepts both
//! encapsulations so that payloads from big-endian peers decode correctly.

use thiserror::Error;

/// Encapsulation header for little-endian plain CDR.
pub const CDR_LE: [u8; 4] = [0x00, 0x01, 0x00, 0x00];
/// Encapsulation header for big-endian plain CDR.
pub const CDR_BE: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
/// Length of the encapsulation header.
pub const ENCAPSULATION_LEN: usize = 4;

/// A CDR encoding or decoding failure.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum CdrError {
    /// The buffer ended before a value could be read.
    #[error("CDR buffer ended at offset {offset}: needed {needed} more bytes")]
    UnexpectedEof {
        /// Body offset of the failed read.
        offset: usize,
        /// Bytes that were required.
        needed: usize,
    },
    /// The encapsulation header is not a supported plain-CDR identifier.
    #[error("unsupported CDR encapsulation {0:02x?}")]
    UnsupportedEncapsulation([u8; 2]),
    /// A string or sequence is longer than a `u32` length can describe.
    #[error("length {0} does not fit a CDR u32 length field")]
    LengthOverflow(usize),
    /// A decoded boolean is neither 0 nor 1.
    #[error("invalid CDR boolean byte {0}")]
    InvalidBool(u8),
    /// A decoded element count cannot possibly fit in the remaining bytes.
    #[error("implausible CDR sequence length {count} with {remaining} bytes remaining")]
    ImplausibleLength {
        /// The decoded count.
        count: usize,
        /// Bytes remaining in the buffer.
        remaining: usize,
    },
    /// A bounded sequence exceeded its declared capacity.
    #[error("sequence of {len} elements exceeds its bound of {bound}")]
    BoundExceeded {
        /// Actual length.
        len: usize,
        /// Declared bound.
        bound: usize,
    },
}

/// Result alias for CDR operations.
pub type Result<T> = core::result::Result<T, CdrError>;

/// A little-endian CDR writer that starts with the encapsulation header.
#[derive(Debug, Clone)]
pub struct CdrWriter {
    buf: Vec<u8>,
}

impl Default for CdrWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl CdrWriter {
    /// A writer holding only the little-endian encapsulation header.
    #[must_use]
    pub fn new() -> Self {
        Self::with_capacity(64)
    }

    /// A writer with room for `capacity` body bytes.
    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        let mut buf = Vec::with_capacity(capacity + ENCAPSULATION_LEN);
        buf.extend_from_slice(&CDR_LE);
        Self { buf }
    }

    /// Number of body bytes written so far (excluding the header).
    #[must_use]
    pub fn body_len(&self) -> usize {
        self.buf.len() - ENCAPSULATION_LEN
    }

    /// The finished payload, header included.
    #[must_use]
    pub fn into_bytes(self) -> Vec<u8> {
        self.buf
    }

    /// Pads with zeros until the body length is a multiple of `align`.
    pub fn align(&mut self, align: usize) {
        let rem = self.body_len() % align;
        if rem != 0 {
            self.buf.resize(self.buf.len() + (align - rem), 0);
        }
    }

    /// Writes one byte.
    pub fn u8(&mut self, v: u8) {
        self.buf.push(v);
    }

    /// Writes one signed byte.
    pub fn i8(&mut self, v: i8) {
        self.buf.push(v.to_le_bytes()[0]);
    }

    /// Writes a boolean as one byte.
    pub fn bool(&mut self, v: bool) {
        self.buf.push(u8::from(v));
    }

    /// Writes an aligned `u16`.
    pub fn u16(&mut self, v: u16) {
        self.align(2);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Writes an aligned `i16`.
    pub fn i16(&mut self, v: i16) {
        self.align(2);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Writes an aligned `u32`.
    pub fn u32(&mut self, v: u32) {
        self.align(4);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Writes an aligned `i32`.
    pub fn i32(&mut self, v: i32) {
        self.align(4);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Writes an aligned `u64`.
    pub fn u64(&mut self, v: u64) {
        self.align(8);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Writes an aligned `i64`.
    pub fn i64(&mut self, v: i64) {
        self.align(8);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Writes an aligned `f32`.
    pub fn f32(&mut self, v: f32) {
        self.align(4);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Writes an aligned `f64`.
    pub fn f64(&mut self, v: f64) {
        self.align(8);
        self.buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Writes raw bytes without alignment.
    pub fn bytes(&mut self, v: &[u8]) {
        self.buf.extend_from_slice(v);
    }

    /// Writes a sequence or string length.
    ///
    /// # Errors
    /// [`CdrError::LengthOverflow`] when `len` exceeds `u32::MAX`.
    pub fn seq_len(&mut self, len: usize) -> Result<()> {
        let n = u32::try_from(len).map_err(|_| CdrError::LengthOverflow(len))?;
        self.u32(n);
        Ok(())
    }

    /// Writes a NUL-terminated CDR string. CDR cannot represent an interior
    /// NUL, so the string is cut at the first one, as C readers would see it.
    ///
    /// # Errors
    /// [`CdrError::LengthOverflow`].
    pub fn string(&mut self, s: &str) -> Result<()> {
        let bytes = s.as_bytes();
        let bytes = bytes
            .iter()
            .position(|&b| b == 0)
            .map_or(bytes, |nul| &bytes[..nul]);
        self.seq_len(bytes.len() + 1)?;
        self.buf.extend_from_slice(bytes);
        self.buf.push(0);
        Ok(())
    }
}

/// A CDR reader over one encapsulated payload.
#[derive(Debug, Clone)]
pub struct CdrReader<'a> {
    body: &'a [u8],
    pos: usize,
    little_endian: bool,
}

impl<'a> CdrReader<'a> {
    /// Reads the encapsulation header and positions at the body start.
    ///
    /// # Errors
    /// [`CdrError::UnexpectedEof`] for a payload shorter than the header and
    /// [`CdrError::UnsupportedEncapsulation`] for a non-plain-CDR identifier.
    pub fn new(payload: &'a [u8]) -> Result<Self> {
        let Some((header, body)) = payload.split_first_chunk::<ENCAPSULATION_LEN>() else {
            return Err(CdrError::UnexpectedEof {
                offset: 0,
                needed: ENCAPSULATION_LEN,
            });
        };
        let little_endian = match [header[0], header[1]] {
            [0x00, 0x01] => true,
            [0x00, 0x00] => false,
            other => return Err(CdrError::UnsupportedEncapsulation(other)),
        };
        Ok(Self {
            body,
            pos: 0,
            little_endian,
        })
    }

    /// Current body offset.
    #[must_use]
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Bytes left after the current position.
    #[must_use]
    pub fn remaining(&self) -> usize {
        self.body.len().saturating_sub(self.pos)
    }

    fn align(&mut self, align: usize) -> Result<()> {
        let rem = self.pos % align;
        if rem != 0 {
            self.take(align - rem)?;
        }
        Ok(())
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        let end = self
            .pos
            .checked_add(n)
            .filter(|end| *end <= self.body.len());
        let Some(end) = end else {
            return Err(CdrError::UnexpectedEof {
                offset: self.pos,
                needed: n,
            });
        };
        let out = &self.body[self.pos..end];
        self.pos = end;
        Ok(out)
    }

    fn array<const N: usize>(&mut self) -> Result<[u8; N]> {
        self.align(N)?;
        let bytes = self.take(N)?;
        let mut out = [0_u8; N];
        out.copy_from_slice(bytes);
        if !self.little_endian {
            out.reverse();
        }
        Ok(out)
    }

    /// Reads one byte.
    pub fn u8(&mut self) -> Result<u8> {
        Ok(self.take(1)?[0])
    }

    /// Reads one signed byte.
    pub fn i8(&mut self) -> Result<i8> {
        Ok(i8::from_le_bytes([self.u8()?]))
    }

    /// Reads a boolean byte, refusing values other than 0 and 1.
    pub fn bool(&mut self) -> Result<bool> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            other => Err(CdrError::InvalidBool(other)),
        }
    }

    /// Reads an aligned `u16`.
    pub fn u16(&mut self) -> Result<u16> {
        Ok(u16::from_le_bytes(self.array()?))
    }

    /// Reads an aligned `i16`.
    pub fn i16(&mut self) -> Result<i16> {
        Ok(i16::from_le_bytes(self.array()?))
    }

    /// Reads an aligned `u32`.
    pub fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.array()?))
    }

    /// Reads an aligned `i32`.
    pub fn i32(&mut self) -> Result<i32> {
        Ok(i32::from_le_bytes(self.array()?))
    }

    /// Reads an aligned `u64`.
    pub fn u64(&mut self) -> Result<u64> {
        Ok(u64::from_le_bytes(self.array()?))
    }

    /// Reads an aligned `i64`.
    pub fn i64(&mut self) -> Result<i64> {
        Ok(i64::from_le_bytes(self.array()?))
    }

    /// Reads an aligned `f32`.
    pub fn f32(&mut self) -> Result<f32> {
        Ok(f32::from_le_bytes(self.array()?))
    }

    /// Reads an aligned `f64`.
    pub fn f64(&mut self) -> Result<f64> {
        Ok(f64::from_le_bytes(self.array()?))
    }

    /// Reads `n` raw bytes without alignment.
    pub fn bytes(&mut self, n: usize) -> Result<&'a [u8]> {
        self.take(n)
    }

    /// Reads a sequence length and checks that `count` elements of at least
    /// `min_element` bytes can fit in the remaining buffer.
    pub fn seq_len(&mut self, min_element: usize) -> Result<usize> {
        let count = self.u32()? as usize;
        self.check_seq_len(count, min_element)?;
        Ok(count)
    }

    /// Reads a sequence length, rejecting counts above `bound` before item
    /// decoding, then checks that the minimum encoded elements fit.
    pub(crate) fn bounded_seq_len(&mut self, min_element: usize, bound: usize) -> Result<usize> {
        let count = self.u32()? as usize;
        if count > bound {
            return Err(CdrError::BoundExceeded { len: count, bound });
        }
        self.check_seq_len(count, min_element)?;
        Ok(count)
    }

    fn check_seq_len(&self, count: usize, min_element: usize) -> Result<()> {
        let remaining = self.remaining();
        if min_element > 0 && count > remaining / min_element {
            return Err(CdrError::ImplausibleLength { count, remaining });
        }
        Ok(())
    }

    /// Reads a CDR string and returns its content bytes without allocating.
    /// A final NUL is removed; missing terminators remain tolerated.
    pub(crate) fn string_bytes(&mut self) -> Result<&'a [u8]> {
        let len = self.seq_len(1)?;
        if len == 0 {
            // Some writers encode an empty string as a bare zero length.
            return Ok(&[]);
        }
        let bytes = self.take(len)?;
        Ok(bytes.strip_suffix(&[0]).unwrap_or(bytes))
    }

    /// Reads a NUL-terminated CDR string. A missing terminator is tolerated
    /// and invalid UTF-8 is replaced, as a lenient reader of foreign data.
    pub fn string(&mut self) -> Result<String> {
        Ok(String::from_utf8_lossy(self.string_bytes()?).into_owned())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alignment_is_relative_to_the_body() {
        let mut w = CdrWriter::new();
        w.u8(1);
        w.f64(2.0);
        let bytes = w.into_bytes();
        assert_eq!(&bytes[..4], &CDR_LE);
        // one byte, seven padding bytes, then the double.
        assert_eq!(bytes.len(), 4 + 16);
        assert_eq!(&bytes[4..12], &[1, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(f64::from_le_bytes(bytes[12..20].try_into().unwrap()), 2.0);
    }

    #[test]
    fn string_round_trip_and_layout() {
        let mut w = CdrWriter::new();
        w.string("map").unwrap();
        let bytes = w.into_bytes();
        assert_eq!(&bytes[4..], &[4, 0, 0, 0, b'm', b'a', b'p', 0]);
        let mut r = CdrReader::new(&bytes).unwrap();
        assert_eq!(r.string().unwrap(), "map");
        assert_eq!(r.remaining(), 0);
    }

    #[test]
    fn strings_are_encoded_and_decoded_leniently() {
        let mut w = CdrWriter::new();
        w.string("a\0b").unwrap();
        let bytes = w.into_bytes();
        assert_eq!(&bytes[4..], &[2, 0, 0, 0, b'a', 0]);

        let payload = [0x00, 0x01, 0x00, 0x00, 3, 0, 0, 0, b'a', 0xff, b'b'];
        let mut r = CdrReader::new(&payload).unwrap();
        assert_eq!(r.string().unwrap(), "a\u{fffd}b");
    }

    #[test]
    fn big_endian_payloads_decode() {
        let payload = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02];
        let mut r = CdrReader::new(&payload).unwrap();
        assert_eq!(r.u32().unwrap(), 0x0102);
    }

    #[test]
    fn truncated_and_implausible_inputs_are_errors() {
        assert!(CdrReader::new(&[0, 1]).is_err());
        let payload = [0x00, 0x01, 0x00, 0x00, 0xff, 0xff, 0xff, 0x7f];
        let mut r = CdrReader::new(&payload).unwrap();
        assert!(matches!(
            r.seq_len(1),
            Err(CdrError::ImplausibleLength { .. })
        ));
        let payload = [0x00, 0x01, 0x00, 0x00, 0x01];
        let mut r = CdrReader::new(&payload).unwrap();
        assert!(r.u32().is_err());
    }

    #[test]
    fn invalid_bool_is_refused() {
        let payload = [0x00, 0x01, 0x00, 0x00, 0x02];
        let mut r = CdrReader::new(&payload).unwrap();
        assert_eq!(r.bool(), Err(CdrError::InvalidBool(2)));
    }
}
