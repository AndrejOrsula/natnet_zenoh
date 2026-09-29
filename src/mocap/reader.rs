//! A bounds-checked little-endian reader for the NatNet bodies that the
//! `natnet` crate keeps opaque (raw frame sections and non-core datasets).

/// Why an opaque body could not be decoded.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum ReadError {
    /// The body ended before a field.
    #[error("truncated at byte {offset}: need {needed} more bytes")]
    Truncated {
        /// Offset of the field.
        offset: usize,
        /// Bytes needed.
        needed: usize,
    },
    /// A count is negative or larger than the remaining bytes allow.
    #[error("implausible count {count} at byte {offset}")]
    Count {
        /// The count read.
        count: i32,
        /// Offset of the count.
        offset: usize,
    },
    /// A string is not NUL-terminated.
    #[error("unterminated string at byte {0}")]
    UnterminatedString(usize),
}

/// A cursor over a byte slice.
#[derive(Debug, Clone)]
pub struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    /// A reader at the start of `data`.
    #[must_use]
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    /// Bytes left.
    #[must_use]
    pub fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }

    /// Current offset.
    #[must_use]
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Takes `n` bytes.
    ///
    /// # Errors
    /// [`ReadError::Truncated`].
    pub fn take(&mut self, n: usize) -> Result<&'a [u8], ReadError> {
        if self.remaining() < n {
            return Err(ReadError::Truncated {
                offset: self.pos,
                needed: n - self.remaining(),
            });
        }
        let out = &self.data[self.pos..self.pos + n];
        self.pos += n;
        Ok(out)
    }

    fn array<const N: usize>(&mut self) -> Result<[u8; N], ReadError> {
        let mut out = [0_u8; N];
        out.copy_from_slice(self.take(N)?);
        Ok(out)
    }

    /// Reads a `u16`.
    ///
    /// # Errors
    /// [`ReadError::Truncated`].
    pub fn u16(&mut self) -> Result<u16, ReadError> {
        Ok(u16::from_le_bytes(self.array()?))
    }

    /// Reads an `i32`.
    ///
    /// # Errors
    /// [`ReadError::Truncated`].
    pub fn i32(&mut self) -> Result<i32, ReadError> {
        Ok(i32::from_le_bytes(self.array()?))
    }

    /// Reads an `f32`.
    ///
    /// # Errors
    /// [`ReadError::Truncated`].
    pub fn f32(&mut self) -> Result<f32, ReadError> {
        Ok(f32::from_le_bytes(self.array()?))
    }

    /// Reads three `f32`.
    ///
    /// # Errors
    /// [`ReadError::Truncated`].
    pub fn vec3(&mut self) -> Result<[f32; 3], ReadError> {
        Ok([self.f32()?, self.f32()?, self.f32()?])
    }

    /// Reads a quaternion `[x, y, z, w]`.
    ///
    /// # Errors
    /// [`ReadError::Truncated`].
    pub fn quat(&mut self) -> Result<[f32; 4], ReadError> {
        Ok([self.f32()?, self.f32()?, self.f32()?, self.f32()?])
    }

    /// Reads a count whose elements take at least `min_element` bytes.
    ///
    /// # Errors
    /// [`ReadError::Count`] for a negative or implausible count.
    pub fn count(&mut self, min_element: usize) -> Result<usize, ReadError> {
        let offset = self.pos;
        let count = self.i32()?;
        let n = usize::try_from(count).map_err(|_| ReadError::Count { count, offset })?;
        if n.saturating_mul(min_element.max(1)) > self.remaining() {
            return Err(ReadError::Count { count, offset });
        }
        Ok(n)
    }

    /// Reads a NUL-terminated string (lossy UTF-8).
    ///
    /// # Errors
    /// [`ReadError::UnterminatedString`].
    pub fn cstr(&mut self) -> Result<String, ReadError> {
        let rest = &self.data[self.pos..];
        let Some(nul) = rest.iter().position(|&b| b == 0) else {
            return Err(ReadError::UnterminatedString(self.pos));
        };
        let s = String::from_utf8_lossy(&rest[..nul]).into_owned();
        self.pos += nul + 1;
        Ok(s)
    }
}

/// A little-endian writer, used to build synthetic model definitions and
/// raw sections.
#[derive(Debug, Default, Clone)]
pub struct Writer {
    /// The bytes written so far.
    pub bytes: Vec<u8>,
}

impl Writer {
    /// Writes a `u16`.
    pub fn u16(&mut self, v: u16) -> &mut Self {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// Writes an `i32`.
    pub fn i32(&mut self, v: i32) -> &mut Self {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// Writes an `f32`.
    pub fn f32(&mut self, v: f32) -> &mut Self {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// Writes several `f32`.
    pub fn f32s(&mut self, v: &[f32]) -> &mut Self {
        for x in v {
            self.f32(*x);
        }
        self
    }

    /// Writes a NUL-terminated string.
    pub fn cstr(&mut self, s: &str) -> &mut Self {
        self.bytes.extend_from_slice(s.as_bytes());
        self.bytes.push(0);
        self
    }

    /// Writes a count.
    pub fn count(&mut self, n: usize) -> &mut Self {
        self.i32(i32::try_from(n).unwrap_or(i32::MAX))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_and_errors() {
        let mut w = Writer::default();
        w.cstr("ab").i32(2).f32s(&[1.0, 2.0]).u16(7);
        let mut r = Reader::new(&w.bytes);
        assert_eq!(r.cstr().unwrap(), "ab");
        assert_eq!(r.count(4).unwrap(), 2);
        assert_eq!(r.f32().unwrap(), 1.0);
        assert_eq!(r.f32().unwrap(), 2.0);
        assert_eq!(r.u16().unwrap(), 7);
        assert_eq!(r.remaining(), 0);
        assert!(matches!(r.i32(), Err(ReadError::Truncated { .. })));

        let mut w = Writer::default();
        w.i32(-1).i32(1000);
        let mut r = Reader::new(&w.bytes);
        assert!(matches!(r.count(1), Err(ReadError::Count { .. })));
        assert!(matches!(r.count(1), Err(ReadError::Count { .. })));
        assert!(matches!(
            Reader::new(b"abc").cstr(),
            Err(ReadError::UnterminatedString(0))
        ));
    }
}
