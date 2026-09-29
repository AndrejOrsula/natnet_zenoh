//! Raw NatNet capture files, read and written as a stream.
//!
//! Two containers are understood, both little-endian:
//!
//! * `.natnet`: no header; records of `u64 receive_ns, u32 length, bytes`.
//!   The receive time counts nanoseconds from the start of the recording.
//! * `.aucap` (auga raw-capture container, version 1): the header
//!   `b"AUGACAP\0", u16 version, u16 reserved`, then records of
//!   `u64 receive_ns, [u8; 4] source ip, u16 source port, u32 length, bytes`.
//!
//! The container is detected from the magic, not the file name.

use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::net::{Ipv4Addr, SocketAddrV4};
use std::path::Path;

/// The `.aucap` magic.
pub const AUCAP_MAGIC: [u8; 8] = *b"AUGACAP\0";
/// The `.aucap` version understood.
pub const AUCAP_VERSION: u16 = 1;
/// The largest datagram accepted in a record (the IPv4 UDP payload limit).
pub const MAX_RECORD: usize = 65_507;
/// Paced playback replays a pause between records up to this long, ns;
/// longer ones (a stopped recording, a corrupt timestamp) are skipped.
pub const MAX_PAUSE_NS: u64 = 60_000_000_000;

/// A capture container.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Format {
    /// `u64 ns, u32 len, bytes` records without a header.
    NatNet,
    /// The auga `.aucap` container.
    Aucap,
}

/// One captured datagram.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Record {
    /// Receive time, nanoseconds since the start of the recording.
    pub time_ns: u64,
    /// Datagram source, when the container records it.
    pub source: Option<SocketAddrV4>,
    /// The raw datagram.
    pub bytes: Vec<u8>,
}

/// A streaming capture reader.
pub struct CaptureReader {
    inner: Box<dyn Read + Send>,
    format: Format,
    index: usize,
    /// Set by the first error: the stream position is lost after it.
    failed: bool,
}

impl std::fmt::Debug for CaptureReader {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CaptureReader")
            .field("format", &self.format)
            .field("index", &self.index)
            .finish_non_exhaustive()
    }
}

impl CaptureReader {
    /// Opens a capture file, detecting its container.
    ///
    /// # Errors
    /// I/O errors, or `InvalidData` for an unsupported `.aucap` version.
    pub fn open(path: &Path) -> io::Result<Self> {
        Self::new(BufReader::with_capacity(1 << 20, File::open(path)?))
    }

    /// Wraps a reader, detecting the container from its first bytes.
    ///
    /// # Errors
    /// I/O errors, or `InvalidData` for an unsupported `.aucap` version.
    pub fn new(mut inner: impl Read + Send + 'static) -> io::Result<Self> {
        let mut head = [0_u8; 12];
        let n = read_full(&mut inner, &mut head)?;
        if n == 12 && head[..8] == AUCAP_MAGIC {
            let version = u16::from_le_bytes([head[8], head[9]]);
            if version != AUCAP_VERSION {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("unsupported .aucap version {version} (expected {AUCAP_VERSION})"),
                ));
            }
            return Ok(Self {
                inner: Box::new(inner),
                format: Format::Aucap,
                index: 0,
                failed: false,
            });
        }
        // Headerless: put the peeked bytes back in front.
        Ok(Self {
            inner: Box::new(io::Cursor::new(head[..n].to_vec()).chain(inner)),
            format: Format::NatNet,
            index: 0,
            failed: false,
        })
    }

    /// The detected container.
    #[must_use]
    pub fn format(&self) -> Format {
        self.format
    }
}

impl Iterator for CaptureReader {
    type Item = io::Result<Record>;

    /// The next record. After an error the iterator ends, since the
    /// following bytes can no longer be framed.
    fn next(&mut self) -> Option<Self::Item> {
        if self.failed {
            return None;
        }
        let item = self.read_record();
        if matches!(item, Some(Err(_))) {
            self.failed = true;
        }
        item
    }
}

impl CaptureReader {
    fn read_record(&mut self) -> Option<io::Result<Record>> {
        let index = self.index;
        let truncated = || {
            io::Error::new(
                io::ErrorKind::UnexpectedEof,
                format!("truncated capture record {index}"),
            )
        };
        let header_len = match self.format {
            Format::NatNet => 12,
            Format::Aucap => 18,
        };
        let mut header = [0_u8; 18];
        match read_full(&mut self.inner, &mut header[..header_len]) {
            Ok(0) => return None,
            Ok(n) if n < header_len => return Some(Err(truncated())),
            Ok(_) => {}
            Err(e) => return Some(Err(e)),
        }
        let le_u64 = |b: &[u8]| {
            let mut a = [0_u8; 8];
            a.copy_from_slice(b);
            u64::from_le_bytes(a)
        };
        let le_u32 = |b: &[u8]| {
            let mut a = [0_u8; 4];
            a.copy_from_slice(b);
            u32::from_le_bytes(a)
        };
        let time_ns = le_u64(&header[..8]);
        let (source, len) = match self.format {
            Format::NatNet => (None, le_u32(&header[8..12]) as usize),
            Format::Aucap => {
                let ip = Ipv4Addr::new(header[8], header[9], header[10], header[11]);
                let port = u16::from_le_bytes([header[12], header[13]]);
                (
                    Some(SocketAddrV4::new(ip, port)),
                    le_u32(&header[14..18]) as usize,
                )
            }
        };
        if len > MAX_RECORD {
            return Some(Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("capture record {index} claims {len} bytes; not a capture file?"),
            )));
        }
        let mut bytes = vec![0_u8; len];
        match read_full(&mut self.inner, &mut bytes) {
            Ok(n) if n == len => {}
            Ok(_) => return Some(Err(truncated())),
            Err(e) => return Some(Err(e)),
        }
        self.index += 1;
        Some(Ok(Record {
            time_ns,
            source,
            bytes,
        }))
    }
}

fn read_full(r: &mut impl Read, buf: &mut [u8]) -> io::Result<usize> {
    let mut filled = 0;
    while filled < buf.len() {
        match r.read(&mut buf[filled..]) {
            Ok(0) => break,
            Ok(n) => filled += n,
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }
    }
    Ok(filled)
}

/// A streaming capture writer.
#[derive(Debug)]
pub struct CaptureWriter<W: Write> {
    inner: W,
    format: Format,
}

impl CaptureWriter<BufWriter<File>> {
    /// Creates (truncating) a capture file.
    ///
    /// # Errors
    /// I/O errors.
    pub fn create(path: &Path, format: Format) -> io::Result<Self> {
        Self::new(BufWriter::new(File::create(path)?), format)
    }

    /// Creates a capture file that must not exist yet.
    ///
    /// # Errors
    /// I/O errors, [`io::ErrorKind::AlreadyExists`] for an existing file.
    pub fn create_new(path: &Path, format: Format) -> io::Result<Self> {
        Self::new(BufWriter::new(File::create_new(path)?), format)
    }
}

impl<W: Write> CaptureWriter<W> {
    /// Wraps a writer, emitting the container header when there is one.
    ///
    /// # Errors
    /// I/O errors.
    pub fn new(mut inner: W, format: Format) -> io::Result<Self> {
        if format == Format::Aucap {
            inner.write_all(&AUCAP_MAGIC)?;
            inner.write_all(&AUCAP_VERSION.to_le_bytes())?;
            inner.write_all(&0_u16.to_le_bytes())?;
        }
        Ok(Self { inner, format })
    }

    /// Appends one record. The source is written as `0.0.0.0:0` in an
    /// `.aucap` file when unknown and dropped in a `.natnet` file.
    ///
    /// # Errors
    /// I/O errors, or `InvalidInput` for an oversized datagram.
    pub fn write(&mut self, record: &Record) -> io::Result<()> {
        if record.bytes.len() > MAX_RECORD {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "datagram too large",
            ));
        }
        self.inner.write_all(&record.time_ns.to_le_bytes())?;
        if self.format == Format::Aucap {
            let source = record
                .source
                .unwrap_or(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0));
            self.inner.write_all(&source.ip().octets())?;
            self.inner.write_all(&source.port().to_le_bytes())?;
        }
        self.inner
            .write_all(&(record.bytes.len() as u32).to_le_bytes())?;
        self.inner.write_all(&record.bytes)
    }

    /// Flushes buffered records.
    ///
    /// # Errors
    /// I/O errors.
    pub fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }

    /// Flushes and returns the inner writer.
    ///
    /// # Errors
    /// I/O errors.
    pub fn into_inner(mut self) -> io::Result<W> {
        self.inner.flush()?;
        Ok(self.inner)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn records() -> Vec<Record> {
        vec![
            Record {
                time_ns: 5,
                source: Some(SocketAddrV4::new(Ipv4Addr::new(10, 0, 0, 2), 1511)),
                bytes: vec![7, 0, 2, 0, 1, 2],
            },
            Record {
                time_ns: 9,
                source: Some(SocketAddrV4::new(Ipv4Addr::new(10, 0, 0, 2), 1510)),
                bytes: vec![],
            },
        ]
    }

    #[test]
    fn round_trip_both_formats() {
        for format in [Format::NatNet, Format::Aucap] {
            let mut w = CaptureWriter::new(Vec::new(), format).unwrap();
            for r in records() {
                w.write(&r).unwrap();
            }
            let bytes = w.into_inner().unwrap();
            let reader = CaptureReader::new(io::Cursor::new(bytes)).unwrap();
            assert_eq!(reader.format(), format);
            let back: Vec<Record> = reader.map(Result::unwrap).collect();
            let mut expected = records();
            if format == Format::NatNet {
                for r in &mut expected {
                    r.source = None;
                }
            }
            assert_eq!(back, expected);
        }
    }

    #[test]
    fn empty_and_truncated() {
        assert_eq!(
            CaptureReader::new(io::Cursor::new(Vec::new()))
                .unwrap()
                .count(),
            0
        );
        let mut w = CaptureWriter::new(Vec::new(), Format::NatNet).unwrap();
        w.write(&records()[0]).unwrap();
        let mut bytes = w.into_inner().unwrap();
        bytes.pop();
        let mut reader = CaptureReader::new(io::Cursor::new(bytes)).unwrap();
        assert_eq!(
            reader.next().unwrap().unwrap_err().kind(),
            io::ErrorKind::UnexpectedEof
        );
    }

    #[test]
    fn rejects_other_aucap_versions_and_garbage() {
        let mut bytes = AUCAP_MAGIC.to_vec();
        bytes.extend_from_slice(&[2, 0, 0, 0]);
        assert!(CaptureReader::new(io::Cursor::new(bytes)).is_err());
        let garbage = vec![0xff_u8; 32];
        let mut reader = CaptureReader::new(io::Cursor::new(garbage)).unwrap();
        assert_eq!(
            reader.next().unwrap().unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
        // The rest cannot be framed, so the iterator ends.
        assert!(reader.next().is_none());
    }
}
