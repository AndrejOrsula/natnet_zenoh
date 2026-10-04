//! Turns raw NatNet datagrams into [`SourceEvent`]s.

use std::time::{Instant, SystemTime};

use natnet::command::ConnectOptions;
use natnet::{Decoder, Message, ParsePolicy, ServerInfo, Version};
use serde::{Deserialize, Serialize};
use tracing::info;

use super::{SourceContext, SourceEvent};

/// Frame `params` bit: the tracked model list changed.
pub const FRAME_MODELS_CHANGED: u16 = 0x02;

/// How strictly datagrams are validated.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Policy {
    /// Reject anything outside the documented grammar.
    Strict,
    /// Accept structurally sound datagrams with unknown extensions.
    #[default]
    Compatible,
}

impl From<Policy> for ParsePolicy {
    fn from(p: Policy) -> Self {
        match p {
            Policy::Strict => ParsePolicy::Strict,
            Policy::Compatible => ParsePolicy::Compatible,
        }
    }
}

/// What a datagram turned out to be.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    /// A SERVERINFO reply.
    ServerInfo(ServerInfo),
    /// A model definition.
    ModelDef,
    /// A frame; `models_changed` mirrors the frame's params bit.
    Frame {
        /// The server flagged a model change.
        models_changed: bool,
    },
    /// A text message from the server.
    Text,
    /// A message this bridge does not use.
    Ignored(u16),
    /// Undecodable, or not decodable yet (no protocol version known).
    Error,
    /// The event receiver is gone; the source should stop.
    Closed,
}

/// Decodes datagrams with the protocol version currently in force.
#[derive(Debug, Clone)]
pub struct Dispatcher {
    policy: ParsePolicy,
    forced: Option<Version>,
    requested: Option<Version>,
    decoder: Option<Decoder>,
}

impl Dispatcher {
    /// A dispatcher; `forced` pins the protocol version regardless of what
    /// SERVERINFO reports.
    #[must_use]
    pub fn new(policy: Policy, forced: Option<Version>) -> Self {
        let policy = ParsePolicy::from(policy);
        let decoder = forced.and_then(|v| Decoder::new(v).ok().map(|d| d.with_policy(policy)));
        Self {
            policy,
            forced,
            requested: None,
            decoder,
        }
    }

    /// Decodes with `requested`, the bitstream version the client asked for
    /// in `NAT_CONNECT`, whenever SERVERINFO shows a server that honours
    /// such a request (NatNet 3 or later).
    #[must_use]
    pub fn with_request(mut self, requested: Option<Version>) -> Self {
        self.requested = requested;
        self
    }

    /// The protocol version in force.
    #[must_use]
    pub fn version(&self) -> Option<Version> {
        self.decoder.map(Decoder::version)
    }

    /// Adopts `version` unless one is forced.
    ///
    /// # Errors
    /// A message for a version `natnet` cannot decode.
    pub fn adopt(&mut self, version: Version) -> Result<(), String> {
        if self.forced.is_some() {
            return Ok(());
        }
        let decoder = Decoder::new(version).map_err(|e| {
            format!(
                "unsupported NatNet protocol version {version}: {e}; set protocol_version to decode with a supported one"
            )
        })?;
        self.decoder = Some(decoder.with_policy(self.policy));
        Ok(())
    }

    /// Decodes a model definition with the version in force.
    ///
    /// # Errors
    /// A message when no version is known or decoding fails.
    pub fn decode_modeldef(&self, datagram: &[u8]) -> Result<(natnet::ModelDef, Version), String> {
        let decoder = self
            .decoder
            .ok_or_else(|| "no protocol version known yet".to_owned())?;
        match decoder
            .parse(datagram)
            .map_err(|e| e.to_string())?
            .into_message()
        {
            Message::ModelDef(md) => Ok((md, decoder.version())),
            _ => Err("not a MODELDEF datagram".to_owned()),
        }
    }

    /// Decodes one datagram and delivers the resulting event.
    pub fn dispatch(
        &mut self,
        ctx: &SourceContext,
        datagram: &[u8],
        received: SystemTime,
        received_mono: Instant,
    ) -> Outcome {
        if datagram.len() < 4 {
            return deliver(
                ctx,
                SourceEvent::DecodeError(format!("{}-byte datagram", datagram.len())),
                Outcome::Error,
            );
        }
        let id = u16::from_le_bytes([datagram[0], datagram[1]]);
        match id {
            natnet::NAT_SERVERINFO => match ServerInfo::parse(datagram) {
                Ok(mut info) => {
                    let stream = ConnectOptions {
                        subscribed_data_only: false,
                        bitstream_version: self.requested,
                    }
                    .stream_version(&info);
                    if stream != info.protocol_version && self.forced.is_none() {
                        info!(
                            "the server reports protocol {}; decoding the requested {stream}",
                            info.protocol_version
                        );
                    }
                    if let Err(e) = self.adopt(stream) {
                        return deliver(ctx, SourceEvent::DecodeError(e), Outcome::Error);
                    }
                    // Downstream (re-encoding raw frames, diagnostics) must
                    // see the version the frames are decoded with.
                    info.protocol_version = self.forced.unwrap_or(stream);
                    deliver(
                        ctx,
                        SourceEvent::ServerInfo(info.clone()),
                        Outcome::ServerInfo(info),
                    )
                }
                Err(e) => deliver(
                    ctx,
                    SourceEvent::DecodeError(format!("SERVERINFO: {e}")),
                    Outcome::Error,
                ),
            },
            natnet::NAT_MODELDEF => match self.decode_modeldef(datagram) {
                Ok((md, version)) => {
                    deliver(ctx, SourceEvent::ModelDef(md, version), Outcome::ModelDef)
                }
                Err(e) => deliver(
                    ctx,
                    SourceEvent::DecodeError(format!("MODELDEF: {e}")),
                    Outcome::Error,
                ),
            },
            natnet::NAT_FRAMEOFDATA => {
                let Some(decoder) = self.decoder else {
                    // Frames before SERVERINFO: nothing to decode them with.
                    return Outcome::Error;
                };
                match decoder
                    .parse(datagram)
                    .map(natnet::DecodedMessage::into_message)
                {
                    Ok(Message::FrameOfData(frame)) => {
                        let models_changed = frame.params & FRAME_MODELS_CHANGED != 0;
                        if ctx.send_frame(frame, received, received_mono) {
                            Outcome::Frame { models_changed }
                        } else {
                            Outcome::Closed
                        }
                    }
                    Ok(_) => Outcome::Error,
                    Err(e) => deliver(
                        ctx,
                        SourceEvent::DecodeError(format!("FRAMEOFDATA: {e}")),
                        Outcome::Error,
                    ),
                }
            }
            natnet::NAT_MESSAGESTRING => match natnet::parse_message_string(datagram) {
                Ok(text) => deliver(ctx, SourceEvent::ServerMessage(text), Outcome::Text),
                Err(e) => deliver(
                    ctx,
                    SourceEvent::DecodeError(format!("MESSAGESTRING: {e}")),
                    Outcome::Error,
                ),
            },
            other => Outcome::Ignored(other),
        }
    }
}

fn deliver(ctx: &SourceContext, event: SourceEvent, outcome: Outcome) -> Outcome {
    let delivered = if matches!(event, SourceEvent::DecodeError(_)) {
        ctx.report(event)
    } else {
        ctx.send(event)
    };
    if delivered { outcome } else { Outcome::Closed }
}

/// Loads a model definition from a file holding one raw MODELDEF datagram.
///
/// # Errors
/// A message for I/O or decoding failures.
pub fn load_modeldef(
    path: &std::path::Path,
    version: Version,
    policy: Policy,
) -> Result<natnet::ModelDef, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut d = Dispatcher::new(policy, Some(version));
    d.adopt(version)?;
    d.decode_modeldef(&bytes)
        .map(|(md, _)| md)
        .map_err(|e| format!("{}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, AtomicU64};

    use natnet::ConnectionInfo;

    use super::*;

    #[test]
    fn a_forced_version_is_the_one_reported() {
        let (tx, rx) = std::sync::mpsc::sync_channel(4);
        let ctx = SourceContext {
            events: tx,
            stop: Arc::new(AtomicBool::new(false)),
            dropped: Arc::new(AtomicU64::new(0)),
            dropped_reports: Arc::new(AtomicU64::new(0)),
            lossless: Arc::new(AtomicBool::new(false)),
        };
        let datagram = ServerInfo {
            application_name: "Motive".into(),
            server_version: Version::new(3, 1, 0, 0),
            protocol_version: Version::new(4, 0, 0, 0),
            high_res_clock_frequency: 10_000_000,
            connection: ConnectionInfo {
                data_port: 1511,
                multicast: true,
                multicast_group: [239, 255, 42, 99],
            },
        }
        .to_datagram()
        .unwrap();
        let mut d = Dispatcher::new(Policy::Compatible, Some(Version::V4_1));
        let outcome = d.dispatch(&ctx, &datagram, SystemTime::now(), Instant::now());
        assert!(
            matches!(&outcome, Outcome::ServerInfo(i) if i.protocol_version == Version::V4_1),
            "{outcome:?}"
        );
        match rx.try_recv() {
            Ok(SourceEvent::ServerInfo(i)) => assert_eq!(i.protocol_version, Version::V4_1),
            other => panic!("{other:?}"),
        }
    }
}
