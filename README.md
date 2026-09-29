# natnet_zenoh

A pure-Rust bridge from OptiTrack Motive (NatNet) to [Zenoh](https://zenoh.io),
with native ROS 2 interoperability through the `rmw_zenoh` protocol.

Stock ROS 2 nodes using `RMW_IMPLEMENTATION=rmw_zenoh_cpp` see the bridge as an
ordinary node. They can subscribe to its topics, call its services, and read
its parameters and type descriptions. The bridge itself needs no ROS 2 install,
no C/C++ toolchain (in the default build), and no code generation. It
reproduces the key expressions, liveliness tokens, attachments, QoS encoding,
RIHS01 type hashes, and CDR payloads of `rmw_zenoh_cpp` byte for byte.
Transient-local topics go through zenoh-ext's AdvancedPublisher and
AdvancedSubscriber, as in `rmw_zenoh_cpp` since Jazzy 0.2.2.

The NatNet protocol comes from the [`natnet`](https://github.com/AndrejOrsula/natnet)
crate, pinned to a git revision in `Cargo.toml`. See
[Development](#development) for building against a local checkout.

## `rmw_zenoh_cpp` versions

Jazzy's `rmw_zenoh_cpp` changed its protocol three times after its first
release, and Humble's has no type hashes. A node that receives an attachment
format it does not expect can abort, and topics whose type hashes differ never
match, so set `ros.rmw_zenoh` to match the other nodes (`ros2 pkg xml
rmw_zenoh_cpp --tag version` prints their release):

| `ros.rmw_zenoh` | Peers | Attachments | Default durability | Transient local |
| --- | --- | --- | --- | --- |
| `current` (default) | Jazzy 0.2.4 and later (the current apt packages), and the current Kilted and Rolling releases | Bare fields | Volatile | Advanced pub/sub, with sample-miss recovery |
| `humble` | Humble (0.1.x) | Bare fields | Volatile | Advanced pub/sub, with sample-miss recovery |
| `jazzy-0.2.3` | Jazzy 0.2.3 | Named fields | Volatile | Advanced pub/sub |
| `jazzy-0.2.2` | Jazzy 0.2.2 | Named fields | Transient local | Advanced pub/sub |
| `jazzy-0.2.0` | Jazzy 0.2.0 and 0.2.1 (for example RoboStack's 0.2.0) | Named fields | Transient local | A queryable cache on `<zid>/<key>` |

The default durability is the reference of the QoS that liveliness tokens
carry, so a mismatch shows up as the wrong durability in `ros2 topic info
-v` and in the QoS that tools such as `ros2 topic echo` adapt to. The bridge
reads either attachment format from others, and answers each service client
in the format the client used.

With `humble`, key expressions and liveliness tokens carry
`TypeHashNotSupported` instead of a RIHS01 hash, as Humble's `rmw_zenoh_cpp`
writes them, and `~/get_type_description` is not served. Humble's `ros2`
daemon fails on that `rmw_zenoh_cpp` for every node, native ones included
(`unknown tag 'rclpy.type_hash.TypeHash'`), so give `ros2 topic echo` and
`ros2 topic info` the `--no-daemon` flag there.

## Quick start

```sh
cargo build --release

# A zenoh router, as rmw_zenoh expects (from any ROS 2 Jazzy+ install):
ros2 run rmw_zenoh_cpp rmw_zenohd

# Bridge a Motive server (unicast or multicast, chosen from its SERVERINFO):
./target/release/natnet_zenoh run --server 192.168.1.10

# ...or discover the server from its multicast stream:
./target/release/natnet_zenoh run

# No Motive at hand? Stream the built-in synthetic scene:
./target/release/natnet_zenoh run --synthetic

# On the ROS 2 side:
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic echo /natnet/rigid_bodies/body_1/pose
ros2 run tf2_ros tf2_echo mocap body_1
ros2 param list /natnet
ros2 param set /natnet timing.max_rate_hz 30.0
```

## Command line

| Command | Purpose |
| --- | --- |
| `run` | Run the bridge from a live server (`--server`, or discovery), a capture (`--replay`), or the synthetic scene (`--synthetic`). |
| `probe` | Connect to a server, report its version, model, frame rate, drops and latency, then exit (`--json` for machines). Fails when no frames arrive. |
| `record` | Record the raw datagrams to a `.natnet` or `.aucap` capture. Refuses to overwrite a file without `--force` and fails when no frames arrive. |
| `serve` | A simulated Motive server that streams the synthetic scene or a capture (`--capture`, with `--model-file` for its model), by multicast or unicast. Use it to test clients without hardware. |
| `config` | Print the default configuration as TOML. |
| `check <file>` | Validate a configuration file: every section, used or not, and the files the selected source reads. |
| `types` | List the ROS 2 types in use with their RIHS01 hashes. |

Options of `run` override their configuration-file counterparts, except
that `--passive`, `--loop` and `--native` can only switch a setting on and
`--no-ros` only off. `--server` and `--passive` also select the live source,
`--protocol-version` applies to replays too, and `--rate` and `--loop` set
`source.replay`. Options that the chosen source would ignore (such as
`--synthetic --data-port`) are rejected. `probe` and `record` read the
configuration file too, for the live source's settings. Run
`natnet_zenoh <command> --help` for the full list. Useful options:

- `-c/--config` (or `NATNET_ZENOH_CONFIG`; an empty value counts as unset)
- `--passive`: listen to multicast only and never send anything
- `--transport auto|multicast|unicast`
- `--protocol-version 4.1`
- `--model-file <MODELDEF datagram>`: for captures or passive streams that lack the model
- `--up-axis y|z`
- `--domain-id`, `--namespace`, `--node-name`, `--world-frame`
- `--native`: native zenoh outputs
- `--no-ros`
- `--connect tcp/host:7447`, `--zenoh-config <file>`
- `--duration <secs>`

`run` exits with an error when a replay ends, or `--duration` elapses,
without a single frame. Logging follows `RUST_LOG` (default
`info,zenoh=warn`) and goes to stderr, colored only on a terminal, so stdout
carries only command output such as `probe --json`. The bridge's log is also
published on `/rosout`. Ctrl-C stops the bridge cleanly, also while it waits
for a zenoh router; a second Ctrl-C exits immediately.

## ROS 2 interface

Names below assume the default node `/natnet`; `~` expands to the node's name.
Stream topics use `[ros.stream_qos]` (reliable, volatile, depth 10 by
default). The exceptions have fixed QoS: `/tf` (depth 100, as tf2 uses),
`~/visualization` (depth 5), `~/rigid_bodies/<name>/path` (depth 1),
`/diagnostics` (depth 10), `/parameter_events` (depth 1000, as rclcpp uses),
and the transient-local topics noted below.

| Topic | Type | Notes |
| --- | --- | --- |
| `/tf` | `tf2_msgs/TFMessage` | Rigid bodies and skeleton bones, parent `world_frame` (`mocap`); with `skeletons.coordinates = "local"`, bones are parented to their parent bone, which needs the model (bones missing from it are skipped). |
| `/tf_static` | `tf2_msgs/TFMessage` | `[[ros.static_transforms]]`, transient local. |
| `~/rigid_bodies/<name>/pose` | `geometry_msgs/PoseStamped` | Per body, created on first sight. |
| `~/rigid_bodies/<name>/odom` | `nav_msgs/Odometry` | Pose plus a finite-difference twist in the body frame. Covariance comes from Motive's mean marker error; an untracked body (`publish_untracked`) carries a pose variance of 1e6. |
| `~/rigid_bodies/<name>/twist` | `geometry_msgs/TwistStamped` | Off by default. |
| `~/rigid_bodies/<name>/path` | `nav_msgs/Path` | Off by default; the last `path_length` poses, sampled and published at 10 Hz. |
| `~/rigid_bodies/poses` | `geometry_msgs/PoseArray` | All tracked bodies, in frame order. Untracked ones are left out, so an index does not identify a body; use the per-body topics or `/tf` for that. |
| `rigid_bodies`, `markers` | `mocap4r2_msgs/RigidBodies`, `Markers` | Off by default; mocap4r2 compatible. |
| `~/markers` | `sensor_msgs/PointCloud2` | Fields `x y z id size residual params`. Unlabeled markers are optional. |
| `~/visualization` | `visualization_msgs/MarkerArray` | Markers, body axes, labels and skeleton lines (each bone joined to its parent in the model), rate limited. |
| `~/skeletons/<name>/poses` | `geometry_msgs/PoseArray` | Tracked bones in `world_frame`, in model order. |
| `~/force_plates/force_plate_<id>/wrench` | `geometry_msgs/WrenchStamped` | Channels matched by name (`Fx`…`Mz`), else by position. |
| `~/force_plates/force_plate_<id>/channels`, `~/devices/<name>/channels` | `std_msgs/Float32MultiArray` | Every channel sub-sample. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Stream health, plus one status per rigid body, stale while no new frames arrive. |
| `~/server_info`, `~/model` | `std_msgs/String` (JSON) | Transient local. |
| `/rosout` | `rcl_interfaces/Log` | Transient local, like rclcpp. |
| `/parameter_events` | `rcl_interfaces/ParameterEvent` | Announces the parameters at start-up and every change. |

`<name>` is the Motive asset name as a ROS name token: characters other than
ASCII letters and digits become `_`, leading underscores are dropped (they
would hide the topics), a leading digit gets an `n` prefix, and a name that
collides with another asset's or with `world_frame` gets `_<id>` appended.
`rigid_bodies.rename` maps Motive names to names of your choice first.

When publishing falls behind the stream, queued frames are skipped so the
newest frame goes out next; the skipped frames count as `queue_drops` in
`natnet/stats` and as dropped frames in `/diagnostics`.

Services:

- `~/refresh_model`, `~/get_server_info`, `~/get_model`, `~/get_stats` and `~/reset_clock_sync` (all `std_srvs/Trigger`).
- The six `rcl_interfaces` parameter services, so `ros2 param list|get|describe|dump|set` work. The parameters mirror the configuration (`ros.world_frame`, `rigid_bodies.path_length`, …) plus `use_sim_time`. The `zenoh` section is withheld because it can carry credentials.
  - These keys can be set at run time; the bridge validates the whole configuration and adopts it between frames:
    - `transform.up_axis`, `transform.scale`, `transform.offset_translation`, `transform.offset_rpy`
    - `timing.policy`, `timing.window_secs`, `timing.max_rate_hz`
    - `rigid_bodies.include`, `rigid_bodies.exclude`, `rigid_bodies.publish_untracked`, `rigid_bodies.velocity_alpha`, `rigid_bodies.velocity_max_gap`
    - `markers.include_unlabeled`, `markers.visualization_rate_hz`, `markers.axis_length`
    - `skeletons.coordinates`
    - `diagnostics.drop_warn_ratio`, `diagnostics.latency_warn_secs`
  - Other keys are read-only; change them in the configuration file and restart.
  - Types are static, as in rclcpp: a double needs a decimal point (`30.0`, not `30`). `[]` clears a list.
  - A change resets velocity history and paths. Clock synchronization restarts only when its settings change.
- `~/get_type_description` (`type_description_interfaces/GetTypeDescription`) for every type the node uses.

Frames follow REP 103: Motive's default Y-up stream is converted to Z-up
(`x, -z, y`). Set `transform.up_axis = "z"` (or `--up-axis z`) when Motive
streams Z-up. Motive does not announce its up axis, so the setting must match
Motive's streaming panel. A world offset (`offset_translation`,
`offset_rpy`) and a scale apply afterwards.

## Native zenoh interface

`--native` (or `[native] enabled = true`) adds plain zenoh keys for non-ROS
consumers, under `native.prefix` (`natnet`). Frames and rigid bodies are only
encoded while someone subscribes, raw frames included.

| Key | Content |
| --- | --- |
| `natnet/frame` | Every frame as JSON. |
| `natnet/rigid_bodies/<name>` | Each rigid body as JSON. |
| `natnet/raw/frame` | Each frame re-encoded as a NatNet datagram (`raw = true`). |
| `natnet/server_info`, `natnet/model` | JSON, put on change and answered to `get`. |
| `natnet/stats` | JSON, put every `diagnostics.period_secs` and answered to `get`. |
| `natnet/refresh_model` | A `get` requests the model again. |
| `natnet/alive` | A liveliness token while the bridge runs (`liveliness().get("natnet/alive")`). |

The documents follow native format version 1. Every JSON sample and reply
carries the encoding `application/json;natnet_zenoh.v1`; within a version,
documents only gain fields, and renaming, removing or retyping one bumps the
version. [`tests/golden/native/`](tests/golden/native/) holds a reference of
each document from the synthetic scene. A raw frame carries
`application/octet-stream;natnet.<version>` (for example
`natnet.4.1.0.0`), the NatNet protocol version to decode it with. The
`model` and `server_info` documents the ROS node publishes are the same
JSON.

## Sources

- **Live.**
  - Handshake: `NAT_CONNECT`, then `SERVERINFO`. The transport follows the server's announcement unless forced. Unanswered handshakes back off to 30 s.
  - Unless unicast is forced, the configured multicast group is joined at once. If Motive streams but its command channel does not answer, frames are decoded as protocol 4.1 (or `protocol_version`) after 3 s.
  - The multicast group is joined on `interface`. Left unspecified, that is the interface routing to `server`, or the default route's without one; a `server` on loopback also joins on the default interface, where a local Motive may send. Set it explicitly on multi-homed hosts when discovering.
  - Keepalives go to unicast servers. Forcing `transport = "unicast"` needs `server`.
  - The model is requested every `model_poll_secs` (30 s) while streaming, since Motive does not flag model changes in its frames. It is also requested when frames carry unknown assets, backing off while the model still lacks them. An unchanged model is not republished.
  - The bridge reconnects after `data_timeout_ms` without data. Without `server`, the server is discovered from the first frame's source and rediscovered after the stream is lost.
  - Multicast sockets share the port with other NatNet clients on the host. On Linux they bind the group address.
  - `request_bitstream` (`--request-bitstream 4.1`) asks for that bitstream version in `NAT_CONNECT`, as the vendor SDK client does, and decodes with it when the server reports NatNet 3 or later; downstream sees it as the protocol version, and the log names the one the server reported. It requires `transport = "unicast"`, since a multicast stream has one version for every client, and excludes `passive` and `protocol_version`. Unset, the hello is the plain one Motive has always accepted. `serve --unicast` honours the request for its synthetic scene, streaming one version at a time: the latest hello decides.
  - `passive = true` never transmits. It synthesizes the server info, so give it `protocol_version` (4.1 when unset), `clock_frequency` (for `clock_sync` stamps) and, for names, `model_file`.
- **Replay.**
  - Reads `.natnet` captures (`u64 ns, u32 len, bytes` records) and `.aucap` captures (the `AUGACAP` container), detected by magic.
  - Recorded `SERVERINFO` and `MODELDEF` datagrams are used when present.
  - `rate = 0` replays as fast as possible without dropping frames. At any rate other than 1, `clock_sync` stamps fall back to `receive`, since the capture's clock no longer tracks the host's.
  - Pauses longer than 60 s in a capture (a recorder left running between sessions) are skipped, as they are by `serve --capture`.
- **Synthetic.** A deterministic scene of circling rigid bodies, labeled and unlabeled markers, a skeleton, a force plate and an analog device. It goes through the real encoder and decoder at any protocol version. A scene has at most 1000 rigid bodies of at most 1000 markers each, and must fit one NatNet datagram.

Frames that arrive out of order (common with multicast over Wi-Fi) are
discarded and counted as late, so outputs never move back in time. A jump
backwards by more than 1000 frames, 60 late frames in a row, or a jump
forwards by more than 100 000 frames is treated as a stream restart; it
resets timing and velocity history. A repeated frame number (a paused
server) is not republished.

## Timing

`[timing] policy` chooses how message stamps are computed:

- `clock_sync` (default): maps Motive's mid-exposure timestamp onto the host clock. The offset is a minimum-delay estimate over `window_secs`. When the estimate drops, stamps slew towards it (by at most 5 % of the elapsed time) instead of stepping back, so they never go backwards.
- `latency_compensated`: reception time minus the server-reported latency.
- `receive`: host reception time.

`max_rate_hz` throttles publishing without affecting velocity estimation.

## Configuration

`natnet_zenoh config` prints every key with its default, except optional keys
that are unset by default: `source.live.server`, `source.live.request_bitstream`,
`protocol_version` and `model_file` (under `source.live` and `source.replay`),
`ros.domain_id`, and
`zenoh.config_file`, `config_json5` and `router_check_attempts`. Unknown keys
are rejected. Sections:

| Section | Contents |
| --- | --- |
| `[source]` | Source kind and queue depth, plus `live`, `replay` and `synthetic` sub-tables. |
| `[zenoh]` | Config file or inline JSON5, `key=value` overrides, router check. |
| `[ros]` | Node, namespace, domain, enclave name, `rmw_zenoh` version, frames, QoS, static transforms, rosout, parameters, type descriptions. |
| `[native]` | Native zenoh outputs. |
| `[transform]` | Up axis, scale, world offset. |
| `[timing]` | Stamp policy, clock sync window, rate limit. |
| `[rigid_bodies]` | Outputs, include/exclude globs, renames, velocity smoothing. |
| `[markers]`, `[skeletons]`, `[analog]` | Their outputs. |
| `[diagnostics]` | Period and warning thresholds. |

The environment variables of `rmw_zenoh` apply too: `ROS_DOMAIN_ID`,
`ZENOH_SESSION_CONFIG_URI`, `ZENOH_CONFIG_OVERRIDE` and
`ZENOH_ROUTER_CHECK_ATTEMPTS`. The configuration file takes precedence over
the environment, with one exception. The zenoh session configuration is built
in this order, later steps winning:

1. `zenoh.config_file` or `config_json5`, else `ZENOH_SESSION_CONFIG_URI`, else the rmw_zenoh defaults
2. `ZENOH_CONFIG_OVERRIDE`
3. `zenoh.overrides` and `--connect`

A configuration of your own must keep `timestamping` enabled, as the
`rmw_zenoh` default does: zenoh-ext's publication cache needs it for
transient-local topics.

## Library

The crate is also a library:

- `natnet_zenoh::ros`: a small ROS 2 client over `rmw_zenoh`, with nodes, publishers, subscriptions, services, clients, QoS, CDR and type hashes. See `examples/ros_talker.rs`.
- `natnet_zenoh::source`: the live client, replay, capture I/O, the simulated server and the synthetic scene.
- `natnet_zenoh::bridge`: frame processing and outputs.

## Development

```sh
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test
cargo test --all-features
RUSTDOCFLAGS=-Dwarnings cargo doc --no-deps --all-features
```

`natnet` is a private git dependency. Cargo fetches it with the git CLI
(`.cargo/config.toml`), so the machine's SSH agent or credential helper
applies. To build against a local checkout instead of the pinned revision:

```sh
cargo --config 'patch."https://github.com/AndrejOrsula/natnet.git".natnet.path="../natnet"' build
```

CI (`.github/workflows/rust.yml`) runs these checks plus a build with the
MSRV (`rust-version` in `Cargo.toml`) on a self-hosted runner. The runner
needs read access to `natnet`: either its own git credentials, or a
`GH_PAT` repository secret holding a token that can read that repository. Only the `cargo fetch` step
sees the token; every later step builds offline.

The tests need no network beyond loopback and no ROS install.
`tests/golden/native/` pins the native JSON documents; after an intended,
additive change, `NATNET_ZENOH_BLESS=1 cargo test --test native_golden`
rewrites the references.

`tests/interop/ros2.sh [jazzy|humble]` checks the bridge against stock ROS 2
tools with the current apt `rmw_zenoh_cpp` in Docker: a pose topic, the
transient-local model, `tf2_echo`, a service call and a parameter. It needs
Docker with host networking, and uses zenoh port 17447 and ROS domain 42 so
it stays apart from other ROS 2 systems on the host. The manually dispatched
`.github/workflows/interop.yml` runs it for both distributions. Beyond that,
interop with real ROS 2 nodes was checked by hand with the `ros2` CLI and `tf2_echo`
(topics, late and early transient-local joiners, services, parameters and
`/parameter_events`) against these `rmw_zenoh_cpp` builds: Jazzy's apt 0.2.10
and a source-built 0.2.3 (`current` and `jazzy-0.2.3`), RoboStack's 0.2.0
(`jazzy-0.2.0`), Humble's apt 0.1.9 (`humble`), Kilted's apt 0.6.8 and
Rolling's apt 0.13.0 (`current`). Rolling's `ros2` daemon also fails for every
node (`unknown tag 'rclpy.topic_endpoint_info.TopicEndpointInfo'`), so use
`--no-daemon` there too. A source-built 0.2.2 (`jazzy-0.2.2`) was checked only
in part: today's Jazzy `rcl` needs a function that build lacks, so several
`ros2` commands fail against any node; `topic info`, `topic hz` and parameters
worked.

The default build is pure Rust: zenoh's TCP and Unix-socket links, which is
what `rmw_zenoh` uses. The `transport_udp`, `transport_tls`, `transport_quic`
and `transport_ws` features add links that depend on `ring` and so need a C
compiler (zenoh's UDP link always builds its QUIC-datagram sibling).
`auth_usrpwd`, `auth_pubkey` and `transport_compression` enable zenoh's
authentication and link compression, configured through a zenoh configuration
file; they stay pure Rust. `shared-memory` enables zenoh SHM.

## Limitations

- Validated against the synthetic scene, the simulated server and recorded
  Motive 4.1 captures; not yet against a live Motive instance.
- Only the keys listed under [Services](#ros-2-interface) can be set at run
  time; everything else needs the configuration file and a restart.
- `rmw_zenoh` security (enclaves with SROS2 keys) and zenoh access control
  are not configured by the bridge; `ros.enclave` only names the enclave in
  the liveliness tokens. For TLS or authentication, build with the matching
  feature and use a zenoh configuration file.
- Subscribing and client-side features of the ROS layer cover what the bridge
  and its tests need, not the whole `rclcpp` API.

## License

Apache-2.0 OR MIT.
