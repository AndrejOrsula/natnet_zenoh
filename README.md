# natnet_zenoh

[![CI](https://github.com/AndrejOrsula/natnet_zenoh/actions/workflows/ci.yml/badge.svg)](https://github.com/AndrejOrsula/natnet_zenoh/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/AndrejOrsula/natnet_zenoh/branch/main/graph/badge.svg)](https://codecov.io/gh/AndrejOrsula/natnet_zenoh/tree/main)

> Rust bridge from the OptiTrack **NatNet** protocol used by Motive to Zenoh

## Installation

### Cargo

```sh
cargo install --locked natnet_zenoh
```

## Hardware Probe

Verify connectivity with Motive running on a local network:

```sh
natnet_zenoh probe
```

## Stream

Run `natnet_zenoh` with the default config:

```sh
natnet_zenoh run
```

### Configuration

You can customize the behaviour of `natnet_zenoh` via a single TOML config ([reference](natnet_zenoh.toml)):

```sh
natnet_zenoh config > nz.toml
natnet_zenoh check    nz.toml
natnet_zenoh run   -c nz.toml
```

### ROS 2

To enable interoperability with ROS 2, you must run a Zenoh router before `natnet_zenoh`:

```sh
ros2 run rmw_zenoh_cpp rmw_zenohd
```

```sh
natnet_zenoh run -c nz.toml
```

```sh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic echo --no-daemon /tf
```

| Topic                                                    | Type                              |
| -------------------------------------------------------- | --------------------------------- |
| `/tf`                                                    | `tf2_msgs/TFMessage`              |
| `/tf_static`                                             | `tf2_msgs/TFMessage`              |
| `~/rigid_bodies/<name>/pose`                             | `geometry_msgs/PoseStamped`       |
| `~/rigid_bodies/poses`                                   | `geometry_msgs/PoseArray`         |
| `~/skeletons/<name>/poses`                               | `geometry_msgs/PoseArray`         |
| `~/rigid_bodies/<name>/odom`                             | `nav_msgs/Odometry`               |
| `~/rigid_bodies/<name>/twist`                            | `geometry_msgs/TwistStamped`      |
| `~/rigid_bodies/<name>/path`                             | `nav_msgs/Path`                   |
| `~/markers`                                              | `sensor_msgs/PointCloud2`         |
| `~/visualization`                                        | `visualization_msgs/MarkerArray`  |
| `rigid_bodies`, `markers`                                | `mocap4r2_msgs`                   |
| `~/force_plates/force_plate_<id>/wrench`                 | `geometry_msgs/WrenchStamped`     |
| `~/force_plates/…/channels`, `~/devices/<name>/channels` | `std_msgs/Float32MultiArray`      |
| `/diagnostics`                                           | `diagnostic_msgs/DiagnosticArray` |
| `~/server_info`, `~/model`                               | `std_msgs/String`                 |
| `/rosout`, `/parameter_events`                           | `rcl_interfaces`                  |

> Note: Motive's default Y-up stream becomes Z-up per REP 103 (`x, -z, y`). If Motive's streaming panel says Z-up, set `--up-axis z`. Motive does not announce this setting.
