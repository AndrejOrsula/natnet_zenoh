//! Bridge OptiTrack NatNet motion capture to Zenoh and ROS 2 (`rmw_zenoh`).
//!
//! Start with the [README](https://github.com/SpaceRL/natnet_zenoh#readme) for
//! the `natnet_zenoh` binary. For library use:
//!
//! - Configure and validate with [`config::Config::load`],
//!   [`config::Config::read`], [`config::Config::default_toml`] and
//!   [`config::Config::validate`].
//! - Open a session with [`bridge::open_context`], start a source with
//!   [`bridge::start_source`], then construct and run the bridge with
//!   [`bridge::Bridge::new`] and [`bridge::Bridge::run`].
//! - Use [`source`] for live, replay and synthetic NatNet sources; [`mocap`]
//!   for model, frame conventions, timing and velocity; and [`ros`] for ROS 2
//!   nodes over `rmw_zenoh`, without a ROS install.

pub mod bridge;
pub mod config;
pub mod mocap;
pub mod ros;
pub mod source;
