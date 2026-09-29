# Stock ROS 2 with rmw_zenoh_cpp from the ROS apt repository, for
# `tests/interop/ros2.sh`. The packages are deliberately unpinned: the check
# runs against the current apt releases, as `ros.rmw_zenoh` promises.
#
# The router and every session use ZENOH_PORT instead of 7447, so the check
# never joins another router on the host.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ARG ZENOH_PORT=17447
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        "ros-${ROS_DISTRO}-rmw-zenoh-cpp" "ros-${ROS_DISTRO}-tf2-ros" \
    && rm -rf /var/lib/apt/lists/*
# Rewrite the port on the configuration lines, not in the comments, and fail
# the build if a default no longer has the line this expects.
RUN set -eu; \
    config="/opt/ros/${ROS_DISTRO}/share/rmw_zenoh_cpp/config"; \
    mkdir /interop; \
    for kind in ROUTER SESSION; do \
        out="/interop/$(echo "$kind" | tr 'A-Z' 'a-z').json5"; \
        sed '/^[[:space:]]*\/\//!s/:7447"/:'"${ZENOH_PORT}"'"/' \
            "$config/DEFAULT_RMW_ZENOH_${kind}_CONFIG.json5" > "$out"; \
        grep -q ":${ZENOH_PORT}\"" "$out"; \
    done
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ZENOH_ROUTER_CONFIG_URI=/interop/router.json5 \
    ZENOH_SESSION_CONFIG_URI=/interop/session.json5
