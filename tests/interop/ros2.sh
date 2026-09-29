#!/usr/bin/env bash
# ROS 2 interoperability check: stock rmw_zenoh_cpp tools in a container
# against the bridge's synthetic scene running on the host.
#
#   tests/interop/ros2.sh [jazzy|humble] [path/to/natnet_zenoh]
#
# Without a binary, the release build is made first. Needs Docker and host
# networking (Linux). The zenoh router listens on ZENOH_PORT (17447) and the
# ROS domain is ROS_DOMAIN_ID (42), so other ROS 2 systems on the host are
# neither joined nor disturbed. Every check asserts on the tool's output.
set -euo pipefail

distro=${1:-jazzy}
case $distro in
    jazzy) rmw=current ;;
    humble) rmw=humble ;;
    *)
        echo "unknown distribution '$distro' (jazzy or humble)" >&2
        exit 2
        ;;
esac
port=${ZENOH_PORT:-17447}
domain=${ROS_DOMAIN_ID:-42}
root=$(cd "$(dirname "$0")/../.." && pwd)
bin=${2:-}
if [ -z "$bin" ]; then
    cargo build --release --locked --manifest-path "$root/Cargo.toml"
    bin=${CARGO_TARGET_DIR:-$root/target}/release/natnet_zenoh
fi
test -x "$bin" || { echo "no executable at $bin" >&2; exit 2; }

image=natnet-zenoh-interop:$distro-$port
docker build --quiet --tag "$image" \
    --build-arg ROS_DISTRO="$distro" --build-arg ZENOH_PORT="$port" \
    "$root/tests/interop" > /dev/null

work=$(mktemp -d)
router=natnet-zenoh-interop-$$
bridge=
cleanup() {
    status=$?
    if [ -n "$bridge" ]; then
        kill "$bridge" 2> /dev/null && wait "$bridge" 2> /dev/null
    fi
    docker rm --force "$router" > /dev/null 2>&1
    if docker container inspect "$router" > /dev/null 2>&1; then
        echo "router container $router is still present" >&2
        status=1
    fi
    if [ "$status" -ne 0 ] && [ -f "$work/bridge.log" ]; then
        echo "--- bridge log" >&2
        tail -n 40 "$work/bridge.log" >&2
    fi
    rm -rf "$work"
    exit "$status"
}
trap cleanup EXIT

docker run --detach --rm --name "$router" --network host "$image" \
    ros2 run rmw_zenoh_cpp rmw_zenohd > /dev/null
deadline=$((SECONDS + 60))
until (exec 3<> "/dev/tcp/127.0.0.1/$port") 2> /dev/null; do
    if [ "$SECONDS" -ge "$deadline" ]; then
        echo "the router did not listen on port $port" >&2
        exit 1
    fi
    sleep 0.5
done

printf '[ros]\nrmw_zenoh = "%s"\n' "$rmw" > "$work/bridge.toml"
"$bin" run --synthetic --config "$work/bridge.toml" --domain-id "$domain" \
    --connect "tcp/127.0.0.1:$port" --duration 600 > "$work/bridge.log" 2>&1 &
bridge=$!

# Runs a ROS 2 command in a fresh container and prints its output.
ros() {
    docker run --rm --network host -e ROS_DOMAIN_ID="$domain" "$image" \
        timeout 30 "$@" 2>&1 || true
}

failures=0
# expect <description> <pattern> <command...>: the output must match.
expect() {
    local what=$1 pattern=$2
    shift 2
    local output
    output=$(ros "$@")
    if grep -qE "$pattern" <<< "$output"; then
        echo "ok: $what"
    else
        echo "FAILED: $what (no match for '$pattern')" >&2
        printf '%s\n' "$output" | tail -n 20 | sed 's/^/    /' >&2
        failures=$((failures + 1))
    fi
}

daemon=()
if [ "$distro" = humble ]; then
    # Humble's ros2 daemon cannot handle this rmw_zenoh_cpp (see README).
    daemon=(--no-daemon)
fi

expect "pose on /natnet/rigid_bodies/body_1/pose" '^  position:' \
    ros2 topic echo "${daemon[@]}" --once /natnet/rigid_bodies/body_1/pose
# `topic echo` truncates long strings; the model document starts with its
# first key.
expect "transient-local model on /natnet/model" "^data: '\\{\"assets\":" \
    ros2 topic echo "${daemon[@]}" --once --qos-durability transient_local /natnet/model
expect "tf from mocap to body_1" '^- Translation: \[' \
    bash -c 'timeout 10 ros2 run tf2_ros tf2_echo mocap body_1; true'
expect "service /natnet/get_server_info" 'success=True' \
    ros2 service call /natnet/get_server_info std_srvs/srv/Trigger
expect "parameter timing.max_rate_hz" '^Double value is: ' \
    ros2 param get "${daemon[@]}" /natnet timing.max_rate_hz

if ! kill -0 "$bridge" 2> /dev/null; then
    echo "FAILED: the bridge exited during the checks" >&2
    failures=$((failures + 1))
fi
if [ "$failures" -ne 0 ]; then
    echo "$failures check(s) failed against ROS 2 $distro" >&2
    exit 1
fi
echo "all checks passed against ROS 2 $distro ($(ros ros2 pkg xml rmw_zenoh_cpp --tag version | tail -n 1))"
