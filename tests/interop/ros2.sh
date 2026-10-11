#!/usr/bin/env bash
# Check stock rmw_zenoh_cpp tools against the host's synthetic bridge.
#
#   tests/interop/ros2.sh [jazzy|humble] [path/to/natnet_zenoh]
#
# Builds a release binary if omitted. Needs Docker and Linux host networking.
# Uses ZENOH_PORT (17447) and ROS_DOMAIN_ID (42) by default. Each check
# requires successful command status and matching, nonempty output.
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
# Refuse to start beside an existing service on the configured Zenoh port.
# This runner has host networking, so an occupied port usually means another
# router or bridge already owns the service.
if (exec 3<> "/dev/tcp/127.0.0.1/$port") 2> /dev/null; then
    echo "Zenoh port $port is already in use on 127.0.0.1" >&2
    exit 1
fi
root=$(cd "$(dirname "$0")/../.." && pwd)
bin=${2:-}
if [ -z "$bin" ]; then
    cargo build --release --locked --manifest-path "$root/Cargo.toml"
    bin=${CARGO_TARGET_DIR:-$root/target}/release/natnet_zenoh
fi
test -x "$bin" || { echo "no executable at $bin" >&2; exit 2; }

image=natnet-zenoh-interop:$distro-$port
work=$(mktemp -d)
router=natnet-zenoh-interop-$$
router_id=
bridge=
remove_router() {
    local failed=0
    [ -n "$router_id" ] || return 0
    if ! docker rm --force "$router_id" > /dev/null 2>&1; then
        echo "failed to remove owned router container $router_id" >&2
        failed=1
    fi
    if docker container inspect "$router_id" > /dev/null 2>&1; then
        echo "owned router container $router_id is still present" >&2
        failed=1
    fi
    [ "$failed" -eq 0 ] || return 1
    router_id=
}
cleanup() {
    status=$?
    if [ -n "$bridge" ]; then
        kill "$bridge" 2> /dev/null && wait "$bridge" 2> /dev/null
    fi
    if [ -n "$router_id" ]; then
        if ! remove_router; then status=1; fi
    fi
    if [ "$status" -ne 0 ] && [ -f "$work/bridge.log" ]; then
        echo "--- bridge log" >&2
        tail -n 40 "$work/bridge.log" >&2
    fi
    rm -rf "$work"
    exit "$status"
}
trap cleanup EXIT

docker build --quiet --tag "$image" \
    --build-arg ROS_DISTRO="$distro" --build-arg ZENOH_PORT="$port" \
    "$root/tests/interop" > /dev/null

if candidate_id=$(docker create --name "$router" --network host "$image" \
    ros2 run rmw_zenoh_cpp rmw_zenohd); then
    :
else
    status=$?
    echo "failed to create router container" >&2
    exit "$status"
fi
if [[ ! "$candidate_id" =~ ^[[:xdigit:]]{12,64}$ ]]; then
    echo "Docker returned an invalid router container ID" >&2
    exit 1
fi
router_id=$candidate_id
if docker start "$router_id" > /dev/null; then
    :
else
    status=$?
    echo "failed to start owned router container $router_id" >&2
    exit "$status"
fi
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

# Run a ROS 2 command in a fresh container; preserve its status and output.
ros() {
    docker run --rm --network host -e ROS_DOMAIN_ID="$domain" "$image" \
        timeout 30 "$@" 2>&1
}

failures=0
# expect <description> <pattern> <command...>: status must be zero and output
# must be nonempty and match.
expect() {
    local what=$1 pattern=$2
    shift 2
    local output status
    if output=$(ros "$@"); then
        status=0
    else
        status=$?
    fi
    if [ "$status" -eq 0 ] && [ -n "$output" ] && grep -qE "$pattern" <<< "$output"; then
        echo "ok: $what"
    else
        echo "FAILED: $what (status $status; expected 0 and output matching '$pattern')" >&2
        printf '%s\n' "$output" | tail -n 20 | sed 's/^/    /' >&2
        failures=$((failures + 1))
    fi
}

# The TF2 echo is a stream. Its 10-second timeout (124) is expected, provided
# the command produced the requested transform.
expect_timeout() {
    local what=$1 pattern=$2
    shift 2
    local output status
    if output=$(ros "$@"); then
        status=0
    else
        status=$?
    fi
    if { [ "$status" -eq 0 ] || [ "$status" -eq 124 ]; } && \
        [ -n "$output" ] && grep -qE "$pattern" <<< "$output"; then
        echo "ok: $what"
    else
        echo "FAILED: $what (status $status; expected 0 or 124 and output matching '$pattern')" >&2
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
expect_timeout "tf from mocap to body_1" '^- Translation: \[' \
    timeout 10 ros2 run tf2_ros tf2_echo mocap body_1
expect "service /natnet/get_server_info" 'success=True' \
    ros2 service call /natnet/get_server_info std_srvs/srv/Trigger
expect "parameter timing.max_rate_hz" '^Double value is: ' \
    ros2 param get "${daemon[@]}" /natnet timing.max_rate_hz

if ! kill -0 "$bridge" 2> /dev/null; then
    echo "FAILED: the bridge exited during the checks" >&2
    failures=$((failures + 1))
fi
version_output=$(ros ros2 pkg xml rmw_zenoh_cpp --tag version) || {
    status=$?
    echo "FAILED: rmw_zenoh_cpp version query (status $status)" >&2
    failures=$((failures + 1))
}
if [ -z "${version_output:-}" ]; then
    echo "FAILED: rmw_zenoh_cpp version query returned no output" >&2
    failures=$((failures + 1))
fi
if [ "$failures" -ne 0 ]; then
    echo "$failures check(s) failed against ROS 2 $distro" >&2
    exit 1
fi
remove_router || exit 1
echo "all checks passed against ROS 2 $distro ($(tail -n 1 <<< "$version_output"))"
