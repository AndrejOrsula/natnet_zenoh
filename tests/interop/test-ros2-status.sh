#!/usr/bin/env bash
# Exercise ROS interop status and router ownership with local shims only.
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
work=$(mktemp -d)
active_server=
active_state=
cleanup() {
    local status=$? wait_status=0 failed=0
    if [ -n "$active_server" ]; then
        if [ -n "$active_state" ]; then : > "$active_state/stop"; fi
        if kill -0 "$active_server" 2> /dev/null; then
            if ! kill -TERM "$active_server"; then
                echo "failed to terminate test router PID $active_server" >&2
                failed=1
            fi
        fi
        if wait "$active_server" 2> /dev/null; then
            wait_status=0
        else
            wait_status=$?
        fi
        if [ "$wait_status" -ne 0 ] && [ "$wait_status" -ne 143 ]; then
            echo "test router PID $active_server exited with status $wait_status" >&2
            failed=1
        fi
        active_server=
    fi
    if [ "${KEEP_TEST_WORK:-0}" = 1 ]; then
        echo "preserved test directory $work" >&2
    else
        if ! rm -rf "$work"; then
            echo "failed to remove test directory $work" >&2
            failed=1
        fi
        if [ -e "$work" ]; then
            echo "test directory $work is still present" >&2
            failed=1
        fi
    fi
    if [ "$failed" -ne 0 ]; then status=1; fi
    exit "$status"
}
trap cleanup EXIT

mkdir -p "$work/bin" "$work/tmp"
cat > "$work/router.py" <<'PY'
import pathlib
import socket
import sys
import time

port = int(sys.argv[1])
start = pathlib.Path(sys.argv[2])
stop = pathlib.Path(sys.argv[3])
while not start.exists() and not stop.exists():
    time.sleep(0.01)
if stop.exists():
    raise SystemExit(0)
sock = socket.socket()
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("127.0.0.1", port))
sock.listen()
sock.settimeout(0.05)
while not stop.exists():
    try:
        client, _ = sock.accept()
        client.close()
    except TimeoutError:
        pass
sock.close()
PY
cat > "$work/bin/docker" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >> "$DOCKER_LOG"
case "$1" in
    build) exit 0 ;;
    create)
        if [ "${CREATE_FAIL:-0}" = 1 ]; then
            echo 'simulated Docker create failure' >&2
            exit 125
        fi
        if [ "${CREATE_BAD_ID:-0}" = 1 ]; then
            printf '%s\n' unrelated-container
            exit 0
        fi
        printf '%s\n' "$FAKE_CONTAINER_ID" > "$SHIM_STATE/container-id"
        : > "$SHIM_STATE/container-present"
        printf '%s\n' "$FAKE_CONTAINER_ID"
        ;;
    start)
        if [ "$2" != "$FAKE_CONTAINER_ID" ] || [ ! -e "$SHIM_STATE/container-present" ]; then
            echo 'start did not use the created container ID' >&2
            exit 2
        fi
        if [ "${START_FAIL:-0}" = 1 ]; then
            echo 'simulated Docker start failure' >&2
            exit 23
        fi
        : > "$SHIM_STATE/start"
        for _ in {1..100}; do
            if (exec 3<> "/dev/tcp/127.0.0.1/$ZENOH_PORT") 2> /dev/null; then
                exit 0
            fi
            sleep 0.01
        done
        echo 'test router did not bind after start' >&2
        exit 1
        ;;
    rm)
        if [ "$3" != "$FAKE_CONTAINER_ID" ]; then
            echo "cleanup used unowned identity: $3" >&2
            exit 2
        fi
        if [ "${RM_FAIL:-0}" = 1 ]; then
            echo 'simulated Docker removal failure' >&2
            exit 24
        fi
        : > "$SHIM_STATE/stop"
        rm -f "$SHIM_STATE/container-present"
        ;;
    container)
        if [ "$2" != inspect ] || [ "$3" != "$FAKE_CONTAINER_ID" ]; then
            echo "inspect used unowned identity: $*" >&2
            exit 2
        fi
        [ -e "$SHIM_STATE/container-present" ]
        ;;
    run)
        if [ "$2" = --detach ]; then
            echo 'legacy docker run --detach path was used' >&2
            exit 2
        fi
        shift
        while [ "$1" != "$FAKE_IMAGE" ]; do shift; done
        shift
        exec "$@"
        ;;
    *) exit 2 ;;
esac
SH
cat > "$work/bin/timeout" <<'SH'
#!/usr/bin/env bash
set -e
limit=$1
shift
if "$@"; then status=0; else status=$?; fi
if [ "$limit" = 10 ] && [ "$status" -eq 0 ]; then exit 124; fi
exit "$status"
SH
cat > "$work/bin/ros2" <<'SH'
#!/usr/bin/env bash
case "$*" in
    *body_1/pose*) echo '  position:' ;;
    */natnet/model*) echo "data: '{\"assets\":" ;;
    *tf2_echo*) echo '- Translation: [0, 0, 0]' ;;
    */natnet/get_server_info*) echo 'success=True' ;;
    *timing.max_rate_hz*) echo 'Double value is: 30.0' ;;
    *'pkg xml rmw_zenoh_cpp'*) echo '0.2.11' ;;
    *) exit 3 ;;
esac
exit "${FAKE_ROS_EXIT:-0}"
SH
cat > "$work/bridge" <<'SH'
#!/usr/bin/env bash
exec python3 -c 'import signal,time; signal.signal(signal.SIGTERM, lambda *_: exit(0)); time.sleep(600)'
SH
chmod +x "$work/bin/docker" "$work/bin/timeout" "$work/bin/ros2" "$work/bridge"

port_open() {
    python3 -c 'import socket,sys; s=socket.socket(); s.settimeout(.2); sys.exit(0 if s.connect_ex(("127.0.0.1",int(sys.argv[1]))) == 0 else 1)' "$1"
}
free_port() {
    python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1]); s.close()'
}
start_router_fixture() {
    local state=$1 port=$2 bind_now=${3:-0}
    mkdir -p "$state"
    python3 "$work/router.py" "$port" "$state/start" "$state/stop" \
        > "$state/router.log" 2>&1 &
    active_server=$!
    active_state=$state
    if [ "$bind_now" = 1 ]; then
        : > "$state/start"
        for _ in {1..100}; do
            if port_open "$port"; then return 0; fi
            sleep 0.01
        done
        echo "test router did not bind busy port $port" >&2
        return 1
    fi
}
stop_router_fixture() {
    local port=$1
    : > "$active_state/stop"
    if wait "$active_server"; then :; else
        echo "test router fixture failed to exit cleanly" >&2
        return 1
    fi
    active_server=
    active_state=
    if port_open "$port"; then
        echo "test router still listens on port $port" >&2
        return 1
    fi
}
run_ros_case() {
    local label=$1 state=$2 port=$3 expected=$4 create_fail=$5 start_fail=$6 rm_fail=$7 bad_id=$8 ros_exit=$9 status
    if env PATH="$work/bin:$PATH" TMPDIR="$work/tmp" \
        SHIM_STATE="$state" DOCKER_LOG="$state/docker.log" \
        FAKE_CONTAINER_ID=0123456789ab FAKE_IMAGE="natnet-zenoh-interop:jazzy-$port" \
        ZENOH_PORT="$port" CREATE_FAIL="$create_fail" START_FAIL="$start_fail" \
        RM_FAIL="$rm_fail" CREATE_BAD_ID="$bad_id" FAKE_ROS_EXIT="$ros_exit" \
        bash "$root/tests/interop/ros2.sh" jazzy "$work/bridge" \
        > "$state/stdout.log" 2> "$state/stderr.log"; then
        status=0
    else
        status=$?
    fi
    if [ "$status" -ne "$expected" ]; then
        cat "$state/stdout.log" "$state/stderr.log" >&2
        echo "$label: expected exit $expected, got $status" >&2
        return 1
    fi
    printf '%s: exit=%s\n' "$label" "$status"
}
check_owned_cleanup() {
    local state=$1 id=0123456789ab
    [ ! -e "$state/container-present" ] || { echo 'owned container marker remains' >&2; return 1; }
    grep -E '^create --name natnet-zenoh-interop-[0-9]+ --network host natnet-zenoh-interop:jazzy-[0-9]+ ros2 run rmw_zenoh_cpp rmw_zenohd$' "$state/docker.log" >/dev/null || { echo 'router create command was not recorded' >&2; return 1; }
    grep -Fx "rm --force $id" "$state/docker.log" >/dev/null || { echo 'cleanup did not use returned ID' >&2; return 1; }
    grep -Fx "container inspect $id" "$state/docker.log" >/dev/null || { echo 'cleanup did not inspect returned ID' >&2; return 1; }
    ! grep -q -- '--detach' "$state/docker.log" || { echo 'legacy detached run was used' >&2; return 1; }
}
assert_no_success_output() {
    if grep -q 'all checks passed' "$1"; then
        echo "unexpected success output in $1" >&2
        return 1
    fi
}

# Normal statuses and the permitted TF timeout still pass, with router cleanup.
good_port=$(free_port)
good="$work/success"
start_router_fixture "$good" "$good_port"
echo "ZENOH_PORT=$good_port" > "$good/env"
run_ros_case 'successful ROS status fixture' "$good" "$good_port" 0 0 0 0 0 0
test "$(grep -c '^ok: ' "$good/stdout.log")" -eq 5
grep -q 'all checks passed against ROS 2 jazzy (0.2.11)' "$good/stdout.log"
check_owned_cleanup "$good"
stop_router_fixture "$good_port"

# Matching text with nonzero command statuses remains a failed check.
bad_ros_port=$(free_port)
bad_ros="$work/nonzero-ros"
start_router_fixture "$bad_ros" "$bad_ros_port"
echo "ZENOH_PORT=$bad_ros_port" > "$bad_ros/env"
run_ros_case 'nonzero ROS command fixture' "$bad_ros" "$bad_ros_port" 1 0 0 0 0 7
grep -q 'status 7; expected 0' "$bad_ros/stderr.log"
assert_no_success_output "$bad_ros/stdout.log"
check_owned_cleanup "$bad_ros"
stop_router_fixture "$bad_ros_port"

# An already listening service must survive and no Docker command may run.
busy_port=$(free_port)
busy="$work/busy-port"
start_router_fixture "$busy" "$busy_port" 1
port_open "$busy_port"
run_ros_case 'busy port refusal' "$busy" "$busy_port" 1 0 0 0 0 0
grep -q "Zenoh port $busy_port is already in use" "$busy/stderr.log"
[ ! -e "$busy/docker.log" ] || { echo 'Docker ran before busy-port refusal' >&2; exit 1; }
port_open "$busy_port"
stop_router_fixture "$busy_port"

# A create/name collision must preserve unrelated identity and never rm it.
create_port=$(free_port)
create="$work/create-failure"
mkdir -p "$create"
echo unrelated > "$create/unrelated-container.marker"
echo "ZENOH_PORT=$create_port" > "$create/env"
run_ros_case 'Docker create failure' "$create" "$create_port" 125 1 0 0 0 0
grep -q 'failed to create router container' "$create/stderr.log"
[ -e "$create/unrelated-container.marker" ] || { echo 'create failure deleted unrelated container' >&2; exit 1; }
! grep -q '^rm ' "$create/docker.log" || { echo 'create failure attempted deletion without an owned ID' >&2; exit 1; }

# Malformed create output must never become a name-based cleanup target.
bad_id_port=$(free_port)
bad_id="$work/bad-create-id"
mkdir -p "$bad_id"
echo unrelated > "$bad_id/unrelated-container.marker"
echo "ZENOH_PORT=$bad_id_port" > "$bad_id/env"
run_ros_case 'malformed Docker create identity' "$bad_id" "$bad_id_port" 1 0 0 0 1 0
grep -q 'Docker returned an invalid router container ID' "$bad_id/stderr.log"
[ -e "$bad_id/unrelated-container.marker" ] || { echo 'malformed ID deleted unrelated container' >&2; exit 1; }
! grep -q '^rm ' "$bad_id/docker.log" || { echo 'malformed ID was used as cleanup target' >&2; exit 1; }

# A failed start still has the create-returned ID and removes exactly that ID.
start_port=$(free_port)
start="$work/start-failure"
start_router_fixture "$start" "$start_port"
echo "ZENOH_PORT=$start_port" > "$start/env"
run_ros_case 'Docker start failure' "$start" "$start_port" 23 0 1 0 0 0
grep -q 'failed to start owned router container 0123456789ab' "$start/stderr.log"
check_owned_cleanup "$start"
stop_router_fixture "$start_port"

# A failed rm and a still-present owned ID must prevent a successful run.
cleanup_port=$(free_port)
failed_cleanup="$work/cleanup-failure"
start_router_fixture "$failed_cleanup" "$cleanup_port"
echo "ZENOH_PORT=$cleanup_port" > "$failed_cleanup/env"
run_ros_case 'failed cleanup fixture' "$failed_cleanup" "$cleanup_port" 1 0 0 1 0 0
grep -q 'failed to remove owned router container 0123456789ab' "$failed_cleanup/stderr.log"
grep -q 'owned router container 0123456789ab is still present' "$failed_cleanup/stderr.log"
assert_no_success_output "$failed_cleanup/stdout.log"
[ -e "$failed_cleanup/container-present" ]
: > "$failed_cleanup/stop"
if wait "$active_server"; then :; else echo 'fixture server failed after test cleanup' >&2; exit 1; fi
active_server=
active_state=
if port_open "$cleanup_port"; then echo 'failed-cleanup fixture server leaked' >&2; exit 1; fi

echo 'ROS interop status and ownership shims passed'
