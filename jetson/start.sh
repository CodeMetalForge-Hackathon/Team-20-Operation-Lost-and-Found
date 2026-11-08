#!/usr/bin/env bash
# GHOST Jetson start-up: ghost-core + robot bringup (camera, cuVSLAM, Nav2) +
# base driver + frame server for the Pi + the GHOST↔Nav2 bridge.
#
#   bash jetson/start.sh                        everything, wheels LIVE
#   DRY_RUN=1 bash jetson/start.sh              wheels print instead of move
#   NO_NAV2=1 bash jetson/start.sh              camera + odometry only (no driving)
#   NO_BRIDGE=1 bash jetson/start.sh            Nav2 up but GHOST not driving (manual goals)
#   CELL_M=0.5 ORIGIN=0.5,0.5 bash jetson/start.sh   indoor grid, launch 0.5 m in from the corner
#   CAM_X=0.12 CAM_Z=0.10 bash jetson/start.sh  camera mount offset from robot centre (m)
#                                               default: measured mount, 6.35 cm ahead, 14.2875 cm up
#   CALIBRATE=0 bash jetson/start.sh            skip the start-up mount calibration (below)
#   CAM_PITCH=-1.2 CAM_ROLL=0 CAM_YAW=-4.4 bash jetson/start.sh
#                                               camera mount angles, deg, ROS sign (pitch + = nose
#                                               down, roll + = right side down, yaw + = turned left);
#                                               set here they win over the calibration for this run
#
# Start-up calibration (every boot unless CALIBRATE=0): camera + cuVSLAM + wheels
# come up alone, jetson/calibrate_mount.py fits the floor (pitch, roll, floor
# height — standing still) and then DRIVES ~20 cm straight forward and back,
# 4 times,
# for the yaw (skipped if something is in the way ahead, or with DRY_RUN); the
# results go to jetson/config/mount_calibration.env and the full stack starts
# with them.  Values failing their checks keep the previous ones.  Adds ~25 s.
#   DRIVE_ARGS="--ids 2,1,4,3 --signs=1,-1,1,-1" bash jetson/start.sh   wheel wiring
#                                               default: this robot, from base_driver.py --identify
#   FILTER_ARGS="--close 0.6" bash jetson/start.sh  obstacle_filter.py options (--help)
#   DASH_PORT=8000 bash jetson/start.sh          dashboard served at http://<jetson-ip>:8000/
#
# Logs: jetson/logs/*.log      Stop: Ctrl-C (wheels are stopped on exit)
set -uo pipefail

GHOST_DIR="$(cd "$(dirname "$0")/.." && pwd)"
J="$GHOST_DIR/jetson"
LOGS="$J/logs"; mkdir -p "$LOGS"
rm -f "$LOGS"/*.log   # no stale logs from a previous run (preflight reads them)
GHOST_ADDR="${GHOST_ADDR:-0.0.0.0:9001}"          # 0.0.0.0 so the Pi can reach the core
CELL_M="${CELL_M:-2.0}"
ORIGIN="${ORIGIN:-0,0}"

# ── ROS environment ───────────────────────────────────────────────────────────
if [ -z "${ROS_DISTRO:-}" ]; then
  ROS_DISTRO="$(ls /opt/ros 2>/dev/null | head -1)"
fi
set +u   # ROS setup files read unset variables
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash" || { echo "no ROS 2 at /opt/ros/$ROS_DISTRO"; exit 1; }
WS="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u
STAMP_ARGS=(); [ "$ROS_DISTRO" = "humble" ] && STAMP_ARGS=(--unstamped)
echo "[ghost] ROS 2 $ROS_DISTRO  workspace ${WS}"

# Job control: each background job gets its own process group (pgid = its PID)
# and, unlike plain `&` in a script, does not start with SIGINT ignored.  So on
# stop we can Ctrl-C each whole group the way a terminal would: ros2 launch
# shuts its nodes down cleanly and base_driver zeroes the wheels.  Plain
# SIGTERM to the top PID left launch children (static TF publishers, cuVSLAM
# container) running, still publishing stale transforms.
set -m
PIDS=()
group_alive() {   # any non-zombie process left in this process group?
  ps -o stat= -g "$1" 2>/dev/null | grep -qv '^Z'
}
any_alive() {
  local p; for p in "${PIDS[@]}"; do group_alive "$p" && return 0; done; return 1
}
signal_all() {
  local p; for p in "${PIDS[@]}"; do kill "-$1" -- "-$p" 2>/dev/null; done
}
cleanup() {
  [ -n "${STOPPING:-}" ] && return; STOPPING=1
  trap '' INT TERM
  echo "[ghost] stopping…"
  signal_all INT
  for _ in $(seq 1 50); do any_alive || break; sleep 0.2; done   # up to 10 s
  if any_alive; then
    echo "[ghost] still running after 10 s — SIGTERM"; signal_all TERM; sleep 3
    any_alive && { echo "[ghost] SIGKILL"; signal_all KILL; }
  fi
  wait 2>/dev/null
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() {   # start NAME CMD…  → background, logged
  local name="$1"; shift
  echo "[ghost] $name → $LOGS/$name.log"
  "$@" > "$LOGS/$name.log" 2>&1 &
  PIDS+=($!)
}

# ── camera mount: saved calibration < fresh calibration < explicit env ──────
CAL_FILE="$J/config/mount_calibration.env"
declare -A EXPLICIT=()
for v in CAM_PITCH CAM_ROLL CAM_YAW FLOOR_Z; do [ -n "${!v:-}" ] && EXPLICIT[$v]="${!v}"; done
load_mount() {   # saved file, then explicit env on top, then built-in defaults
  # shellcheck disable=SC1090
  [ -f "$CAL_FILE" ] && source "$CAL_FILE"
  local v; for v in "${!EXPLICIT[@]}"; do printf -v "$v" '%s' "${EXPLICIT[$v]}"; done
  CAM_PITCH="${CAM_PITCH:--1.2}"; CAM_ROLL="${CAM_ROLL:-0.0}"
  CAM_YAW="${CAM_YAW:--4.4}"; FLOOR_Z="${FLOOR_Z:--0.004}"
}
load_mount
robot_args() {   # robot.launch.py arguments for the current mount
  echo color:="${COLOR:-false}" cam_x:="${CAM_X:-0.0635}" cam_z:="${CAM_Z:-0.142875}" \
    cam_pitch_deg:="$CAM_PITCH" cam_roll_deg:="$CAM_ROLL" cam_yaw_deg:="$CAM_YAW"
}

HW=(--hardware); [ -n "${DRY_RUN:-}" ] && HW=()
# wheel wiring found on the bench with base_driver.py --identify (motor ids in
# fl, fr, rl, rr order, and direction signs); override with DRIVE_ARGS=...
DRIVE_ARGS="${DRIVE_ARGS:-"--ids 1,3,2,4 --signs=1,-1,1,-1"}"

if [ "${CALIBRATE:-1}" != "0" ]; then
  echo "[ghost] mount calibration (pitch $CAM_PITCH, roll $CAM_ROLL, yaw $CAM_YAW to start) → $LOGS/calibrate.log"
  # shellcheck disable=SC2046
  start cal_robot ros2 launch "$J/launch/robot.launch.py" nav2:=false $(robot_args)
  # shellcheck disable=SC2086
  start cal_base python3 "$J/base_driver.py" "${HW[@]}" --nav "${STAMP_ARGS[@]}" ${DRIVE_ARGS}
  CAL_ARGS=(); [ -n "${DRY_RUN:-}" ] && CAL_ARGS=(--no-drive)
  # a tracked background job + wait, so Ctrl-C / docker stop interrupts it
  # mid-drive (bash defers traps until a FOREGROUND command finishes)
  start calibrate timeout 180 python3 "$J/calibrate_mount.py" --out "$CAL_FILE.new" --floor-z "$FLOOR_Z" "${CAL_ARGS[@]}"
  wait "${PIDS[-1]}"; CAL_RC=$?
  if [ "$CAL_RC" = 0 ] && [ -s "$CAL_FILE.new" ]; then
    mv "$CAL_FILE.new" "$CAL_FILE"
  else
    rm -f "$CAL_FILE.new"; echo "[ghost] mount calibration FAILED — keeping the saved mount (see calibrate.log)" >&2
  fi
  grep -E "floor:|yaw:|result:" "$LOGS/calibrate.log" | sed 's/^/[ghost]   /'
  # tear the calibration stack down (camera USB must be free for the real one)
  signal_all INT
  for _ in $(seq 1 50); do any_alive || break; sleep 0.2; done
  any_alive && { signal_all TERM; sleep 3; }
  any_alive && signal_all KILL
  wait 2>/dev/null; PIDS=(); sleep 2
  load_mount
fi
echo "[ghost] mount: pitch $CAM_PITCH°, roll $CAM_ROLL°, yaw $CAM_YAW°, floor z $FLOOR_Z m"

# ── ghost-core ────────────────────────────────────────────────────────────────
CORE_BIN="$GHOST_DIR/core/target/release/ghost"
[ -x "$CORE_BIN" ] || (cd "$GHOST_DIR/core" && cargo build --release) || exit 1
GHOST_ADDR="$GHOST_ADDR" start core "$CORE_BIN"

# ── robot: camera + cuVSLAM (+ Nav2) ──────────────────────────────────────────
NAV2=true; [ -n "${NO_NAV2:-}" ] && NAV2=false
# shellcheck disable=SC2046
start robot ros2 launch "$J/launch/robot.launch.py" nav2:="$NAV2" $(robot_args)

# ── obstacle filter: floor removed in the robot frame + ego-motion check ─────
# Feeds Nav2's costmaps and Collision Monitor (see jetson/obstacle_filter.py).
# Without it the Collision Monitor's source times out and the robot won't move.
if [ "$NAV2" = "true" ]; then
  start filter python3 "$J/obstacle_filter.py" --floor-z "$FLOOR_Z" ${FILTER_ARGS:-}
fi

# ── wheels ────────────────────────────────────────────────────────────────────
# shellcheck disable=SC2086
start base python3 "$J/base_driver.py" "${HW[@]}" --nav "${STAMP_ARGS[@]}" ${DRIVE_ARGS}

# ── camera frames → Pi detector ──────────────────────────────────────────────
FRAME_TOPIC=/camera/left_ir/image_raw/compressed
[ "${COLOR:-false}" = "true" ] && FRAME_TOPIC=/camera/color/image_raw/compressed
start frames python3 "$J/frame_server.py" --topic "$FRAME_TOPIC"

# ── dashboard for PCs on the network: http://<jetson-ip>:8000/ ───────────────
DASH_PORT="${DASH_PORT:-8000}"
start dashboard python3 -m http.server "$DASH_PORT" --bind 0.0.0.0 --directory "$GHOST_DIR/dashboard"

# ── bridge (waits for Nav2, and drives only once the Pi's sensors report) ────
if [ "$NAV2" = "true" ] && [ -z "${NO_BRIDGE:-}" ]; then
  sleep 2
  start bridge python3 "$J/ghost_bridge.py" --ghost-ws "ws://127.0.0.1:${GHOST_ADDR##*:}/ws" \
    --cell-m "$CELL_M" --origin "$ORIGIN"
fi

echo "[ghost] running — core ws://$(hostname -I 2>/dev/null | awk '{print $1}'):${GHOST_ADDR##*:}/ws   Ctrl-C to stop"
echo "[ghost] dashboard http://$(hostname -I 2>/dev/null | awk '{print $1}'):$DASH_PORT/"
echo "[ghost] watch: tail -f $LOGS/*.log"
wait -n "${PIDS[@]}"
echo "[ghost] a process exited — see $LOGS/" >&2
