#!/usr/bin/env bash
# Send ONE Nav2 goal relative to the launch point (odom frame), with feedback.
# Tests Nav2 + base without GHOST.  Stop the bridge first so it doesn't
# replace the goal.
#
#   bash bench/nav_goal.sh 0.5 0      # half a metre forward
#   bash bench/nav_goal.sh 0 0        # back to the launch point
set -euo pipefail
X="${1:?x metres}"; Y="${2:-0}"
ROS_DISTRO="${ROS_DISTRO:-$(ls /opt/ros | head -1)}"
set +u   # ROS setup files read unset variables
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u
# If this script is interrupted or killed (Ctrl-C, `timeout`, SIGTERM), CANCEL
# the goal: killing the client alone leaves Nav2 driving to it.
cancel() {
  trap - INT TERM
  [ -n "${GOAL_PID:-}" ] && kill -INT "$GOAL_PID" 2>/dev/null
  timeout 5 ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}" >/dev/null 2>&1 \
    && echo "nav_goal: goal cancelled" || echo "nav_goal: CANCEL FAILED — run bench/stop.sh / use the kill" >&2
  exit 130
}
trap cancel INT TERM
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {w: 1.0}}}}" &
GOAL_PID=$!
wait "$GOAL_PID"   # wait (unlike a foreground command) lets the trap run at once
STATUS=$?
trap - INT TERM
exit "$STATUS"
