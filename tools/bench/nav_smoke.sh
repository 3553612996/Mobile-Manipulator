#!/usr/bin/env bash
# Offline Nav2 smoke test: fake base + map raycaster -> navigation.launch.py
# -> NavigateToPose. No robot hardware is touched.
#
#   tools/bench/nav_smoke.sh
#   tools/bench/nav_smoke.sh --goal 1.5 -0.5 0 --timeout 120
#   tools/bench/nav_smoke.sh --map mobile_manipulator_ws/src/robot_bringup/maps/room1.yaml
#
# Requires ROS 2 Humble and Nav2. The workspace is built automatically if
# install/ is missing. This is a smoke test of the stack and parameters; the
# arrival error printed here is a simulation number, not the acceptance result
# on the floor.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$REPO/mobile_manipulator_ws"
MAP="$WS/src/robot_bringup/maps/room2.yaml"
GOAL=(1.5 -0.5 0)
TIMEOUT=120
NAV_LOG="${TMPDIR:-/tmp}/m5_nav2.log"
SIM_LOG="${TMPDIR:-/tmp}/m5_nav_sim.log"
GOAL_LOG="${TMPDIR:-/tmp}/m5_nav_goal.log"

usage() { sed -n '2,12p' "$0"; exit 0; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP="$2"; shift 2 ;;
    --goal) GOAL=("$2" "$3" "$4"); shift 4 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# ROS setup scripts read unset variables; keep nounset off while sourcing.
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
elif [[ -f "$WS/install/setup.sh" ]]; then
  source "$WS/install/setup.sh"
else
  echo "workspace not built: $WS/install/setup.{bash,sh} missing" >&2
  exit 2
fi

# Stay off domain 0 by default: the offline stack must not collide with a real
# robot stack (same node names, same action server names).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

descendants() {
  local parent=$1 child
  for child in $(pgrep -P "$parent" 2>/dev/null); do
    echo "$child"
    descendants "$child"
  done
}

pids=()
cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  # ros2 launch does not always forward SIGTERM to its children. Collect the
  # whole process tree first, then kill only our own PIDs -- never pkill by
  # node name, that would take down an unrelated navigation stack.
  local victims=() child
  for pid in "${pids[@]}"; do
    victims+=("$pid")
    while IFS= read -r child; do victims+=("$child"); done < <(descendants "$pid")
  done
  kill "${victims[@]}" 2>/dev/null || true
  sleep 1
  kill -9 "${victims[@]}" 2>/dev/null || true
  exit "$rc"
}
trap cleanup EXIT INT TERM

echo "== start navigation.launch.py (log: $NAV_LOG)"
ros2 launch robot_bringup navigation.launch.py rviz:=false >"$NAV_LOG" 2>&1 &
pids+=("$!")

sleep 3
echo "== start fake base (log: $SIM_LOG)"
python3 "$REPO/tools/bench/nav_sim.py" --map "$MAP" >"$SIM_LOG" 2>&1 &
pids+=("$!")

echo "== goal: ${GOAL[*]} (timeout ${TIMEOUT}s)"
python3 "$REPO/tools/bench/nav_goal.py" --goal "${GOAL[@]}" \
    --amcl-timeout 20 --nav-active-timeout 60 --timeout "$TIMEOUT" >"$GOAL_LOG" 2>&1
rc=$?

cat "$GOAL_LOG"
if [[ $rc -ne 0 ]]; then
  echo "== nav2 tail ($NAV_LOG)"
  tail -30 "$NAV_LOG"
fi
exit $rc
