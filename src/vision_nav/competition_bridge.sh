#!/usr/bin/env bash
# Basic competition plumbing. Vision navigation uses bridge.sh instead.
set -e
source /opt/ros/jazzy/setup.bash
HERE=$(dirname "$0")
CONFIG="$HERE/competition_bridge.yaml"
ARGS=()
NS=/bluerov2_bridge
if [ "${1:-}" = sauvc ]; then
  CONFIG=$(mktemp /tmp/mira-bridge.XXXXXX.yaml)
  trap 'rm -f "$CONFIG"' EXIT
  cat "$HERE/competition_bridge.yaml" "$HERE/sauvc_bridge.yaml" > "$CONFIG"
  NS=/sauvc_bridge
  # Commands are ROS -> Gazebo only; services retain their original endpoint.
  ARGS+=( '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
          '/world/pool_world/create@ros_gz_interfaces/srv/SpawnEntity' )
fi
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="$CONFIG" -r __ns:="$NS" &
PID=$!
if [ "${1:-}" = sauvc ]; then
  # The installed Jazzy bridge ignores config_file when positional bridges
  # are supplied. Keep command/service plumbing separate from the YAML topics.
  ros2 run ros_gz_bridge parameter_bridge "${ARGS[@]}" --ros-args \
    -r __ns:="$NS" -r __node:=commands_bridge &
  PID="$PID $!"
fi
trap 'kill $PID 2>/dev/null || true; if [ "$CONFIG" != "$HERE/competition_bridge.yaml" ]; then rm -f "$CONFIG"; fi' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait -n $PID
