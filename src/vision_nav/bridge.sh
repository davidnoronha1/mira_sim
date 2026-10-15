#!/usr/bin/env bash
# Gazebo -> ROS plumbing for vision navigation. Runs inside the mira_sim
# container (see `make bringup-vision`):
#   - RGB-D image / depth / camera_info from the bluerov2_heavy front_camera_depth
#   - ground-truth odometry from the OdometryPublisher system on the model
#   - /clock so ROS nodes can run on sim time (SITL is lockstepped to Gazebo)
#   - static TF base_link -> front_camera_depth_link -> _optical (sensor
#     pose from model.sdf + the standard x-forward -> z-forward optical rotation)
#   - static TF odom -> world_origin (makes `odom` exist for RViz)
set -e
source /opt/ros/jazzy/setup.bash

# Camera mount: must match front_camera_depth's <pose> in
# bluerov2_gz/models/bluerov2_heavy/model.sdf (25 deg nose-down pitch)
CAMERA_PITCH=0.4363
ros2 run tf2_ros static_transform_publisher \
  --x 0.25 --y 0 --z 0.05 --pitch $CAMERA_PITCH \
  --frame-id base_link --child-frame-id front_camera_depth_link \
  --ros-args -p use_sim_time:=true &
TF_PID=$!
ros2 run tf2_ros static_transform_publisher \
  --yaw -1.5707963 --roll -1.5707963 \
  --frame-id front_camera_depth_link --child-frame-id front_camera_depth_optical \
  --ros-args -p use_sim_time:=true &
TF_PID="$TF_PID $!"
# Anchor the `odom` frame (= Gazebo world, ENU) in TF so RViz can use it as
# its fixed frame; everything goal_bridge.py draws is expressed in it.
ros2 run tf2_ros static_transform_publisher \
  --frame-id odom --child-frame-id world_origin \
  --ros-args -p use_sim_time:=true &
TF2_PID=$!
trap 'kill $TF_PID $TF2_PID 2>/dev/null' EXIT

# Keep the shell alive so its EXIT trap also cleans up static TF publishers.
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="$(dirname "$0")/vision_bridge.yaml" &
BRIDGE_PID=$!
trap 'kill $TF_PID $TF2_PID $BRIDGE_PID 2>/dev/null || true' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait "$BRIDGE_PID"
