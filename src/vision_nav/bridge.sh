#!/usr/bin/env bash
# Gazebo -> ROS plumbing for vision navigation. Runs inside the mira_sim
# container (see `make bringup-vision`):
#   - RGB-D image / depth / camera_info from the bluerov2_heavy front_camera_depth
#   - ground-truth odometry from the OdometryPublisher system on the model
#   - /clock so ROS nodes can run on sim time (SITL is lockstepped to Gazebo)
#   - static TF base_link -> front_camera_depth_optical (sensor pose from
#     model.sdf + the standard x-forward -> z-forward optical rotation)
set -e
source /opt/ros/jazzy/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0.25 --y 0 --z 0.05 --yaw -1.5707963 --pitch 0 --roll -1.5707963 \
  --frame-id base_link --child-frame-id front_camera_depth_optical \
  --ros-args -p use_sim_time:=true &
TF_PID=$!
trap 'kill $TF_PID 2>/dev/null' EXIT

exec ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /sim/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /sim/front_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image \
  /sim/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /model/bluerov2_heavy/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  --ros-args \
  -r /sim/front_camera/image:=/vision/image \
  -r /sim/front_camera/depth_image:=/vision/depth \
  -r /sim/front_camera/camera_info:=/vision/camera_info \
  -r /model/bluerov2_heavy/odometry:=/vision/gt_odom
