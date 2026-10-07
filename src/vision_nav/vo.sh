#!/usr/bin/env bash
# RTAB-Map RGB-D visual odometry on the bridged front_camera_depth topics
# (see bridge.sh). Publishes nav_msgs/Odometry on /vision/vo_odom in an
# odom frame anchored at the vehicle's start pose (x = initial forward).
# Extra ROS args/rtabmap parameters are appended verbatim, e.g.
#   vo.sh -p Reg/Strategy:="'1'"    (0=visual, 1=ICP on depth, 2=visual+ICP)
set -e
source /opt/ros/jazzy/setup.bash

exec ros2 run rtabmap_odom rgbd_odometry \
  --ros-args \
  -p use_sim_time:=true \
  -p frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p publish_tf:=false \
  -p wait_for_transform:=0.2 \
  -p approx_sync:=true \
  -p approx_sync_max_interval:=0.01 \
  -p publish_null_when_lost:=true \
  -p Odom/ResetCountdown:="'0'" \
  -r rgb/image:=/vision/image \
  -r depth/image:=/vision/depth \
  -r rgb/camera_info:=/vision/camera_info \
  -r odom:=/vision/vo_odom \
  "$@"
