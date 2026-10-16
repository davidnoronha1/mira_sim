"""Gazebo -> ROS plumbing for the independent simulator (bringup.py "bridge" window).

Runs inside the simulator container, next to gz sim:
  - cameras (front RGB-D, bottom), IMU, ground-truth odometry and /clock
    (topics in ../config/bridge.yaml)
  - static TFs for the camera/IMU mounts; they must match the <pose>s in
    bluerov2_gz/models/bluerov2_heavy/model.sdf
  - static TF odom -> world_origin (makes `odom` exist for RViz)
"""
from math import pi
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = Path(__file__).resolve().parents[1] / 'config'
# x-forward link -> z-forward optical frame
OPTICAL = {'yaw': -pi / 2, 'roll': -pi / 2}
# (parent, child, pose): pose keys as static_transform_publisher's options.
MOUNTS = [
    # front_camera_depth: 25 deg nose-down pitch
    ('base_link', 'front_camera_depth_link', {'x': 0.25, 'z': 0.05, 'pitch': 0.4363}),
    ('front_camera_depth_link', 'front_camera_depth_optical', OPTICAL),
    # bottom_camera: pitched straight down
    ('base_link', 'bottom_camera_link', {'z': -0.05, 'pitch': pi / 2}),
    ('bottom_camera_link', 'bottom_camera_optical', OPTICAL),
    # imu_sensor (ArduPilot FRD, roll pi); the bridged sensor_msgs/Imu carries
    # this Gazebo-scoped frame id.
    ('base_link', 'bluerov2_heavy/base_link/imu_sensor', {'roll': pi}),
    # `odom` = Gazebo world (ENU), anchored so RViz can use it as fixed frame.
    ('odom', 'world_origin', {}),
]


def static_tf(parent, child, pose):
    arguments = [item for key, value in pose.items() for item in (f'--{key}', str(value))]
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name=f'tf_{child.replace("/", "_")}', output='log',
                arguments=[*arguments, '--frame-id', parent, '--child-frame-id', child],
                parameters=[{'use_sim_time': True}])


def generate_launch_description():
    return LaunchDescription([
        *[static_tf(*mount) for mount in MOUNTS],
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sim_bridge', output='screen',
             parameters=[{'config_file': str(CONFIG / 'bridge.yaml')}]),
    ])
