"""Vision nav bridges to ArduSub, in one process group:

  vision_bridge.py      odometry -> MAVLink ODOMETRY (udp 14555)
  obstacle_distance.py  depth image -> MAVLink OBSTACLE_DISTANCE (udp 14558)
  goal_bridge.py        RViz /goal_pose -> GUIDED target (udp 14557)

Run without a package: ros2 launch /workspace/vision_nav/nav.launch.py source:=vo start_x:=7 start_y:=-10
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))


def script(name, *args):
    return ExecuteProcess(cmd=['python3', '-u', os.path.join(HERE, f'{name}.py'), *args],
                          name=name, output='screen')


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='vo', description='vo | gt'),
        DeclareLaunchArgument('start_x', default_value='0', description='AUV start, world east (m)'),
        DeclareLaunchArgument('start_y', default_value='0', description='AUV start, world north (m)'),
        DeclareLaunchArgument('depth', default_value='1.2', description='depth for 2D goals (m)'),
        script('vision_bridge', '--source', arg('source'), '--start-enu', arg('start_x'), arg('start_y')),
        script('obstacle_distance'),
        script('goal_bridge', '--depth', arg('depth')),
    ])
