"""RViz plus the scene markers it draws (world SDF + AUV at EKF/truth poses).

Run without a package: ros2 launch /workspace/vision_nav/viz.launch.py world:=/workspace/worlds/tacc.world
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', description='world SDF that scene_markers draws'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(HERE, 'vision_nav.rviz')),
        ExecuteProcess(
            cmd=['python3', '-u', os.path.join(HERE, 'scene_markers.py'), LaunchConfiguration('world')],
            name='scene_markers', output='screen'),
        ExecuteProcess(
            cmd=['rviz2', '-d', LaunchConfiguration('rviz_config'),
                 '--ros-args', '-p', 'use_sim_time:=true'],
            name='rviz2', output='screen'),
    ])
