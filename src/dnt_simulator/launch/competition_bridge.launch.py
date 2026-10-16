"""Basic Gazebo -> ROS plumbing for bringup.py up --mode sim.

competition:=sauvc adds the SAUVC topics (../config/sauvc_bridge.yaml) and the
ROS -> Gazebo command/spawn bridges. The Mira stack uses bridge.launch.py instead.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def bridges(context):
    sauvc = context.launch_configurations['competition'] == 'sauvc'
    namespace = '/sauvc_bridge' if sauvc else '/bluerov2_bridge'
    nodes = [Node(package='ros_gz_bridge', executable='parameter_bridge', namespace=namespace,
                  name='competition_bridge', output='screen',
                  parameters=[{'config_file': str(CONFIG / 'competition_bridge.yaml')}])]
    if sauvc:
        nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge', namespace=namespace,
                          name='sauvc_topics_bridge', output='screen',
                          parameters=[{'config_file': str(CONFIG / 'sauvc_bridge.yaml')}]))
        # The Jazzy bridge ignores config_file when positional bridges are
        # given, so commands/services run as their own node.
        nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge', namespace=namespace,
                          name='commands_bridge', output='screen',
                          arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                                     '/world/pool_world/create@ros_gz_interfaces/srv/SpawnEntity']))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('competition', default_value='', description="'sauvc' or empty"),
        OpaqueFunction(function=bridges),
    ])
