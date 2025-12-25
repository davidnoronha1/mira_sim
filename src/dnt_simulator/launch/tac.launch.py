from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml
from launch_ros.actions import Node

def generate_launch_description():

    simulation_data = PathJoinSubstitution([FindPackageShare('common_resources'), 'data'])
    scenario_desc = PathJoinSubstitution([FindPackageShare('vortex_simulator'), 'scenarios', 'tacc.scn'])
    simulation_rate = '300.0'
    window_res_x = '1200'
    window_res_y = '800'
    rendering_quality = 'high'

    params_path = Path(get_package_share_directory('vortex_simulator')) / 'config' / 'tacc_config.yaml'
    # pprint(params_path)
    with open(params_path, 'r') as f:
        params = yaml.safe_load(f)

    stonefish_node = Node(
            package='stonefish_ros2',
            executable='stonefish_simulator',
            namespace='stonefish_ros2',
            name='stonefish_simulator',
            arguments=[simulation_data, scenario_desc, simulation_rate, window_res_x, window_res_y, rendering_quality],
            output='screen',
            parameters=[ params ]
            #prefix=['xterm -e gdb -ex run --args']
    )

    return LaunchDescription([
        stonefish_node
    ])