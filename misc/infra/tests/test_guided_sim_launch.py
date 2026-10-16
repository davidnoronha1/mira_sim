"""Exercise the connection launch's argument forwarding without installed ROS."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

LAUNCH = Path(__file__).resolve().parents[4] / 'mira/src/mira2_hardware/launch/guided_sim.launch.py'


class Configuration:
    def __init__(self, key):
        self.key = key

    def perform(self, context):
        return context[self.key]


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


def load_launch():
    modules = {}
    for name in ('launch', 'launch.actions', 'launch.launch_description_sources', 'launch.substitutions',
                 'launch_ros', 'launch_ros.substitutions'):
        modules[name] = types.ModuleType(name)
    modules['launch'].LaunchDescription = lambda actions: actions
    for name in ('DeclareLaunchArgument', 'IncludeLaunchDescription', 'OpaqueFunction', 'SetEnvironmentVariable'):
        setattr(modules['launch.actions'], name, Action)
    modules['launch.launch_description_sources'].PythonLaunchDescriptionSource = Action
    modules['launch.substitutions'].LaunchConfiguration = Configuration
    modules['launch.substitutions'].PathJoinSubstitution = Action
    modules['launch_ros.substitutions'].FindPackageShare = Action
    spec = importlib.util.spec_from_file_location('guided_sim_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


@unittest.skipUnless(LAUNCH.is_file(), 'sibling Mira checkout unavailable')
class GuidedSimLaunchTests(unittest.TestCase):
    def test_localhost_links_and_ros_topics_forward_to_regular_guided(self):
        module = load_launch()
        context = {**module.DEFAULTS, 'config': '/test/navigation.yaml'}
        include = module.include_guided(context)[0]
        arguments = dict(include.kwargs['launch_arguments'])
        self.assertEqual(arguments['goal_link'], 'udpin:127.0.0.1:14557')
        self.assertEqual(arguments['vision_link'], 'udpin:127.0.0.1:14555')
        self.assertEqual(arguments['obstacle_link'], 'udpin:127.0.0.1:14558')
        self.assertEqual(arguments['rgb_topic'], '/vision/image')
        self.assertEqual(arguments['depth_topic'], '/vision/depth')
        self.assertEqual(arguments['simulation'], 'true')
        self.assertEqual(arguments['auto_arm'], 'false')
        self.assertEqual(include.args[0].args[0].args[0][-1], 'guided.launch.py')

    def test_topic_pose_and_bind_overrides_are_preserved(self):
        module = load_launch()
        context = {**module.DEFAULTS, 'config': '/test/navigation.yaml', 'bind_ip': '0.0.0.0',
                   'rgb_topic': '/custom/rgb', 'pose_source': 'optical_flow',
                   'flow_scene_config': '/test/floor.yaml'}
        arguments = dict(module.include_guided(context)[0].kwargs['launch_arguments'])
        self.assertEqual(arguments['goal_link'], 'udpin:0.0.0.0:14557')
        self.assertEqual(arguments['rgb_topic'], '/custom/rgb')
        self.assertEqual(arguments['pose_source'], 'optical_flow')
        self.assertEqual(arguments['flow_scene_config'], '/test/floor.yaml')
