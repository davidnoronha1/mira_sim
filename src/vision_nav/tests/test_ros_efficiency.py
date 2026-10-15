"""Run in the Gazebo image with ROS sourced; no SITL/GPU needed."""
import os
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from goal_bridge import GoalBridge, M
from obstacle_distance import ObstacleDistance


class RosEfficiencyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def goal_node(self, visualize='true'):
        mav = Mock()
        hb = Mock(autopilot=M.MAV_AUTOPILOT_ARDUPILOTMEGA)
        hb.get_srcSystem.return_value = 1
        hb.get_srcComponent.return_value = 1
        mav.recv_match.return_value = hb
        args = SimpleNamespace(visualize=visualize, path_max_points=3, path_rate=2.0,
                               mavlink='unused', radius=0.3, depth=1.2)
        with patch('goal_bridge.mavutil.mavlink_connection', return_value=mav), \
                patch('goal_bridge.threading.Thread.start'):
            node = GoalBridge(args)
        self.addCleanup(node.destroy_node)
        node.attitude = (0, 0, 0)
        return node

    def test_pose_and_tf_are_not_path_rate_limited(self):
        node = self.goal_node()
        node.pub_pose = Mock()
        node.tf = Mock()
        node.pub_ekf_path = Mock()
        node.pub_gt_path = Mock()
        for x in range(20):
            node._publish_ekf(SimpleNamespace(x=x, y=0, z=1.2))
        self.assertEqual(node.pub_pose.publish.call_count, 20)
        self.assertEqual(node.tf.sendTransform.call_count, 20)
        node.pub_ekf_path.publish.assert_not_called()
        node._publish_paths()
        path = node.pub_ekf_path.publish.call_args.args[0]
        self.assertEqual(len(path.poses), 3)
        self.assertEqual(path.poses[-1].pose.position.y, 19)
        self.assertEqual(path.header.frame_id, 'odom')

    def test_paths_are_not_serialized_without_subscribers(self):
        node = self.goal_node()
        node.ekf_path.append(PoseStamped())
        node.pub_ekf_path = Mock()
        node.pub_gt_path = Mock()
        node.pub_ekf_path.get_subscription_count.return_value = 0
        node.pub_gt_path.get_subscription_count.return_value = 0
        node._publish_paths()
        node.pub_ekf_path.publish.assert_not_called()
        node.pub_gt_path.publish.assert_not_called()

    def test_no_visualization_keeps_goals_and_tf(self):
        node = self.goal_node('false')
        node.pub_pose = Mock()
        node.tf = Mock()
        node._publish_ekf(SimpleNamespace(x=0, y=0, z=1.2))
        self.assertEqual(node.ekf_path.snapshot(), [])
        node.tf.sendTransform.assert_called_once()
        with patch.object(node, '_gz_goal_markers') as markers:
            goal = PoseStamped()
            goal.header.frame_id = 'odom'
            goal.pose.position.x = 2.0
            goal.pose.orientation.w = 1.0
            node._on_goal(goal)
            self.assertEqual(node.goal[:3], (0.0, 2.0, 1.2))
            markers.assert_not_called()

    def test_depth_qos_and_mavlink_without_rviz(self):
        args = SimpleNamespace(mavlink='unused', band=0.25, min_range=0.2, max_range=8)
        with patch('obstacle_distance.mavutil.mavlink_connection', return_value=Mock()):
            node = ObstacleDistance(args)
        self.addCleanup(node.destroy_node)
        depth_sub = next(s for s in node.subscriptions if s.topic_name == '/vision/depth')
        self.assertEqual(depth_sub.qos_profile.depth, 1)
        self.assertEqual(depth_sub.qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT)
        node.info = CameraInfo()
        rays = np.zeros((2, 2, 3))
        rays[..., 0] = 1
        node.geom = dict(rays=rays, origin=np.zeros(3), hfov=1.2)
        node.pub_scan = Mock()
        node.pub_scan.get_subscription_count.return_value = 0
        msg = Image(height=4, width=4, encoding='32FC1', step=16,
                    data=np.ones((4, 4), dtype=np.float32).tobytes())
        node._on_depth(msg)
        node.mav.mav.obstacle_distance_send.assert_called_once()
        node.pub_scan.publish.assert_not_called()
        node.pub_scan.get_subscription_count.return_value = 1
        node._on_depth(msg)
        self.assertEqual(node.mav.mav.obstacle_distance_send.call_count, 2)
        node.pub_scan.publish.assert_called_once()

    def test_bridge_configs_are_directional_lazy_and_small(self):
        for name in ('vision_bridge', 'competition_bridge', 'sauvc_bridge'):
            with open(os.path.join(HERE, name + '.yaml')) as file:
                configs = yaml.safe_load(file)
            for config in configs:
                self.assertEqual(config['direction'], 'GZ_TO_ROS')
                self.assertEqual(config['publisher_queue'], 1)
                self.assertEqual(config['subscriber_queue'], 1)
                self.assertEqual(config['lazy'], config.get('topic_name') != '/clock')
        # Concatenation used by competition_bridge.sh must remain valid YAML.
        with open(os.path.join(HERE, 'competition_bridge.yaml')) as base, \
                open(os.path.join(HERE, 'sauvc_bridge.yaml')) as extra:
            self.assertEqual(len(yaml.safe_load(base.read() + extra.read())), 5)


if __name__ == '__main__':
    unittest.main()
