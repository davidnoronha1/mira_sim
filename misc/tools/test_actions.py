"""Exercise the compiled BT's contracts in an isolated ROS domain (no vehicle).

ROS_DOMAIN_ID=79 python3 /sim/misc/tools/test_actions.py
Source ROS and /workspace/install/setup.bash first. Live simulator navigation
must be tested separately; these cases inject detections, TF and EKF feedback.
"""
import os
import copy
import re
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class ActionsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if int(os.environ.get('ROS_DOMAIN_ID', '0')) != 79:
            raise RuntimeError('Use ROS_DOMAIN_ID=79 to isolate synthetic feedback from the vehicle')
        rclpy.init()
        cls.executable = str(Path(get_package_prefix('mira2_actions')) /
                             'lib/mira2_actions/mira2_actions_exe')

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def exercise(self, mode):
        node = rclpy.create_node('actions_test')
        objects = node.create_publisher(Detection2DArray, '/mira/perception/front/objects', qos_profile_sensor_data)
        feedback = node.create_publisher(PoseStamped, '/vision/ekf_pose', 10)
        broadcaster = (StaticTransformBroadcaster(node) if mode in ('continuous', 'continuous_outlier', 'gate_cancel', 'gate_stale', 'scan', 'scan_cancel')
                       else TransformBroadcaster(node))
        goals, holds = [], []
        node.create_subscription(PoseStamped, '/goal_pose', goals.append, 10)
        # Gate crossings use the direct (no-OA) line; record them in the same order.
        node.create_subscription(PoseStamped, '/goal_pose_direct', goals.append, 10)

        def hold(request, response):
            holds.append(True)
            response.success = True
            return response

        node.create_service(Trigger, '/mira/guidance/hold', hold)
        xml = '''<root BTCPP_format="4"><BehaviorTree ID="Test"><Sequence>
          <SelectObject class_id="test" object="{target}"/>
          <NavigateToPose object="{target}" stand_off="0.5" timeout="1.5"/>
          </Sequence></BehaviorTree></root>'''
        if mode in ('continuous', 'continuous_outlier', 'gate_cancel', 'gate_stale'):
            xml = (Path(get_package_share_directory('mira2_actions')) / 'config/gate.xml').read_text()
            xml = re.sub(r'<ScanArea[^>]*/>', '', xml)
        if mode in ('scan', 'scan_cancel', 'scan_invalid', 'scan_shallow'):
            xml = '<root BTCPP_format="4"><BehaviorTree ID="Test"><ScanArea radius=".5" sweep_degrees="90" step_degrees="45" depth="1" dwell=".05" timeout="5"/></BehaviorTree></root>'
            if mode == 'scan_invalid':
                xml = xml.replace('radius=".5"', 'radius="-.5"')
            if mode == 'scan_shallow':
                xml = xml.replace('depth="1"', 'depth=".3"')
        if mode == 'through':
            xml = xml.replace('stand_off="0.5"', 'through_distance="1.5"').replace('timeout="1.5"', 'timeout="5"')
        if mode == 'invalid_offsets':
            xml = xml.replace('stand_off="0.5"', 'stand_off="0.5" through_distance="1.5"')
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml') as tree:
            tree.write(xml)
            tree.flush()
            process = subprocess.Popen([self.executable, '--ros-args', '-p', 'tree_xml:='+tree.name,
                                        '-p', 'groot_port:=17979', '-p', 'target_class:=test'],
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            deadline = time.monotonic() + 8
            cancelled = False
            outlier_samples = 0
            approach_samples = 0
            try:
                while process.poll() is None and time.monotonic() < deadline:
                    stamp = node.get_clock().now().to_msg()
                    transforms = []
                    for parent, child in [('odom', 'base_link'), ('base_link', 'test_camera')]:
                        tf = TransformStamped()
                        tf.header.frame_id, tf.child_frame_id, tf.header.stamp = parent, child, stamp
                        tf.transform.rotation.w = 1.
                        transforms.append(tf)
                    broadcaster.sendTransform(transforms)
                    pose = PoseStamped()
                    pose.header.frame_id, pose.header.stamp = 'odom', stamp
                    pose.pose.orientation.w = 1.
                    if mode in ('success', 'through', 'continuous', 'continuous_outlier', 'scan') and goals:
                        pose.pose = copy.deepcopy(goals[-1].pose)
                        if mode in ('continuous', 'continuous_outlier') and len(goals) == 1:
                            # Switch while still 0.5 m before the staging goal.
                            approach_samples += 1
                            pose.pose.position.x = .5 if approach_samples <= 15 else 3.5
                    if not (mode in ('stale_feedback', 'gate_stale') and goals):
                        feedback.publish(pose)
                    detection = Detection2D()
                    detection.header.frame_id, detection.header.stamp = 'test_camera', stamp
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = 'test'
                    hypothesis.hypothesis.score = 0.9
                    hypothesis.pose.pose.orientation.w = 1.
                    hypothesis.pose.pose.position.z = 0. if mode == 'unlocalized' else 5.
                    if mode in ('through', 'invalid_offsets', 'continuous', 'continuous_outlier', 'gate_cancel', 'gate_stale'):
                        hypothesis.pose.pose.position.x = 5.
                        hypothesis.pose.pose.position.z = 2.
                    # Changing subsequent observations must not replace the goal.
                    if goals and mode not in ('continuous', 'continuous_outlier', 'gate_cancel', 'gate_stale'):
                        hypothesis.pose.pose.position.z = 20.
                    if goals and mode in ('continuous', 'continuous_outlier', 'gate_cancel', 'gate_stale'):
                        hypothesis.pose.pose.position.x = 5.25
                        if mode == 'continuous_outlier' and outlier_samples < 5:
                            hypothesis.pose.pose.position.x = 2.0
                            outlier_samples += 1
                    detection.results = [hypothesis]
                    array = Detection2DArray(header=detection.header)
                    array.detections = [] if mode == 'missing' else [detection]
                    objects.publish(array)
                    rclpy.spin_once(node, timeout_sec=.02)
                    if mode in ('cancel', 'gate_cancel', 'scan_cancel') and goals and not cancelled:
                        process.send_signal(signal.SIGINT)
                        cancelled = True
                    time.sleep(.02)
                if process.poll() is None:
                    process.kill()
                    self.fail(f'{mode} did not finish')
                output = process.communicate(timeout=2)[0]
                print(f'\n{mode}:\n{output}', flush=True)
                self.assertEqual(process.returncode, 0 if mode in ('success', 'through', 'continuous', 'continuous_outlier', 'scan') else 1)
                self.assertEqual(len(goals), 0 if mode in ('missing', 'unlocalized', 'invalid_offsets', 'scan_invalid', 'scan_shallow') else (5 if mode == 'scan' else (2 if mode in ('continuous', 'continuous_outlier', 'through') else 1)))
                if mode == 'scan':
                    self.assertAlmostEqual(goals[0].pose.position.x, 0., places=3)
                    self.assertAlmostEqual(goals[0].pose.position.y, 0., places=3)
                    self.assertAlmostEqual(goals[1].pose.position.x, .5, places=3)
                    self.assertAlmostEqual(goals[3].pose.position.y, .5, places=3)
                    self.assertAlmostEqual(goals[4].pose.position.x, 0., places=3)
                    self.assertAlmostEqual(goals[4].pose.position.y, 0., places=3)
                    self.assertTrue(all(g.pose.position.z == -1 for g in goals))
                elif mode in ('continuous', 'continuous_outlier'):
                    self.assertAlmostEqual(goals[0].pose.position.x, 3.0, places=3)  # 2 m staging
                    # Crossing goal: averaged gate (5.0 first, 5.25 after) + 1.5 m.
                    self.assertGreater(goals[1].pose.position.x, 6.5)
                    self.assertLessEqual(goals[1].pose.position.x, 6.75 + 1e-6)
                    self.assertAlmostEqual(goals[1].pose.position.z, 2., places=3)
                elif goals and mode not in ('gate_cancel', 'gate_stale', 'scan_cancel'):
                    self.assertAlmostEqual(goals[0].pose.position.z, 2. if mode == 'through' else 4.5, places=3)
                    if mode == 'through':
                        self.assertAlmostEqual(goals[0].pose.position.x, 3.5, places=3)
                        self.assertAlmostEqual(goals[1].pose.position.x, 6.5, places=3)
                        self.assertAlmostEqual(goals[1].pose.position.z, 2., places=3)
                if mode in ('timeout', 'stale_feedback', 'cancel', 'gate_cancel', 'gate_stale', 'scan_cancel'):
                    self.assertEqual(len(holds), 1, 'failure/cancel must request hold before exit')
                else:
                    self.assertFalse(holds)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                node.destroy_node()

    def test_success_and_latched_goal(self):
        self.exercise('success')

    def test_missing_target(self):
        self.exercise('missing')

    def test_unlocalized_box(self):
        self.exercise('unlocalized')

    def test_timeout_hold(self):
        self.exercise('timeout')

    def test_stale_feedback_hold(self):
        self.exercise('stale_feedback')

    def test_cancellation_hold(self):
        self.exercise('cancel')

    def test_through_goal_keeps_gate_depth(self):
        self.exercise('through')

    def test_gate_switches_while_moving_with_fresh_observation(self):
        self.exercise('continuous')

    def test_gate_rejects_foreground_depth_jump(self):
        self.exercise('continuous_outlier')

    def test_gate_cancellation_hold(self):
        self.exercise('gate_cancel')

    def test_gate_stale_feedback_hold(self):
        self.exercise('gate_stale')

    def test_scan_area_radius_sweep_and_return(self):
        self.exercise('scan')

    def test_scan_cancellation_hold(self):
        self.exercise('scan_cancel')

    def test_invalid_scan_area_rejected(self):
        self.exercise('scan_invalid')

    def test_scan_refuses_lateral_motion_above_ring_clearance(self):
        self.exercise('scan_shallow')

    def test_conflicting_offsets_rejected(self):
        self.exercise('invalid_offsets')


if __name__ == '__main__':
    unittest.main(verbosity=2)
