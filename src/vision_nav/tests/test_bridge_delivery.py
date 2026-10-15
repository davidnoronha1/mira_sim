"""Exercise the installed ros_gz_bridge, not a mocked configuration parser."""
import os
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class BridgeDeliveryTests(unittest.TestCase):
    def test_competition_profiles_load_sensors_and_commands(self):
        for profile, count in (('', 4), ('sauvc', 5)):
            with self.subTest(profile=profile), tempfile.TemporaryFile(mode='w+') as log:
                args = ['bash', os.path.join(HERE, 'competition_bridge.sh')]
                if profile:
                    args.append(profile)
                bridge = subprocess.Popen(args, stdout=log, stderr=log, start_new_session=True)
                try:
                    deadline = time.monotonic() + 8
                    output = ''
                    while time.monotonic() < deadline:
                        self.assertIsNone(bridge.poll(), 'competition bridge exited')
                        log.seek(0)
                        output = log.read()
                        if output.count('Creating GZ->ROS Bridge') == count:
                            break
                        time.sleep(0.1)
                    self.assertEqual(output.count('Creating GZ->ROS Bridge'), count, output)
                    self.assertEqual(output.count('Creating ROS->GZ Bridge'), bool(profile), output)
                    self.assertEqual(output.count('Creating ROS->GZ service bridge'), bool(profile), output)
                    self.assertEqual(output.count('(Lazy 1)'), count, output)
                    self.assertNotIn('Failed', output)
                finally:
                    import signal
                    try:
                        os.killpg(bridge.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    bridge.wait(timeout=5)

    def test_lazy_depth_bridge_delivers_to_both_qos_profiles(self):
        rclpy.init()
        node = rclpy.create_node('bridge_delivery_test')
        reliable, best_effort = [], []
        node.create_subscription(Image, '/vision/depth', reliable.append, 1)
        node.create_subscription(Image, '/vision/depth', best_effort.append,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        # One float32 pixel at 1m. The optical frame and simulation timestamp
        # must survive the bridge unchanged.
        body = ('header { stamp { sec: 12 nsec: 34 } '
                'data { key: "frame_id" value: "front_camera_depth_optical" } } '
                'width: 1 height: 1 step: 4 pixel_format_type: R_FLOAT32 '
                'data: "\\000\\000\\200\\077"')
        with tempfile.TemporaryFile(mode='w+') as log:
            bridge = subprocess.Popen(
                ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args',
                 '-p', 'config_file:=' + os.path.join(HERE, 'vision_bridge.yaml')],
                stdout=log, stderr=log, start_new_session=True)
            try:
                for _ in range(10):
                    self.assertIsNone(bridge.poll(), 'bridge exited during startup')
                    subprocess.run(['gz', 'topic', '-t', '/sim/front_camera/depth_image',
                                    '-m', 'gz.msgs.Image', '-p', body],
                                   check=True, capture_output=True, timeout=5)
                    deadline = time.monotonic() + 0.5
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    if reliable and best_effort:
                        break
                log.seek(0)
                output = log.read()
                self.assertTrue(reliable, output)
                self.assertTrue(best_effort, output)
                for msg in (reliable[-1], best_effort[-1]):
                    self.assertEqual(msg.encoding, '32FC1')
                    self.assertEqual(msg.header.frame_id, 'front_camera_depth_optical')
                    self.assertEqual(msg.header.stamp.sec, 12)
                    self.assertEqual(msg.header.stamp.nanosec, 34)
                    self.assertEqual(bytes(msg.data), b'\x00\x00\x80\x3f')
            finally:
                # ros2 run has a child process; terminate this test's group only.
                import signal
                try:
                    os.killpg(bridge.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                bridge.wait(timeout=5)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
