"""Log EKF heading against Gazebo truth for N seconds (flow-only heading check).

Usage: python3 yaw_drift.py SECONDS
Prints every 5 s: EKF yaw - true yaw (deg), and the floor-heading message rate.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

rclpy.init()
node = rclpy.create_node('yaw_drift')
state = {'ekf': None, 'gt': None, 'headings': 0}


def yaw(q):
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX')[0]


node.create_subscription(PoseStamped, '/vision/ekf_attitude', lambda m: state.update(ekf=yaw(m.pose.orientation)), 10)
node.create_subscription(Odometry, '/vision/gt_odom', lambda m: state.update(gt=yaw(m.pose.pose.orientation)), 10)
node.create_subscription(PoseWithCovarianceStamped, '/mira/perception/bottom/floor_heading',
                         lambda m: state.update(headings=state['headings'] + 1), 10)
end, next_print, errors = time.monotonic() + float(sys.argv[1]), time.monotonic() + 5, []
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
    if state['ekf'] is not None and state['gt'] is not None:
        errors.append(math.degrees((state['ekf'] - state['gt'] + math.pi) % (2 * math.pi) - math.pi))
    if time.monotonic() > next_print and errors:
        print(f'yaw error {errors[-1]:+6.2f} deg  floor headings {state["headings"] / 5:.1f}/s', flush=True)
        state['headings'], next_print = 0, next_print + 5
if errors:
    print(f'max |yaw error| {max(map(abs, errors)):.2f} deg over {sys.argv[1]} s')
