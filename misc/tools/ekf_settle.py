"""Log EKF horizontal speed (from /vision/ekf_pose) against truth while idle.

Usage: python3 ekf_settle.py SECONDS
Every 2 s: seconds since the first EKF pose, EKF speed, true speed, EKF and true roll/pitch.
"""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

rclpy.init()
node = rclpy.create_node('ekf_settle')
ekf, truth = [], []
tilt = {'ekf': None, 'truth': None}


def roll_pitch(q):
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX', degrees=True)[[2, 1]]


node.create_subscription(PoseStamped, '/vision/ekf_attitude', lambda m: tilt.update(ekf=roll_pitch(m.pose.orientation)), 10)
node.create_subscription(Odometry, '/vision/gt_odom', lambda m: tilt.update(truth=roll_pitch(m.pose.pose.orientation)), 10)
node.create_subscription(PoseStamped, '/vision/ekf_pose',
                         lambda m: ekf.append((time.monotonic(), m.pose.position.x, m.pose.position.y)), 10)
node.create_subscription(Odometry, '/vision/gt_odom',
                         lambda m: truth.append((time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y)), 10)


def speed(rows, since):
    a = np.array([r for r in rows if r[0] >= since])
    if len(a) < 2:
        return float('nan')
    return np.linalg.norm(a[-1, 1:] - a[0, 1:]) / (a[-1, 0] - a[0, 0])


end = time.monotonic() + float(sys.argv[1])
start = None
next_print = time.monotonic() + 2
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
    if ekf and start is None:
        start = ekf[0][0]
    if time.monotonic() > next_print:
        now = time.monotonic()
        label = f'{now - start:6.1f}s since first EKF pose' if start else 'no EKF pose yet'
        rp = ' '.join(f'{k} roll/pitch {v[0]:+6.1f} {v[1]:+6.1f}' for k, v in tilt.items() if v is not None)
        print(f'{label}: EKF speed {speed(ekf, now - 2):6.2f} m/s, truth {speed(truth, now - 2):5.2f} m/s  {rp}', flush=True)
        next_print += 2
