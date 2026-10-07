#!/usr/bin/env python3
"""Print visual-odometry drift against Gazebo ground truth.

Both streams are re-expressed relative to their own first sample (VO starts
at identity, GT starts at the spawn pose), with the GT start heading removed,
so the printed error is pure VO drift in the vehicle's start frame (FLU).
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class CompareOdom(Node):
    def __init__(self):
        super().__init__('compare_odom')
        self.gt = None
        self.gt0 = None
        self.vo0_seen = False
        self.lost = 0
        self.create_subscription(Odometry, '/vision/gt_odom', self._on_gt, 10)
        self.create_subscription(Odometry, '/vision/vo_odom', self._on_vo, 10)
        self.create_timer(1.0, self._report)
        self.last = None

    def _on_gt(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        if self.gt0 is None:
            self.gt0 = (np.array([p.x, p.y, p.z]), yaw)
        pos0, yaw0 = self.gt0
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        d = np.array([p.x, p.y, p.z]) - pos0
        self.gt = (np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]]), yaw - yaw0)

    def _on_vo(self, msg):
        if msg.pose.covariance[0] >= 1000.0:
            self.lost += 1
            return
        if self.gt is None:
            return
        p = msg.pose.pose.position
        vo = np.array([p.x, p.y, p.z])
        gt_pos, gt_yaw = self.gt
        err = vo - gt_pos
        yaw_err = math.degrees(math.remainder(yaw_of(msg.pose.pose.orientation) - gt_yaw, 2 * math.pi))
        self.last = (np.linalg.norm(gt_pos), np.linalg.norm(err[:2]), err[2], yaw_err)

    def _report(self):
        if self.last is None:
            self.get_logger().info('waiting for /vision/gt_odom and /vision/vo_odom ...')
            return
        travelled, xy, z, yaw = self.last
        self.get_logger().info(
            f'GT offset {travelled:6.2f} m | VO err xy {xy:5.2f} m  z {z:+5.2f} m  yaw {yaw:+6.1f} deg'
            f' | lost frames {self.lost}')


def main():
    rclpy.init()
    node = CompareOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
