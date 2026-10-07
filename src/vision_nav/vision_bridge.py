#!/usr/bin/env python3
"""Forward a ROS odometry stream to ArduSub as an external-nav (vision) pose.

Subscribes to nav_msgs/Odometry and sends MAVLink ODOMETRY (#331), which
ArduSub fuses when VISO_TYPE=1 and EK3_SRC1_POSXY/VELXY=6 (see
docker/ardusub_vision.parm).

Sources:
  gt  /vision/gt_odom  Gazebo ground truth, world ENU / body FLU
  vo  /vision/vo_odom  RTAB-Map visual odometry, start-pose FLU / body FLU

Both are converted to a local NED frame with a body FRD child frame. The
first sample is rotated about the vertical so its heading matches ArduSub's
current AHRS yaw (compass), so a VO frame anchored at "whatever way the
vehicle was facing" lines up with NED and waypoints mean the same thing for
both sources.

On startup the EKF origin is set via SET_GPS_GLOBAL_ORIGIN (there is no GPS
to set it), matching --custom-location in docker-compose.yml.
"""
import argparse
import math
import os
import threading
import time

os.environ.setdefault('MAVLINK20', '1')

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from rclpy.node import Node

TOPICS = {'gt': '/vision/gt_odom', 'vo': '/vision/vo_odom'}

# Fixed frame changes (rotation matrices, applied as R @ v)
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])
ENU_TO_NED = np.array([[0.0, 1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, -1.0]])
# VO odom frame is FLU at the start pose; treat its FRD as "NED with north
# = initial heading" and let the yaw alignment rotate it onto true NED.
SRC_TO_NED = {'gt': ENU_TO_NED, 'vo': FLU_TO_FRD}

# rtabmap publishes covariance 9999 when it has lost tracking
LOST_COVARIANCE = 1000.0

NAN21 = [float('nan')] * 21


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat_wxyz(r):
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    return [w, x, y, z]


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class VisionBridge(Node):
    def __init__(self, args):
        super().__init__('vision_bridge')
        self.args = args
        self.src_to_ned = SRC_TO_NED[args.source]
        self.align = None  # rotation about NED down, latched on first sample
        self.ahrs_yaw = None
        self.origin_set = False
        self.sent = 0
        self.dropped = 0

        self.mav = mavutil.mavlink_connection(
            args.mavlink, source_system=args.sysid,
            source_component=mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)
        self.get_logger().info(f'Waiting for ArduSub heartbeat on {args.mavlink} ...')
        self.mav.wait_heartbeat()
        self.get_logger().info(
            f'Heartbeat from system {self.mav.target_system}; forwarding {TOPICS[args.source]}')

        threading.Thread(target=self._mav_rx, daemon=True).start()
        self.create_timer(1.0, self._housekeeping)
        self.create_subscription(Odometry, TOPICS[args.source], self._on_odom, 10)

    def _mav_rx(self):
        while rclpy.ok():
            msg = self.mav.recv_match(type=['ATTITUDE', 'GPS_GLOBAL_ORIGIN'], blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() == 'ATTITUDE':
                self.ahrs_yaw = msg.yaw
            elif not self.origin_set:
                self.origin_set = True
                self.get_logger().info(
                    f'EKF origin set: {msg.latitude / 1e7:.6f}, {msg.longitude / 1e7:.6f}')

    def _housekeeping(self):
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if not self.origin_set:
            self.mav.mav.set_gps_global_origin_send(
                self.mav.target_system,
                int(self.args.origin_lat * 1e7), int(self.args.origin_lon * 1e7),
                int(self.args.origin_alt * 1000), int(time.time() * 1e6))
        if self.sent or self.dropped:
            self.get_logger().info(
                f'sent {self.sent} / dropped {self.dropped} (lost tracking) in last 1s',
                throttle_duration_sec=10.0)
        self.sent = self.dropped = 0

    def _on_odom(self, msg):
        if msg.pose.covariance[0] >= LOST_COVARIANCE:
            self.dropped += 1
            return

        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        r_src_body = quat_to_rot(q.x, q.y, q.z, q.w)
        r_ned_frd = self.src_to_ned @ r_src_body @ FLU_TO_FRD

        if self.align is None:
            if self.args.no_align:
                self.align = np.eye(3)
            elif self.ahrs_yaw is None:
                self.get_logger().warn('No ATTITUDE from ArduSub yet, holding off', throttle_duration_sec=5.0)
                return
            else:
                src_yaw = math.atan2(r_ned_frd[1, 0], r_ned_frd[0, 0])
                delta = wrap_pi(self.ahrs_yaw - src_yaw)
                self.align = rot_z(delta)
                self.get_logger().info(
                    f'Aligned {self.args.source} heading to AHRS: rotate {math.degrees(delta):+.1f} deg')

        pos = self.align @ self.src_to_ned @ np.array([p.x, p.y, p.z])
        r_ned_frd = self.align @ r_ned_frd

        lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
        vel = FLU_TO_FRD @ np.array([lin.x, lin.y, lin.z])
        rates = FLU_TO_FRD @ np.array([ang.x, ang.y, ang.z])

        stamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        self.mav.mav.odometry_send(
            stamp_us,
            mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            float(pos[0]), float(pos[1]), float(pos[2]),
            rot_to_quat_wxyz(r_ned_frd),
            float(vel[0]), float(vel[1]), float(vel[2]),
            float(rates[0]), float(rates[1]), float(rates[2]),
            NAN21, NAN21,
            0,  # reset_counter
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO,
            100)  # quality
        self.sent += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--source', choices=TOPICS, default='gt')
    ap.add_argument('--mavlink', default='udpin:0.0.0.0:14555')
    ap.add_argument('--sysid', type=int, default=1)
    ap.add_argument('--origin-lat', type=float, default=63.52)
    ap.add_argument('--origin-lon', type=float, default=10.35)
    ap.add_argument('--origin-alt', type=float, default=0.0)
    ap.add_argument('--no-align', action='store_true',
                    help='skip rotating the first sample onto the AHRS heading')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=['vision_bridge', *ros_args])
    node = VisionBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
