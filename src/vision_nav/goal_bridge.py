#!/usr/bin/env python3
"""Click-to-go for ArduSub from RViz, plus live display of what it believes.

  /goal_pose (RViz "2D Goal Pose")  ->  GUIDED SET_POSITION_TARGET_LOCAL_NED
  ArduSub LOCAL_POSITION_NED+ATTITUDE  ->  /vision/ekf_pose, /vision/ekf_path
  /vision/gt_odom (Gazebo truth)       ->  /vision/gt_path  (for comparison)

Everything is published in the `odom` frame, which is Gazebo's world frame
(ENU, x = east, y = north). The EKF origin is the vehicle's spawn point at the
world origin and its heading is true north (see docs/VISION_NAV.md), so
ArduSub's local NED maps onto it directly: east = ned.y, north = ned.x,
up = -ned.z.

A 2D goal has no height, so it is flown at --depth (metres below the
surface); a PoseStamped with z < 0 (e.g. from `ros2 topic pub`) uses -z.
The first goal switches ArduSub to GUIDED and arms it.
"""
import argparse
import math
import os
import threading
import time

os.environ.setdefault('MAVLINK20', '1')

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.parameter import Parameter

from vision_bridge import ENU_TO_NED, FLU_TO_FRD, quat_to_rot, rot_to_quat_wxyz

M = mavutil.mavlink
MASK_POS_YAW = 0b100111111000  # position + yaw, ignore velocity/accel/yaw rate
GUIDED = 4                       # ArduSub custom mode
PATH_STEP = 0.05                 # m between recorded path points


def euler_to_rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ])


class GoalBridge(Node):
    def __init__(self, args):
        super().__init__('goal_bridge', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.args = args
        self.goal = None  # (n, e, d, yaw_ned)
        self.goal_reached = False
        self.attitude = None
        self.armed = False
        self.mode = None
        self.ekf_path = self._new_path()
        self.gt_path = self._new_path()

        self.pub_pose = self.create_publisher(PoseStamped, '/vision/ekf_pose', 10)
        self.pub_ekf_path = self.create_publisher(Path, '/vision/ekf_path', 10)
        self.pub_gt_path = self.create_publisher(Path, '/vision/gt_path', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/vision/goal', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.create_subscription(Odometry, '/vision/gt_odom', self._on_gt, 10)

        self.mav = mavutil.mavlink_connection(args.mavlink, source_system=253)
        self.get_logger().info(f'Waiting for ArduSub heartbeat on {args.mavlink} ...')
        while True:
            hb = self.mav.recv_match(type='HEARTBEAT', blocking=True)
            if hb.autopilot != M.MAV_AUTOPILOT_INVALID:
                self.mav.target_system, self.mav.target_component = hb.get_srcSystem(), hb.get_srcComponent()
                break
        for msg_id in (M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, M.MAVLINK_MSG_ID_ATTITUDE):
            self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                           M.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, 1e5, 0, 0, 0, 0, 0)
        self.get_logger().info('Connected. Click "2D Goal Pose" in RViz to send the vehicle somewhere.')

        threading.Thread(target=self._mav_rx, daemon=True).start()
        self.create_timer(1.0, self._resend_goal)

    def _new_path(self):
        p = Path()
        p.header.frame_id = 'odom'
        return p

    @staticmethod
    def _append(path, pose):
        if path.poses:
            a, b = path.poses[-1].pose.position, pose.pose.position
            if math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) < PATH_STEP:
                return False
        path.poses.append(pose)
        path.header.stamp = pose.header.stamp
        return True

    # ---- ArduSub -> ROS -------------------------------------------------
    def _mav_rx(self):
        while rclpy.ok():
            msg = self.mav.recv_match(type=['HEARTBEAT', 'ATTITUDE', 'LOCAL_POSITION_NED', 'STATUSTEXT'],
                                      blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'HEARTBEAT':
                if msg.autopilot != M.MAV_AUTOPILOT_INVALID:
                    self.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.mode = msg.custom_mode
            elif t == 'ATTITUDE':
                self.attitude = (msg.roll, msg.pitch, msg.yaw)
            elif t == 'STATUSTEXT':
                self.get_logger().info(f'ArduSub: {msg.text}')
            elif self.attitude is not None:
                self._publish_ekf(msg)

    def _publish_ekf(self, pos):
        r_enu_flu = ENU_TO_NED.T @ euler_to_rot(*self.attitude) @ FLU_TO_FRD
        w, x, y, z = rot_to_quat_wxyz(r_enu_flu)
        ps = PoseStamped()
        ps.header.frame_id = 'odom'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos.y, pos.x, -pos.z
        ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = w, x, y, z
        self.pub_pose.publish(ps)
        if self._append(self.ekf_path, ps):
            self.pub_ekf_path.publish(self.ekf_path)

        if self.goal and not self.goal_reached:
            n, e, d, _ = self.goal
            dist = math.dist((pos.x, pos.y, pos.z), (n, e, d))
            if dist < self.args.radius:
                self.goal_reached = True
                self.get_logger().info(f'Goal reached (EKF says {dist:.2f} m away)')
            else:
                self.get_logger().info(f'{dist:.2f} m to goal', throttle_duration_sec=2.0)

    def _on_gt(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = 'odom'
        ps.pose = msg.pose.pose
        if self._append(self.gt_path, ps):
            self.pub_gt_path.publish(self.gt_path)

    # ---- ROS -> ArduSub -------------------------------------------------
    def _on_goal(self, msg):
        if msg.header.frame_id not in ('', 'odom'):
            self.get_logger().warn(f'Goal in frame "{msg.header.frame_id}"; set RViz Fixed Frame to "odom"')
            return
        p, q = msg.pose.position, msg.pose.orientation
        depth = -p.z if p.z < -0.05 else self.args.depth
        r_enu_flu = quat_to_rot(q.x, q.y, q.z, q.w)
        yaw_enu = math.atan2(r_enu_flu[1, 0], r_enu_flu[0, 0])
        yaw_ned = math.atan2(math.cos(yaw_enu), math.sin(yaw_enu))  # pi/2 - yaw_enu, wrapped
        self.goal = (p.y, p.x, depth, yaw_ned)
        self.goal_reached = False
        self.get_logger().info(
            f'Goal: north {p.y:+.2f} m, east {p.x:+.2f} m, depth {depth:.2f} m, '
            f'heading {round(math.degrees(yaw_ned)) % 360} deg')

        echo = PoseStamped()
        echo.header.frame_id = 'odom'
        echo.header.stamp = self.get_clock().now().to_msg()
        echo.pose = msg.pose
        echo.pose.position.z = -depth
        self.pub_goal.publish(echo)

        if self.mode != GUIDED:
            self.mav.set_mode_apm(GUIDED)
        if not self.armed:
            self.mav.arducopter_arm()
        self._send_target()

    def _send_target(self):
        n, e, d, yaw = self.goal
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component, M.MAV_FRAME_LOCAL_NED,
            MASK_POS_YAW, n, e, d, 0, 0, 0, 0, 0, 0, yaw, 0)

    def _resend_goal(self):
        # UDP can drop the first target or arm/mode commands; keep them flowing
        if self.goal is None:
            return
        if self.mode != GUIDED:
            self.mav.set_mode_apm(GUIDED)
        elif not self.armed:
            self.mav.arducopter_arm()
        self._send_target()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mavlink', default='udpin:0.0.0.0:14557')
    ap.add_argument('--depth', type=float, default=1.0, help='depth for 2D goals (m)')
    ap.add_argument('--radius', type=float, default=0.3, help='goal reached radius (m)')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=['goal_bridge', *ros_args])
    node = GoalBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
