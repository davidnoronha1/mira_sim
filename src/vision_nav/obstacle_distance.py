#!/usr/bin/env python3
"""Depth camera -> MAVLink OBSTACLE_DISTANCE, for ArduSub object avoidance.

A port of ArduPilot's d4xx_to_mavlink.py (RealSense depth-camera obstacle
avoidance), running on the same RGB-D camera that visual odometry uses:

  /vision/depth (+ camera_info)  ->  72 sectors over the camera's HFOV
     -> OBSTACLE_DISTANCE (MAV_FRAME_BODY_FRD) -> ArduSub proximity
        (PRX1_TYPE=2) -> object database -> OA path planner (OA_TYPE)
     -> /vision/obstacles (sensor_msgs/LaserScan, base_link) for RViz

The camera is pitched down (for VO), so every pixel is projected into the
AUV's level body frame using the camera's mount pose from TF. Only points
within --band metres of the AUV's centre height count, so the pool floor and
water surface aren't reported as obstacles; each sector reports the nearest
horizontal range among them. Sectors with nothing in range report
max_distance + 1 ("no obstacle").
"""
import argparse
import math
import os

os.environ.setdefault('MAVLINK20', '1')

import numpy as np
import rclpy
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformListener

from vision_bridge import quat_to_rot

M = mavutil.mavlink
SECTORS = 72
STRIDE = 2  # use every 2nd pixel in each direction; plenty for 72 sectors


class ObstacleDistance(Node):
    def __init__(self, args):
        super().__init__('obstacle_distance')
        self.args = args
        self.geom = None
        self.info = None
        self.sent = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_scan = self.create_publisher(LaserScan, '/vision/obstacles', 10)
        self.create_subscription(CameraInfo, '/vision/camera_info', self._on_info, 10)
        self.create_subscription(Image, '/vision/depth', self._on_depth, 10)

        self.mav = mavutil.mavlink_connection(args.mavlink, source_system=1,
                                              source_component=M.MAV_COMP_ID_OBSTACLE_AVOIDANCE)
        self.get_logger().info(f'Waiting for ArduSub heartbeat on {args.mavlink} ...')
        self.mav.wait_heartbeat()
        self.get_logger().info('Connected; sending OBSTACLE_DISTANCE from the depth camera')
        self.create_timer(1.0, self._heartbeat)

    def _heartbeat(self):
        self.mav.mav.heartbeat_send(M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if self.sent:
            self.get_logger().info(f'{self.sent} OBSTACLE_DISTANCE/s', throttle_duration_sec=15.0)
        self.sent = 0

    def _on_info(self, msg):
        self.info = msg

    def _setup(self, frame_id):
        """Precompute per-pixel body-frame rays and sector bins (once)."""
        try:
            tf = self.tf_buffer.lookup_transform('base_link', frame_id, Time())
        except Exception as e:  # noqa: BLE001 - tf2 raises several types
            self.get_logger().warn(f'waiting for TF base_link <- {frame_id}: {e}', throttle_duration_sec=5.0)
            return False
        q, t = tf.transform.rotation, tf.transform.translation
        rot = quat_to_rot(q.x, q.y, q.z, q.w)  # optical -> base_link (FLU)

        k = self.info
        fx, cx, fy, cy = k.k[0], k.k[2], k.k[4], k.k[5]
        u, v = np.meshgrid(np.arange(0, k.width, STRIDE), np.arange(0, k.height, STRIDE))
        rays_opt = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, dtype=float)], axis=-1)
        rays_body = rays_opt @ rot.T  # per unit of optical depth
        hfov = 2 * math.atan2(k.width / 2, fx)
        self.geom = dict(rays=rays_body, origin=np.array([t.x, t.y, t.z]), hfov=hfov)
        self.get_logger().info(
            f'depth camera {k.width}x{k.height}, HFOV {math.degrees(hfov):.1f} deg, mount pitch '
            f'{math.degrees(math.asin(-rot[2, 2])):.1f} deg down; obstacles = points within '
            f'+/-{self.args.band} m of the AUV height')
        return True

    def _on_depth(self, msg):
        if self.info is None or msg.encoding != '32FC1':
            return
        if self.geom is None and not self._setup(msg.header.frame_id):
            return
        g = self.geom
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)[::STRIDE, ::STRIDE]
        valid = np.isfinite(depth) & (depth > 0.05)
        pts = g['rays'] * np.where(valid, depth, 0)[..., None] + g['origin']   # base_link FLU
        in_band = valid & (np.abs(pts[..., 2]) <= self.args.band)
        x, y = pts[..., 0][in_band], pts[..., 1][in_band]
        rng = np.hypot(x, y)
        bearing = np.arctan2(-y, x)  # clockwise positive (MAVLink / FRD)

        half = g['hfov'] / 2
        keep = (np.abs(bearing) < half) & (rng >= self.args.min_range)
        sector_idx = ((bearing[keep] + half) / g['hfov'] * SECTORS).astype(int).clip(0, SECTORS - 1)
        sectors = np.full(SECTORS, np.inf)
        np.minimum.at(sectors, sector_idx, rng[keep])

        max_cm = int(self.args.max_range * 100)
        dist_cm = np.where(sectors <= self.args.max_range, np.round(sectors * 100), max_cm + 1).astype(int)

        inc_deg = math.degrees(g['hfov']) / SECTORS
        stamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        self.mav.mav.obstacle_distance_send(
            stamp_us, M.MAV_DISTANCE_SENSOR_LASER, [int(d) for d in dist_cm], 0,
            int(self.args.min_range * 100), max_cm,
            # ArduPilot puts sector j at angle_offset + j*increment, so offset
            # to the centre of sector 0
            inc_deg, -math.degrees(half) + inc_deg / 2, M.MAV_FRAME_BODY_FRD)
        self.sent += 1

        # Same data for RViz: LaserScan is counter-clockwise from angle_min,
        # so reverse the clockwise MAVLink sector order.
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = 'base_link'
        scan.angle_increment = math.radians(inc_deg)
        scan.angle_min = -half + scan.angle_increment / 2
        scan.angle_max = scan.angle_min + scan.angle_increment * (SECTORS - 1)
        scan.range_min = self.args.min_range
        scan.range_max = self.args.max_range
        scan.ranges = [float(r) if r <= self.args.max_range else float('inf') for r in sectors[::-1]]
        self.pub_scan.publish(scan)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mavlink', default='udpin:0.0.0.0:14558')
    # hull half-height 0.127 m + 0.12 m vertical margin: an obstacle edge
    # (e.g. a gate's top bar) that close above/below the hull counts, one
    # further away is something the AUV passes over/under
    ap.add_argument('--band', type=float, default=0.25,
                    help='half-height of the slice around the AUV that counts as obstacles (m)')
    ap.add_argument('--min-range', type=float, default=0.2)
    ap.add_argument('--max-range', type=float, default=8.0)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=['obstacle_distance', *ros_args])
    node = ObstacleDistance(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
