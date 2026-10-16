#!/usr/bin/env python3
"""External Gazebo spawn-scan/gate recorder and cuVSLAM SLAM reconstruction.

Run inside the Mira container with ROS and /workspace/install sourced. The action
package owns all vehicle behavior; this tool only launches its gate routine
when --run-gate is supplied. Benchmark replay never publishes vehicle goals,
TF, or navigation odometry. Native cuVSLAM SLAM is replayed after the pass.
"""
import argparse
import json
import math
import os
from pathlib import Path
import pickle
import signal
import subprocess
import time
import threading
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message, deserialize_message
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import CameraInfo, Image, Imu
from vision_msgs.msg import Detection2DArray
from tf2_ros import Buffer, TransformListener
from mira2_vision.cuvslam_odometry import CuVslamOdometry


# 1920x1080 composite: third-person view and SLAM map on top; onboard RGB,
# depth, bottom camera and the route below. (rows, columns) per panel.
CHASE = (slice(90, 619), slice(20, 960))
MAP = (slice(90, 560), slice(980, 1900))
BOTTOM_W, BOTTOM_H = 455, 341
BOTTOM_TOP = 680
BOTTOM = [(slice(BOTTOM_TOP, BOTTOM_TOP+BOTTOM_H), slice(20+i*475, 20+i*475+BOTTOM_W)) for i in range(4)]
TITLES = [('THIRD PERSON / Gazebo chase camera on the vehicle', (20, 78)),
          ('cuVSLAM SLAM MAP / registered depth at optimized keyframe poses', (980, 78)),
          ('ONBOARD RGB / detections', (20, BOTTOM_TOP-12)),
          ('REGISTERED DEPTH / 0-8 m', (495, BOTTOM_TOP-12)),
          ('BOTTOM CAMERA / floor tracks', (970, BOTTOM_TOP-12)),
          ('ROUTE / top view, metres', (1445, BOTTOM_TOP-12))]


def stamp(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def pose_row(message):
    p = message.pose.pose
    return [stamp(message)*1e-9, p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]


def label(canvas, value, xy, scale=.7, color=(230, 235, 240)):
    cv2.putText(canvas, value, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def cloud_view(points, bounds=None):
    canvas = np.full((470, 920, 3), (25, 22, 18), np.uint8)
    if points is None or len(points) == 0:
        label(canvas, 'Waiting for mapped keyframes', (220, 235))
        return canvas
    xyz, rgb = points[:, :3], points[:, 3:6].astype(np.uint8)
    # Fixed oblique view of the estimated SLAM frame (never use ground truth).
    view = Rotation.from_euler('xyz', [62, 0, -30], degrees=True).apply(xyz)
    if bounds is None:
        bounds = np.percentile(view[:, :2], [0.2, 99.8], axis=0)
    low, high = np.asarray(bounds)
    center = (low+high)/2
    scale = min(850/max(1, high[0]-low[0]), 360/max(1, high[1]-low[1]))
    uv = np.column_stack((460 + scale*(view[:, 0]-center[0]),
                          225 - scale*(view[:, 1]-center[1]))).astype(int)
    rgb = np.uint8(np.clip(rgb.astype(float)*1.8, 0, 255))
    valid = (uv[:, 0] >= 0) & (uv[:, 0] < 920) & (uv[:, 1] >= 0) & (uv[:, 1] < 470)
    order = np.argsort(view[valid, 2])
    u = uv[valid][order]
    canvas[u[:, 1], u[:, 0]] = rgb[valid][order, ::-1]
    origin = Rotation.from_euler('xyz', [62, 0, -30], degrees=True).apply([0., 0., 0.])
    start_pixel = (round(460+scale*(origin[0]-center[0])), round(225-scale*(origin[1]-center[1])))
    cv2.circle(canvas, start_pixel, 5, (100, 240, 140), -1)
    label(canvas, 'start', (start_pixel[0]+8, start_pixel[1]-8), .5, (100, 240, 140))
    label(canvas, f'{len(points):,} colored map points', (20, 440), .6)
    return canvas


PANEL_W = 620  # three columns of 620 px with 20 px gutters on a 1920 px canvas


def flow_panel(previous, current, flow, truth_velocity):
    """Bottom camera with the frame-to-frame floor tracks and the flow velocity."""
    view = cv2.cvtColor(current, cv2.COLOR_GRAY2BGR)
    if previous is not None and previous.shape == current.shape:
        p0 = cv2.goodFeaturesToTrack(previous, 120, 0.01, 8)
        if p0 is not None:
            p1, status, _ = cv2.calcOpticalFlowPyrLK(previous, current, p0, None)
            for a, b, ok in zip(p0.reshape(-1, 2), p1.reshape(-1, 2), status.ravel()):
                if ok:
                    tip = a + (b - a) * 4  # exaggerated so slow motion is visible
                    cv2.arrowedLine(view, tuple(map(int, a)), tuple(map(int, tip)), (80, 230, 120), 1,
                                    cv2.LINE_AA, tipLength=0.3)
    view = cv2.resize(view, (PANEL_W, 450))
    cv2.rectangle(view, (0, 380), (PANEL_W, 450), (20, 18, 14), -1)
    if flow is None:
        label(view, 'optical flow off (slam_optical_flow:=false)', (12, 420), .6)
    else:
        label(view, f'flow   fwd {flow[1]:+.2f}  left {flow[2]:+.2f} m/s', (12, 405), .6, (120, 235, 150))
        if truth_velocity is not None:
            label(view, f'truth  fwd {truth_velocity[0]:+.2f}  left {truth_velocity[1]:+.2f} m/s (reference)',
                  (12, 435), .55, (170, 175, 185))
    return view


def speed_plot(flow, truth, now, window=20.0):
    """Forward/left body speed from optical flow against ground truth, last window s."""
    canvas = np.full((470, PANEL_W, 3), (25, 22, 18), np.uint8)
    left, right, top, bottom = 50, PANEL_W-15, 30, 420
    cv2.rectangle(canvas, (left, top), (right, bottom), (70, 70, 70), 1)
    to_px = lambda t, v: (int(left + (t-(now-window))/window*(right-left)),
                          int((top+bottom)/2 - v/1.2*(bottom-top)/2))
    for v in (-1.0, -0.5, 0.0, 0.5, 1.0):
        y = to_px(now, v)[1]
        cv2.line(canvas, (left, y), (right, y), (55, 55, 55) if v else (95, 95, 95), 1)
        label(canvas, f'{v:+.1f}', (5, y+5), .4, (150, 150, 150))
    series = [(truth, 1, (150, 150, 150), 'truth fwd'), (truth, 2, (110, 110, 110), 'truth left'),
              (flow, 1, (110, 235, 140), 'flow fwd'), (flow, 2, (230, 190, 90), 'flow left')]
    for i, (rows, k, color, name) in enumerate(series):
        points = [to_px(r[0], r[k]) for r in rows if r[0] >= now-window]
        if len(points) > 1:
            cv2.polylines(canvas, [np.array(points, np.int32)], False, color, 2 if 'flow' in name else 1, cv2.LINE_AA)
        label(canvas, name, (left+10+i*135, 450), .5, color)
    if not flow:
        label(canvas, 'optical flow off', (PANEL_W//2-80, 240), .7)
    return canvas


def route_view(truth, ekf, goal):
    """Top view of the pass: Gazebo truth (verification) against the EKF the vehicle flew on."""
    canvas = np.full((BOTTOM_H, BOTTOM_W, 3), (25, 22, 18), np.uint8)
    scale = 26  # px per metre; y (along the course) runs left to right
    px = lambda x, y: (int(20 + (y+12)*scale), int(BOTTOM_H/2 + (x-7)*scale))
    for y in range(-12, 4, 2):
        cv2.line(canvas, px(-20, y), px(20, y), (45, 42, 38), 1)
        label(canvas, f'{y}', (px(0, y)[0]-6, BOTTOM_H-8), .4, (130, 130, 130))
    for x in range(1, 14, 2):
        cv2.line(canvas, px(x, -20), px(x, 20), (45, 42, 38), 1)
    ring = [px(6.3, -10.7), px(7.7, -9.3)]
    cv2.rectangle(canvas, ring[0], ring[1], (210, 210, 210), 1)
    cv2.circle(canvas, px(7, -3), 4, (0, 120, 255), -1)
    label(canvas, 'flare', (px(7, -3)[0]-16, px(7, -3)[1]-10), .45, (0, 160, 255))
    cv2.line(canvas, px(6.25, 0), px(7.75, 0), (200, 200, 200), 2)
    cv2.circle(canvas, px(6.25, 0), 4, (60, 60, 230), -1)
    cv2.circle(canvas, px(7.75, 0), 4, (60, 200, 60), -1)
    label(canvas, 'gate', (px(7, 0)[0]+8, px(7, 0)[1]-30), .45)
    for rows, color in ((truth, (120, 225, 140)), (ekf, (255, 180, 110))):
        points = [px(r[1], r[2]) for r in rows[::5]]
        if len(points) > 1:
            cv2.polylines(canvas, [np.array(points, np.int32)], False, color, 2, cv2.LINE_AA)
    if goal is not None:
        cv2.drawMarker(canvas, px(goal[0], goal[1]), (255, 240, 80), cv2.MARKER_CROSS, 14, 2)
    if truth:
        yaw = Rotation.from_quat(truth[-1][4:8]).as_euler('xyz')[2]
        x, y = truth[-1][1:3]
        cv2.arrowedLine(canvas, px(x, y), px(x+.8*np.cos(yaw), y+.8*np.sin(yaw)),
                        (120, 225, 140), 2, tipLength=.4)
    label(canvas, 'green truth  blue EKF  + goal', (10, 20), .45, (200, 205, 210))
    return canvas


def metrics(estimates, truth):
    """First-pose SE(3) alignment, metric scale fixed; interpolate truth in time."""
    estimates, truth = np.asarray(estimates, float), np.asarray(truth, float)
    if len(estimates) < 2 or len(truth) < 2:
        return {'error': 'insufficient valid poses'}
    # Remove duplicate timestamps before Slerp.
    truth = truth[np.unique(truth[:, 0], return_index=True)[1]]
    estimates = estimates[(estimates[:, 0] >= truth[0, 0]) & (estimates[:, 0] <= truth[-1, 0])]
    if len(estimates) < 2:
        return {'error': 'no overlapping ground truth'}
    t = estimates[:, 0]
    gt_p = np.column_stack([np.interp(t, truth[:, 0], truth[:, axis]) for axis in (1, 2, 3)])
    gt_r = Slerp(truth[:, 0], Rotation.from_quat(truth[:, 4:8]))(t)
    est_r = Rotation.from_quat(estimates[:, 4:8])
    align_r = gt_r[0] * est_r[0].inv()
    aligned_p = align_r.apply(estimates[:, 1:4]-estimates[0, 1:4]) + gt_p[0]
    aligned_r = align_r * est_r
    errors = np.linalg.norm(aligned_p-gt_p, axis=1)
    angles = (gt_r.inv()*aligned_r).magnitude()*180/np.pi
    # One-second relative pose error at the nearest sample, within half a frame.
    rpe, rpe_rot = [], []
    for i in range(len(t)):
        j = np.searchsorted(t, t[i]+1.0)
        if j >= len(t) or abs(t[j]-t[i]-1) > .15:
            continue
        gt_delta = gt_r[i].inv().apply(gt_p[j]-gt_p[i])
        est_delta = est_r[i].inv().apply(estimates[j, 1:4]-estimates[i, 1:4])
        rpe.append(float(np.linalg.norm(est_delta-gt_delta)))
        rpe_rot.append(float(((gt_r[i].inv()*gt_r[j]).inv() *
                              (est_r[i].inv()*est_r[j])).magnitude()*180/np.pi))
    return {'samples': len(t), 'ate_rmse_m': float(np.sqrt(np.mean(errors**2))),
            'ate_median_m': float(np.median(errors)), 'ate_max_m': float(errors.max()),
            'orientation_rmse_deg': float(np.sqrt(np.mean(angles**2))),
            'rpe_1s_translation_rmse_m': float(np.sqrt(np.mean(np.square(rpe)))) if rpe else None,
            'rpe_1s_rotation_rmse_deg': float(np.sqrt(np.mean(np.square(rpe_rot)))) if rpe_rot else None,
            'endpoint_error_m': float(errors[-1]),
            'truth_distance_m': float(np.linalg.norm(np.diff(gt_p, axis=0), axis=1).sum())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--run-gate', action='store_true', help='launch mira2_actions gate routine')
    parser.add_argument('--seconds', type=float, default=260)
    parser.add_argument('--replay-only', action='store_true', help='reuse sequence.pkl without running a vehicle action')
    for name, default in [('radius', .75), ('sweep', 360), ('step', 15), ('depth', 1.0)]:
        parser.add_argument('--scan-'+name, type=float, default=default)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    out = args.output.resolve()
    rclpy.init()
    node = rclpy.create_node('gate_demo_recorder')
    node.dist_coeffs = None
    bridge = CvBridge()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    processes, files = [], []
    data, rgb_cache, depth_cache = {}, {}, {}
    truth, frames, video_stamps, video_phases, imu = [], [], [], [], []
    truth_twist, flow_rows, bottom = [], [], {}  # optical-flow display only
    estimates = {'cuvslam': {}}
    received = {'cuvslam': set()}
    clouds = []
    writer = None

    def start(command, name):
        stream = (out / (name+'.log')).open('w')
        files.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def receive(msg, key):
        data[key] = msg
        if key not in ('rgb', 'depth'):
            return
        cache = rgb_cache if key == 'rgb' else depth_cache
        cache[stamp(msg)] = msg
        for s in sorted(set(rgb_cache) & set(depth_cache)):
            rgb, depth = rgb_cache.pop(s), depth_cache.pop(s)
            if 'info' in data and (not frames or s-frames[-1][0] >= 65_000_000):
                frames.append((s, serialize_message(rgb), serialize_message(depth),
                               serialize_message(data['info'])))
        for cache in (rgb_cache, depth_cache):
            while len(cache) > 20:
                del cache[min(cache)]

    def odometry(msg, key):
        received[key].add(stamp(msg))
        if all(math.isfinite(v) and v < 9999 for v in msg.pose.covariance[::7]):
            estimates[key][stamp(msg)] = pose_row(msg)

    node.create_subscription(Image, '/mira/sensors/camera/front/image_raw', lambda m: receive(m, 'rgb'), qos_profile_sensor_data)
    node.create_subscription(Image, '/mira/sensors/camera/front/depth/image_raw', lambda m: receive(m, 'depth'), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/mira/sensors/camera/front/camera_info', lambda m: receive(m, 'info'), qos_profile_sensor_data)
    node.create_subscription(Image, '/mira/sim/camera/chase/image_raw', lambda m: receive(m, 'third'), qos_profile_sensor_data)
    ekf_path = []
    node.create_subscription(PoseStamped, '/mira/state/pose', lambda m: ekf_path.append(
        [stamp(m)*1e-9, m.pose.position.x, m.pose.position.y, m.pose.position.z]), 10)
    node.create_subscription(PoseStamped, '/mira/guidance/goal', lambda m: data.update(
        goal=(m.pose.position.x, m.pose.position.y)), 10)
    node.create_subscription(Odometry, '/mira/odometry/visual', lambda m: receive(m, 'live_vo'), 10)
    def ground_truth(m):
        truth.append(pose_row(m))
        v = m.twist.twist.linear  # body frame
        truth_twist.append((stamp(m)*1e-9, v.x, v.y))

    def bottom_image(m):
        gray = cv2.cvtColor(bridge.imgmsg_to_cv2(m, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
        bottom['previous'], bottom['current'] = bottom.get('current'), cv2.resize(gray, (320, 240))

    node.create_subscription(Odometry, '/mira/sim/ground_truth/odom', ground_truth, 100)
    node.create_subscription(Image, '/mira/sensors/camera/bottom/image_raw', bottom_image, qos_profile_sensor_data)
    node.create_subscription(TwistWithCovarianceStamped, '/mira/odometry/flow_velocity', lambda m: flow_rows.append(
        (stamp(m)*1e-9, m.twist.twist.linear.x, m.twist.twist.linear.y)), 50)
    # Same 1 kHz IMU the live odometry fuses; replayed for a with/without-IMU comparison.
    node.create_subscription(Imu, '/mira/sensors/imu', lambda m: imu.append(
        (stamp(m), m.header.frame_id, (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z),
         (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))), 1000)
    node.create_subscription(Detection2DArray, '/mira/perception/front/objects', lambda m: receive(m, 'boxes'), 10)
    node.create_subscription(Odometry, '/mira/replay/cuvslam', lambda m: odometry(m, 'cuvslam'), 100)
    gate = None
    recorded_extrinsics = None
    try:
        if not args.replay_only:
            deadline = time.monotonic()+60
            while not all(k in data for k in ('rgb', 'depth', 'info', 'third', 'boxes', 'live_vo')) or not truth or data['live_vo'].pose.covariance[0] >= 1000:
                spin(.1)
                if time.monotonic() > deadline:
                    raise RuntimeError('Missing RGB/depth/chase camera/boxes/truth; check the simulator bridge')
            writer = cv2.VideoWriter(str(out/'capture.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 10, (1920, 1080))
            if not writer.isOpened():
                raise RuntimeError('Cannot create demo video')
            frames.clear()
            truth[:] = truth[-2:]
            imu[:] = imu[-200:]
            start_time = time.monotonic()
            next_frame, finished = start_time, None
            if args.run_gate:
                # The behavior routine owns acquisition, alignment, traversal and hold.
                gate = start(['ros2', 'launch', 'mira2_actions', 'gate.launch.py', 'use_sim_time:=true',
                              *[f'scan_{name}:={getattr(args, "scan_"+name)}' for name in ('radius', 'sweep', 'step', 'depth')]], 'gate_action')
            print('CAPTURE started; gate behavior is owned by mira2_actions', flush=True)
            while time.monotonic()-start_time < args.seconds:
                rclpy.spin_once(node, timeout_sec=.005)
                now = time.monotonic()
                if gate is not None and gate.poll() is not None and finished is None:
                    finished = now
                if finished is not None and now-finished > 3:
                    break
                if now < next_frame:
                    continue
                next_frame = now+.1
                canvas = np.full((1080, 1920, 3), (20, 18, 14), np.uint8)
                action_log = (out/'gate_action.log').read_text() if (out/'gate_action.log').exists() else ''
                phase = ('COMPLETE' if 'Gate crossing complete' in action_log else
                         'CROSSING GATE' if 'Gate corridor aligned' in action_log else
                         'APPROACHING GATE' if 'Spawn scan complete' in action_log else 'SCANNING START AREA')
                label(canvas, 'MIRA | '+phase, (24, 40), 1)
                label(canvas, f'{now-start_time:5.1f}s | Gazebo RGB-D | cuVSLAM SLAM | ArduSub GUIDED + BendyRuler',
                      (1000, 40), .65)
                for title, origin in TITLES:
                    label(canvas, title, origin, .6)
                canvas[CHASE] = cv2.resize(bridge.imgmsg_to_cv2(data['third'], desired_encoding='bgr8'), (940, 529))
                canvas[MAP] = cloud_view(None)
                label(canvas, 'Built from the recorded RGB-D by replaying cuVSLAM SLAM after the pass', (980, 595), .55,
                      (170, 175, 185))
                rgb = bridge.imgmsg_to_cv2(data['rgb'], desired_encoding='bgr8').copy()
                for box in data['boxes'].detections:
                    h, w = rgb.shape[:2]
                    b = box.bbox
                    name = box.results[0].hypothesis.class_id if box.results else ''
                    if name in ('gate', 'flare'):
                        x, y = b.center.position.x*w, b.center.position.y*h
                        cv2.rectangle(rgb, (int(x-b.size_x*w/2), int(y-b.size_y*h/2)),
                                      (int(x+b.size_x*w/2), int(y+b.size_y*h/2)), (0, 220, 255), 2)
                        label(rgb, name, (int(x-b.size_x*w/2), max(18, int(y-b.size_y*h/2)-6)), .6, (0, 220, 255))
                depth = bridge.imgmsg_to_cv2(data['depth'], desired_encoding='passthrough')
                valid = np.isfinite(depth) & (depth > 0)
                scaled = np.uint8(np.clip(np.nan_to_num(depth, nan=0, posinf=8)/8, 0, 1)*255)
                depth_rgb = cv2.applyColorMap(255-scaled, cv2.COLORMAP_TURBO)
                depth_rgb[~valid] = 0
                sim_now = stamp(data['rgb'])*1e-9
                truth_now = next((v[1:] for v in reversed(truth_twist) if v[0] <= sim_now), None)
                bottom_view = (flow_panel(bottom.get('previous'), bottom['current'],
                                          flow_rows[-1] if flow_rows else None, truth_now)
                               if 'current' in bottom else np.zeros((450, PANEL_W, 3), np.uint8))
                for region, image in zip(BOTTOM, (rgb, depth_rgb, bottom_view,
                                                  route_view(truth, ekf_path, data.get('goal')))):
                    canvas[region] = cv2.resize(image, (BOTTOM_W, BOTTOM_H))
                events = [line.split('[mira2_actions]: ', 1)[-1] for line in action_log.splitlines()
                          if '[mira2_actions]: ' in line and 'WARN' not in line]
                if events:
                    label(canvas, events[-1][:150], (20, 1055), .6, (120, 225, 140))
                writer.write(canvas)
                video_stamps.append(stamp(data['rgb']))
                video_phases.append(phase)
                cv2.imwrite(str(out/'depth.png'), depth_rgb)
                cv2.imwrite(str(out/'third_person.png'), canvas[CHASE])
            writer.release()
            writer = None
            # Freeze the captured input set before running the benchmark.
            dataset, gt = frames[:], truth[:]
            frames.clear()
            for topic_cache in (rgb_cache, depth_cache):
                topic_cache.clear()
            recorded_extrinsics = serialize_message(tf_buffer.lookup_transform(
                'base_link', data['info'].header.frame_id, rclpy.time.Time()))
            imu_samples = imu[:]
            imu_extrinsics = (serialize_message(tf_buffer.lookup_transform(
                'base_link', imu_samples[-1][1], rclpy.time.Time())) if imu_samples else None)
            with (out/'sequence.pkl').open('wb') as stream:
                pickle.dump({'frames': dataset, 'truth': gt, 'video_stamps': video_stamps, 'video_phases': video_phases,
                             'extrinsics': recorded_extrinsics, 'imu': imu_samples, 'imu_extrinsics': imu_extrinsics}, stream)
        else:
            with (out/'sequence.pkl').open('rb') as stream:
                saved = pickle.load(stream)
            dataset, gt, video_stamps = saved['frames'], saved['truth'], saved['video_stamps']
            video_phases = saved.get('video_phases', [])
            recorded_extrinsics = saved.get('extrinsics')
            imu_samples, imu_extrinsics = saved.get('imu', []), saved.get('imu_extrinsics')
        print(f'CAPTURE complete: {len(dataset)} matched RGB-D frames; replaying cuVSLAM SLAM', flush=True)
        # Remove live input subscriptions: replay consumes exactly the recorded set.
        for sub in list(node.subscriptions):
            if sub.topic_name in ('/mira/sensors/camera/front/image_raw', '/mira/sensors/camera/front/depth/image_raw', '/mira/sensors/camera/front/camera_info',
                                  '/mira/sim/ground_truth/odom', '/mira/odometry/visual', '/mira/sensors/imu', '/mira/sim/camera/chase/image_raw',
                                  '/mira/perception/front/objects', '/mira/sensors/camera/bottom/image_raw',
                                  '/mira/odometry/flow_velocity'):
                node.destroy_subscription(sub)

        def replay(topic, use_imu, slam):
            """Replay the recorded frames (and IMU, when requested) through cuVSLAM."""
            imu_frame = imu_samples[-1][1] if use_imu else ''
            cu = CuVslamOdometry(node, topic, max_depth=8, odom_frame='demo_cu_odom', enable_slam=slam,
                                 tracking_mode='multisensor', imu_topic='/mira/replay/imu' if use_imu else '',
                                 imu_frame=imu_frame)
            if recorded_extrinsics is not None:
                cu.tf_buffer.set_transform_static(deserialize_message(recorded_extrinsics, TransformStamped), 'recording')
            if use_imu:
                cu.tf_buffer.set_transform_static(deserialize_message(imu_extrinsics, TransformStamped), 'recording')
            spin(1)
            times, fed = [], 0
            for index, (s, rgb_bytes, depth_bytes, info_bytes) in enumerate(dataset):
                rgb = deserialize_message(rgb_bytes, Image)
                depth = deserialize_message(depth_bytes, Image)
                info = deserialize_message(info_bytes, CameraInfo)
                depth_array = bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough').copy()
                depth_array[~np.isfinite(depth_array) | (depth_array > 8) | (depth_array <= 0)] = 0
                frame = SimpleNamespace(header=rgb.header, image=bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8'),
                                        matrix=np.array(info.k).reshape(3, 3), depth=SimpleNamespace(image=depth_array))
                if use_imu:
                    # Deliver IMU exactly as live: everything up to this frame's time.
                    while fed < len(imu_samples) and imu_samples[fed][0] <= s:
                        with cu.imu_lock:
                            cu.imu_samples.append(imu_samples[fed])
                        fed += 1
                if index == 0 and not cu._build(frame):
                    raise RuntimeError('Camera/IMU extrinsics or IMU samples missing for replay')
                tick = time.monotonic()
                cu.process(frame)
                times.append(time.monotonic()-tick)
                spin(.01)
                if index % 100 == 0:
                    print(f'cuVSLAM {"RGB-D-inertial" if use_imu else "RGB-D"} REPLAY {index}/{len(dataset)}', flush=True)
            spin(2)
            return cu, times

        have_imu = bool(imu_samples) and imu_extrinsics is not None
        # Primary replay matches the live configuration (IMU fused when recorded)
        # and produces the map; the comparison replay drops the IMU on the same frames.
        cu, cu_times = replay('/mira/replay/cuvslam', have_imu, True)
        optimized = {int(p.timestamp_ns): p.pose for p in cu.tracker.get_all_slam_poses()}
        saved_map = []
        save_done = threading.Event()
        def saved(ok):
            saved_map.append(bool(ok))
            save_done.set()
        cu.tracker.save_map(str(out/'cuvslam_map'), saved)
        if not save_done.wait(30) or not saved_map[0]:
            raise RuntimeError('cuVSLAM native map export failed')
        # Dense visualization uses only registered depth and native cuVSLAM
        # optimized poses. It is not a second SLAM/mapping backend.
        voxels = {}
        last_snapshot = 0
        for index, (s, rgb_bytes, depth_bytes, info_bytes) in enumerate(dataset):
            if s not in optimized or index % 3:
                continue
            rgb = bridge.imgmsg_to_cv2(deserialize_message(rgb_bytes, Image), desired_encoding='rgb8')
            depth = bridge.imgmsg_to_cv2(deserialize_message(depth_bytes, Image), desired_encoding='passthrough')
            info = deserialize_message(info_bytes, CameraInfo)
            ys, xs = np.mgrid[0:depth.shape[0]:6, 0:depth.shape[1]:6]
            z = depth[::6, ::6]
            valid = np.isfinite(z) & (z > .1) & (z <= 8)
            x = (xs[valid]-info.k[2])*z[valid]/info.k[0]
            y = (ys[valid]-info.k[5])*z[valid]/info.k[4]
            optical = np.column_stack((x, y, z[valid]))
            base = cu.camera_rotation.apply(optical)+cu.camera_translation
            pose = optimized[s]
            xyz = Rotation.from_quat(pose.rotation).apply(base)+np.array(pose.translation)
            colors = rgb[::6, ::6][valid]
            keys = np.floor(xyz/.06).astype(np.int32)
            _, unique = np.unique(keys, axis=0, return_index=True)
            values = np.column_stack((xyz, colors)).astype(np.float32)
            for j in unique:
                voxels[tuple(keys[j])] = values[j]
            if not clouds or s-last_snapshot >= 1_000_000_000:
                clouds.append((s, np.array(list(voxels.values()), np.float32)))
                last_snapshot = s
        cloud = np.array(list(voxels.values()), np.float32)
        data['cloud'] = cloud
        cu.close()
        comparison = None
        if have_imu:
            estimates['rgbd_only'], received['rgbd_only'] = {}, set()
            node.create_subscription(Odometry, '/mira/replay/cuvslam_rgbd', lambda m: odometry(m, 'rgbd_only'), 100)
            comparison, _ = replay('/mira/replay/cuvslam_rgbd', False, False)
            comparison.close()
        result = {'backend': 'cuVSLAM only', 'tracking_mode': cu.tracking_mode,
                  'scan': {name: getattr(args, 'scan_'+name) for name in ('radius', 'sweep', 'step', 'depth')}, 'input_frames': len(dataset),
                  'input_rate_hz': round(1e9/np.median(np.diff([f[0] for f in dataset])), 2),
                  'depth_limit_m': 8, 'map_backend': 'native cuVSLAM SLAM',
                  'map_visualization': 'registered RGB-D fused using optimized cuVSLAM SLAM poses',
                  'native_map_saved': saved_map[0], 'optimized_slam_poses': len(optimized),
                  'map_points': len(cloud), 'cuvslam_mean_track_ms': float(np.mean(cu_times)*1000),
                  'gate_launch_returncode': gate.poll() if gate else None,
                  'gate_action_returncode': (0 if (out/'gate_action.log').exists() and
                      'Behavior tree finished: SUCCESS' in (out/'gate_action.log').read_text() else 1),
                  'video_phase_frames': {p: video_phases.count(p) for p in set(video_phases)},
                  'scope': 'One simulated spawn scan and gate pass; Gazebo truth is used only for verification'}
        poses = estimates['cuvslam']
        result['cuvslam'] = {'received_frames': len(received['cuvslam']), 'valid_frames': len(poses),
                             'coverage': len(poses)/max(1, len(dataset)),
                             'all_valid_metrics': metrics(list(poses.values()), gt)}
        result['imu_fused'] = have_imu
        result['imu_samples'] = len(imu_samples)
        if comparison is not None:
            result['cuvslam_rgbd_only_same_frames'] = {
                'valid_frames': len(estimates['rgbd_only']),
                'all_valid_metrics': metrics(list(estimates['rgbd_only'].values()), gt)}
        result['slam_metrics'] = metrics([[s*1e-9, *map(float,p.translation), *map(float,p.rotation)]
                                         for s,p in sorted(optimized.items())], gt)
        # Gate aperture and obstacle geometry are used for verification only.
        crossings = []
        for a, b in zip(gt, gt[1:]):
            if a[2] < 0 <= b[2]:
                f = -a[2]/(b[2]-a[2])
                crossings.append([a[i]+f*(b[i]-a[i]) for i in range(4)])
        scan_times = [s*1e-9 for s,p in zip(video_stamps, video_phases) if p == 'SCANNING START AREA']
        if len(scan_times) > 1:
            scan_truth = np.array([p for p in gt if scan_times[0] <= p[0] <= scan_times[-1]])
            if len(scan_truth) > 1:
                radii = np.linalg.norm(scan_truth[:, 1:3]-scan_truth[0, 1:3], axis=1)
                yaw = np.unwrap(Rotation.from_quat(scan_truth[:, 4:8]).as_euler('xyz')[:, 2])
                result['scan_verification'] = {
                    'maximum_radius_from_start_m': float(radii.max()),
                    'return_distance_from_start_m': float(radii[-1]),
                    'yaw_range_degrees': float(np.ptp(yaw)*180/np.pi),
                    'truth_samples': len(scan_truth)}
        # Verification geometry only: four physical starting-frame bars at
        # z=-0.2, each 1.4 m long and 0.04 m radius. The enclosing hull sphere
        # includes body dimensions, its centre offset, and a clearance allowance.
        ring_segments = [(np.array([6.3,-10.7,-.2]), np.array([6.3,-9.3,-.2])),
                         (np.array([7.7,-10.7,-.2]), np.array([7.7,-9.3,-.2])),
                         (np.array([6.3,-10.7,-.2]), np.array([7.7,-10.7,-.2])),
                         (np.array([6.3,-9.3,-.2]), np.array([7.7,-9.3,-.2]))]
        xyz_truth = np.asarray(gt)[:,1:4]
        clearance = float('inf')
        for a,b in ring_segments:
            ab = b-a
            f = np.clip((xyz_truth-a)@ab/np.dot(ab,ab),0,1)
            d = np.linalg.norm(xyz_truth-(a+f[:,None]*ab),axis=1)
            clearance = min(clearance, float(d.min())-.04-.45)
        # Bound motion between 50 Hz truth samples as well as at samples.
        motion_bound = float(np.linalg.norm(np.diff(xyz_truth,axis=0),axis=1).max())/2
        clearance -= motion_bound
        result['starting_ring_min_clearance_m'] = clearance
        result['starting_ring_cleared_conservative_hull'] = clearance > 0
        result['gate_crossings_txyz'] = crossings
        result['gate_aperture_passed_with_hull_margin'] = any(
            6.56 < p[1] < 7.44 and -2.05 < p[3] < -.85 for p in crossings)
        result['min_flare_center_distance_m'] = min(math.hypot(p[1]-7, p[2]+3) for p in gt)
        (out/'accuracy.json').write_text(json.dumps(result, indent=2))
        (out/'trajectories.json').write_text(json.dumps({'ground_truth': gt,
            **{key: list(poses.values()) for key, poses in estimates.items()}}))
        if len(cloud) == 0:
            raise RuntimeError('No cuVSLAM reconstruction points; inspect saved SLAM poses')
        cloud = data['cloud']
        with (out/'map.ply').open('w') as stream:
            stream.write('ply\nformat ascii 1.0\nelement vertex '+str(len(cloud))+
                         '\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\n'
                         'property uchar green\nproperty uchar blue\nend_header\n')
            for x, y, z, r, g, b in cloud:
                stream.write(f'{x:.4f} {y:.4f} {z:.4f} {int(r)} {int(g)} {int(b)}\n')
        final_view = Rotation.from_euler('xyz', [62, 0, -30], degrees=True).apply(cloud[:, :3])
        map_bounds = np.percentile(final_view[:, :2], [0.2, 99.8], axis=0)
        cv2.imwrite(str(out/'map.png'), cloud_view(cloud, map_bounds))
        with (out/'map_snapshots.pkl').open('wb') as stream:
            pickle.dump(clouds, stream)
        reader = cv2.VideoCapture(str(out/'capture.mp4'))
        writer = cv2.VideoWriter(str(out/'composite.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 10, (1920, 1080))
        snapshot = 0
        for i, s in enumerate(video_stamps):
            ok, canvas = reader.read()
            if not ok:
                break
            while snapshot+1 < len(clouds) and clouds[snapshot+1][0] <= s:
                snapshot += 1
            points = clouds[snapshot][1] if clouds and clouds[snapshot][0] <= s else None
            canvas[MAP] = cloud_view(points, map_bounds)
            writer.write(canvas)
            if i == len(video_stamps)-15:
                cv2.imwrite(str(out/'gate_demo.jpg'), canvas)
        reader.release()
        writer.release()
        writer = None
        ffmpeg = os.environ.get('FFMPEG', 'ffmpeg')
        subprocess.run([ffmpeg, '-y', '-loglevel', 'error', '-i', str(out/'composite.mp4'),
                        '-c:v', 'libx264', '-crf', '20', '-pix_fmt', 'yuv420p',
                        '-movflags', '+faststart', str(out/'gate_demo.mp4')], check=True)
        print(json.dumps(result, indent=2), flush=True)
        print(f'Saved {out}/gate_demo.mp4', flush=True)
        if result['gate_action_returncode'] != 0:
            raise RuntimeError('Gate routine failed; inspect gate_action.log')
        if not result['starting_ring_cleared_conservative_hull']:
            raise RuntimeError('Starting ring clearance validation failed; inspect accuracy.json')
        if not result['gate_aperture_passed_with_hull_margin']:
            raise RuntimeError('Vehicle did not cross the gate aperture with the configured hull margin')
    finally:
        if writer is not None:
            writer.release()
        # SIGINT lets the behavior routine request hold and the mapper flush its DB.
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        for stream in files:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
