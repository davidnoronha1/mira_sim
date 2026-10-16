"""Record actual Gazebo RGB, detections and pose telemetry during the Mira BT.

Run inside the Mira container after fresh bringup with RUN_ACTIONS=0. Writes an MP4
and the actual BT process log. The top-down panel is telemetry visualization;
the camera panel is the unmodified Gazebo RGB stream with detection overlays.
"""
import math
import json
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def main():
    output = Path(sys.argv[1] if len(sys.argv) > 1 else '/tmp/mira_navigation.mp4')
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('record_mira_navigation')
    bridge = CvBridge()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    data, paths = {}, {'ekf': [], 'truth': []}
    truth_samples = []
    # Full poses (t, x, y, z, qx, qy, qz, qw) in odom for 3D replay.
    poses = {'ekf': [], 'truth': []}

    def image(msg):
        data['image'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()

    def pose(key, msg):
        p = msg.pose if key == 'ekf' else msg.pose.pose
        data[key] = p
        paths[key].append((p.position.x, p.position.y))
        q = p.orientation
        poses[key].append([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                           p.position.x, p.position.y, p.position.z, q.x, q.y, q.z, q.w])
        if key == 'truth':
            truth_samples.append([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                                  p.position.x, p.position.y, p.position.z])
            data['flare_min'] = min(data.get('flare_min', float('inf')),
                                    math.hypot(p.position.x-7, p.position.y+3))
            if len(truth_samples) > 1:
                a, b = truth_samples[-2:]
                if a[2] < 0 <= b[2]:
                    f = -a[2]/(b[2]-a[2])
                    x, z = a[1]+f*(b[1]-a[1]), a[3]+f*(b[3]-a[3])
                    data['gate_passed'] = 6.56 < x < 7.44 and -2.05 < z < -.85

    node.create_subscription(Image, '/mira/sensors/camera/front/image_raw', image, qos_profile_sensor_data)
    node.create_subscription(Detection2DArray, '/mira/perception/front/objects',
                             lambda m: data.update(boxes=m), qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/mira/state/pose', lambda m: pose('ekf', m), 10)
    node.create_subscription(Odometry, '/mira/sim/ground_truth/odom', lambda m: pose('truth', m), 10)
    node.create_subscription(Odometry, '/mira/odometry/visual', lambda m: data.update(vo=m), 10)
    def goal(msg):
        data['goal'] = msg.pose
        data.setdefault('goals', []).append([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                                             msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    node.create_subscription(PoseStamped, '/mira/guidance/goal', goal, 10)
    def scan(msg):
        data['scan'] = msg
        data['scan_samples'] = data.get('scan_samples', 0)+1
    node.create_subscription(LaserScan, '/mira/perception/front/obstacles', scan, qos_profile_sensor_data)
    # Optional third-person view from the vehicle model's chase camera.
    node.create_subscription(Image, '/mira/sim/camera/chase/image_raw',
                             lambda m: data.update(chase=m), qos_profile_sensor_data)
    deadline = time.monotonic() + 45
    while not all(k in data for k in ('image', 'boxes', 'ekf', 'truth', 'vo')):
        rclpy.spin_once(node, timeout_sec=.05)
        if time.monotonic() > deadline:
            raise RuntimeError('Missing simulator image/detections/pose/VO')
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'mp4v'), 15, (1280, 800))
    if not writer.isOpened():
        raise RuntimeError('Cannot open video writer')
    chase_deadline = time.monotonic() + 10
    while 'chase' not in data and time.monotonic() < chase_deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    chase_path = output.with_name(output.stem + '_3d.mp4')
    chase_writer = (cv2.VideoWriter(str(chase_path), cv2.VideoWriter_fourcc(*'mp4v'), 15, (1280, 720))
                    if 'chase' in data else None)
    if chase_writer is None:
        print('No chase camera images; writing only the telemetry video', flush=True)
    log_path = output.with_suffix('.log')
    log = log_path.open('w')
    process, finished = None, None
    start = next_frame = time.monotonic()
    colors = {'ekf': (255, 180, 110), 'truth': (160, 225, 100)}

    def text(canvas, value, x, y, scale=.6, color=(225, 232, 240)):
        cv2.putText(canvas, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    def point(x, y):
        return round(870 + (x-7)*34), round(572 - (y+10)*34)

    try:
        while time.monotonic()-start < 100:
            rclpy.spin_once(node, timeout_sec=.005)
            now = time.monotonic()
            elapsed = now-start
            if (process is None and elapsed >= 3 and
                    tf_buffer.can_transform('odom', data['boxes'].header.frame_id,
                                            Time.from_msg(data['boxes'].header.stamp))):
                executable = Path(get_package_prefix('mira2_actions'))/'lib/mira2_actions/mira2_actions_exe'
                process = subprocess.Popen([str(executable), '--ros-args', '-p', 'use_sim_time:=true',
                                            '-p', 'target_class:=gate', '-p', 'groot_port:=17980',
                                            '-p', 'tree_xml:='+str(Path(get_package_share_directory('mira2_actions'))/'config/gate.xml')],
                                           stdout=log, stderr=subprocess.STDOUT)
            if process is not None and process.poll() is not None and finished is None:
                finished = now
            if finished is not None and now-finished > 4:
                break
            if now < next_frame:
                continue
            canvas = np.full((800, 1280, 3), (24, 19, 14), np.uint8)
            text(canvas, 'MIRA  /  AUTONOMOUS NAVIGATION', 28, 40, .9)
            text(canvas, 'Gate traversal  /  cuVSLAM GPU  /  ArduSub GUIDED + BendyRuler', 28, 72)
            text(canvas, f'{elapsed:05.1f}s  |  actual recorded pass', 922, 40, .55)
            view = cv2.resize(data['image'], (640, 480))
            for detection in data['boxes'].detections:
                box = detection.bbox
                cx, cy = box.center.position.x*640, box.center.position.y*480
                w, h = box.size_x*640, box.size_y*480
                label = detection.results[0].hypothesis.class_id if detection.results else '?'
                color = (40, 210, 255) if label in ('flare', 'gate') else (145, 160, 145)
                cv2.rectangle(view, (round(cx-w/2), round(cy-h/2)), (round(cx+w/2), round(cy+h/2)), color, 1)
                if label in ('flare', 'gate'):
                    text(view, label, max(0, round(cx-w/2)), max(20, round(cy-h/2)-8), .6, color)
            canvas[120:600, 24:664] = view
            text(canvas, 'ONBOARD CAMERA / Gazebo boxes substitute for YOLO', 24, 104, .55)
            text(canvas, 'TOP VIEW / measured pose and trajectory', 710, 104, .55)
            for x in range(3, 18):
                cv2.line(canvas, point(x, -11), point(x, 3), (50, 45, 40), 1)
            for y in range(-11, 4):
                cv2.line(canvas, point(3, y), point(17, y), (50, 45, 40), 1)
            for label, xy in [('spawn', (7, -10)), ('flare', (7, -3)), ('gate', (7, 0))]:
                p = point(*xy)
                cv2.circle(canvas, p, 5, (40, 210, 255), -1)
                text(canvas, label, p[0]+10, p[1]-8, .5)
            cv2.line(canvas, point(6.25, 0), point(7.75, 0), (40, 210, 255), 3)
            cv2.circle(canvas, point(7, -3), round(.575*34), (50, 80, 130), 1)
            for key, color in colors.items():
                points = np.array([point(x, y) for x, y in paths[key]], np.int32)
                if len(points) > 1:
                    cv2.polylines(canvas, [points], False, color, 2)
                p = data[key]
                q = p.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                x, y = p.position.x, p.position.y
                cv2.arrowedLine(canvas, point(x, y), point(x+.65*math.cos(yaw), y+.65*math.sin(yaw)), color, 3, tipLength=.35)
            if 'scan' in data:
                pose_now = data['ekf']
                q = pose_now.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                scan_now = data['scan']
                for index, distance in enumerate(scan_now.ranges):
                    if not math.isfinite(distance):
                        continue
                    angle = yaw+scan_now.angle_min+index*scan_now.angle_increment
                    pixel = point(pose_now.position.x+distance*math.cos(angle),
                                  pose_now.position.y+distance*math.sin(angle))
                    if 710 < pixel[0] < 1250 and 120 < pixel[1] < 600:
                        cv2.circle(canvas, pixel, 2, (50, 160, 255), -1)
            text(canvas, 'Blue: EKF (cuVSLAM input)   Green: Gazebo truth', 710, 622, .5)
            text(canvas, f"Min flare distance: {data.get('flare_min', 0):.2f} m  |  Gate: " +
                 ('crossed' if data.get('gate_passed') else 'pending'), 710, 649, .5)
            if 'goal' in data:
                g = data['goal'].position
                cv2.drawMarker(canvas, point(g.x, g.y), (255, 240, 80), cv2.MARKER_CROSS, 18, 2)
                p = data['ekf'].position
                distance = math.dist((p.x, p.y, p.z), (g.x, g.y, g.z))
                text(canvas, f'Goal distance  {distance:.2f} m    |    Depth  {-p.z:.2f} m', 24, 640, .65)
            logs = log_path.read_text()
            tree_success = process is not None and process.poll() == 0
            success = tree_success and data.get('gate_passed') and data.get('flare_min', 0) > .575
            status = 'SUCCESS / gate crossed, flare avoided' if success else (
                'VALIDATION FAILED / route missed gate or clearance' if tree_success else
                'FAILED / see navigation log' if finished else ('THROUGH GATE / AVOID FLARE' if 'goal' in data else 'ACQUIRING GATE'))
            text(canvas, status, 24, 690, .85, (160, 225, 100) if success else (40, 210, 255))
            events = [line.split('[mira2_actions]: ', 1)[-1] for line in logs.splitlines()
                      if any(word in line for word in ('Selected localized', 'Goal sent:', 'Gate approach aligned', 'Arrival position', 'Navigation failed'))]
            for i, line in enumerate(events[-2:]):
                text(canvas, line[:105], 24, 727+i*25, .55)
            text(canvas, 'Orange points: depth proximity sent to ArduPilot. Boxes substitute for YOLO. No RTAB-Map node.', 24, 785, .5, (155, 165, 175))
            scene = None
            if chase_writer is not None:
                scene = cv2.resize(bridge.imgmsg_to_cv2(data['chase'], desired_encoding='bgr8'), (1280, 720))
                shade = scene.copy()
                cv2.rectangle(shade, (0, 0), (1280, 92), (24, 19, 14), -1)
                cv2.rectangle(shade, (0, 650), (1280, 720), (24, 19, 14), -1)
                scene = cv2.addWeighted(shade, .62, scene, .38, 0)
                text(scene, 'MIRA  /  THIRD-PERSON VIEW', 28, 40, .9)
                text(scene, 'Gazebo chase camera on the vehicle  /  cuVSLAM GPU  /  ArduSub GUIDED + BendyRuler', 28, 72, .55)
                text(scene, f'{elapsed:05.1f}s', 1170, 40, .6)
                inset = cv2.resize(view, (384, 288))
                cv2.rectangle(inset, (0, 0), (383, 287), (40, 210, 255), 2)
                scene[346:634, 880:1264] = inset
                text(scene, 'ONBOARD CAMERA', 884, 338, .5)
                p = data['truth'].position
                text(scene, status, 28, 692, .75, (160, 225, 100) if success else (40, 210, 255))
                text(scene, f'Depth {-p.z:.2f} m   |   Flare clearance {data.get("flare_min", 0):.2f} m', 760, 692, .55)
            while next_frame <= now:
                writer.write(canvas)
                if scene is not None:
                    chase_writer.write(scene)
                next_frame += 1/15
        if process is None or process.poll() != 0:
            raise RuntimeError('Navigation did not complete successfully; inspect '+str(log_path))
        crossings = []
        for a, b in zip(truth_samples, truth_samples[1:]):
            if a[2] < 0 <= b[2]:
                fraction = -a[2]/(b[2]-a[2])
                crossings.append([a[i]+fraction*(b[i]-a[i]) for i in range(4)])
        clearance = min(math.hypot(p[1]-7, p[2]+3) for p in truth_samples)
        passed = any(6.56 < p[1] < 7.44 and -2.05 < p[3] < -.85 for p in crossings)
        evidence = {'tree_success': True, 'gate_crossings_txyz': crossings,
                    'gate_aperture_passed_with_hull_margin': passed,
                    'min_flare_center_distance_m': clearance,
                    'flare_cleared_conservative_hull_radius': clearance > .575,
                    'depth_proximity_samples': data.get('scan_samples', 0),
                    'truth_txyz': truth_samples,
                    'truth_pose_txyzq': poses['truth'], 'ekf_pose_txyzq': poses['ekf'],
                    'goals_txyz': data.get('goals', []),
                    'events': [line for line in log_path.read_text().splitlines() if '[mira2_actions]' in line]}
        output.with_suffix('.json').write_text(json.dumps(evidence, indent=2))
        if not passed or clearance <= .575:
            raise RuntimeError('Tree arrived, but gate/flare ground-truth validation failed: '+str(evidence)[:400])
        print(f'Saved {output}; gate crossing and flare clearance verified ({clearance:.3f} m)', flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        writer.release()
        if chase_writer is not None:
            chase_writer.release()
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
