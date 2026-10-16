"""Run the gate BT with lightweight telemetry; no video or SLAM replay.

Simulator truth is recorded for diagnosis only and never sent to navigation.
Run in the simulation container after a fresh bringup with RUN_ACTIONS=0.
"""
import json
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # Register PoseStamped transforms.


def main():
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('gate_telemetry')
    tf = Buffer()
    listener = TransformListener(tf, node)
    rows = {key: [] for key in ('truth', 'vo', 'ekf', 'goals', 'gate', 'obstacles', 'flow', 'truth_twist')}

    def record(key, msg):
        if isinstance(msg, Odometry) and key == 'truth':
            v = msg.twist.twist.linear  # Gazebo odometry twist is in the body frame
            rows['truth_twist'].append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, v.x, v.y, v.z])
        pose = msg.pose.pose if isinstance(msg, Odometry) else msg.pose
        p, q = pose.position, pose.orientation
        rows[key].append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                          p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def objects(msg):
        for detection in msg.detections:
            for result in detection.results:
                if result.hypothesis.class_id != 'gate':
                    continue
                p = PoseStamped(header=msg.header, pose=result.pose.pose)
                try:
                    record('gate', tf.transform(p, 'odom'))
                except Exception:
                    pass

    def obstacles(msg):
        # Pixhawk proximity input: closest return, its bearing (CCW, rad) and
        # the straight-ahead range, so OA detours can be matched to obstacles.
        r = [(v, msg.angle_min+i*msg.angle_increment) for i, v in enumerate(msg.ranges)]
        near = min(r)
        rows['obstacles'].append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                                  near[0], near[1], r[len(r)//2][0]])

    node.create_subscription(TwistWithCovarianceStamped, '/vision/flow_velocity', lambda m: rows['flow'].append(
        [m.header.stamp.sec+m.header.stamp.nanosec*1e-9, m.twist.twist.linear.x, m.twist.twist.linear.y,
         m.twist.covariance[0]]), 50)
    node.create_subscription(LaserScan, '/mira/perception/front/obstacles', obstacles, 10)
    for topic, key, cls in [('/vision/gt_odom', 'truth', Odometry),
                             ('/vision/vo_odom', 'vo', Odometry),
                             ('/vision/ekf_pose', 'ekf', PoseStamped),
                             ('/goal_pose', 'goals', PoseStamped)]:
        node.create_subscription(cls, topic, lambda m, k=key: record(k, m), 100)
    node.create_subscription(Detection2DArray, '/mira/perception/front/objects', objects, 10)
    deadline = time.monotonic()+60
    # This tool itself subscribes to goals. Wait for the real controller too;
    # its own subscription must not make the BT believe control is ready.
    while (not all(rows[k] for k in ('truth', 'ekf')) or
           node.count_subscribers('/goal_pose') < 2):
        rclpy.spin_once(node, timeout_sec=.05)
        if time.monotonic() > deadline:
            raise RuntimeError('Missing navigation telemetry')
    with (out/'gate_action.log').open('w') as log:
        action = subprocess.Popen(['ros2', 'launch', 'mira2_actions', 'gate.launch.py',
                                   'use_sim_time:=true'], stdout=log, stderr=subprocess.STDOUT)
        try:
            end = time.monotonic()+250
            while action.poll() is None and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
        finally:
            if action.poll() is None:
                action.terminate()
                action.wait(timeout=10)
            (out/'telemetry.json').write_text(json.dumps(rows))
            node.destroy_node()
            rclpy.shutdown()
    print(f'Saved telemetry to {out}', flush=True)


if __name__ == '__main__':
    main()
