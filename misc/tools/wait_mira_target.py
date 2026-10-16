"""Wait for a localized visible object and fresh, settled EKF feedback before the demo.

Settled: the vehicle is not commanded yet, so EKF motion means the estimate has
not converged. In optical-flow mode the unaided EKF drifts at metres per second
for the first ~15 s; starting then made the vehicle chase it 24 m off.
"""
import collections
import math
import sys
import time


def localized(message, label):
    for detection in message.detections:
        for result in detection.results:
            p = result.pose.pose.position
            if (result.hypothesis.class_id == label and p.z > 0 and
                    all(math.isfinite(v) for v in (p.x, p.y, p.z))):
                return detection.header
    return None


def main():
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from vision_msgs.msg import Detection2DArray
    label = sys.argv[1]
    timeout = float(sys.argv[2])
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('ACTION_WAIT must be positive')
    rclpy.init()
    node = rclpy.create_node('wait_mira_target', parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf_buffer = Buffer(node=node)
    tf_listener = TransformListener(tf_buffer, node)
    samples = {'object': None, 'pose': None}
    track = collections.deque()  # (monotonic s, x, y) of the EKF pose, last SETTLE_S
    SETTLE_S, SETTLE_M = 3.0, 0.1
    def detections(msg):
        samples['object'] = localized(msg, label)
    def pose(msg):
        samples['pose'] = msg.header if msg.header.frame_id == 'odom' else None
        now = time.monotonic()
        track.append((now, msg.pose.position.x, msg.pose.position.y))
        while track[0][0] < now - SETTLE_S - 0.5:
            track.popleft()
    def settled():
        if not track or track[-1][0] - track[0][0] < SETTLE_S:
            return False
        _, x, y = track[-1]
        return all(math.isfinite(v) for v in (x, y)) and max(math.hypot(a - x, b - y) for _, a, b in track) < SETTLE_M
    node.create_subscription(Detection2DArray, '/mira/perception/front/objects', detections, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/mira/state/pose', pose, qos_profile_sensor_data)
    def fresh(header):
        if header is None:
            return False
        age = node.get_clock().now().nanoseconds * 1e-9 - header.stamp.sec - header.stamp.nanosec * 1e-9
        return bool(header.frame_id) and 0 <= age <= .5
    deadline = time.monotonic() + timeout
    node.get_logger().info(f'Waiting for localized class {label!r} and EKF pose (no movement yet)')
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if (all(fresh(header) for header in samples.values()) and settled() and
                    tf_buffer.can_transform('odom', samples['object'].frame_id,
                                            Time.from_msg(samples['object'].stamp))):
                return 0
        node.get_logger().error('No localized target/settled EKF feedback. Check model labels, visibility, RGB-D and VO; no demo goal sent.')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
