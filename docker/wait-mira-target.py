"""Wait for a localized visible object and fresh EKF feedback before the demo."""
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
    samples = {'object': None, 'pose': None}
    def detections(msg):
        samples['object'] = localized(msg, label)
    def pose(msg):
        samples['pose'] = msg.header if msg.header.frame_id == 'odom' else None
    node.create_subscription(Detection2DArray, '/mira/perception/front/objects', detections, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/vision/ekf_pose', pose, qos_profile_sensor_data)
    def fresh(header):
        if header is None:
            return False
        age = node.get_clock().now().nanoseconds * 1e-9 - header.stamp.sec - header.stamp.nanosec * 1e-9
        return bool(header.frame_id) and 0 <= age <= .5
    deadline = time.monotonic() + timeout
    node.get_logger().info(f'Waiting for localized YOLO class {label!r} and EKF pose (no movement yet)')
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if all(fresh(header) for header in samples.values()):
                return 0
        node.get_logger().error('No localized target/EKF feedback. Check model labels, visibility, RGB-D and VO; no demo goal sent.')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
