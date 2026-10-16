"""Compare cuVSLAM GPU/CPU on identical timestamp-matched simulator RGB-D frames.

Run inside the Mira container with ROS and the Mira install sourced. This diagnostic
does not publish odometry or command the vehicle. A moving scene is required to
distinguish a frozen pose from valid stationary tracking.
"""
import json
import time
from collections import OrderedDict

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from mira2_vision.cuvslam_odometry import camera_model, load_cuvslam


def main():
    rclpy.init()
    node = rclpy.create_node('probe_cuvslam')
    bridge = CvBridge()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    images, depths, frames, calibration = OrderedDict(), OrderedDict(), [], []

    def receive(msg, cache):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        cache[stamp] = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').copy()
        if stamp in images and stamp in depths:
            rgb = np.ascontiguousarray(images.pop(stamp)[:, :, :3])
            depth = np.nan_to_num(depths.pop(stamp) * 1000, nan=0, posinf=0)
            depth[depth > 8000] = 0
            frames.append((stamp, rgb, np.clip(depth, 0, 65535).astype(np.uint16)))
        while len(cache) > 20:
            cache.popitem(last=False)

    node.create_subscription(Image, '/mira/sensors/camera/front/image_raw', lambda m: receive(m, images), qos_profile_sensor_data)
    node.create_subscription(Image, '/mira/sensors/camera/front/depth/image_raw', lambda m: receive(m, depths), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/mira/sensors/camera/front/camera_info', lambda m: calibration.append(m), qos_profile_sensor_data)
    deadline = time.monotonic() + 30
    while (len(frames) < 90 or not calibration) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    if len(frames) < 30 or not calibration:
        raise RuntimeError(f'Insufficient matched RGB-D samples: {len(frames)}')
    cuvslam = load_cuvslam()
    info = calibration[-1]
    transform = tf_buffer.lookup_transform('base_link', info.header.frame_id, rclpy.time.Time())
    q, t = transform.transform.rotation, transform.transform.translation
    print(json.dumps({'version': cuvslam.get_version()[0], 'frames': len(frames),
                      'sim_duration': (frames[-1][0]-frames[0][0])*1e-9}), flush=True)
    for gpu in (False, True):
        camera = camera_model(cuvslam, np.array(info.k).reshape(3, 3), (info.width, info.height), info.d)
        camera.rig_from_camera = cuvslam.Pose([q.x, q.y, q.z, q.w], [t.x, t.y, t.z])
        settings = cuvslam.Tracker.OdometryRGBDSettings()
        settings.depth_scale_factor = 1000
        settings.depth_camera_id = 0
        config = cuvslam.Tracker.OdometryConfig(
            odometry_mode=cuvslam.Tracker.OdometryMode.RGBD,
            rgbd_settings=settings, use_gpu=gpu, async_sba=gpu)
        start = time.monotonic()
        try:
            tracker = cuvslam.Tracker(cuvslam.Rig([camera]), config)
            positions, observations = [], []
            for stamp, rgb, depth in frames:
                estimate, _ = tracker.track(stamp, images=[rgb], depths=[depth])
                observations.append(len(tracker.get_last_observations(0)))
                if estimate.world_from_rig is not None:
                    positions.append([float(v) for v in estimate.world_from_rig.pose.translation])
            print(json.dumps({'use_gpu': gpu, 'valid_poses': len(positions),
                              'first': positions[0] if positions else None,
                              'last': positions[-1] if positions else None,
                              'observations_min_max': [min(observations), max(observations)],
                              'wall_seconds': time.monotonic()-start}), flush=True)
            del tracker
        except Exception as error:
            print(json.dumps({'use_gpu': gpu, 'error': str(error)}), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
