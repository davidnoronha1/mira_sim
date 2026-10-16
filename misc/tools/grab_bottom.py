"""Save N bottom-camera frames (+ camera_info) to a directory as PNG/npz."""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

out, count, every = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]) if len(sys.argv) > 3 else 1
rclpy.init()
node = rclpy.create_node('grab_bottom')
bridge = CvBridge()
state = {'n': 0, 'seen': 0, 'K': None}


def on_info(m):
    state['K'] = np.array(m.k).reshape(3, 3)


def on_image(m):
    state['seen'] += 1
    if state['seen'] % every:
        return
    img = bridge.imgmsg_to_cv2(m, 'bgr8')
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    cv2.imwrite(f'{out}/f{state["n"]:04d}_{t:.3f}.png', img)
    state['n'] += 1


node.create_subscription(CameraInfo, '/vision/bottom/camera_info', on_info, 10)
node.create_subscription(Image, '/vision/bottom/image', on_image, qos_profile_sensor_data)
while rclpy.ok() and state['n'] < count:
    rclpy.spin_once(node, timeout_sec=0.5)
np.save(f'{out}/K.npy', state['K'])
print('saved', state['n'], 'K', state['K'])
