#!/usr/bin/env python3
"""Draw the Gazebo world and the AUV in RViz.

Parses the world SDF (includes, nested models, links, visuals) and publishes
its static geometry as a MarkerArray on /vision/scene, in the `odom` frame
(= Gazebo world, ENU). Boxes, cylinders, spheres and planes become primitive
markers; meshes are loaded by RViz from file:// paths inside the container.

The AUV model is left out of the scene and instead drawn twice on
/vision/auv: solid at ArduSub's EKF pose (/vision/ekf_pose, what it steers
by) and as a translucent green ghost at Gazebo's true pose (/vision/gt_odom).

Object poses come from the world file, so objects that drift in the sim
(non-static models) are shown where they started.
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from vision_bridge import quat_to_rot, rot_to_quat_wxyz

AUV_MODEL = 'bluerov2_heavy'
DEFAULT_RGBA = (0.7, 0.7, 0.7, 1.0)


def resource_paths():
    return [p for p in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':') if p]


def pose_matrix(text):
    vals = [float(v) for v in (text or '').split()] + [0.0] * 6
    x, y, z, roll, pitch, yaw = vals[:6]
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    t = np.eye(4)
    t[:3, :3] = [[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                 [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                 [-sp, sr * cp, cr * cp]]
    t[:3, 3] = (x, y, z)
    return t


def child_pose(elem):
    return pose_matrix(elem.findtext('pose'))


def resolve_uri(uri, model_dir):
    """model://name/sub, file://rel, absolute or relative path -> local path."""
    uri = uri.strip()
    if uri.startswith('model://'):
        rest = uri[len('model://'):]
        for root in resource_paths():
            if os.path.exists(os.path.join(root, rest)):
                return os.path.join(root, rest)
        return None
    if uri.startswith('file://'):
        uri = uri[len('file://'):]
    if uri.startswith(('http://', 'https://')):
        return None
    path = uri if os.path.isabs(uri) else os.path.join(model_dir or '', uri)
    return path if os.path.exists(path) else None


def model_sdf_path(model_dir):
    cfg = os.path.join(model_dir, 'model.config')
    if os.path.exists(cfg):
        sdf = ET.parse(cfg).getroot().findtext('sdf')
        if sdf and os.path.exists(os.path.join(model_dir, sdf.strip())):
            return os.path.join(model_dir, sdf.strip())
    path = os.path.join(model_dir, 'model.sdf')
    return path if os.path.exists(path) else None


def material_rgba(visual):
    mat = visual.find('material')
    rgba = DEFAULT_RGBA
    if mat is not None:
        for tag in ('diffuse', 'ambient'):
            text = mat.findtext(tag)
            if text:
                vals = [float(v) for v in text.split()] + [1.0]
                rgba = tuple(vals[:4])
                break
    transparency = float(visual.findtext('transparency') or 0.0)
    return rgba[:3] + (rgba[3] * (1.0 - transparency),)


class Visual:
    def __init__(self, owner, tf, geom, rgba, model_dir):
        self.owner, self.tf, self.geom, self.rgba, self.model_dir = owner, tf, geom, rgba, model_dir


def collect(elem, tf, model_dir, owner, out, depth=0):
    """Walk an SDF <world>/<model>, appending Visuals with world transforms."""
    if depth > 8:
        return
    for inc in elem.findall('include'):
        uri = inc.findtext('uri') or ''
        path = resolve_uri(uri, model_dir)
        if not path or not os.path.isdir(path):
            continue
        sdf = model_sdf_path(path)
        if not sdf:
            continue
        model = ET.parse(sdf).getroot().find('model')
        if model is None:
            continue
        name = (inc.findtext('name') or model.get('name') or '').strip()
        inc_tf = tf @ (child_pose(inc) if inc.find('pose') is not None else child_pose(model))
        collect(model, inc_tf, path, owner or name, out, depth + 1)
    for model in elem.findall('model'):
        collect(model, tf @ child_pose(model), model_dir, owner or model.get('name'), out, depth + 1)
    for link in elem.findall('link'):
        link_tf = tf @ child_pose(link)
        for vis in link.findall('visual'):
            geom = vis.find('geometry')
            if geom is not None and len(geom):
                out.append(Visual(owner, link_tf @ child_pose(vis), geom[0], material_rgba(vis), model_dir))


def to_marker(v, ns, mid, frame='odom'):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.action = ns, mid, Marker.ADD
    g = v.geom
    if g.tag == 'box':
        m.type = Marker.CUBE
        m.scale.x, m.scale.y, m.scale.z = [float(s) for s in g.findtext('size').split()]
    elif g.tag == 'cylinder':
        m.type = Marker.CYLINDER
        r, length = float(g.findtext('radius')), float(g.findtext('length'))
        m.scale.x, m.scale.y, m.scale.z = 2 * r, 2 * r, length
    elif g.tag == 'sphere':
        m.type = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = 2 * float(g.findtext('radius'))
    elif g.tag == 'plane':
        m.type = Marker.CUBE
        sx, sy = [float(s) for s in (g.findtext('size') or '1 1').split()]
        m.scale.x, m.scale.y, m.scale.z = sx, sy, 0.005
    elif g.tag == 'mesh':
        path = resolve_uri(g.findtext('uri') or '', v.model_dir)
        if not path:
            return None
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = 'file://' + path
        m.mesh_use_embedded_materials = path.lower().endswith('.dae')
        sx, sy, sz = [float(s) for s in (g.findtext('scale') or '1 1 1').split()]
        m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
    else:
        return None
    m.pose.position.x, m.pose.position.y, m.pose.position.z = v.tf[:3, 3]
    w, x, y, z = rot_to_quat_wxyz(v.tf[:3, :3])
    m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = w, x, y, z
    m.color.r, m.color.g, m.color.b, m.color.a = v.rgba
    return m


def label(name, tf, mid):
    m = Marker()
    m.header.frame_id = 'odom'
    m.ns, m.id, m.type, m.action = 'labels', mid, Marker.TEXT_VIEW_FACING, Marker.ADD
    m.text = name
    m.pose.position.x, m.pose.position.y = tf[0, 3], tf[1, 3]
    m.pose.position.z = max(tf[2, 3], -2.0) + 2.3
    m.pose.orientation.w = 1.0
    m.scale.z = 0.35
    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
    return m


class SceneMarkers(Node):
    def __init__(self, world):
        super().__init__('scene_markers')
        visuals = []
        root = ET.parse(world).getroot()
        collect(root.find('world'), np.eye(4), os.path.dirname(world), None, visuals)
        scene = [v for v in visuals if v.owner != AUV_MODEL]
        auv = [v for v in visuals if v.owner == AUV_MODEL]

        self.scene = MarkerArray()
        labelled = {}
        for i, v in enumerate(scene):
            m = to_marker(v, 'scene', i)
            if m is not None:
                self.scene.markers.append(m)
            if v.owner and v.owner not in labelled and not v.owner.startswith(('pool', 'water', 'wall')):
                labelled[v.owner] = v.tf
        for i, (name, tf) in enumerate(labelled.items()):
            self.scene.markers.append(label(name, tf, i))

        # AUV visuals relative to the model frame: undo its world spawn pose
        self.auv_visuals = []
        if auv:
            spawn = self._spawn_tf(root, auv)
            inv = np.linalg.inv(spawn)
            self.auv_visuals = [Visual(v.owner, inv @ v.tf, v.geom, v.rgba, v.model_dir) for v in auv]
        self.get_logger().info(
            f'{os.path.basename(world)}: {len(self.scene.markers)} scene markers, '
            f'{len(self.auv_visuals)} AUV visuals')

        self.pub_scene = self.create_publisher(MarkerArray, '/vision/scene', 1)
        self.pub_auv = self.create_publisher(MarkerArray, '/vision/auv', 10)
        self.create_subscription(PoseStamped, '/vision/ekf_pose', self._on_ekf, 10)
        self.create_subscription(Odometry, '/vision/gt_odom', self._on_gt, 10)
        self.create_timer(1.0, lambda: self.pub_scene.publish(self.scene))
        self.last_pub = {}

    @staticmethod
    def _spawn_tf(root, auv):
        for inc in root.find('world').iter('include'):
            if AUV_MODEL in (inc.findtext('uri') or '') or (inc.findtext('name') or '') == AUV_MODEL:
                return child_pose(inc)
        return np.eye(4)

    def _publish_auv(self, ns, pose, tint):
        now = self.get_clock().now().nanoseconds
        if now - self.last_pub.get(ns, 0) < 5e7:  # 20 Hz is plenty for display
            return
        self.last_pub[ns] = now
        q, p = pose.orientation, pose.position
        body = np.eye(4)
        body[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
        body[:3, 3] = (p.x, p.y, p.z)
        arr = MarkerArray()
        for i, v in enumerate(self.auv_visuals):
            m = to_marker(Visual(v.owner, body @ v.tf, v.geom, v.rgba, v.model_dir), ns, i)
            if m is None:
                continue
            if tint:
                m.mesh_use_embedded_materials = False
                m.color.r, m.color.g, m.color.b, m.color.a = tint
            arr.markers.append(m)
        self.pub_auv.publish(arr)

    def _on_ekf(self, msg):
        self._publish_auv('auv_ekf', msg.pose, None)

    def _on_gt(self, msg):
        self._publish_auv('auv_truth', msg.pose.pose, (0.2, 0.9, 0.2, 0.35))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world', help='world SDF file')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=['scene_markers', *ros_args])
    node = SceneMarkers(args.world)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
