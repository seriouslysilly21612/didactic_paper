#!/usr/bin/env python3
"""snap_view.py - one view of the camera for a figure of the camera application (paper, Fig. 10 (a)-(d): the
view of the camera with the detection box, the hover point and the tool). Read-only.

Saves the color frame that the detector processed (OUT.png) and, in OUT.json, what the figure is drawn from
afterwards: the detections of that same frame (same capture stamp), the newest 3-D target in the base frame
(/pick_target_base), the newest joint states of the bridge of the controller (/joint_states: the tool by
forward kinematics), the camera model (camera_info) and the transform base_link -> camera_color_optical_frame.

Run while the pipeline and the controller (with its bridge) run, the arm at rest over the object:

    python3 tools/snap_view.py OUT [--timeout 10]          e.g. view_p3  ->  view_p3.png, view_p3.json
Needs the ROS 2 environment (source /opt/ros/jazzy/setup.bash and ros2/install/setup.bash).
"""
import argparse
import collections
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from my_interfaces.msg import DetectionArray, PickTarget3D
import tf2_ros

TOPIC_IMG = '/camera/camera/color/image_raw'
TOPIC_INFO = '/camera/camera/color/camera_info'
BASE, OPTICAL = 'base_link', 'camera_color_optical_frame'


def stamp_ns(s):
    return int(s.sec) * 1000000000 + int(s.nanosec)


def to_bgr(msg):
    """sensor_msgs/Image (rgb8, bgr8, rgba8, bgra8, mono8) -> BGR image (copy)"""
    enc = msg.encoding.lower()
    ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1, '8uc1': 1}.get(enc)
    if ch is None:
        raise ValueError('image encoding %s not handled' % msg.encoding)
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * ch].reshape(msg.height, msg.width, ch) if ch > 1 else rows[:, :msg.width]
    conv = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR, 'bgra8': cv2.COLOR_BGRA2BGR,
            'mono8': cv2.COLOR_GRAY2BGR, '8uc1': cv2.COLOR_GRAY2BGR}.get(enc)
    return cv2.cvtColor(img, conv) if conv is not None else img.copy()


def fields(msg, names):
    out = {}
    for n in names:
        v = getattr(msg, n, None)
        if hasattr(v, 'sec') and hasattr(v, 'nanosec'):
            v = stamp_ns(v)
        elif v is not None and not isinstance(v, (str, bool, int, float)):
            v = [float(x) for x in v]
        out[n] = v
    return out


class Snap(Node):
    def __init__(self):
        super().__init__('snap_view')
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.images = collections.OrderedDict()     # capture stamp [ns] -> Image, the newest 30
        self.pair = None                            # (Image, DetectionArray) of one capture
        self.info = self.target = self.joints = None
        self.create_subscription(Image, TOPIC_IMG, self.on_img, be)
        self.create_subscription(CameraInfo, TOPIC_INFO, self.on_info, be)
        self.create_subscription(DetectionArray, '/detections', self.on_det, 10)
        self.create_subscription(PickTarget3D, '/pick_target_base', self.on_target, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)

    def on_img(self, m):
        self.images[stamp_ns(m.header.stamp)] = m
        while len(self.images) > 30:
            self.images.popitem(last=False)

    def on_det(self, m):
        k = stamp_ns(m.header.stamp)
        if self.pair is None and len(m.detections) > 0 and k in self.images:
            self.pair = (self.images[k], m)

    def on_info(self, m):
        self.info = m

    def on_target(self, m):
        self.target = m

    def on_joints(self, m):
        self.joints = m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', help='prefix of OUT.png and OUT.json')
    ap.add_argument('--timeout', type=float, default=10.0, help='[s] to wait for a frame with a detection')
    a = ap.parse_args()
    rclpy.init()
    node = Snap()
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.pair is not None and node.info is not None and node.target is not None \
                    and node.joints is not None:
                break
        if node.pair is None:
            print('snap_view: no frame with a detection in %.0f s (pipeline running, object in view?)' % a.timeout)
            return 1
        img, det = node.pair
        tf = {'from': BASE, 'to': OPTICAL}
        try:
            tr = node.tf.lookup_transform(BASE, OPTICAL, rclpy.time.Time()).transform
            tf.update(t=[tr.translation.x, tr.translation.y, tr.translation.z],
                      q_xyzw=[tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
        except Exception as ex:                       # noqa: BLE001 - recorded, the frame is still useful
            tf['error'] = str(ex)
        doc = {
            'image': {'png': os.path.basename(a.out) + '.png', 'capture_stamp_ns': stamp_ns(img.header.stamp),
                      'width': int(img.width), 'height': int(img.height), 'encoding': img.encoding,
                      'frame_id': img.header.frame_id},
            'detections': [fields(d, ('class_id', 'class_name', 'confidence', 'center_x', 'center_y', 'width',
                                      'height')) for d in det.detections],
            'target_base': None if node.target is None else dict(
                fields(node.target, ('capture_stamp', 'target_valid', 'depth_valid', 'class_name', 'confidence',
                                     'x', 'y', 'z')), frame_id=node.target.header.frame_id,
                stamp_ns=stamp_ns(node.target.header.stamp)),
            'joint_states': None if node.joints is None else dict(
                name=list(node.joints.name), position=[float(x) for x in node.joints.position],
                stamp_ns=stamp_ns(node.joints.header.stamp)),
            'camera_info': None if node.info is None else dict(
                width=int(node.info.width), height=int(node.info.height), frame_id=node.info.header.frame_id,
                K=[float(x) for x in node.info.k], D=[float(x) for x in node.info.d],
                distortion_model=node.info.distortion_model),
            'tf': tf,
            'saved_at_ns': time.time_ns(),
        }
        os.makedirs(os.path.dirname(os.path.abspath(a.out)) or '.', exist_ok=True)
        if not cv2.imwrite(a.out + '.png', to_bgr(img)):
            print('snap_view: cannot write %s.png' % a.out)
            return 1
        with open(a.out + '.json', 'w') as f:
            json.dump(doc, f, indent=1)
        miss = [k for k in ('target_base', 'joint_states', 'camera_info') if doc[k] is None]
        print('snap_view: %s.png (%dx%d, %d detection(s): %s), %s.json%s%s' % (
            a.out, img.width, img.height, len(doc['detections']),
            ', '.join('%s %.2f' % (d['class_name'], d['confidence']) for d in doc['detections']), a.out,
            '' if not miss else '; MISSING ' + ', '.join(miss) + ' (the controller with its bridge running?)',
            '' if 'error' not in tf else '; no transform ' + BASE + ' -> ' + OPTICAL))
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
