#!/usr/bin/env python3
"""calib_grab.py - the images of the hand-eye calibration (paper, Sec. V-C, eq. (10)).

Run while the camera pipeline is up (it owns the camera; ros2/README) and the controller holds the arm in
gravity compensation with the servo on ('r', 'g', 'h'). Per pose the operator
  1. guides the arm by hand so that the flange AprilTag faces the camera,
  2. presses Enter here: the newest frame is checked for the tag and saved only when the tag is detected
     (image%04d.png),
  3. presses 's' in the controller: pose_rPe_<N>.yaml with the same index N (the counter of the controller
     and the one here must stay in step, which is why the image is confirmed first).
Arm the capture of the controller first: 'w' once. Without it 's' only prints the pose ("(print only)") and
writes nothing, so that a stray 's' cannot rewrite pose_rPe_N while image000N and pose_cPo_N stay; the
banner of 'w' shows the next N, which must agree with the number here.

camera_info is saved once (camera_info.yaml): the factory intrinsics, the model that the 3-D reconstruction
of the pipeline uses. The frames are the raw color images (the topic of the detector: it exists whenever the
pipeline runs; JPEG would blur the corners of the tag). OpenCV 4.x: the ArUco API changed in 4.7
(ArucoDetector), both are handled. One tag in view: a frame with more than one is not saved.

Run:  source /opt/ros/jazzy/setup.bash && source ros2/install/setup.bash
      python3 calib/calib_grab.py          (from the root of the repository)
Output: calib/data/ (CALIB_DIR of indy7/INDY7.cfg). Then calib/calib_cpo.py, calib/eye_to_hand_calib.py and
calib/calib_to_tf.py.
"""
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

OUT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data'))
TOPIC_IMG = '/camera/camera/color/image_raw'
TOPIC_INFO = '/camera/camera/color/camera_info'


def aruco_detector():
    """detect(img) -> (corners, ids) for the AprilTag 36h11 family with OpenCV < 4.7
    (DetectorParameters_create, detectMarkers) and >= 4.7 (ArucoDetector)"""
    adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        det = cv2.aruco.ArucoDetector(adict, cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)[:2]
    params = cv2.aruco.DetectorParameters_create()
    return lambda img: cv2.aruco.detectMarkers(img, adict, parameters=params)[:2]


def to_bgr(msg):
    """sensor_msgs/Image (rgb8, bgr8, rgba8, bgra8, mono8) -> BGR image (copy)"""
    enc = msg.encoding.lower()
    ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1, '8uc1': 1}.get(enc)
    if ch is None:
        raise ValueError('image encoding %s not handled' % msg.encoding)
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * ch].reshape(msg.height, msg.width, ch) if ch > 1 else rows[:, :msg.width]
    conv = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR, 'bgra8': cv2.COLOR_BGRA2BGR,
            'mono8': cv2.COLOR_GRAY2BGR, '8uc1': cv2.COLOR_GRAY2BGR}.get(enc)
    return cv2.cvtColor(img, conv) if conv is not None else img.copy()


class Grabber(Node):
    def __init__(self):
        super().__init__('calib_grab')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.latest = None          # the newest sensor_msgs/Image
        self.info = None
        self.create_subscription(Image, TOPIC_IMG, self._on_img, qos)
        self.create_subscription(CameraInfo, TOPIC_INFO, self._on_info, qos)
        self.detect = aruco_detector()

    def _on_img(self, msg):
        self.latest = msg           # one reference, replaced atomically

    def _on_info(self, msg):
        if self.info is None:
            self.info = msg

    def snap(self):
        msg = self.latest
        if msg is None:
            return None, 'no frame yet'
        try:
            img = to_bgr(msg)
        except ValueError as ex:
            return None, str(ex)
        corners, ids = self.detect(img)
        ids = np.asarray(ids if ids is not None else [], int).flatten()     # (N, 1) or (N,) by version
        if len(ids) == 0:
            return None, 'TAG NOT FOUND'
        if len(ids) > 1:            # calib_cpo.py takes the first marker: which one would be chance
            return None, f'{len(ids)} TAGS IN VIEW (ids {sorted(ids.tolist())}) - keep one'
        c = np.asarray(corners[0], float).reshape(-1, 2)
        side = float(np.linalg.norm(c[0] - c[1]))
        return img, f'tag id={int(ids[0])}, side ~{side:.0f} px'


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rclpy.init()
    node = Grabber()
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()

    print(f'[grab] waiting for camera_info + first frame on {TOPIC_IMG} ...')
    t0 = time.time()
    while (node.info is None or node.latest is None) and time.time() - t0 < 30:
        time.sleep(0.2)
    if node.info is None or node.latest is None:
        print('[grab] no camera data - is the pipeline running?')
        return 1

    info = node.info
    with open(os.path.join(OUT_DIR, 'camera_info.yaml'), 'w') as f:
        f.write(f'width: {info.width}\nheight: {info.height}\n')
        f.write(f'distortion_model: {info.distortion_model}\n')
        f.write('K: [' + ', '.join(f'{v:.6f}' for v in info.k) + ']\n')
        f.write('D: [' + ', '.join(f'{v:.6f}' for v in info.d) + ']\n')
    print(f'[grab] camera_info saved ({info.width}x{info.height}, '
          f'fx={info.k[0]:.1f} fy={info.k[4]:.1f})')
    print("[grab] per pose: place the arm, then Enter here (tag check + save), then 's' in the controller")
    print('[grab] Enter: capture, q and Enter: quit\n')

    n = 0
    while True:
        try:
            line = input(f'#{n + 1} - Enter: ')
        except EOFError:
            break
        if line.strip().lower() == 'q':
            break
        img, detail = node.snap()
        if img is None:
            print(f'   {detail} - not saved; adjust the pose, then Enter again')
            continue
        n += 1
        path = os.path.join(OUT_DIR, f'image{n:04d}.png')
        cv2.imwrite(path, img)
        print(f'   SAVED image{n:04d}.png ({detail})')
        print(f"   now 's' in the controller (it must become pose_rPe_{n}.yaml): the controller prints "
              f'"*** CAPTURED ***"; if it prints "(print only)", arm the capture with \'w\' first')

    print(f'\n[grab] {n} images captured: check that the controller has written as many pose_rPe files')
    return 0


if __name__ == '__main__':
    # end without tearing rclpy down: rclpy.shutdown() while the spin thread waits inside rclpy can abort
    # the process ("terminate called without an active exception") after the files were written. Every
    # file is closed here, so the process just ends.
    try:
        code = main()
    except KeyboardInterrupt:
        code = 130
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
