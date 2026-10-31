#!/usr/bin/env python3
"""calib_to_tf.py - the hand-eye calibration into the static transform of the camera pipeline (paper,
Sec. V-C).

calib/eye_to_hand_calib.py gives rMc, the pose of the optical frame of the COLOR camera in the base frame
(rPc.yaml: tx ty tz and the rotation vector theta-u), because the tag poses come from the color image. The
launch file of the pipeline publishes base_link -> camera_link; the RealSense driver publishes
camera_link -> camera_color_optical_frame (and the depth frames, in which the 3-D targets are). So

    base_link -> camera_link = rMc * (camera_link -> camera_color_optical_frame)^-1

  python3 calib/calib_to_tf.py --live                              (pipeline running: TF read at full precision)
  python3 calib/calib_to_tf.py --link-to-optical "x y z qx qy qz qw"
  python3 calib/calib_to_tf.py --tf-echo echo.txt                  (output of: ros2 run tf2_ros tf2_echo
                                                                     camera_link camera_color_optical_frame)
  options: --rpc FILE (default calib/data/rPc.yaml), --launch FILE (default the launch file of
           ros2/src/system_bringup_pkg), --write (replace the numbers of base_to_camera_tf in the launch
           file; a backup is kept)

Printed: the new transform, the one in the launch file, and their difference (mm, deg), i.e. how far the new
calibration moves every goal. After --write, rebuild the workspace (colcon build; no rebuild with
--symlink-install) and restart the pipeline. Needs numpy; --live needs the ROS 2 environment (rclpy, tf2_ros).
"""
import argparse
import datetime
import os
import re
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..'))
DEF_RPC = os.path.join(HERE, 'data', 'rPc.yaml')
DEF_LAUNCH = os.path.join(REPO, 'ros2', 'src', 'system_bringup_pkg', 'launch', 'pick_place_openvino.launch.py')
ARG = re.compile(r"'--(x|y|z|qx|qy|qz|qw)',\s*'(-?[0-9.]+(?:[eE][-+]?[0-9]+)?)'")


def rotvec_to_R(v):
    v = np.asarray(v, float)
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def quat_to_R(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    """(x, y, z, w), w >= 0"""
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    q = np.array(q)
    return q if q[3] >= 0 else -q


def T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


def read_rpc(path):
    with open(path) as f:
        text = f.read()
    vals = [float(v) for v in re.findall(r'\[\s*(-?[0-9.]+(?:[eE][-+]?[0-9]+)?)\s*\]', text.split('data:', 1)[-1])]
    if len(vals) != 6:
        raise ValueError('%s: expected 6 values under data:, found %d' % (path, len(vals)))
    return T(rotvec_to_R(vals[3:]), vals[:3])


def parse_tf_echo(text):
    tr = re.search(r'Translation:\s*\[([^\]]+)\]', text)
    qu = re.search(r'Quaternion[^\[]*\[([^\]]+)\]', text)
    if not tr or not qu:
        raise ValueError('no "Translation: [...]" and "Quaternion ... [...]" in the tf2_echo output')
    t = [float(v) for v in tr.group(1).split(',')]
    q = [float(v) for v in qu.group(1).split(',')]
    return T(quat_to_R(q), t)


def live_link_to_optical(timeout=15.0):
    import time
    import rclpy
    import rclpy.time
    from rclpy.node import Node
    import tf2_ros
    rclpy.init()
    node = Node('calib_to_tf')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tr = buf.lookup_transform('camera_link', 'camera_color_optical_frame', rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            t = tr.transform.translation
            r = tr.transform.rotation
            return T(quat_to_R([r.x, r.y, r.z, r.w]), [t.x, t.y, t.z])
        raise RuntimeError('camera_link -> camera_color_optical_frame not in TF after %.0f s (driver running?)'
                           % timeout)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def read_launch(path):
    with open(path) as f:
        text = f.read()
    v = dict((k, float(x)) for k, x in ARG.findall(text))
    if set(v) != {'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'}:
        raise ValueError('%s: the arguments --x .. --qw of base_to_camera_tf not found' % path)
    return text, T(quat_to_R([v['qx'], v['qy'], v['qz'], v['qw']]), [v['x'], v['y'], v['z']])


def fmt(M):
    q = R_to_quat(M[:3, :3])
    t = M[:3, 3]
    return {'x': '%.6f' % t[0], 'y': '%.6f' % t[1], 'z': '%.6f' % t[2],
            'qx': '%.8f' % q[0], 'qy': '%.8f' % q[1], 'qz': '%.8f' % q[2], 'qw': '%.8f' % q[3]}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--live', action='store_true', help='read camera_link -> camera_color_optical_frame from TF')
    src.add_argument('--link-to-optical', help='"x y z qx qy qz qw" of camera_link -> camera_color_optical_frame')
    src.add_argument('--tf-echo', help='a file with the output of tf2_echo camera_link camera_color_optical_frame')
    ap.add_argument('--rpc', default=DEF_RPC)
    ap.add_argument('--launch', default=DEF_LAUNCH)
    ap.add_argument('--write', action='store_true', help='write the new transform into the launch file')
    a = ap.parse_args()
    base_opt = read_rpc(a.rpc)
    if a.live:
        link_opt = live_link_to_optical()
    elif a.tf_echo:
        with open(a.tf_echo) as f:
            link_opt = parse_tf_echo(f.read())
    else:
        v = [float(x) for x in a.link_to_optical.replace(',', ' ').split()]
        if len(v) != 7:
            sys.exit('calib_to_tf: --link-to-optical needs 7 numbers')
        link_opt = T(quat_to_R(v[3:]), v[:3])
    base_link = base_opt @ np.linalg.inv(link_opt)
    new = fmt(base_link)
    text, old_T = read_launch(a.launch)
    d = base_link @ np.linalg.inv(old_T)
    # how far a goal at the table moves: the change of the camera pose applied to a point 0.85 m in front of it
    p = old_T @ link_opt @ np.array([0.0, 0.0, 0.85, 1.0])
    p_new = base_link @ link_opt @ np.array([0.0, 0.0, 0.85, 1.0])
    print('rPc (base -> color optical) from %s' % a.rpc)
    print('camera_link -> camera_color_optical_frame: t %s, q %s' % (
        np.array2string(link_opt[:3, 3], precision=5), np.array2string(R_to_quat(link_opt[:3, :3]), precision=6)))
    print('new  base_link -> camera_link: ' + ' '.join("--%s %s" % (k, new[k]) for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')))
    old = fmt(old_T)
    print('launch base_link -> camera_link: ' + ' '.join("--%s %s" % (k, old[k]) for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')))
    print('difference: camera position %.1f mm, rotation %.2f deg; a point 0.85 m in front of the camera moves %.1f mm'
          % (1e3 * np.linalg.norm(base_link[:3, 3] - old_T[:3, 3]), angle_deg(d[:3, :3]),
             1e3 * np.linalg.norm(p_new[:3] - p[:3])))
    if a.write:
        bak = '%s.bak_%s' % (a.launch, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        shutil.copy2(a.launch, bak)
        out = ARG.sub(lambda m: "'--%s', '%s'" % (m.group(1), new[m.group(1)]), text)
        with open(a.launch, 'w') as f:
            f.write(out)
        print('wrote %s (backup %s); restart the pipeline' % (a.launch, bak))
    return 0


if __name__ == '__main__':
    sys.exit(main())
