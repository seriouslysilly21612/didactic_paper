#!/usr/bin/env python3
"""calib_cpo.py - the pose of the flange tag in the camera frame, pose_cPo_<N>.yaml, from image<N>.png of
calib/calib_grab.py (the hand-eye calibration, paper, Sec. V-C, eq. (10): cT_t(i)).

Detects the flange AprilTag (36h11) in every captured image and solves the planar pose with
SOLVEPNP_IPPE_SQUARE (well conditioned for one square); it reports the ratio of the reprojection errors of
the two solutions of a square (alt/best), and marks a frame AMBIGUOUS below 2: such pairs, and pairs whose
residual in calib/eye_to_hand_calib.py stands out, are moved to data/excluded/ by hand before the final
solution (in the paper, 11 of 16 observations were retained). The output is the vpPoseVector YAML of ViSP
(tx ty tz and theta-u), which calib/eye_to_hand_calib.py reads.

The intrinsics come from camera_info.yaml of calib_grab.py: the factory calibration, the camera model of the
3-D reconstruction of the pipeline.

Usage: python3 calib/calib_cpo.py --tag-mm 65.0 [--dir calib/data]
"""
import argparse
import glob
import os
import re
import sys

import cv2
import numpy as np
import yaml


def load_intrinsics(path):
    with open(path) as f:
        info = yaml.safe_load(f)
    K = np.array(info['K'], dtype=np.float64).reshape(3, 3)
    D = np.array(info['D'], dtype=np.float64)
    return K, D


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag-mm', type=float, required=True,
                    help='black square outer side length [mm]')
    ap.add_argument('--dir', default=os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')),
                    help='the images and camera_info.yaml (default calib/data, CALIB_DIR of indy7/INDY7.cfg)')
    args = ap.parse_args()

    s = args.tag_mm / 1000.0
    # Corner order must match detectMarkers: TL, TR, BR, BL (marker frame:
    # x right, y up, z out of the tag face) - the same as estimatePoseSingleMarkers.
    obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0],
                    [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], dtype=np.float64)

    K, D = load_intrinsics(os.path.join(args.dir, 'camera_info.yaml'))
    print(f'[cpo] tag side {args.tag_mm:.1f} mm, fx={K[0,0]:.1f} fy={K[1,1]:.1f}')

    adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    # OpenCV < 4.7: DetectorParameters_create + detectMarkers;
    # >= 4.7: DetectorParameters() + ArucoDetector (the old calls are gone)
    new_api = hasattr(cv2.aruco, 'ArucoDetector')
    # Best corners first, but fall back - CORNER_REFINE_APRILTAG can drop a
    # marker the default detector accepts. A missing index
    # leaves that pair out (eye_to_hand_calib.py pairs by index), so a
    # coarser detection beats no detection.
    param_sets = []
    for method in ('CORNER_REFINE_APRILTAG', 'CORNER_REFINE_SUBPIX', None):
        p = cv2.aruco.DetectorParameters() if new_api else cv2.aruco.DetectorParameters_create()
        if method is not None:
            try:
                p.cornerRefinementMethod = getattr(cv2.aruco, method)
            except AttributeError:
                continue
        if new_api:
            det = cv2.aruco.ArucoDetector(adict, p)
            param_sets.append((method or 'default', lambda img, det=det: det.detectMarkers(img)[:2]))
        else:
            param_sets.append((method or 'default',
                               lambda img, p=p: cv2.aruco.detectMarkers(img, adict, parameters=p)[:2]))

    def detect(img):
        for name, fn in param_sets:
            corners, ids = fn(img)
            if ids is not None and len(ids) > 0:
                if len(ids) > 1:
                    print(f'   {len(ids)} tags in view, the first one taken (ids {np.asarray(ids).flatten().tolist()})')
                return corners, name
        return None, None

    imgs = sorted(glob.glob(os.path.join(args.dir, 'image*.png')))
    if not imgs:
        print('[cpo] no images found'); return 1

    n_ok = 0
    for p in imgs:
        m = re.search(r'image(\d+)\.png$', p)
        idx = int(m.group(1))
        img = cv2.imread(p)
        corners, used = detect(img)
        if corners is None:
            print(f'image{idx:04d}: NO TAG - skipped (pose pair {idx} unusable)')
            continue
        pts = np.asarray(corners[0], np.float64).reshape(4, 2)

        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            print(f'image{idx:04d}: solvePnP FAILED'); continue
        errs = np.asarray(errs).flatten()
        best = int(np.argmin(errs))
        rvec, tvec = rvecs[best].flatten(), tvecs[best].flatten()
        ratio = (errs[1 - best] / max(errs[best], 1e-9)) if len(errs) > 1 else np.inf
        flag = '  AMBIGUOUS!' if ratio < 2.0 else ''

        out = os.path.join(args.dir, f'pose_cPo_{idx}.yaml')
        with open(out, 'w') as f:
            f.write('rows: 6\ncols: 1\ndata: \n')
            for v in list(tvec) + list(rvec):
                f.write(f'  - [{v:.9f}]\n')
        n_ok += 1
        note = '' if used == 'CORNER_REFINE_APRILTAG' else f'  [{used}]'
        print(f'image{idx:04d}: z={tvec[2]:.3f} m  reproj={errs[best]:.2f} px  '
              f'alt/best={ratio:5.1f}{flag}{note}')

    print(f'[cpo] {n_ok}/{len(imgs)} pose_cPo written to {args.dir}')
    return 0 if n_ok == len(imgs) else 1


if __name__ == '__main__':
    sys.exit(main())
