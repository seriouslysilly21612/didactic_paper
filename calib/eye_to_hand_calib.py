#!/usr/bin/env python3
"""eye_to_hand_calib.py - the pose of the camera in the base frame (rMc) by hand-eye calibration (paper,
Sec. V-C, eq. (10): bT_f(i) fT_t = bT_c cT_t(i)), from the pairs
pose_rPe_N.yaml (base -> flange, written by the controller, key 's' after 'w') and
pose_cPo_N.yaml (camera -> tag, calib/calib_cpo.py), paired by their index N.

  python3 calib/eye_to_hand_calib.py [--dir DIR] [--no-plot]

DIR: the directory of the pairs (default: calib/data, CALIB_DIR of indy7/INDY7.cfg). Pairs with an index on one
side only are reported and skipped. The rotation of the flange-to-tag transform from the relative motions of
all pairs (Park and Martin), then its translation by least squares; rMc is the mean over the pairs of
rMe eMo cMo^-1. Writes rPc.yaml, rPc.txt and rMc_visualization.png into DIR and prints the residual of every
pair (its rMc against the mean): pairs that stand out, and those that calib_cpo.py marks AMBIGUOUS, are moved
to DIR/excluded/ by hand and the solution is run again. calib/calib_to_tf.py puts the result into the launch
file of the camera pipeline.
"""
import argparse
import glob
import os
import re
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

def load_pose_yaml(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
    # Remove comment lines
    lines = [l for l in content.split('\n') if not l.strip().startswith('#')]
    data = yaml.safe_load('\n'.join(lines))
    vals = [v[0] for v in data['data']]
    tx, ty, tz = vals[0], vals[1], vals[2]
    tux, tuy, tuz = vals[3], vals[4], vals[5]
    theta = np.sqrt(tux**2 + tuy**2 + tuz**2)
    if theta < 1e-10:
        R = np.eye(3)
    else:
        R = Rotation.from_rotvec([tux, tuy, tuz]).as_matrix()
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [tx, ty, tz]
    return M

def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def rot_to_vec(R):
    return Rotation.from_matrix(R).as_rotvec()

def solve_eye_to_hand(rMe_list, cMo_list):
    """
    Solve AX = ZB for eye-to-hand:
      rMe_i * eMo = rMc * cMo_i
      A_i = rMe_i, B_i = cMo_i, X = eMo, Z = rMc
    Using Park & Martin (1994) approach adapted for AX=ZB.
    """
    n = len(rMe_list)
    assert n == len(cMo_list)

    # Build relative motions: A_ij = rMe_i^{-1} * rMe_j,  B_ij = cMo_i^{-1} * cMo_j
    A_list, B_list = [], []
    for i in range(n):
        for j in range(i+1, n):
            Aij = np.linalg.inv(rMe_list[i]) @ rMe_list[j]
            Bij = np.linalg.inv(cMo_list[i]) @ cMo_list[j]
            A_list.append(Aij)
            B_list.append(Bij)

    # Rotation part: solve Ra * Rx = Rx * Rb, i.e. AX = XB on rotations
    M_rot = np.zeros((3, 3))
    for A, B in zip(A_list, B_list):
        Ra = A[:3, :3]
        Rb = B[:3, :3]
        alpha = rot_to_vec(Ra)
        beta  = rot_to_vec(Rb)
        M_rot += np.outer(beta, alpha)

    U, S, Vt = np.linalg.svd(M_rot)
    Rx = Vt.T @ np.diag([1, 1, np.linalg.det(Vt.T) * np.linalg.det(U)]) @ U.T

    # Translation: solve for tx from (Ra - I)*tx = Rx*tb - ta
    C_list, d_list = [], []
    for A, B in zip(A_list, B_list):
        Ra = A[:3, :3]
        ta = A[:3, 3]
        tb = B[:3, 3]
        C_list.append(Ra - np.eye(3))
        d_list.append(Rx @ tb - ta)

    C = np.vstack(C_list)
    d = np.concatenate(d_list)
    tx, _, _, _ = np.linalg.lstsq(C, d, rcond=None)

    X = np.eye(4)
    X[:3, :3] = Rx
    X[:3, 3]  = tx
    return X

def indexed(data_dir, prefix):
    """{N: path} of prefix_N.yaml"""
    out = {}
    for path in glob.glob(os.path.join(data_dir, prefix + '_*.yaml')):
        m = re.fullmatch(prefix + r'_(\d+)\.yaml', os.path.basename(path))
        if m:
            out[int(m.group(1))] = path
    return out


# Load data
here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--dir', default=os.path.join(here, 'data'))
ap.add_argument('--no-plot', action='store_true', help='no rMc_visualization.png and no window')
args = ap.parse_args()
data_dir = os.path.abspath(args.dir)
rPe = indexed(data_dir, 'pose_rPe')
cPo = indexed(data_dir, 'pose_cPo')
pairs = sorted(set(rPe) & set(cPo))
only_r = sorted(set(rPe) - set(cPo))
only_c = sorted(set(cPo) - set(rPe))
print(f"{data_dir}: {len(rPe)} rPe files, {len(cPo)} cPo files, {len(pairs)} pairs by index: "
      + ' '.join(str(n) for n in pairs))
if only_r:
    print(f"WARNING: rPe without cPo (the tag was not detected?), skipped: {only_r}")
if only_c:
    print(f"WARNING: cPo without rPe ('s' not pressed?), skipped: {only_c}")
if len(pairs) < 3:
    sys.exit('eye_to_hand_calib: at least 3 pairs are needed')

rMe_list, cMo_list = [], []
for n in pairs:
    rMe_list.append(load_pose_yaml(rPe[n]))
    cMo_list.append(load_pose_yaml(cPo[n]))

# Solve AX=ZB: rMe * eMo = rMc * cMo
# -> the equivalent AX = XB on relative motions gives eMo
eMo = solve_eye_to_hand(rMe_list, cMo_list)

# rMc from each measurement: rMc = rMe * eMo * cMo^{-1}
rMc_list = []
for rMe, cMo in zip(rMe_list, cMo_list):
    rMc_i = rMe @ eMo @ np.linalg.inv(cMo)
    rMc_list.append(rMc_i)

# Average translations
t_mean = np.mean([m[:3,3] for m in rMc_list], axis=0)
# Average rotations via SVD
R_sum = np.zeros((3,3))
for m in rMc_list:
    R_sum += m[:3,:3]
U, S, Vt = np.linalg.svd(R_sum)
R_mean = U @ np.diag([1,1,np.linalg.det(U)*np.linalg.det(Vt)]) @ Vt

rMc = np.eye(4)
rMc[:3,:3] = R_mean
rMc[:3,3]  = t_mean

print("\n=== Eye-to-Hand Calibration Result ===")
print("\nrMc (robot base -> camera) homogeneous matrix:")
print(np.array2string(rMc, precision=6, suppress_small=True))

t = rMc[:3, 3]
R = rMc[:3, :3]
rpy = Rotation.from_matrix(R).as_euler('xyz', degrees=True)
rotvec = Rotation.from_matrix(R).as_rotvec()

print(f"\nTranslation [m]:  X={t[0]:.4f}  Y={t[1]:.4f}  Z={t[2]:.4f}")
print(f"Distance base -> camera: {np.linalg.norm(t)*100:.1f} cm")
print(f"Rotation RPY [deg]:  R={rpy[0]:.2f}  P={rpy[1]:.2f}  Y={rpy[2]:.2f}")
print(f"Rotation theta-u [rad]: {rotvec[0]:.4f}  {rotvec[1]:.4f}  {rotvec[2]:.4f}")
q = Rotation.from_matrix(R).as_quat()      # x, y, z, w
print(f"Quaternion (x y z w):   {q[0]:.8f}  {q[1]:.8f}  {q[2]:.8f}  {q[3]:.8f}")
print("(rMc is base -> the optical frame of the color camera, the frame of the tag poses; the static transform "
      "of the launch file is base_link -> camera_link, so the optical rotation of the RealSense driver must be "
      "composed: calib/calib_to_tf.py does it.)")

# Residuals
errors_t, errors_r = [], []
for rMe, cMo in zip(rMe_list, cMo_list):
    rMc_i = rMe @ eMo @ np.linalg.inv(cMo)
    dt = np.linalg.norm(rMc_i[:3, 3] - rMc[:3, 3])
    dR = np.linalg.norm(Rotation.from_matrix(rMc_i[:3,:3] @ rMc[:3,:3].T).as_rotvec()) * 180/np.pi
    errors_t.append(dt)
    errors_r.append(dR)
print(f"\nResiduals of the {len(pairs)} pairs (rMc of each pair against the mean):")
for n, dt, dR in zip(pairs, errors_t, errors_r):
    print(f"  pair {n:3d}: translation {dt*1000:6.2f} mm, rotation {dR:5.2f} deg")
print(f"Mean residual - translation: {np.mean(errors_t)*100:.2f} cm ({np.mean(errors_t)*1000:.1f} mm, "
      f"max {np.max(errors_t)*1000:.1f} mm)")
print(f"Mean residual - rotation:    {np.mean(errors_r):.2f} deg (max {np.max(errors_r):.2f} deg)")

# Visualization
if args.no_plot:
    plt = None
else:
    import matplotlib
    if not os.environ.get('DISPLAY'):
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

if plt is not None:
    def draw_frame(ax, R, t, label, scale=0.1):
        colors = ['r', 'g', 'b']
        for i, c in enumerate(colors):
            ax.quiver(t[0], t[1], t[2],
                      R[0,i]*scale, R[1,i]*scale, R[2,i]*scale,
                      color=c, linewidth=2)
        ax.text(t[0]+0.02, t[1]+0.02, t[2]+0.02, label, fontsize=9, fontweight='bold')

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    draw_frame(ax, np.eye(3), np.zeros(3), 'Robot Base')
    draw_frame(ax, rMc[:3,:3], rMc[:3,3], 'Camera')

    ax.quiver(0,0,0, t[0],t[1],t[2], color='purple', linewidth=1.5,
              linestyle='dashed', arrow_length_ratio=0.1)

    # End-effector poses
    for rMe in rMe_list:
        p = rMe[:3,3]
        ax.scatter(p[0], p[1], p[2], c='orange', s=20)

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('Eye-to-hand: rMc (robot base -> camera)\norange dots: flange positions')

    lim = max(np.linalg.norm(t)*1.5, 0.5)
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)

    plt.tight_layout()
    plt.savefig(os.path.join(data_dir, 'rMc_visualization.png'), dpi=150)
    print("\nSaved: rMc_visualization.png")

# Save rPc.yaml
yaml_path = os.path.join(data_dir, 'rPc.yaml')
tu = Rotation.from_matrix(R_mean).as_rotvec()
with open(yaml_path, 'w') as f:
    f.write("# Robot reference to camera frames transformation (rMc)\n")
    f.write("rows: 6\n")
    f.write("cols: 1\n")
    f.write("data: \n")
    for v in list(t_mean) + list(tu):
        f.write(f"  - [{v}]\n")
print(f"Saved: {yaml_path}")

# Save rPc.txt (homogeneous matrix)
txt_path = os.path.join(data_dir, 'rPc.txt')
with open(txt_path, 'w') as f:
    for i in range(3):
        f.write("  ".join(f"{rMc[i,j]}" for j in range(4)) + "\n")
    f.write("0  0  0  1\n")
print(f"Saved: {txt_path}")

if plt is not None and os.environ.get('DISPLAY'):
    plt.show()
