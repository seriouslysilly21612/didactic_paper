#!/usr/bin/env python3
"""sim_view.py - look at the arm: live from the controller, a replayed log, or a picture.

The controller and the simulation run on the real-time computer; this script only displays.

  live    SCENE --host HOST [--port 7430]
          the joint angles that the controller streams (cfg VIEW_PORT, indy7/src/base/ViewStream.h), in the
          viewer of MuJoCo
  replay  SCENE LOG.csv [--speed 1.0]
          a log of the controller in the viewer: a JOINT TEST log (jt_*.csv) or a high-level test log
          (hl_*.csv), columns t, q0..q5
  render  SCENE --out FIG.png  (--q "q0 ... q5" | LOG.csv --t T | LOG.csv --ghost T1,T2,...)
          [--width 1600] [--height 1200] [--camera figure] [--alpha 0.3]
          a picture without a display; --ghost draws the postures at T1, T2, ... translucent
          and the last one solid, to show a motion in one picture

  record  SCENE LOG.csv [LOG.csv ...] --out VIDEO.mp4 [--fps 30] [--speed 1.0] [--frames T1,T2,...]
          [--view X,Y,Z,DIST,AZIM,ELEV | --camera figure] [--width 1280] [--height 960] [--no-label]
          a video of high-level test logs (hl_*.csv) without a display: the arm, the path of the tool
          drawn into the scene (blue, from its start, black), and the accepted goals (orange; earlier
          ones light) with a line down to the floor; several logs play one after another. --frames also
          writes the pictures at these times of the first log (path up to them, no label; the path and
          the goals cast no shadow there), for figures (paper, Figs. 6 and 8).
          The video needs  pip install imageio imageio-ffmpeg ; without it, the frames go into a folder.

SCENE is the model with meshes from  tools/urdf2mjcf.py URDF SCENE --scene --meshdir DIR
(tools/make_view_bundle.sh puts it, the meshes, and this script in one folder).
live and replay need a display (Windows or a Linux desktop); render does not
(on a Linux without a display: MUJOCO_GL=egl, or MUJOCO_GL=osmesa).
Needs Python >= 3.10 and:  pip install mujoco numpy pillow
"""
import argparse
import csv
import os
import socket
import sys
import time

import numpy as np

try:
    import mujoco
except ImportError:
    sys.exit('sim_view: the mujoco package is missing: pip install mujoco numpy pillow')

DOF = 6


def load(scene):
    m = mujoco.MjModel.from_xml_path(scene)
    adr = []
    for i in range(DOF):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'joint%d' % i)
        if j < 0:
            sys.exit('sim_view: %s has no joint "joint%d"' % (scene, i))
        adr.append(m.jnt_qposadr[j])
    return m, mujoco.MjData(m), adr


def set_q(m, d, adr, q):
    for a, v in zip(adr, q):
        d.qpos[a] = v
    mujoco.mj_forward(m, d)


def read_log(path):
    """a log with the columns t (or timestamp_ns) and q0..q5 -> (t [s] from the first row, q [n x 6])"""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows or 'q0' not in rows[0]:
        sys.exit('sim_view: %s has no columns q0..q%d' % (path, DOF - 1))
    q = np.array([[float(r['q%d' % i]) for i in range(DOF)] for r in rows])
    if 'timestamp_ns' in rows[0]:
        t = np.array([float(r['timestamp_ns']) for r in rows]) * 1e-9
        t -= t[0]
    elif 't' in rows[0]:
        t = np.array([float(r['t']) for r in rows])
        t -= t[0]
    else:
        t = np.arange(len(rows)) * 1e-3
    return t, q


def view_live(a):
    m, d, adr = load(a.scene)
    sock = None
    buf = b''
    n = 0
    t_rate = time.time()
    if a.no_gui:
        viewer = None
    else:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(m, d)
    try:
        while viewer is None or viewer.is_running():
            if sock is None:
                try:
                    sock = socket.create_connection((a.host, a.port), timeout=2.0)
                    sock.settimeout(0.2)
                    buf = b''
                    print('sim_view: connected to %s:%d' % (a.host, a.port))
                except OSError as e:
                    print('sim_view: no stream at %s:%d (%s), retrying' % (a.host, a.port, e))
                    time.sleep(1.0)
                    if viewer is not None:
                        viewer.sync()
                    continue
            try:
                chunk = sock.recv(65536)
                if not chunk:
                    raise ConnectionError('closed')
                buf += chunk
            except socket.timeout:
                pass
            except OSError as e:
                print('sim_view: stream lost (%s), reconnecting' % e)
                sock.close()
                sock = None
                continue
            lines = buf.split(b'\n')
            buf = lines[-1]
            last = None
            for line in lines[:-1]:
                if line.startswith(b'#') or not line.strip():
                    continue
                last = line
                n += 1
            if last is not None:
                v = last.split()
                set_q(m, d, adr, [float(x) for x in v[1:1 + DOF]])
                if viewer is not None:
                    viewer.sync()
            if a.no_gui and time.time() - t_rate >= 1.0:
                print('sim_view: %d samples/s, q = %s' % (n, np.round(d.qpos[adr], 4)))
                n = 0
                t_rate = time.time()
                a.frames -= 1
                if a.frames <= 0:
                    break
    finally:
        if sock is not None:
            sock.close()
        if viewer is not None:
            viewer.close()


def view_replay(a):
    m, d, adr = load(a.scene)
    t, q = read_log(a.log)
    import mujoco.viewer
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            t0 = time.time()
            k = 0
            while viewer.is_running() and k < len(t):
                now = (time.time() - t0) * a.speed
                k = int(np.searchsorted(t, now))
                if k >= len(t):
                    break
                set_q(m, d, adr, q[k])
                viewer.sync()
                time.sleep(1.0 / 60.0)
            time.sleep(1.0)        # then play it again until the window is closed


def render(a):
    m, d, adr = load(a.scene)
    from PIL import Image
    if a.q is not None:
        poses = [[float(x) for x in a.q.split()]]
    elif a.log is not None:
        t, q = read_log(a.log)
        times = [float(x) for x in a.ghost.split(',')] if a.ghost else [a.t]
        poses = [q[min(int(np.searchsorted(t, x)), len(t) - 1)] for x in times]
    else:
        sys.exit('sim_view: render needs --q or a log')
    r = mujoco.Renderer(m, height=a.height, width=a.width)
    set_q(m, d, adr, poses[-1])
    r.update_scene(d, camera=a.camera)
    if len(poses) > 1:
        opt = mujoco.MjvOption()
        pert = mujoco.MjvPerturb()
        dg = mujoco.MjData(m)
        for p in poses[:-1]:
            n0 = r.scene.ngeom
            set_q(m, dg, adr, p)
            mujoco.mjv_addGeoms(m, dg, opt, pert, mujoco.mjtCatBit.mjCAT_DYNAMIC, r.scene)
            for g in range(n0, r.scene.ngeom):
                r.scene.geoms[g].rgba[3] = a.alpha
    Image.fromarray(r.render()).save(a.out)
    print('sim_view: wrote %s (%dx%d, %d posture(s))' % (a.out, a.width, a.height, len(poses)))


C_PATH = (31 / 255, 94 / 255, 168 / 255, 1.0)       # blue of the figures: path of the tool
C_GOAL = (214 / 255, 110 / 255, 20 / 255, 1.0)      # orange: goals
F_GOAL = 64                                         # flag of the high-level log: a goal has been accepted


def read_hl_log(path):
    """high-level test log (hl_*.csv) -> t, q, tool, accepted goal (NaN before the first), flags"""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    need = ['t', 'flags', 'tool_x', 'acc_x'] + ['q%d' % i for i in range(DOF)]
    if not rows or any(k not in rows[0] for k in need):
        sys.exit('sim_view: %s is not a high-level test log (hl_*.csv)' % path)
    num = lambda r, k: float(r[k]) if r[k] != '' else np.nan
    t = np.array([num(r, 't') for r in rows])
    q = np.array([[num(r, 'q%d' % i) for i in range(DOF)] for r in rows])
    tool = np.array([[num(r, 'tool_%s' % a) for a in 'xyz'] for r in rows])
    acc = np.array([[num(r, 'acc_%s' % a) for a in 'xyz'] for r in rows])
    flags = np.array([int(float(r['flags'])) for r in rows])
    acc[(flags & F_GOAL) == 0] = np.nan
    return t - t[0], q, tool, acc, flags


def _add_geom(scn, gtype, size, pos, rgba, mat=None):
    if scn.ngeom >= scn.maxgeom:
        return None
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, gtype, np.asarray(size, float), np.asarray(pos, float),
                        np.eye(3).flatten() if mat is None else np.asarray(mat, float).flatten(),
                        np.asarray(rgba, np.float32))
    scn.ngeom += 1
    return g


def _add_segment(scn, p1, p2, width, rgba):
    g = _add_geom(scn, mujoco.mjtGeom.mjGEOM_CAPSULE, [0, 0, 0], [0, 0, 0], rgba)
    if g is not None:
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, np.asarray(p1, float), np.asarray(p2, float))


def decorate(scn, tool, acc, i, step=0.004):
    """draw into the scene: the path of the tool up to row i, its start, and the accepted goals up to row i"""
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(tool[:i + 1], axis=0), axis=1))] if i > 0 else np.zeros(1)
    idx = [0]
    for k in range(1, i + 1):
        if seg[k] - seg[idx[-1]] >= step:
            idx.append(k)
    if idx[-1] != i:
        idx.append(i)
    for a, b in zip(idx[:-1], idx[1:]):
        if np.linalg.norm(tool[b] - tool[a]) > 1e-6:
            _add_segment(scn, tool[a], tool[b], 0.0035, C_PATH)
    _add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.008, 0, 0], tool[0], (0.1, 0.1, 0.1, 1.0))
    goals = []                                   # distinct accepted goals up to row i
    for k in range(i + 1):
        if np.all(np.isfinite(acc[k])) and (not goals or np.linalg.norm(acc[k] - goals[-1]) > 1e-9):
            goals.append(acc[k].copy())
    for n, gp in enumerate(goals):
        cur = n == len(goals) - 1
        rgba = C_GOAL if cur else (C_GOAL[0], C_GOAL[1], C_GOAL[2], 0.35)
        _add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.012, 0, 0], gp, rgba)
        _add_segment(scn, gp, [gp[0], gp[1], 0.0], 0.0012, (0.45, 0.45, 0.45, 0.6 if cur else 0.3))


def record(a):
    m, d, adr = load(a.scene)
    from PIL import Image, ImageDraw
    logs = [read_hl_log(p) for p in a.logs]
    r = mujoco.Renderer(m, height=a.height, width=a.width)
    cam = a.camera
    if a.view and not a.camera:
        v = [float(x) for x in a.view.split(',')]
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = v[0:3]
        cam.distance, cam.azimuth, cam.elevation = v[3], v[4], v[5]

    def picture(t, q, tool, acc, i, label=None):
        set_q(m, d, adr, q[i])
        r.update_scene(d, camera=cam)
        r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 1
        decorate(r.scene, tool, acc, i)
        img = Image.fromarray(r.render())
        if label:
            ImageDraw.Draw(img).text((12, 10), label, fill=(40, 40, 40))
        return img

    def clean_picture(t, q, tool, acc, i):
        """as picture(), but the path and the goals cast no shadow: the arm with its shadow, and on top
        the pixels in which the path and the goals are seen (rendered without shadows, so occluded
        parts stay hidden)"""
        set_q(m, d, adr, q[i])
        flag = mujoco.mjtRndFlag.mjRND_SHADOW
        r.update_scene(d, camera=cam)
        r.scene.flags[flag] = 1                            # the flags stay in the scene: set each time
        base = r.render().astype(np.int16)                 # arm and floor with shadows
        r.update_scene(d, camera=cam)
        r.scene.flags[flag] = 0
        plain = r.render().astype(np.int16)                # without shadows
        r.update_scene(d, camera=cam)
        r.scene.flags[flag] = 0
        decorate(r.scene, tool, acc, i)
        deco = r.render().astype(np.int16)                 # without shadows, with path and goals
        mask = np.abs(deco - plain).max(axis=2) > 6
        out = base.copy()
        out[mask] = deco[mask]
        r.scene.flags[flag] = 1
        return Image.fromarray(out.astype(np.uint8))

    if a.frames:                                 # pictures for the figures, from the first log
        t, q, tool, acc, _ = logs[0]
        base = os.path.splitext(a.out)[0]
        for n, tk in enumerate(float(x) for x in a.frames.split(',')):
            i = min(int(np.searchsorted(t, tk - 1e-9)), len(t) - 1)
            path = '%s_frame%d_%.3fs.png' % (base, n, t[i])
            clean_picture(t, q, tool, acc, i).save(path)
            print('sim_view: wrote %s' % path)
    try:
        import imageio
        writer = imageio.get_writer(a.out, fps=a.fps, codec='libx264', quality=8, macro_block_size=1)
        put = lambda img: writer.append_data(np.asarray(img))
        folder = None
    except Exception as ex:                      # no imageio or no ffmpeg: a folder of pictures
        folder = os.path.splitext(a.out)[0] + '_frames'
        os.makedirs(folder, exist_ok=True)
        print('sim_view: no video writer (%s): pictures into %s' % (ex, folder))
        writer = None
        count = [0]

        def put(img):
            img.save(os.path.join(folder, 'f%05d.png' % count[0]))
            count[0] += 1
    nf = 0
    for n, (t, q, tool, acc, _) in enumerate(logs):
        steps = int(t[-1] * a.fps / a.speed) + 1
        for k in range(steps + int(0.5 * a.fps)):            # and half a second at the end
            i = min(int(np.searchsorted(t, k * a.speed / a.fps - 1e-9)), len(t) - 1)
            label = None if a.no_label else '%s   t = %.2f s' % (os.path.basename(a.logs[n])[:40], t[i])
            put(picture(t, q, tool, acc, i, label))
            nf += 1
    if writer is not None:
        writer.close()
    print('sim_view: %d frames of %d log(s) -> %s' % (nf, len(logs), folder or a.out))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='mode', required=True)
    p = sub.add_parser('live')
    p.add_argument('scene')
    p.add_argument('--host', required=True, help='the address of the controller')
    p.add_argument('--port', type=int, default=7430)
    p.add_argument('--no-gui', action='store_true', help='print the stream instead (test)')
    p.add_argument('--frames', type=int, default=5, help='with --no-gui: seconds to print')
    p = sub.add_parser('replay')
    p.add_argument('scene')
    p.add_argument('log')
    p.add_argument('--speed', type=float, default=1.0)
    p = sub.add_parser('render')
    p.add_argument('scene')
    p.add_argument('log', nargs='?')
    p.add_argument('--out', required=True)
    p.add_argument('--q', help='one posture, "q0 ... q5" [rad]')
    p.add_argument('--t', type=float, default=0.0, help='time in the log [s]')
    p.add_argument('--ghost', help='times in the log [s], comma separated')
    p.add_argument('--alpha', type=float, default=0.3)
    p.add_argument('--camera', default='figure')
    p.add_argument('--width', type=int, default=1600)
    p.add_argument('--height', type=int, default=1200)
    p = sub.add_parser('record')
    p.add_argument('scene')
    p.add_argument('logs', nargs='+', help='high-level test logs (hl_*.csv), played one after another')
    p.add_argument('--out', required=True, help='video, e.g. jump.mp4')
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--speed', type=float, default=1.0, help='> 1 plays faster')
    p.add_argument('--frames', help='times in the first log [s], comma separated: pictures for figures')
    p.add_argument('--view', default='0.40,-0.10,0.38,1.55,130,-22',
                   help='free camera: lookat x,y,z, distance, azimuth, elevation (default: the goal-jump test)')
    p.add_argument('--camera', help='a camera of the scene instead, e.g. figure')
    p.add_argument('--no-label', action='store_true', help='no file name and time in the video')
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=960)
    a = ap.parse_args()
    {'live': view_live, 'replay': view_replay, 'render': render, 'record': record}[a.mode](a)


if __name__ == '__main__':
    main()
