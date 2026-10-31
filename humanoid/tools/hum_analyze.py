#!/usr/bin/env python3
"""hum_analyze.py - the joint test of the humanoid (paper, Sec. VI, Fig. 11 (e)) against the prediction of
eq. (8), as tools/jt_predict.py does for the arm.

Reads joint_test.csv and joint_test_meta.txt of  bin/humanoid_sim --mode joint  and the MuJoCo model of the
humanoid, and prints, per joint that moves, the largest |e| of the simulation and the predictions
  dde    e'' + Kd e'(t - tau_d) + Kp e(t - tau_d) = qdd_d(t) - qdd_d(t - tau_d)
  dde+g  plus tau_d M(q_d)^-1 (dg/dq)(q_d) qd_d(t), with M and dg/dq of the whole robot (29 joints)
with tau_d = 2.5 ms (the state 2 steps old, the torque 2 steps later, held over the step; one task, so no
period of a pipeline), and their ratio; the criterion: every ratio at most 1.5 where the prediction exceeds
1 urad. The joints that are held (legs, waist, the other arm joints) are listed with their largest error,
which the motion of the arms causes through the coupling that CTC cancels only up to the delay.

  python3 humanoid/tools/hum_analyze.py <joint test dir> humanoid/models/g1_mujoco.xml
"""
import os
import sys

import numpy as np

TAU_D = 2.5e-3


def read_meta(path):
    meta = {'joints': [], 'kp': {}, 'kd': {}, 'qr': {}, 'move': {}}
    for line in open(path):
        p = line.split()
        if not p:
            continue
        if p[0] in ('T', 'dwell'):
            meta[p[0]] = float(p[1])
        elif p[0] == 'joint':
            n = p[1]
            meta['joints'].append(n)
            meta['kp'][n], meta['kd'][n], meta['qr'][n] = float(p[3]), float(p[5]), float(p[7])
        elif p[0] == 'move':
            meta['move'][p[1]] = float(p[2])
    return meta


def reference(t, dq, T, dwell):
    t = np.asarray(t, dtype=float)
    q = np.zeros((len(t), len(dq)))
    qd = np.zeros_like(q)
    qdd = np.zeros_like(q)

    def leg(s, sign, base):
        p = 10 * s**3 - 15 * s**4 + 6 * s**5
        pd = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
        pdd = (60 * s - 180 * s**2 + 120 * s**3) / T**2
        return base + sign * np.outer(p, dq), sign * np.outer(pd, dq), sign * np.outer(pdd, dq)

    m0 = (t >= 0) & (t < T)
    m1 = (t >= T) & (t < T + dwell)
    m2 = (t >= T + dwell) & (t < 2 * T + dwell)
    if m0.any():
        q[m0], qd[m0], qdd[m0] = leg(t[m0] / T, 1.0, 0.0)
    q[m1] = dq
    if m2.any():
        q[m2], qd[m2], qdd[m2] = leg((t[m2] - T - dwell) / T, -1.0, dq)
    return q, qd, qdd


class Model:
    def __init__(self, mjcf, names):
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(mjcf)
        self.d = mujoco.MjData(self.m)
        self.qa = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
        self.va = [self.m.jnt_dofadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
        self.n = len(names)

    def _set(self, q):
        self.d.qpos[:] = 0
        self.d.qvel[:] = 0
        self.d.qpos[self.qa] = q
        self.mj.mj_forward(self.m, self.d)

    def g(self, q):
        self._set(q)
        return self.d.qfrc_bias[self.va].copy()

    def M(self, q):
        self._set(q)
        full = np.zeros((self.m.nv, self.m.nv))
        e, col = np.zeros(self.m.nv), np.zeros(self.m.nv)
        for j in range(self.m.nv):              # column by column: mj_mulM has the same signature in every MuJoCo 3
            e[j] = 1.0
            self.mj.mj_mulM(self.m, self.d, col, e)
            e[j] = 0.0
            full[:, j] = col
        return full[np.ix_(self.va, self.va)]

    def gravity_term(self, q, qd, tau_d, h=1e-6):
        moving = np.nonzero(qd)[0]
        G = np.zeros((self.n, self.n))
        for j in moving:                       # dg/dq qd needs only the columns of the moving joints
            e = np.zeros(self.n)
            e[j] = h
            G[:, j] = (self.g(q + e) - self.g(q - e)) / (2 * h)
        return tau_d * np.linalg.solve(self.M(q), G @ qd)


def predict(dq, T, dwell, Kp, Kd, tau_d, model=None, q0=None, h=1e-5, t_end=None):
    m = int(round(tau_d / h))
    t_end = t_end or (2 * T + 2 * dwell)
    n = int(round(t_end / h)) + 1
    t = np.arange(n) * h
    _, _, qdd = reference(t, dq, T, dwell)
    f = qdd.copy()
    f[m:] -= qdd[:-m]
    if model is not None:
        tg = np.arange(int(np.ceil(t_end / 1e-3)) + 1) * 1e-3
        qr, qdr, _ = reference(tg, dq, T, dwell)
        fg = np.zeros((len(tg), len(dq)))
        for k in range(len(tg)):
            if np.any(qdr[k] != 0.0):
                fg[k] = model.gravity_term(q0 + qr[k], qdr[k], tau_d)
        f += np.array([np.interp(t, tg, fg[:, j]) for j in range(len(dq))]).T
    e = np.zeros((n, len(dq)))
    ed = np.zeros_like(e)
    for s in range(0, n - 1, m):
        r = min(s + m, n - 1)
        edd = f[s:r].copy()
        if s >= m:
            edd -= Kd * ed[s - m:r - m] + Kp * e[s - m:r - m]
        ed[s + 1:r + 1] = ed[s] + h * np.cumsum(edd, axis=0)
        e[s + 1:r + 1] = e[s] + h * np.cumsum(ed[s + 1:r + 1], axis=0)
    return t, e


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    d, mjcf = sys.argv[1], sys.argv[2]
    meta = read_meta(os.path.join(d, 'joint_test_meta.txt'))
    names = meta['joints']
    with open(os.path.join(d, 'joint_test.csv')) as f:
        head = f.readline().strip().split(',')
    data = np.loadtxt(os.path.join(d, 'joint_test.csv'), delimiter=',', skiprows=1)
    col = {h: i for i, h in enumerate(head)}
    e_sim = np.array([data[:, col['e:%s' % n]] for n in names]).T
    dq = np.array([meta['move'].get(n, 0.0) for n in names])
    Kp = np.array([meta['kp'][n] for n in names])
    Kd = np.array([meta['kd'][n] for n in names])
    q0 = np.array([meta['qr'][n] for n in names])
    T, dwell = meta['T'], meta['dwell']
    _, e1 = predict(dq, T, dwell, Kp, Kd, TAU_D)
    model = Model(mjcf, names)
    _, e2 = predict(dq, T, dwell, Kp, Kd, TAU_D, model=model, q0=q0)
    sim = np.abs(e_sim).max(axis=0) * 1e3
    p1 = np.abs(e1).max(axis=0) * 1e3
    p2 = np.abs(e2).max(axis=0) * 1e3
    print('humanoid joint test: T %.1f s, dwell %.1f s, tau_d %.1f ms; largest |e| [mrad]' % (T, dwell, TAU_D * 1e3))
    print('%-28s %8s %8s %8s %8s %7s' % ('joint (moving)', 'dq rad', 'sim.', 'dde', 'dde+g', 'ratio'))
    ratios = []
    for i, n in enumerate(names):
        if dq[i] == 0.0:
            continue
        r = sim[i] / p2[i] if p2[i] > 1e-3 else float('nan')
        if p2[i] > 1e-3:
            ratios.append(r)
        print('%-28s %8.2f %8.4f %8.4f %8.4f %7.2f' % (n, dq[i], sim[i], p1[i], p2[i], r))
    held = [(sim[i], p2[i], n) for i, n in enumerate(names) if dq[i] == 0.0]
    held.sort(reverse=True)
    print('held joints, largest |e| [mrad] (simulation / dde+g): ' +
          ', '.join('%s %.4f / %.4f' % (n, s, p) for s, p, n in held[:6]))
    ok = all(r <= 1.5 for r in ratios)
    print('ratios %.2f to %.2f: %s (criterion: at most 1.5 where the prediction exceeds 1 urad)' % (
        min(ratios), max(ratios), 'PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
