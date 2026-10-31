#!/usr/bin/env python3
"""jt_predict.py - the tracking error that the delay alone causes under CTC (paper, Sec. IV-B, eq. (8)).

The JOINT TEST reference: from q0 at rest to q0 + dq in T (a quintic polynomial from rest to rest), a dwell,
back to q0 in T, a dwell. With a model that matches the arm and no friction, CTC leaves only the error of the
delay tau_d between the measurement and the torque: on average (D + 1/2) T_s in the simulation (the torque
acts D steps after the state; indy7/src/sim/SimArm.h), plus one period of the pipeline of the tasks (the
torque computed in a period is sent in the next one; indy7/src/TaskSlots.h): (D + 3/2) T_s = 3.5 ms, the
tau_d of Table 7. Predictions of the logged error e = q_d - (the measurement the controller used), per joint:

  dde    the delayed feedback alone: the reference and the feedback are tau_d late,
             e''(t) + Kd e'(t - tau_d) + Kp e(t - tau_d) = qdd_d(t) - qdd_d(t - tau_d)
  dde+g  eq. (8): the same plus the gravity torque, which CTC computes at the posture tau_d old: it is wrong
         by about (dg/dq) qd tau_d, which the joints receive as the acceleration M^-1 (dg/dq) qd tau_d,
             ... = qdd_d(t) - qdd_d(t - tau_d) + tau_d M(q_d)^-1 (dg/dq)(q_d) qd_d(t)
         (M + B and dg/dq from the MuJoCo model at the reference, which starts at q0). At the light wrist,
         this term dominates; the delayed inertia and Coriolis terms are small (checked against the full
         simulation, ratios 0.8-1.0 on all joints).

Usage:
  python3 tools/jt_predict.py [--cfg indy7/INDY7.cfg] [--test fast|slow|both] [--q0 "q1 .. q6"]
                              [--csv-prefix PRED]
Prints the largest |e| per joint in mrad. dde+g needs the mujoco package and the model (--mjcf, default
indy7/models/indy7_mujoco.xml); q0 defaults to SIM_INITIAL_Q of the cfg.
"""
import argparse
import os
import sys

import numpy as np

DOF = 6
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CFG = os.path.join(HERE, '..', 'indy7', 'INDY7.cfg')
DEFAULT_MJCF = os.path.join(HERE, '..', 'indy7', 'models', 'indy7_mujoco.xml')

# the defaults of the cfg (JOINT_TEST_* keys), used when a key is missing
DEFAULT_TESTS = {
    'fast': {'dq': [0.3, 0.3, 0.3, 0.0, 0.0, 0.0], 'T': 1.0},
    'slow': {'dq': [0.9, 0.6, 0.9, 0.9, 0.9, 0.0], 'T': 3.0},
}
DEFAULT_DWELL = 1.0
DEFAULT_Q0 = [0.0, 0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0]


# ---------------------------------------------------------------------------------------------- cfg
def cfg_get(path, section, key, default=None):
    """Same lookup as GetPrivateProfileString of the controller: the first line after [section] that
    starts with key (case-insensitive), value after the last '=', comment after ';' removed."""
    try:
        with open(path) as f:
            lines = f.readlines()
    except OSError:
        return default
    tag = '[%s]' % section.lower()
    start = None
    for i, line in enumerate(lines):
        if line.lower().startswith(tag):
            start = i + 1
            break
    if start is None:
        return default
    for line in lines[start:]:
        if line.lower().startswith(key.lower()):
            val = line.rsplit('=', 1)[-1] if '=' in line else ''
            return val.split(';', 1)[0].strip()
    return default


def _num(v, default):
    try:
        return float(v.split()[0]) if v else default
    except ValueError:
        return default


def _vec(v, default):
    if not v:
        return list(default)
    try:
        x = [float(s) for s in v.split()]
    except ValueError:
        return list(default)
    return (x + list(default)[len(x):])[:len(default)]


def pipeline_periods(path):
    """1 if the cfg runs the control law in a Dynamics task of its own (indy7/src/TaskSlots.h: the torque of a
    period is sent in the next one), else 0"""
    n = int(_num(cfg_get(path, 'RT_TASKS', 'NO_OF_TASKS'), 0))
    for k in range(n):
        if 'dynamics' in (cfg_get(path, 'TASK%d' % k, 'NAME') or '').lower():
            return 1
    return 0


def cfg_params(path):
    """Gains, tests, timing, and the initial posture of the simulation, from INDY7.cfg."""
    g = lambda k, d: _num(cfg_get(path, 'SYSTEM', k), d)
    p = {
        'Kp': [g('KP_%d' % i, 0.0) for i in range(DOF)],
        'Kd': [g('KD_%d' % i, 0.0) for i in range(DOF)],
        'B': [g('ROTOR_INERTIA_%d' % i, 0.0) for i in range(DOF)],
        'dwell': g('JOINT_TEST_DWELL', DEFAULT_DWELL),
        'a': int(g('SIM_STATE_DELAY_STEPS', 2)),
        'D': int(g('SIM_TORQUE_DELAY_STEPS', 2)),
        'Ts': 1e-9 * _num(cfg_get(path, 'TASK0', 'TASK_PERIOD'), 1e6),
        'P': pipeline_periods(path),
        'q0': _vec(cfg_get(path, 'SYSTEM', 'SIM_INITIAL_Q'), DEFAULT_Q0),
        'tests': {},
    }
    for name, d in DEFAULT_TESTS.items():
        key = name.upper()
        p['tests'][name] = {'dq': _vec(cfg_get(path, 'SYSTEM', 'JOINT_TEST_%s_DQ' % key), d['dq']),
                            'T': g('JOINT_TEST_%s_T' % key, d['T'])}
    return p


# ---------------------------------------------------------------------------------------- reference
def reference(t, dq, T, dwell):
    """JOINT TEST reference relative to q0 at times t [s]: (q_d, qd_d, qdd_d), each len(t) x len(dq)."""
    t = np.asarray(t, dtype=float)
    dq = np.asarray(dq, dtype=float)
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


def test_duration(T, dwell):
    return 2 * T + 2 * dwell


# ------------------------------------------------------------------------------------------ gravity
class Model:
    """M(q) and dg/dq of the MuJoCo model of the simulation (the same numbers as RBDL's to 1e-8,
    tools/model_check.cpp)."""

    def __init__(self, mjcf=DEFAULT_MJCF, armature=None):
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(mjcf)
        self.d = mujoco.MjData(self.m)
        self.adr = []
        for i in range(DOF):
            j = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, 'joint%d' % i)
            self.adr.append((self.m.jnt_qposadr[j], self.m.jnt_dofadr[j]))
        self.set_armature(armature)

    def set_armature(self, armature):
        """the rotor inertias B [kg m^2] on the diagonal of M, as in the controller and the simulation (paper,
        eq. (4)); None or zeros: the URDF alone"""
        b = list(armature) if armature is not None else [0.0] * DOF
        for (_, va), v in zip(self.adr, b):
            self.m.dof_armature[va] = max(float(v), 0.0)
        self.armature = tuple(round(float(v), 9) for v in b[:DOF])

    def _set(self, q):
        self.d.qpos[:] = 0
        self.d.qvel[:] = 0
        for (qa, _), v in zip(self.adr, q):
            self.d.qpos[qa] = v
        self.mj.mj_forward(self.m, self.d)

    def g(self, q):
        self._set(q)
        return np.array([self.d.qfrc_bias[va] for _, va in self.adr])

    def M(self, q):
        self._set(q)
        full = np.zeros((self.m.nv, self.m.nv))
        e, col = np.zeros(self.m.nv), np.zeros(self.m.nv)
        for j in range(self.m.nv):              # column by column: mj_mulM has the same signature in every MuJoCo 3
            e[j] = 1.0
            self.mj.mj_mulM(self.m, self.d, col, e)
            e[j] = 0.0
            full[:, j] = col
        idx = [va for _, va in self.adr]
        return full[np.ix_(idx, idx)]

    def gravity_term(self, q, qd, tau_d, h=1e-6):
        """tau_d M(q)^-1 (dg/dq)(q) qd"""
        G = np.zeros((DOF, DOF))
        for j in range(DOF):
            e = np.zeros(DOF)
            e[j] = h
            G[:, j] = (self.g(q + e) - self.g(q - e)) / (2 * h)
        return tau_d * np.linalg.solve(self.M(q), G @ qd)


def gravity_forcing(model, q0, dq, T, dwell, tau_d, t):
    """The term of dde+g on the times t (computed every 1 ms of motion and interpolated)."""
    t_end = t[-1]
    tg = np.arange(int(np.ceil(t_end / 1e-3)) + 1) * 1e-3
    qr, qdr, _ = reference(tg, dq, T, dwell)
    fg = np.zeros((len(tg), DOF))
    q0 = np.asarray(q0, dtype=float)
    for k in range(len(tg)):
        if np.any(qdr[k] != 0.0):
            fg[k] = model.gravity_term(q0 + qr[k], qdr[k], tau_d)
    return np.array([np.interp(t, tg, fg[:, j]) for j in range(DOF)]).T


# --------------------------------------------------------------------------------------- prediction
def predict(dq, T, dwell, Kp, Kd, tau_d, h=1e-5, model=None, q0=None, t_end=None):
    """Integrate the delay equation (semi-implicit Euler, step h, tau_d a multiple of h), with e = 0
    before the start. With model and q0, the gravity term is included (dde+g). Returns t, e."""
    m = int(round(tau_d / h))
    if m < 1 or abs(m * h - tau_d) > 1e-12:
        raise ValueError('tau_d must be a positive multiple of h')
    if t_end is None:
        t_end = test_duration(T, dwell)
    n = int(round(t_end / h)) + 1
    t = np.arange(n) * h
    _, _, qdd = reference(t, dq, T, dwell)
    f = qdd.copy()
    f[m:] -= qdd[:-m]
    if model is not None:
        f += gravity_forcing(model, q0, dq, T, dwell, tau_d, t)
    Kp = np.asarray(Kp, dtype=float)
    Kd = np.asarray(Kd, dtype=float)
    e = np.zeros((n, len(dq)))
    ed = np.zeros_like(e)
    # blocks of m steps: inside a block, the delayed terms are already known
    for s in range(0, n - 1, m):
        r = min(s + m, n - 1)
        edd = f[s:r].copy()
        if s >= m:
            edd -= Kd * ed[s - m:r - m] + Kp * e[s - m:r - m]
        ed[s + 1:r + 1] = ed[s] + h * np.cumsum(edd, axis=0)
        e[s + 1:r + 1] = e[s] + h * np.cumsum(ed[s + 1:r + 1], axis=0)
    return t, e


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cfg', default=DEFAULT_CFG)
    ap.add_argument('--mjcf', default=DEFAULT_MJCF)
    ap.add_argument('--test', default='both', choices=['fast', 'slow', 'both'])
    ap.add_argument('--q0', help='start posture [rad], default SIM_INITIAL_Q of the cfg')
    ap.add_argument('--tau-d', type=float, default=None, help='[s], default (D + 1/2 + P) T_s')
    ap.add_argument('--pipeline', type=int, default=None,
                    help='[periods] P that the tasks add (default: 1 if the cfg has a Dynamics task, else 0)')
    ap.add_argument('--h', type=float, default=1e-5, help='integration step [s]')
    ap.add_argument('--no-gravity', action='store_true', help='only dde (no mujoco needed)')
    ap.add_argument('--csv-prefix', help='write PREFIX_<test>.csv: t and e of both predictions, every 1 ms')
    a = ap.parse_args()
    p = cfg_params(a.cfg)
    pipe = a.pipeline if a.pipeline is not None else p['P']
    tau_d = a.tau_d if a.tau_d is not None else (p['D'] + 0.5 + pipe) * p['Ts']
    q0 = [float(x) for x in a.q0.split()] if a.q0 else p['q0']
    model = None
    if not a.no_gravity:
        try:
            model = Model(a.mjcf, armature=p['B'])
        except Exception as ex:          # no mujoco: dde only
            print('jt_predict: no dde+g (%s)' % ex)
    print('cfg %s: Kp %s, Kd %s; tau_d %.4g s (pipeline +%d period); q0 %s' % (os.path.relpath(a.cfg), p['Kp'], p['Kd'], tau_d, pipe,
                                                        np.round(q0, 4).tolist()))
    for name in (['fast', 'slow'] if a.test == 'both' else [a.test]):
        tst = p['tests'][name]
        t, e1 = predict(tst['dq'], tst['T'], p['dwell'], p['Kp'], p['Kd'], tau_d, a.h)
        print('%s test: dq %s rad, T %.2f s, dwell %.2f s' % (name, tst['dq'], tst['T'], p['dwell']))
        print('  max |e| [mrad]  dde   ' + ' '.join('%7.4f' % (1e3 * x) for x in np.abs(e1).max(0)))
        e2 = None
        if model is not None:
            _, e2 = predict(tst['dq'], tst['T'], p['dwell'], p['Kp'], p['Kd'], tau_d, a.h, model, q0)
            print('  max |e| [mrad]  dde+g ' + ' '.join('%7.4f' % (1e3 * x) for x in np.abs(e2).max(0)))
        if a.csv_prefix:
            step = int(round(1e-3 / a.h))
            cols = [t[::step], e1[::step]] + ([e2[::step]] if e2 is not None else [])
            hdr = 't,' + ','.join('e_dde%d' % j for j in range(DOF))
            if e2 is not None:
                hdr += ',' + ','.join('e_ddeg%d' % j for j in range(DOF))
            out = '%s_%s.csv' % (a.csv_prefix, name)
            np.savetxt(out, np.column_stack(cols), delimiter=',', header=hdr, comments='', fmt='%.6e')
            print('  wrote', out)


if __name__ == '__main__':
    sys.exit(main())
