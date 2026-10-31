#!/usr/bin/env python3
"""identify_rotor.py - the rotor inertia B_ii of one joint from its sine test (JOINT TEST 7; paper, Sec. IV-A,
eq. (6), Table 7), by least squares.

The joint followed q0 + A sin(2 pi f t), on a slow sweep at the wrist, under CTC (with the friction feedforward)
while the others were held. The torque that the rigid-body model does not explain is fitted by the rotor
inertia, a Coulomb friction torque, and a viscous coefficient:

    tau_i - [M(q) qdd]_i - h_i(q, qd) = B_ii qdd_i + f_c sign(qd_i) + f_v qd_i

tau_i: the commanded torque (column tau<i>, after the torque limit); [M(q) qdd]_i with the whole row of M(q)
(columns mrow<k>, the URDF without B), sum_k M_ik qdd_k, since the held joints still move a little;
h_i = tau_g<i> + tau_c<i>; qdd: the measured velocities differentiated with a zero-phase Savitzky-Golay filter
(41 samples, cubic). Only samples with |qd_i| >= --qd-min enter the fit (the joint slides there), and the
torque of period k is paired with the state logged --shift periods later, three by default: two from a state
to the torque that acts on it, plus the period of the pipeline (the torque computed in period k is sent in
period k + 1, indy7/src/TaskSlots.h); the simulation reproduces this. With --tau-act, the torque that the drives report
(tau_act<i>, robot only; it comes with the state of the same frame, shift 2) is fitted instead, as a check:
the commanded torque leaves out the dynamics of the current loop.

  python3 tools/identify_rotor.py jt_sine_*.csv [...] [--qd-min 0.05] [--window 41] [--shift 3] [--tau-act]
                                  [--pool 4,5] [--apply] [--cfg indy7/INDY7.cfg]

Every file is fitted on its own (joint from its .json, test "sine"), then the files of each joint together.
The acceptance is printed at the end: the runs of a joint agree within 10 % (spread of the per-run B over
the joint's B), and J4 and J5 (the same drive module) within 20 %. R^2 is printed but not used: at the wrist
the friction terms alone explain most of the variance. --pool 4,5 (1-based) fits one B for the files of those
joints, with a Coulomb and a viscous term per joint: the fallback when J4 and J5 fail on their own.
--apply writes the B of every joint that passed the acceptance into ROTOR_INERTIA_<i> of the cfg
(tools/cfgset.py; the changed lines are printed); restart the controller afterwards. Without a J6 file,
ROTOR_INERTIA_5 = B5 (J6 then runs its own sine test from it) once J5 has passed; with J6 files, B6 only if
it passes (otherwise the value stays). When ROTOR_INERTIA_5 is written, the J6 entry of JOINT_TEST_SLOW_DQ
becomes 0.9 if it is 0 (the controller moves J6 only with its rotor inertia). A joint that fails is not
written (rerun its sine test, e.g. at 5-6 Hz); nothing is written if no joint passes or a key is missing.
The simulated arm has the known armature SIM_ARMATURE (default ROTOR_INERTIA_n): the sine test in the
simulation checks that the identification recovers it.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cfgset  # noqa: E402


def sg_derivative(y, window, order, dt):
    """first derivative by a centered Savitzky-Golay filter (symmetric kernel: zero phase); NaN at the edges"""
    half = window // 2
    x = np.arange(-half, half + 1) * dt
    A = np.vander(x, order + 1, increasing=True)            # columns 1, x, x^2, ...
    coef = np.linalg.pinv(A)[1]                              # the row that gives the slope at x = 0
    d = np.convolve(y, coef[::-1], mode='same')
    d[:half] = np.nan
    d[-half:] = np.nan
    return d


def load(path):
    base = os.path.splitext(path)[0]
    with open(base + '.json') as f:
        meta = json.load(f)
    if meta.get('test') != 'sine':
        raise ValueError('%s: test "%s", not a sine test' % (path, meta.get('test')))
    with open(path) as f:
        head = next(csv.reader(f))
    data = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    col = {h: k for k, h in enumerate(head)}
    return meta, data, col


def regressors(meta, data, col, a):
    j = int(meta['sine_joint']) - 1
    for name in ('qd%d' % j, 'tau%d' % j, 'tau_g%d' % j, 'tau_c%d' % j, 'mii%d' % j):
        if name not in col:
            raise ValueError('column %s missing' % name)
    dt = float(meta['period'])
    qd = data[:, col['qd%d' % j]]
    qdd = sg_derivative(qd, a.window, 3, dt)
    tau = data[:, col[('tau_act%d' if a.tau_act else 'tau%d') % j]]
    # the commanded torque acts 2 periods after its state, plus the period of the pipeline in which the
    # EtherCAT task sends it; the torque the drives report (--tau-act) comes with the state of the same frame
    pipe = int(meta.get('pipeline_periods', 1))
    s = a.shift if a.shift is not None else 2 + (0 if a.tau_act else pipe)
    meta['_shift'] = s
    n = len(qd) - s
    tau_k = tau[:n]                                          # the torque of period k ...
    qd_s, qdd_s = qd[s:], qdd[s:]                            # ... acts on the state logged at k + shift
    h_s = data[s:, col['tau_g%d' % j]] + data[s:, col['tau_c%d' % j]]
    m_s = data[s:, col['mii%d' % j]]
    nj = len([h for h in col if h.startswith('qd') and h[2:].isdigit()])
    if 'mrow0' in col and np.any(data[:, col['mrow%d' % j]] != 0.0):
        # [M(q) qdd]_j with the whole row: the held joints still move a little and couple into joint j
        mq = np.zeros(n)
        ok_all = np.ones(n, bool)
        for k in range(nj):
            qdd_k = sg_derivative(data[:, col['qd%d' % k]], a.window, 3, dt)[s:]
            mq += data[s:, col['mrow%d' % k]] * np.nan_to_num(qdd_k)
            ok_all &= np.isfinite(qdd_k)
    else:
        mq = m_s * np.nan_to_num(qdd_s)
        ok_all = np.isfinite(qdd_s)
    y = tau_k - mq - h_s
    X = np.column_stack([qdd_s, np.sign(qd_s), qd_s])
    ok = ok_all & np.isfinite(qdd_s) & (np.abs(qd_s) >= a.qd_min) & (m_s > 0.0)
    return j, X[ok], y[ok], meta


def fit(X, y):
    th, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ th
    r2 = 1.0 - (r @ r) / max(((y - y.mean()) @ (y - y.mean())), 1e-30)
    # standard errors of the parameters from the residual variance
    dof = max(len(y) - X.shape[1], 1)
    cov = (r @ r) / dof * np.linalg.pinv(X.T @ X)
    return th, np.sqrt(np.diag(cov)), r2, np.sqrt(r @ r / len(y))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--qd-min', type=float, default=0.05, help='[rad/s] samples in which the joint slides')
    ap.add_argument('--window', type=int, default=41, help='Savitzky-Golay window [samples], odd')
    ap.add_argument('--shift', type=int, default=None,
                    help='[periods] from a torque to the state on which it acts (default: 3 for the commanded torque, '
                         '2 with --tau-act)')
    ap.add_argument('--tau-act', action='store_true', help='fit the torque the drives report (robot)')
    ap.add_argument('--pool', default='', help='joints (1-based, e.g. 4,5) fitted with one common B')
    ap.add_argument('--spread', type=float, default=10.0, help='[%%] acceptance: spread of the runs of a joint')
    ap.add_argument('--j45', type=float, default=20.0, help='[%%] acceptance: difference of J4 and J5')
    ap.add_argument('--apply', action='store_true', help='write the accepted B into the cfg')
    ap.add_argument('--cfg', default=cfgset.CFG_MAIN, help='the cfg (default indy7/INDY7.cfg)')
    a = ap.parse_args()
    if a.window % 2 == 0 or a.window < 7:
        sys.exit('identify_rotor: --window must be odd and at least 7')
    per_joint, per_run = {}, {}
    print('%-46s %3s %7s %10s %9s %9s %8s %6s' % ('log', 'J', 'samples', 'B kg m^2', 'f_c N m', 'f_v N m s',
                                                   'rms N m', 'R^2'))
    for p in a.logs:
        try:
            j, X, y, meta = regressors(*load(p), a)
        except (OSError, ValueError, KeyError) as ex:
            print('identify_rotor: skipped %s (%s)' % (p, ex))
            continue
        if len(y) < 100:
            print('identify_rotor: %s: only %d samples above %.3f rad/s' % (p, len(y), a.qd_min))
            continue
        th, se, r2, rms = fit(X, y)
        b_cfg = (meta.get('rotor_inertia') or [0.0] * 6)[j]
        print('%-46s %3d %7d %10.5f %9.3f %9.3f %8.4f %6.3f  (+-%.5f; B in the controller %.5f; shift %d)' % (
            os.path.basename(p)[:46], j + 1, len(y), th[0], th[1], th[2], rms, r2, se[0], b_cfg, meta['_shift']))
        per_joint.setdefault(j, []).append((X, y))
        per_run.setdefault(j, []).append(th[0])
    if not per_joint:
        return 1
    print('\nper joint, all files together:')
    joint_b = {}
    for j in sorted(per_joint):
        X = np.vstack([x for x, _ in per_joint[j]])
        y = np.concatenate([v for _, v in per_joint[j]])
        th, se, r2, rms = fit(X, y)
        joint_b[j] = th[0]
        print('  J%d: B = %.5f +- %.5f kg m^2, f_c %.3f N m, f_v %.3f N m s, R^2 %.3f, %d files -> '
              'ROTOR_INERTIA_%d=%.5f' % (j + 1, th[0], se[0], th[1], th[2], r2, len(per_joint[j]), j, th[0]))
    print('\nacceptance (runs within %g %%, J4 and J5 within %g %%; R^2 not used):' % (a.spread, a.j45))
    accepted = {}
    for j in sorted(per_joint):
        b = per_run[j]
        if len(b) < 2:
            print('  J%d: %d run - repeat it (at least 2 runs)' % (j + 1, len(b)))
            continue
        spread = 100.0 * (max(b) - min(b)) / abs(joint_b[j]) if joint_b[j] != 0.0 else float('inf')
        verdict = 'OK' if joint_b[j] > 0.0 and spread <= a.spread else \
            ('B <= 0 - not usable' if joint_b[j] <= 0.0 else 'NOT accepted')
        accepted[j] = verdict == 'OK'
        print('  J%d: runs %s -> spread %.1f %% -> %s' % (j + 1, ' '.join('%.5f' % v for v in b), spread, verdict))
    if 3 in joint_b and 4 in joint_b:
        mean = 0.5 * (joint_b[3] + joint_b[4])
        d = 100.0 * abs(joint_b[3] - joint_b[4]) / abs(mean) if mean != 0.0 else float('inf')
        print('  J4 vs J5: %.5f vs %.5f -> %.1f %% -> %s' % (joint_b[3], joint_b[4], d,
                                                          'OK' if d <= a.j45 else 'NOT accepted (try --pool 4,5)'))
        if d > a.j45:
            # the same drive module: one of the two is wrong, neither is used
            for j in (3, 4):
                if accepted.get(j):
                    accepted[j] = False
    if a.pool:
        pool = sorted({int(v) - 1 for v in a.pool.split(',') if v.strip()})
        have = [j for j in pool if j in per_joint]
        if len(have) < 2:
            print('\n--pool %s: files of fewer than two of these joints' % a.pool)
            return apply_b(joint_b, accepted, a) if a.apply else 0
        rows_x, rows_y = [], []
        for idx, j in enumerate(have):
            for X, y in per_joint[j]:
                Z = np.zeros((len(y), 1 + 2 * len(have)))
                Z[:, 0] = X[:, 0]                       # the common B
                Z[:, 1 + 2 * idx] = X[:, 1]             # Coulomb of this joint
                Z[:, 2 + 2 * idx] = X[:, 2]             # viscous of this joint
                rows_x.append(Z)
                rows_y.append(y)
        th, se, r2, rms = fit(np.vstack(rows_x), np.concatenate(rows_y))
        print('\npooled %s: B = %.5f +- %.5f kg m^2 (one B, friction per joint), R^2 %.3f -> %s' % (
            '+'.join('J%d' % (j + 1) for j in have), th[0], se[0], r2,
            ', '.join('ROTOR_INERTIA_%d=%.5f' % (j, th[0]) for j in have)))
    if a.apply:
        return apply_b(joint_b, accepted, a)
    return 0


def apply_b(joint_b, accepted, a):
    """ROTOR_INERTIA_<i> of the cfg; 0 written, 2 nothing written"""
    print('\n--apply:')
    items, done = [], set()
    for j in range(6):
        if accepted.get(j):
            items.append(('ROTOR_INERTIA_%d' % j, '%.5f' % joint_b[j]))
            done.add(j)
        elif j in joint_b:
            print('  J%d: not accepted - not written (rerun its sine test, e.g. at 5-6 Hz; J4, J5: or --pool 4,5 by '
                  'hand)' % (j + 1))
    if 5 not in joint_b and accepted.get(4):
        # no J6 file yet: J6 starts from the B of J5 (the same drive module family), then its own sine test
        items.append(('ROTOR_INERTIA_5', '%.5f' % joint_b[4]))
        done.add(5)
        print('  J6: no sine test yet: ROTOR_INERTIA_5 = B5 = %.5f (then sine_cfg.py 6, u 7 x 3)' % joint_b[4])
    if not items:
        print('  no joint passed - nothing written')
        return 2
    if 5 in done:
        dq = cfgset.values(a.cfg, ['JOINT_TEST_SLOW_DQ'])['JOINT_TEST_SLOW_DQ']
        v = dq.split() if dq else []
        if len(v) >= 6 and float(v[5]) == 0.0:
            v[5] = '0.9'
            items.append(('JOINT_TEST_SLOW_DQ', ' '.join(v)))
            print('  J6 joins the slow test: JOINT_TEST_SLOW_DQ J6 0 -> 0.9 rad')
    ok, res = cfgset.apply_files([a.cfg], items, tag='identify_rotor --apply')
    if not ok:
        print('  ' + '\n  '.join(res) + '\n  nothing written')
        return 2
    print('  ROTOR_INERTIA written for %s; restart the controller'
          % ', '.join('J%d' % (j + 1) for j in sorted(done)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
