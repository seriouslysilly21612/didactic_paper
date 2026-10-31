#!/usr/bin/env python3
"""hl_analyze.py - results of the high-level tests (paper, Sec. IV-C and V-B: Table 11, Figs. 6 and 7).

Reads the logs of the controller (hl_*.csv with their .json, in the cfg JOINT_TEST_LOG_DIR) and prints
  1. every run: result, goals accepted/rejected, lead-bound resets, final distance, timing faults
  2. the reference check: the reference contract (Sec. IV-C) in every period of every run -
     q_d the integral of qd_d (C2); |qd_d|, |qdd_d|, and the change of qdd_d per period (jerk) within the
     limits of the generator, except right after a lead-bound reset, and q_d inside the joint limits (C3);
     the lead |q_d - q| below ell, except at a reset (C4); the time of the high-level step and no
     allocation (C5)
  3. the results of the high-level tests:
     (1) random goals (test 1, cfg HL_RAND_*, the last completed run of every goal): goals reached, final
         distance (median and largest), time to reach the goal against the planned duration (+ the 0.2 s of
         the rest condition, largest deviation), IK rejections, goals not at rest after 20 s; with the fixed
         goal (HL_RAND_GOALS=0): final distance, spread of the final positions over the trials, time to reach
     (2) random goal jumps (test 2, cfg HL_RAND_JUMP*, the last completed run of every jump): jumps
         completed, final distance to the new goal (median and largest), time to reach it against the plan
         made at the jump, largest change of qdd_d per period (prediction jerk_max T_s) and of the torque
         (prediction: the largest change per period of the torque of the reference,
         (M(q_d) + B) qdd_d + h(q_d, qd_d)), and the largest ratio of the two over the runs; with the fixed
         jump (HL_RAND_JUMPS=0): the two changes and the final distance
     (3) residual motion (peak-to-peak motion of the tool in the last second) with delta and without
     all: largest lead, lead-bound resets
     distances and deviations as median / 95th percentile / largest
  4. Table 11, per platform: the final distance of every random goal of test 1 over its runs (mean, SD,
     largest; the goal p_g in the base frame), and with --static-err the estimate set before the runs,
         Est. = e_tol + || |J_p(q_g)| e_bar ||,
     the IK tolerance plus the errors at rest of the joint test carried to the tool, with q_g the IK solution
     of the goal (the target of the generator at the end of the run), |J_p| the element-wise absolute value of
     the position Jacobian of the tool point, and e_bar_i the larger magnitude of the mean errors at rest of
     joint i at q1 and at q0 (tools/jt_analyze.py prints it; Table 10)
  5. the step baseline (cfg HL_STEP_BASELINE=1, simulation only: the IK solution as a step, without the
     generator and the lead bound) against the high-level controller on the same jumps: largest change of
     the torque per period, share of the periods until rest with a torque at its limit, largest joint speed,
     largest lead, time to rest, final distance, largest tool speed, and length of the tool path
     (median / 95th percentile / largest over the jumps that both ran); the contract of 2 does not apply to it
Runs with timing faults, a controller fault or a drive dropout are listed but left out of 2-5 (none in the
simulation, see jt_analyze.py). Only the per-run results are kept in memory (1000 runs fit); --plot keeps
the logs as well.
Usage:
  python3 tools/hl_analyze.py [LOG_OR_DIR ...] [--static-err "e1 ... e6"] [--plot OUT_PREFIX]
  (default indy7/jt_logs; --static-err in mrad)
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DIR = os.path.join(HERE, '..', 'indy7', 'jt_logs')
DEFAULT_MJCF = os.path.join(HERE, '..', 'indy7', 'models', 'indy7_mujoco.xml')
DEFAULT_CFG = os.path.join(HERE, '..', 'indy7', 'INDY7.cfg')
B_MAX = 2.0             # [kg m^2] the controller bounds the rotor inertias it uses (Table 7, note a)
DOF = 6
F_ACCEPTED, F_CONVERGED, F_REJECTED, F_ACTIVE, F_NEW, F_REST, F_GOAL = 1, 2, 4, 8, 16, 32, 64
_MODELS = {}
_ARMATURE = None        # rotor inertias B [kg m^2] of the controller (--rotor-inertia)


def load(path, keep=True, mjcf=None):
    """one run: its meta data and the per-run results; the arrays of the log only with keep (--plot)"""
    with open(os.path.splitext(path)[0] + '.json') as f:
        meta = json.load(f)
    with open(path) as f:
        header = f.readline().strip().split(',')
    data = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2,
                      converters=lambda v: float(v) if v.strip() else np.nan)   # empty: no goal yet
    cols = {name: data[:, i] for i, name in enumerate(header)}
    g = lambda name: np.column_stack([cols['%s%d' % (name, i)] for i in range(DOF)])
    r = {'meta': meta, 'path': path, 't': cols['t'], 'flags': cols['flags'].astype(int),
         'otg': cols['otg'].astype(int), 'reset': cols['lead_reset'].astype(int), 'hl_us': cols['hl_us'],
         'otg_dur': cols['otg_dur'],
         'goal': np.column_stack([cols['goal_x'], cols['goal_y'], cols['goal_z']]),
         'acc': np.column_stack([cols['acc_x'], cols['acc_y'], cols['acc_z']]),
         'tool': np.column_stack([cols['tool_x'], cols['tool_y'], cols['tool_z']]),
         'clamp': cols['clamp'].astype(int) if 'clamp' in cols else np.zeros(len(data), dtype=int)}
    for name in ('target', 'q_ref', 'qd_ref', 'qdd_ref', 'q', 'qd', 'tau'):
        r[name] = g(name)
    del cols, data
    if meta.get('step_baseline'):
        meta['test'] = meta['test'] + '-step'      # kept apart from the tests of the controller
    # per-run results, so that the arrays can be dropped
    r['n'] = len(r['t'])
    r['contract'] = contract(r)
    r['planned'] = planned_reach(r)
    r['resid'] = residual(r)
    r['tool_last'] = r['tool'][-1].copy()
    r['goal_last'] = r['acc'][-1].copy()            # the accepted goal p_g
    r['q_goal'] = r['target'][-1].copy()            # its IK solution q_g (the target of the generator)
    r['motion'] = motion_stats(r)
    r['pred_dtau'] = ref_torque_change([r], mjcf) if (meta['test'] == 'jump' and mjcf) else None
    if not keep:
        for name in ('t', 'flags', 'otg', 'reset', 'hl_us', 'otg_dur', 'goal', 'acc', 'tool', 'clamp',
                     'target', 'q_ref', 'qd_ref', 'qdd_ref', 'q', 'qd', 'tau'):
            del r[name]
    return r


def motion_stats(r):
    """the quantities of the step baseline (section 5), for any run: largest change of the torque per period,
    share of the periods until rest with a torque at its limit, largest joint speed (per joint), largest lead,
    largest tool speed, length of the tool path"""
    m = r['meta']
    t, tool = r['t'], r['tool']
    rest = m.get('rest_time', -1)
    upto = (t <= rest) if rest is not None and rest >= 0 else np.ones(len(t), dtype=bool)
    v = np.linalg.norm(np.gradient(tool, t, axis=0), axis=1) if len(t) > 1 else np.zeros(1)
    return {'dtau': np.abs(np.diff(r['tau'], axis=0)).max() if len(t) > 1 else 0.0,
            'clamp_share': float((r['clamp'][upto] != 0).mean()) if upto.any() else 0.0,
            'qd_max': np.abs(r['qd']).max(axis=0),
            'lead': np.abs(r['q_ref'] - r['q']).max(),
            'tool_vmax': float(v.max()),
            'path': float(np.linalg.norm(np.diff(tool, axis=0), axis=1).sum()) if len(t) > 1 else 0.0}


def platform(m):
    if m['platform'] != 'simulation':
        return 'robot'
    return 'sim+fr' if m.get('sim_friction') else 'sim'


def contract(r):
    """the contract in every period; returns a dict of the worst values and violation counts"""
    m = r['meta']
    T = m['period']
    qr, qdr, qddr, q = r['q_ref'], r['qd_ref'], r['qdd_ref'], r['q']
    reset = r['reset']
    after_reset = np.zeros(len(reset), dtype=int)
    after_reset[1:] = reset[:-1]          # the reference jumps in the period after a reset
    ok_j = np.ones((len(reset) - 1, DOF), dtype=bool)
    for i in range(DOF):
        ok_j[:, i] = (after_reset[1:] & (1 << i)) == 0
    integ = np.abs(qr[1:] - qr[:-1] - 0.5 * T * (qdr[1:] + qdr[:-1]))
    jerk = np.abs(qddr[1:] - qddr[:-1]) / T
    lead = np.abs(qr - q)
    ok_lead = np.ones_like(lead, dtype=bool)
    for i in range(DOF):
        ok_lead[:, i] = (reset & (1 << i)) == 0
    lo, hi = np.array(m['lim_lo']), np.array(m['lim_hi'])
    res = {
        'vel': np.abs(qdr).max(), 'acc': np.abs(qddr).max(),
        'jerk': jerk[ok_j].max() if ok_j.any() else 0.0,
        'integ': integ[ok_j].max() if ok_j.any() else 0.0,
        'pos_margin': min((qr - lo).min(), (hi - qr).min()),
        'lead': lead[ok_lead].max() if ok_lead.any() else 0.0,
        'hl_us': r['hl_us'].max(),
    }
    tol = 1e-6
    res['n_vel'] = int((np.abs(qdr) > m['vel_max'] * (1 + 1e-9) + 1e-9).sum())
    res['n_acc'] = int((np.abs(qddr) > m['acc_max'] * (1 + 1e-9) + 1e-9).sum())
    res['n_jerk'] = int((jerk[ok_j] > m['jerk_max'] * (1 + 1e-6) + 1e-3).sum())
    res['n_integ'] = int((integ[ok_j] > 1e-7).sum())
    res['n_pos'] = int(((qr < lo - tol) | (qr > hi + tol)).sum())
    res['n_lead'] = int((lead[ok_lead] >= m['lead']).sum())
    res['n_all'] = res['n_vel'] + res['n_acc'] + res['n_jerk'] + res['n_integ'] + res['n_pos'] + res['n_lead']
    return res


def ref_torque_change(rs, mjcf):
    """prediction of the change of the commanded torque per period: the largest change per period of the
    torque of the reference itself, (M(q_d) + B) qdd_d + h(q_d, qd_d), along the logged reference of the
    runs rs (MuJoCo model; the feedback adds little while the arm follows); None without MuJoCo"""
    try:
        import mujoco
    except ImportError:
        return None
    if mjcf not in _MODELS:
        mm = mujoco.MjModel.from_xml_path(mjcf)
        if _ARMATURE is not None:                    # B on the diagonal of M, as in the controller
            for i, v in enumerate(_ARMATURE):
                j = mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_JOINT, 'joint%d' % i)
                if j >= 0:
                    mm.dof_armature[mm.jnt_dofadr[j]] = max(float(v), 0.0)
        _MODELS[mjcf] = (mm, mujoco.MjData(mm))
    m, d = _MODELS[mjcf]
    Mqdd = np.zeros(m.nv)
    best = 0.0
    for r in rs:
        tau = np.empty_like(r['q_ref'])
        for k in range(len(tau)):
            d.qpos[:] = r['q_ref'][k]
            d.qvel[:] = r['qd_ref'][k]
            mujoco.mj_forward(m, d)
            mujoco.mj_mulM(m, d, Mqdd, np.ascontiguousarray(r['qdd_ref'][k], dtype=float))
            tau[k] = Mqdd + d.qfrc_bias
        if len(tau) > 1:
            best = max(best, np.abs(np.diff(tau, axis=0)).max())
    return best


def residual(r, window=1.0):
    """peak-to-peak motion of the tool in the last `window` s [m], largest over the axes"""
    msk = r['t'] > r['t'][-1] - window
    tl = r['tool'][msk]
    return (tl.max(0) - tl.min(0)).max()


def planned_reach(r):
    """planned time to reach the goal of one run: start of the last plan + its duration + the rest condition"""
    m = r['meta']
    new = np.where((r['flags'] & F_NEW) != 0)[0]
    if not len(new):
        return None
    return r['t'][new[-1]] + r['otg_dur'][new[-1]] + m['rest_cycles'] * m['period']


def random_goals(rs):
    """the random goals of test 1 (or jumps of test 2): the last completed (or not at rest) run of every index"""
    best = {}
    for r in rs:
        m = r['meta']
        if m.get('goal_index', -1) >= 0 and m['result'].startswith(('completed', 'timeout')):
            best[m['goal_index']] = r          # the logs are sorted by their time stamp: the last run wins
    return [best[k] for k in sorted(best)]


def random_goal_rows(runs_of, m0):
    """rows (1) of the table for the random goals of test 1"""
    def cells(fn):
        out = []
        for plat in ('robot', 'sim', 'sim+fr'):
            rs = random_goals(runs_of('fixed', plat))
            v = fn(rs) if rs else None
            out.append(('%s (%d)' % (v, len(rs))) if v is not None else '-')
        return out

    def reached(rs):
        n = rs[0]['meta'].get('goals', len(rs))
        ok = sum(1 for r in rs if r['meta']['result'].startswith('completed') and r['meta']['rejected'] == 0)
        return '%d of %d' % (ok, n)

    def dist(rs):
        d = [1e3 * r['meta']['final_distance'] for r in rs if r['meta']['rejected'] == 0]
        return mpm(d, 3)

    def dev(rs):
        v = []
        for r in rs:
            p = r['planned']
            if p and r['meta']['rest_time'] >= 0:
                v.append(100.0 * (r['meta']['rest_time'] / p - 1.0))
        return mpm(np.abs(v), 2) if v else None

    def rej(rs):
        return str(sum(r['meta']['rejected'] for r in rs))

    def not_rest(rs):
        return str(sum(1 for r in rs if r['meta']['result'].startswith('timeout')))

    for label, pred, fn in (('(1) random goals reached', 'all', reached),
                            ('    final dist., med./p95/max (mm)', '<= %.2g+stat.' % (1e3 * m0['tol']), dist),
                            ('    reach vs planned, |dev.| m/p95/max (%)', '0', dev),
                            ('    IK rejections', '0', rej),
                            ('    not at rest after 20 s', '0', not_rest)):
        print('   %-40s %12s %24s %24s %24s' % (label, pred, *cells(fn)))


def random_jump_rows(runs_of, m0, mjcf):
    """rows (2) of the table for the random goal jumps of test 2"""
    def pred_of(r):
        return r.get('pred_dtau')

    def cells(fn):
        out = []
        for plat in ('robot', 'sim', 'sim+fr'):
            rs = random_goals(runs_of('jump', plat))
            v = fn(rs) if rs else None
            out.append(('%s (%d)' % (v, len(rs))) if v is not None else '-')
        return out

    def done(rs):
        n = rs[0]['meta'].get('goals', len(rs))
        ok = sum(1 for r in rs if r['meta']['result'].startswith('completed') and r['meta']['rejected'] == 0)
        return '%d of %d' % (ok, n)

    def dist(rs):
        d = [1e3 * r['meta']['final_distance'] for r in rs if r['meta']['rejected'] == 0]
        return mpm(d, 3)

    def dev(rs):
        v = []
        for r in rs:
            p = r['planned']
            if p and r['meta']['rest_time'] >= 0:
                v.append(100.0 * (r['meta']['rest_time'] / p - 1.0))
        return mpm(np.abs(v), 2) if v else None

    def dqdd(rs):
        return fmt(max(r['meta']['max_dqdd_cycle'] for r in rs), 4)

    def dtau(rs):
        return fmt(max(r['meta']['max_dtau_cycle'] for r in rs), 3)

    def ratio(rs):
        v = [r['meta']['max_dtau_cycle'] / pred_of(r) for r in rs if pred_of(r)]
        return fmt(max(v), 2) if v else None

    def rej(rs):
        return str(sum(r['meta']['rejected'] for r in rs))

    rs_pred = random_goals(runs_of('jump', 'sim')) or random_goals(runs_of('jump', 'robot'))
    preds = [pred_of(r) for r in rs_pred]
    pred = max(preds) if preds and all(p is not None for p in preds) else None
    for label, pr, fn in (('(2) random goal jumps completed', 'all', done),
                          ('    final dist., med./p95/max (mm)', '<= %.2g+stat.' % (1e3 * m0['tol']), dist),
                          ('    reach vs planned, |dev.| m/p95/max (%)', '0', dev),
                          ('    max change of qdd_d per period', '%.3f' % (m0['jerk_max'] * m0['period']), dqdd),
                          ('    max change of tau per period (N m)', fmt(pred, 3), dtau),
                          ('    tau change / prediction, max', '1', ratio),
                          ('    IK rejections', '0', rej)):
        print('   %-40s %12s %24s %24s %24s' % (label, pr, *cells(fn)))


def fmt(v, p=3):
    return ('%.*f' % (p, v)) if v is not None and np.isfinite(v) else '-'


def mpm(v, p=3):
    """median / 95th percentile / largest"""
    return '%s / %s / %s' % (fmt(np.median(v), p), fmt(np.percentile(v, 95), p), fmt(max(v), p)) if len(v) else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='*')
    ap.add_argument('--keep-faulty', action='store_true', help='keep runs with timing faults or a controller fault (diagnosis)')
    ap.add_argument('--plot', help='write OUT.png and OUT.pdf')
    ap.add_argument('--mjcf', default=DEFAULT_MJCF, help='model for the torque prediction of (2)')
    ap.add_argument('--rotor-inertia', help='"b1 ... b6" [kg m^2]: B of the controller; default: ROTOR_INERTIA_n '
                    'of indy7/INDY7.cfg, bounded at %.1f as in the controller' % B_MAX)
    ap.add_argument('--static-err', help='"e1 ... e6" [mrad]: e_bar of the joint test, for the estimate of Table 11 '
                    '(tools/jt_analyze.py prints it)')
    a = ap.parse_args()
    global _ARMATURE
    if a.rotor_inertia:
        _ARMATURE = [float(x) for x in a.rotor_inertia.replace(',', ' ').split()]
    else:
        try:
            import jt_predict
            _ARMATURE = [min(max(b, 0.0), B_MAX) for b in jt_predict.cfg_params(DEFAULT_CFG)['B']]
        except Exception:                            # noqa: BLE001 - no cfg: the URDF alone
            _ARMATURE = None
    paths = []
    for item in (a.logs or [DEFAULT_DIR]):
        paths += sorted(f for f in glob.glob(os.path.join(item, 'hl_*.csv')) if '_seed' not in os.path.basename(f)) \
            if os.path.isdir(item) else [item]
    runs = []
    for p in paths:
        try:
            runs.append(load(p, keep=bool(a.plot), mjcf=a.mjcf))
        except (OSError, ValueError, KeyError) as ex:
            print('hl_analyze: skipped %s (%s)' % (p, ex))
    if not runs:
        sys.exit('hl_analyze: no logs')

    print('\n1. Runs')
    print('   %-44s %-7s %-12s %6s %5s %5s %5s %9s %8s %6s %6s' % ('log', 'plat.', 'result', 'timing', 'acc', 'rej',
                                                                   'reset', 'dist mm', 'rest s', 'alloc', 'hl us'))
    for r in runs:
        m = r['meta']
        print('   %-44s %-7s %-12s %6s %5d %5d %5d %9s %8s %6s %6s' % (
            os.path.basename(r['path'])[:44], platform(m), m['result'][:12],
            str(m['timing_faults']) if m['timing_faults'] >= 0 else '-', m['accepted'], m['rejected'],
            m['lead_resets'], fmt(1e3 * m['final_distance'], 4), fmt(m['rest_time']) if m['rest_time'] >= 0 else '-',
            str(m['allocations']) if m['allocations'] >= 0 else '-', fmt(m['max_hl_us'], 1)))
    faulty = [r for r in runs if r['meta']['timing_faults'] > 0]
    if faulty and not a.keep_faulty:
        print('   %d run(s) with timing faults, left out below' % len(faulty))
        runs = [r for r in runs if r['meta']['timing_faults'] <= 0]
        if not runs:
            return 1
    # a controller fault (a non-finite state or torque) ended the run there and switched the drives off
    cfault = [r for r in runs if r['meta'].get('controller_faults', 0) > 0]
    if cfault and not a.keep_faulty:
        print('   %d run(s) ended by a controller fault, left out below: %s' % (
            len(cfault), ', '.join(os.path.basename(r['path']) for r in cfault)))
        runs = [r for r in runs if r['meta'].get('controller_faults', 0) <= 0]
        if not runs:
            return 1
    # a drive dropout (a drive left Operation enabled, e.g. the E-stop) ended the run there and switched
    # every axis off
    dfault = [r for r in runs if r['meta'].get('drive_dropouts', 0) > 0]
    if dfault and not a.keep_faulty:
        print('   %d run(s) ended by a drive dropout, left out below: %s' % (
            len(dfault), ', '.join(os.path.basename(r['path']) for r in dfault)))
        runs = [r for r in runs if r['meta'].get('drive_dropouts', 0) <= 0]
        if not runs:
            return 1

    print('\n2. Reference check: the contract in every period')
    print('   %-10s %-7s %6s %8s %8s %8s %9s %10s %8s %7s %6s' % ('test', 'plat.', 'periods', 'max qd', 'max qdd',
                                                                 'max jerk', 'integ', 'pos margin', 'max lead',
                                                                 'hl us', 'viol.'))
    total = 0
    n_base = 0
    for r in runs:
        c = r['contract']
        if r['meta']['test'].endswith('-step'):
            n_base += 1                   # the baseline does not meet the contract by construction
        else:
            total += c['n_all'] + max(0, r['meta']['allocations'])
        print('   %-10s %-7s %6d %8.4f %8.4f %8.4f %9.1e %10.4f %8.5f %7.1f %6d' % (
            r['meta']['test'], platform(r['meta']), r['n'], c['vel'], c['acc'], c['jerk'], c['integ'],
            c['pos_margin'], c['lead'], c['hl_us'], c['n_all']))
    m0 = runs[0]['meta']
    if n_base:
        print('   %d run(s) of the step baseline (test *-step): listed, not counted below' % n_base)
    print('   limits: %.2f rad/s, %.2f rad/s^2, %.1f rad/s^3; integral 1e-7 rad; lead %.2f rad' % (
        m0['vel_max'], m0['acc_max'], m0['jerk_max'], m0['lead']))
    print('   %s: %d violation(s) (allocations included)' % ('PASS' if total == 0 else 'FAIL', total))

    def runs_of(test, plat):
        return [r for r in runs if r['meta']['test'] == test and platform(r['meta']) == plat]

    print('\n3. Results of the high-level controller')
    print('   %-40s %12s %24s %24s %24s' % ('test, metric', 'prediction', 'robot', 'sim.', 'sim.+fr.'))

    def row(label, pred, fn, tests):
        cells = []
        for plat in ('robot', 'sim', 'sim+fr'):
            rs = [r for t in tests for r in runs_of(t, plat)]
            v = fn(rs) if rs else None
            cells.append(('%s (%d)' % (v, len(rs))) if v is not None else '-')
        print('   %-40s %12s %24s %24s %24s' % (label, pred, *cells))

    def final_dist(rs):
        return fmt(1e3 * np.mean([r['meta']['final_distance'] for r in rs]), 3)

    def spread(rs):
        if len(rs) < 2:
            return None
        P = np.array([r['tool_last'] for r in rs])
        return fmt(1e3 * max(np.linalg.norm(P[i] - P[j]) for i in range(len(P)) for j in range(i + 1, len(P))), 4)

    def reach(rs):
        v = [r['meta']['rest_time'] for r in rs if r['meta']['rest_time'] >= 0]
        return fmt(np.mean(v), 3) if v else None

    def planned(rs):
        v = [r['planned'] for r in rs if r['planned']]
        return fmt(np.mean(v), 3) if v else None

    def dqdd(rs):
        return fmt(max(r['meta']['max_dqdd_cycle'] for r in rs), 4)

    def dtau(rs):
        return fmt(max(r['meta']['max_dtau_cycle'] for r in rs), 3)

    def resid(rs):
        return fmt(1e3 * np.mean([r['resid'] for r in rs]), 3)

    def lead(rs):
        return fmt(max(r['meta']['max_lead'] for r in rs), 4)

    def resets(rs):
        return str(sum(r['meta']['lead_resets'] for r in rs))

    rs_fixed = [r for r in runs if r['meta']['test'] == 'fixed']
    if any(r['meta'].get('goal_index', -1) >= 0 for r in rs_fixed):
        random_goal_rows(runs_of, m0)
    else:
        pred_reach = planned(rs_fixed) if rs_fixed else '-'
        row('(1) final distance (mm)', '<= %.2g+stat.' % (1e3 * m0['tol']), final_dist, ['fixed'])
        row('    spread of final positions (mm)', '0', spread, ['fixed'])
        row('    time to reach the goal (s)', pred_reach or '-', reach, ['fixed'])
    rs_jump = [r for r in runs if r['meta']['test'] == 'jump']
    if any(r['meta'].get('goal_index', -1) >= 0 for r in rs_jump):
        random_jump_rows(runs_of, m0, a.mjcf)
    else:
        row('(2) max change of qdd_d per period', '%.3f' % (m0['jerk_max'] * m0['period']), dqdd, ['jump'])
        pp = [r['pred_dtau'] for r in rs_jump if r.get('pred_dtau') is not None]
        pred_dtau = max(pp) if pp else None
        row('    max change of tau per period (N m)', fmt(pred_dtau, 3), dtau, ['jump'])
        row('    final distance (mm)', '<= %.2g+stat.' % (1e3 * m0['tol']), final_dist, ['jump'])
    row('(3) residual motion, delta (mm)', '0', resid, ['noise'])
    row('    residual motion, delta = 0 (mm)', '-', resid, ['noise0'])
    row('all max lead (rad)', '-', lead, ['fixed', 'jump', 'noise', 'noise0', 'refcheck'])
    row('    lead-bound resets', '0', resets, ['fixed', 'jump', 'noise', 'noise0', 'refcheck'])
    print('   time to reach: until at rest (generator done, |qd| <= %.2f rad/s for %d ms); prediction: the planned'
          ' duration + %d ms' % (m0['rest_qd'], m0['rest_cycles'], m0['rest_cycles']))
    print('   final distance: prediction the IK tolerance e_tol plus the static error (slow JOINT TEST, through J_p)')
    print('   change of tau: prediction from the torque of the reference, (M(q_d) + B) qdd_d + h(q_d, qd_d), along'
          ' the logged reference')

    goal_table(runs, a)
    baseline(runs)
    if a.plot:
        plot(runs, a.plot)
    return 0 if total == 0 else 1


def tool_jacobian(mjcf):
    """a function q -> J_p(q) (3 x 6) of the tool point (body "tcp" of the MuJoCo model), or None without MuJoCo"""
    try:
        import mujoco
    except ImportError:
        return None
    m = mujoco.MjModel.from_xml_path(mjcf)
    d = mujoco.MjData(m)
    tcp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'tcp')
    jnt = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'joint%d' % i) for i in range(DOF)]
    if tcp < 0 or min(jnt) < 0:
        return None
    qadr = [m.jnt_qposadr[j] for j in jnt]
    vadr = [m.jnt_dofadr[j] for j in jnt]

    def jp(q):
        d.qpos[:] = 0.0
        for a, v in zip(qadr, q):
            d.qpos[a] = v
        mujoco.mj_forward(m, d)
        jacp = np.zeros((3, m.nv))
        mujoco.mj_jacBody(m, d, jacp, None, tcp)
        return jacp[:, vadr]
    return jp


def goal_table(runs, a):
    """section 4: Table 11, the final distance per random goal of test 1 over its runs, and the estimate"""
    ebar = None
    if a.static_err:
        ebar = 1e-3 * np.array([float(x) for x in a.static_err.replace(',', ' ').split()][:DOF])
        if len(ebar) != DOF:
            print('\n4. --static-err needs %d values' % DOF)
            ebar = None
    jp = tool_jacobian(a.mjcf) if ebar is not None else None
    if ebar is not None and jp is None:
        print('\n4. no estimate: the mujoco package or the model %s is missing' % a.mjcf)
    for plat in ('robot', 'sim', 'sim+fr'):
        rs = [r for r in runs if r['meta']['test'] == 'fixed' and platform(r['meta']) == plat and
              r['meta'].get('goal_index', -1) >= 0 and r['meta']['result'].startswith('completed') and
              r['meta']['rejected'] == 0]
        if not rs:
            continue
        by = {}
        for r in rs:
            by.setdefault(r['meta']['goal_index'], []).append(r)
        print('\n4. Final distance per goal [mm] (%s; Table 11): mean, SD and largest over the runs of a goal%s' % (
            plat, '; Est. = e_tol + || |J_p(q_g)| e_bar ||' if jp is not None else ''))
        print('   %-5s %-22s %4s %8s %8s %8s %8s' % ('goal', 'p_g (x, y, z) mm', 'runs', 'mean', 'SD', 'max.',
                                                    'Est.' if jp is not None else ''))
        over = []
        for k in sorted(by):
            v = 1e3 * np.array([r['meta']['final_distance'] for r in by[k]])
            p = 1e3 * by[k][-1]['goal_last']
            est = ''
            if jp is not None:
                e = 1e3 * by[k][-1]['meta']['tol'] + 1e3 * np.linalg.norm(np.abs(jp(by[k][-1]['q_goal'])) @ ebar)
                est = '%8.2f' % e
                if v.mean() > e:
                    over.append('G%d' % (k + 1))
            print('   G%-4d (%4.0f, %4.0f, %4.0f)      %4d %8.2f %8s %8.2f %s' % (
                k + 1, p[0], p[1], p[2], len(v), v.mean(), '%.2f' % v.std(ddof=1) if len(v) > 1 else '-', v.max(), est))
        if jp is not None:
            print('   goals beyond the estimate: %s' % (', '.join(over) if over else 'none'))


def baseline(runs):
    """section 5: the step baseline against the controller, on the jumps that both ran (same index)"""
    def by_index(test):
        out = {}
        for r in runs:
            m = r['meta']
            if m['test'] == test and platform(m) == 'sim' and m.get('goal_index', -1) >= 0:
                out[m['goal_index']] = r
        return out
    ctl, stp = by_index('jump'), by_index('jump-step')
    if not stp:
        return
    common = sorted(set(stp) & set(ctl)) if ctl else sorted(stp)
    print('\n5. Step baseline (simulation): the IK solution as a step, without the generator and the lead bound')
    print('   %d jump(s) of the baseline, %d of the controller, %d in both%s' % (
        len(stp), len(ctl), len(set(stp) & set(ctl)), '' if ctl else ' (baseline alone)'))
    rows = (('max change of tau per period (N m)', lambda r: r['motion']['dtau'], 3),
            ('periods until rest at a torque limit (%)', lambda r: 100.0 * r['motion']['clamp_share'], 1),
            ('max joint speed, any joint (rad/s)', lambda r: float(r['motion']['qd_max'].max()), 2),
            ('max joint speed, J1-J5 (rad/s)', lambda r: float(r['motion']['qd_max'][:5].max()), 2),
            ('max lead |q_d - q| (rad)', lambda r: r['motion']['lead'], 4),
            ('time to rest (s)', lambda r: r['meta']['rest_time'], 3),
            ('final distance (mm)', lambda r: 1e3 * r['meta']['final_distance'], 3),
            ('max tool speed (m/s)', lambda r: r['motion']['tool_vmax'], 3),
            ('tool path length (m)', lambda r: r['motion']['path'], 3))
    print('   %-42s %28s %28s' % ('metric, median / p95 / max', 'controller', 'step baseline'))
    for label, fn, p in rows:
        c = mpm([fn(ctl[k]) for k in common], p) if ctl else '-'
        s_ = mpm([fn(stp[k]) for k in common], p)
        print('   %-42s %28s %28s' % (label, c, s_))
    not_rest = sum(1 for k in common if not stp[k]['meta']['result'].startswith('completed'))
    print('   baseline runs not completed: %d; joint speed limit of the generator %.2f rad/s' % (
        not_rest, stp[common[0]]['meta']['vel_max']))


def plot(runs, out):
    """quick look in the layout of Fig. 6: (a) path of the tool to a fixed goal, (b) speed of the tool with
    the goal jump, (c) tool position with goal noise, with and without delta"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'serif', 'font.serif': ['Times New Roman', 'Times', 'Nimbus Roman',
                         'STIXGeneral'], 'mathtext.fontset': 'stix', 'font.size': 8, 'legend.fontsize': 7,
                         'xtick.labelsize': 7, 'ytick.labelsize': 7, 'lines.linewidth': 0.9})
    first = lambda t: next((r for r in runs if r['meta']['test'] == t), None)
    fig, ax = plt.subplots(1, 3, figsize=(7.16, 2.1))
    r = first('fixed')
    if r is not None:
        p0 = np.array(r['meta']['p0'])
        ax[0].plot(1e3 * (r['tool'][:, 0] - p0[0]), 1e3 * (r['tool'][:, 2] - p0[2]), 'C0-', label='tool')
        g = r['acc'][-1] - p0
        ax[0].plot(1e3 * g[0], 1e3 * g[2], 'kx', label='goal')
        ax[0].set_xlabel('$x$ (mm)')
        ax[0].set_ylabel('$z$ (mm)')
        ax[0].set_aspect('equal', 'datalim')
        ax[0].legend(frameon=False)
    r = first('jump')
    if r is not None:
        v = np.linalg.norm(np.gradient(r['tool'], r['t'], axis=0), axis=1)
        ax[1].plot(r['t'], v, 'C0-')
        acc = np.where((r['flags'] & F_ACCEPTED) != 0)[0]
        for k in acc[1:]:
            ax[1].axvline(r['t'][k], color='k', lw=0.6, ls=':')
        ax[1].set_xlabel('$t$ (s)')
        ax[1].set_ylabel('tool speed (m/s)')
    for test, st, lab in (('noise', 'C0-', r'with $\delta$'), ('noise0', 'C3-', r'$\delta = 0$')):
        r = first(test)
        if r is not None:
            y = r['tool'][:, 1]
            ax[2].plot(r['t'], 1e3 * (y - y[-1]), st, label=lab)
    ax[2].set_xlabel('$t$ (s)')
    ax[2].set_ylabel('$y$ rel. to the end (mm)')
    ax[2].set_ylim(-5, 5)
    ax[2].legend(frameon=False)
    for i, x in enumerate(ax):
        x.set_title('(%s)' % 'abc'[i], fontsize=8)
    fig.tight_layout(pad=0.3)
    for ext in ('png', 'pdf'):
        fig.savefig('%s.%s' % (out, ext), dpi=300)
    print('\nwrote %s.png and %s.pdf' % (out, out))


if __name__ == '__main__':
    sys.exit(main())
