#!/usr/bin/env python3
"""jt_analyze.py - results of the JOINT TESTs (paper, Sec. V-A: Table 10, Figs. 4 and 5).

Reads the logs of the controller (jt_*.csv with their .json, cfg JOINT_TEST_LOG_DIR) and prints:
  1. every run: the largest |e| per joint [mrad] (e = reference minus the measurement the controller used),
     and the stalls of each run (stalls(): in the moves, wherever the reference moves at least 0.1 rad/s,
     the joint advances along it at less than 0.2 of its speed for at least 50 ms; tools/kf_step.py chooses
     K_f with the same rule)
  2. the CTC check: CTC in the simulation without friction against the predictions of tools/jt_predict.py,
     dde and dde+g (eq. (8)); PASS if, on every joint whose prediction exceeds 1 urad, the error is at most
     X times dde+g (Fig. 4; Table 10: 0.90-0.99 times the prediction)
  3. Table 10, per platform, test and law: the RMSE and the largest |e| of each run while the reference
     moves (the two moves of the test), and the error at rest, the mean of e over the last 0.1 s of the rest
     at q1 and at q0 (signed); mean +- standard deviation over the runs. Law "CTC + FF": the friction
     feedforward with K_f > 0 (JOINT TEST 4); "CTC, K_f = 0": JOINT TEST 3, or 4 with every K_f zero. Then
     e_bar, the larger magnitude of the two errors at rest of each joint (CTC + FF on the robot), which the
     estimate of Table 11 carries to the tool (tools/hl_analyze.py --static-err).
Runs with timing faults, a controller fault, or a drive dropout are listed but left out of 2 and 3. In the
simulation the EtherCAT task steps the simulator once per period, so the simulated loop has the timing of the
host; on a host configured as in Table 3 there are none.
Usage:
  python3 tools/jt_analyze.py [LOG_OR_DIR ...] [--x 1.5] [--plot OUT_PREFIX] [--mjcf MJCF]
Without arguments: indy7/jt_logs. --plot writes OUT_PREFIX.png and .pdf (a quick look in the layout of
Figs. 4 and 5: the positions and errors of one run of each law, with the prediction for the simulation).
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jt_predict as jp  # noqa: E402

DOF = 6
DEFAULT_DIR = os.path.join(HERE, '..', 'indy7', 'jt_logs')
LAW_NAME = {'pdg': 'PD + g', 'ctc': 'CTC, K_f = 0', 'ctcff': 'CTC + FF'}
FLOOR = 1e-6          # [rad] predictions below this are not compared
# the stall of a joint (tools/kf_step.py uses the same rule)
STALL_QD_REF = 0.1    # [rad/s] the reference moves at least this fast
STALL_RATIO = 0.2     # the joint advances along it at less than this fraction of its speed
STALL_MIN_S = 0.05    # [s] for at least this long
REST_WINDOW = 0.1     # [s] the error at rest: the mean of e over the end of the rest


def load(csv_path):
    """One run: metadata (the .json) and the columns of the CSV as arrays."""
    js = os.path.splitext(csv_path)[0] + '.json'
    with open(js) as f:
        meta = json.load(f)
    with open(csv_path) as f:
        header = f.readline().strip().split(',')
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1, ndmin=2)
    cols = {name: data[:, i] for i, name in enumerate(header)}
    n = meta.get('dq') and len(meta['dq']) or DOF

    def group(g):
        return np.column_stack([cols['%s%d' % (g, i)] for i in range(n)])

    run = {'meta': meta, 'path': csv_path, 't': cols['t'], 'phase': cols['phase'].astype(int),
           'clamp': cols['clamp'].astype(int)}
    for g in ('q_ref', 'qd_ref', 'qdd_ref', 'q', 'qd', 'e', 'tau', 'tau_g', 'tau_c', 'tau_mqdd', 'tau_fb',
              'tau_ff'):
        run[g] = group(g)
    return run


def platform(meta):
    if meta['platform'] != 'simulation':
        return 'robot'
    return 'sim+fr' if meta.get('sim_friction') else 'sim'


def law_of(meta):
    """the law of the run: CTC + FF with every K_f zero is CTC without the feedforward"""
    law = meta['law']
    if law == 'ctcff' and not any(float(k) > 0.0 for k in (meta.get('Kf') or [])):
        return 'ctc'
    return law


def stalls(run):
    """per joint: (number of stalls, their total time [s]). A stall: in the moves (phases 0 and 2), the
    reference moves at least STALL_QD_REF and the joint advances along it at less than STALL_RATIO of its
    speed (or goes back), for at least STALL_MIN_S"""
    qd, qdr, ph, t = run['qd'], run['qd_ref'], run['phase'], run['t']
    dt = float(run['meta'].get('period', 0.001))
    moving = (ph == 0) | (ph == 2)
    out = []
    for j in range(qd.shape[1]):
        ref = qdr[:, j]
        lag = moving & (np.abs(ref) >= STALL_QD_REF) & (qd[:, j] * np.sign(ref) < STALL_RATIO * np.abs(ref))
        edges = np.diff(np.concatenate(([0], lag.astype(np.int8), [0])))
        starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)     # [start, end)
        n, tot = 0, 0.0
        for a, b in zip(starts, ends):
            dur = float(t[b - 1] - t[a]) + dt
            if dur >= STALL_MIN_S - 1e-9:
                n += 1
                tot += dur
        out.append((n, tot))
    return out


def static_error(run, phase, window=REST_WINDOW):
    """mean e over the last `window` s of a rest (phase 1 at q1, phase 3 at q0)"""
    m = run['phase'] == phase
    if not m.any():
        return np.full(run['e'].shape[1], np.nan)
    t_end = run['t'][m][-1]
    m &= run['t'] > t_end - window
    return run['e'][m].mean(0)


def moving_stats(run):
    """RMSE and largest |e| per joint while the reference moves (phases 0 and 2)"""
    m = (run['phase'] == 0) | (run['phase'] == 2)
    if not m.any():
        nan = np.full(run['e'].shape[1], np.nan)
        return nan, nan
    e = run['e'][m]
    return np.sqrt((e * e).mean(0)), np.abs(e).max(0)


class Predictor:
    """Predictions per (test, gains, q0), computed once."""

    def __init__(self, mjcf):
        self.cache = {}
        self.model = None
        try:
            self.model = jp.Model(mjcf)
        except Exception as ex:          # noqa: BLE001 - no mujoco or no model: dde only
            print('jt_analyze: no dde+g prediction (%s)' % ex)

    def __call__(self, meta):
        dq, T, dwell = meta['dq'], meta['T'], meta['dwell']
        tau_d = tau_d_of(meta)
        b = meta.get('rotor_inertia') or [0.0] * len(dq)          # B of the controller in the test
        if self.model is not None:
            self.model.set_armature(b)
        key = (tuple(np.round(dq, 6)), T, dwell, tuple(meta['Kp']), tuple(meta['Kd']), round(tau_d, 9),
               tuple(np.round(b, 9)), tuple(np.round(meta['q0'], 4)))
        if key not in self.cache:
            t, e1 = jp.predict(dq, T, dwell, meta['Kp'], meta['Kd'], tau_d)
            e2 = None
            if self.model is not None:
                _, e2 = jp.predict(dq, T, dwell, meta['Kp'], meta['Kd'], tau_d, model=self.model, q0=meta['q0'])
            self.cache[key] = (t, e1, e2)
        return self.cache[key]


def fmt(v, w=7, p=4):
    return ('%*.*f' % (w, p, v)) if np.isfinite(v) else '%*s' % (w, '-')


def pipeline_of(meta):
    """periods that the tasks add between the receive and the send that acts on it: 1, the torque of the
    Dynamics task is sent by the EtherCAT task in the next period (indy7/src/TaskSlots.h)"""
    return int(meta.get('pipeline_periods', 1))


def tau_d_of(meta):
    """delay of eq. (8): (D + 1/2) T_s in the simulation, 2.5 T_s assumed on the robot (Sec. III-B), plus the
    pipeline: 3.5 ms (Table 7)"""
    base = (meta.get('sim_torque_delay_steps', 2) + 0.5) if meta['platform'] == 'simulation' else 2.5
    return (base + pipeline_of(meta)) * meta['period']


def mean_sd(v, scale=1e3, w=14, p=2):
    """'mean +- sd' over the runs of a column (n >= 2), or the value of a single run"""
    v = np.asarray(v, dtype=float) * scale
    if len(v) == 0 or not np.all(np.isfinite(v)):
        return '%*s' % (w, '-')
    if len(v) == 1:
        return '%*.*f' % (w, p, v[0])
    return '%*s' % (w, '%.*f +- %.*f' % (p, v.mean(), p, v.std(ddof=1)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='*', help='jt_*.csv files or directories (default indy7/jt_logs)')
    ap.add_argument('--x', type=float, default=1.5, help='CTC check: error <= X x prediction (dde+g)')
    ap.add_argument('--mjcf', default=jp.DEFAULT_MJCF)
    ap.add_argument('--keep-faulty', action='store_true',
                    help='keep runs with timing faults, a controller fault or a drive dropout (diagnosis)')
    ap.add_argument('--plot', help='write OUT.png and OUT.pdf')
    a = ap.parse_args()

    paths = []
    for item in (a.logs or [DEFAULT_DIR]):
        # the torque-step, breakaway and sine tests have tools of their own
        paths += sorted(f for f in glob.glob(os.path.join(item, 'jt_*.csv'))
                        if not os.path.basename(f).startswith(('jt_step_', 'jt_brk_', 'jt_sine_'))) \
            if os.path.isdir(item) else [item]
    runs = []
    for p in paths:
        try:
            runs.append(load(p))
        except (OSError, ValueError, KeyError) as ex:
            print('jt_analyze: skipped %s (%s)' % (p, ex))
    if not runs:
        sys.exit('jt_analyze: no logs')
    pred = Predictor(a.mjcf)

    # 1. every run
    print('\n1. Runs: largest |e| per joint [mrad]')
    print('   %-48s %-8s %-12s %-14s %6s' % ('log', 'platform', 'law', 'result', 'timing') +
          ''.join('%8s' % ('J%d' % (i + 1)) for i in range(DOF)))
    for r in runs:
        m = r['meta']
        tf = m.get('timing_faults', -1)
        bad = '' if m['result'] == 'at rest' and m['dropped'] == 0 else '  <-- check'
        lim = '  (torque limited in %d cycles)' % m['clamped_cycles'] if m['clamped_cycles'] else ''
        st = stalls(r)
        stx = ', '.join('J%d %d (%.2f s)' % (j + 1, n, d) for j, (n, d) in enumerate(st) if n)
        print('   %-48s %-8s %-12s %-14s %6s' % (os.path.basename(r['path'])[:48], platform(m),
                                               LAW_NAME.get(law_of(m), m['law'])[:12], m['result'][:14],
                                               str(tf) if tf >= 0 else '-') +
              ''.join(fmt(1e3 * x, 8) for x in np.abs(r['e']).max(0)) + bad + lim +
              (('  stalls: ' + stx) if stx else ''))
    if not a.keep_faulty:
        for key, what in (('timing_faults', 'with timing faults'), ('controller_faults', 'ended by a controller fault'),
                          ('drive_dropouts', 'ended by a drive dropout')):
            out = [r for r in runs if r['meta'].get(key, 0) > 0]
            if out:
                print('   %d run(s) %s, left out below: %s' % (len(out), what,
                                                               ', '.join(os.path.basename(r['path']) for r in out)))
                runs = [r for r in runs if r['meta'].get(key, 0) <= 0]
        if not runs:
            return 1

    # 2. CTC check
    print('\n2. CTC check (simulation without friction, CTC): largest |e| [mrad] vs the predictions')
    verdict = None
    for test in ('fast', 'slow'):
        rs = [r for r in runs if r['meta']['test'] == test and law_of(r['meta']) == 'ctc' and platform(r['meta']) == 'sim']
        if not rs:
            continue
        emax = np.array([np.abs(r['e']).max(0) for r in rs])
        _, e1, e2 = pred(rs[0]['meta'])
        p1, p2 = np.abs(e1).max(0), (np.abs(e2).max(0) if e2 is not None else None)
        print('   %s test, %d run(s); tau_d %.1f ms (pipeline +%d period)' % (
            test, len(rs), 1e3 * tau_d_of(rs[0]['meta']), pipeline_of(rs[0]['meta'])))
        print('     simulation (mean)  ' + ''.join(fmt(1e3 * x, 8) for x in emax.mean(0)))
        if len(rs) > 1:
            print('     spread (max-min)   ' + ''.join(fmt(1e3 * x, 8) for x in emax.max(0) - emax.min(0)))
        print('     prediction dde     ' + ''.join(fmt(1e3 * x, 8) for x in p1))
        if p2 is not None:
            print('     prediction dde+g   ' + ''.join(fmt(1e3 * x, 8) for x in p2))
            ratio = np.where(p2 > FLOOR, emax.max(0) / np.maximum(p2, FLOOR), np.nan)
            print('     ratio (worst run)  ' + ''.join(fmt(x, 8, 2) for x in ratio))
            ok = bool(np.all(ratio[np.isfinite(ratio)] <= a.x))
            print('     %s: every ratio <= X = %.2f' % ('PASS' if ok else 'FAIL', a.x))
            verdict = ok if verdict is None else (verdict and ok)
    if verdict is None:
        print('   no CTC runs in the simulation without friction (or no dde+g)')

    # 3. Table 10
    print('\n3. Joint tracking [mrad] (Table 10): RMSE and largest |e| while the reference moves, error at rest '
          '(mean of e over the last %.1f s of the rest at q1 and at q0); mean +- sd over the runs' % REST_WINDOW)
    keys = sorted({(platform(r['meta']), r['meta']['test'], law_of(r['meta'])) for r in runs})
    ebar = None
    for k in keys:
        rs = [r for r in runs if (platform(r['meta']), r['meta']['test'], law_of(r['meta'])) == k]
        stats = [moving_stats(r) for r in rs]
        rmse = np.array([s[0] for s in stats])
        emax = np.array([s[1] for s in stats])
        s1 = np.array([static_error(r, 1) for r in rs])
        s3 = np.array([static_error(r, 3) for r in rs])
        print('\n   %s, %s test, %s: %d run(s)' % (k[0], k[1], LAW_NAME.get(k[2], k[2]), len(rs)))
        print('   %4s %14s %14s %14s %14s' % ('', 'RMSE', 'max. per run', 'rest at q1', 'rest at q0'))
        n = rmse.shape[1]
        for j in range(n):
            print('   J%-3d %s %s %s %s' % (j + 1, mean_sd(rmse[:, j]), mean_sd(emax[:, j]), mean_sd(s1[:, j]),
                                          mean_sd(s3[:, j])))
        if k[0] == 'robot' and k[2] == 'ctcff' and k[1] == 'slow':
            ebar = np.maximum(np.abs(s1.mean(0)), np.abs(s3.mean(0)))
    if ebar is not None:
        print('\n   e_bar (robot, slow, CTC + FF) [mrad]: %s' % ' '.join('%.2f' % (1e3 * x) for x in ebar))
        print('   (tools/hl_analyze.py --static-err "%s": the estimate of Table 11)' %
              ' '.join('%.2f' % (1e3 * x) for x in ebar))

    if a.plot:
        plot(runs, pred, a.plot)
    return 0 if verdict in (None, True) else 1


def plot(runs, pred, out):
    """Quick look in the layout of Figs. 4 and 5: per joint, (top) q_d and q of one run, (bottom) the error of
    one run of each law and platform, with the prediction (dde+g) for the simulation without friction."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'serif', 'font.serif': ['Times New Roman', 'Times', 'Nimbus Roman',
                         'STIXGeneral'], 'mathtext.fontset': 'stix', 'font.size': 8, 'axes.labelsize': 8,
                         'legend.fontsize': 7, 'xtick.labelsize': 7, 'ytick.labelsize': 7, 'lines.linewidth': 0.9})
    slow = [r for r in runs if r['meta']['test'] == 'slow'] or runs
    groups = []
    for r in slow:
        k = (platform(r['meta']), law_of(r['meta']))
        if k not in [g[0] for g in groups]:
            groups.append((k, r))
    first = groups[0][1]
    n = first['e'].shape[1]
    fig, ax = plt.subplots(2, n, figsize=(7.16, 2.8), sharex=True)
    ax = np.atleast_2d(ax)
    for j in range(n):
        ax[0, j].plot(first['t'], first['q_ref'][:, j], 'k-', label='$q_d$')
        ax[0, j].plot(first['t'], first['q'][:, j], 'C0--', label='$q$')
        ax[0, j].set_title('J%d' % (j + 1), fontsize=8)
        for c, ((plat, law), r) in enumerate(groups):
            ax[1, j].plot(r['t'], 1e3 * r['e'][:, j], 'C%d-' % c, label='%s, %s' % (plat, LAW_NAME.get(law, law)))
        sims = [r for (plat, law), r in groups if plat == 'sim' and law == 'ctc']
        if sims:
            t, e1, e2 = pred(sims[0]['meta'])
            ax[1, j].plot(t, 1e3 * (e2 if e2 is not None else e1)[:, j], 'k:', label='prediction')
        ax[1, j].set_xlabel('$t$ (s)')
    ax[0, 0].set_ylabel('$q$ (rad)')
    ax[1, 0].set_ylabel('$e$ (mrad)')
    ax[0, 0].legend(frameon=False)
    ax[1, 0].legend(frameon=False)
    fig.tight_layout(pad=0.3)
    for ext in ('png', 'pdf'):
        fig.savefig('%s.%s' % (out, ext), dpi=300)
    print('\nwrote %s.png and %s.pdf' % (out, out))


if __name__ == '__main__':
    sys.exit(main())
