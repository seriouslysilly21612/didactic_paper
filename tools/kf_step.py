#!/usr/bin/env python3
"""kf_step.py - the friction feedforward K_f of J1-J6 (paper, Sec. IV-A, eq. (7), Table 7) in one sweep of the
slow joint test with the feedforward ('u' '4': slow, CTC + FF, every joint moves).

  python3 tools/kf_step.py LOGDIR --start [--max-steps 5] [--step 0.5] [--cfg indy7/INDY7.cfg]
      the start of the sweep: KF_0..5 = 0 in the cfg (the record without the feedforward), and the state of
      the sweep in LOGDIR/kf_step.json (tau_s from SIM_TAU_S, tools/jt_breakaway.py --apply; the step; the
      start time); then restart the controller, 'u' '4' twice
  python3 tools/kf_step.py LOGDIR
      after every step (restart, 'u' '4' twice): the runs of the last K_f (the logs jt_slow_ctcff_* of LOGDIR
      since the start, grouped by their K_f, the key Kf of their .json) are analyzed, and either the next K_f
      goes into the cfg ("next: restart, u 4 x 2"), or K_f is chosen ("done")
LOGDIR is JOINT_TEST_LOG_DIR of the cfg (indy7/jt_logs by default).

The rule (per joint; the stall is that of tools/jt_analyze.py, stalls()):
  - the first step is 0.5 tau_s; then a joint that stalled gets +step (at most tau_s);
  - a joint is fixed at the smallest value (<= tau_s; K_f = 0, the record without the feedforward, counts)
    with no stall at which the next step does not reduce its largest |e| (the mean over the runs of the step)
    by more than 10 %: a joint without a stall is therefore stepped once more to see that, then set back to
    the value chosen; a joint that stalls again at a larger value keeps the last value without a stall;
  - after --max-steps steps, a joint not fixed takes the best value so far (its last value without a stall)
    or, if it stalled at every step, the largest value tried, with a warning;
  - a value a joint keeps while others step (at tau_s) counts with its latest step: a stall of another joint
    moves this one too;
  - a joint that does not move in the slow test (JOINT_TEST_SLOW_DQ 0) or has no tau_s keeps K_f = 0.
Runs ended by a controller fault, a drive dropout, timing faults (on the robot) or a stop are left out. Every
step needs --min-runs runs (default 2). The cfg must have the K_f of the last runs (the controller reads the
cfg at its start: restart after every step). Standard library and numpy.
"""
import argparse
import glob
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cfgset  # noqa: E402
import jt_analyze  # noqa: E402

DOF = 6
STATE = 'kf_step.json'
GAIN = 0.10           # the next step must reduce the largest |e| by more than this to be worth it


def fmt_kf(v):
    return ' '.join('%.3f' % x for x in v)


def kf_items(v):
    return [('KF_%d' % i, '%.3f' % x) for i, x in enumerate(v)]


def load_runs(logdir, t0):
    """the runs of the slow test with the feedforward since t0: [(mtime, Kf, run, stalls)]"""
    out, skipped = [], []
    for p in sorted(glob.glob(os.path.join(logdir, 'jt_slow_ctcff_*.csv'))):
        js = os.path.splitext(p)[0] + '.json'
        try:
            mt = os.path.getmtime(js)
        except OSError:
            continue                     # the test is still running (the .json comes at its end)
        if mt < t0:
            continue
        try:
            r = jt_analyze.load(p)
        except (OSError, ValueError, KeyError) as ex:
            skipped.append('%s (%s)' % (os.path.basename(p), ex))
            continue
        m = r['meta']
        why = None
        if m.get('controller_faults', 0) > 0:
            why = 'controller fault'
        elif m.get('drive_dropouts', 0) > 0:
            why = 'drive dropout'
        elif m.get('platform') != 'simulation' and m.get('timing_faults', 0) > 0:
            why = 'timing faults'
        elif str(m.get('result', '')).startswith(('stopped', 'aborted')):
            why = 'stopped'
        if why:
            skipped.append('%s (%s)' % (os.path.basename(p), why))
            continue
        kf = tuple(round(float(x), 3) for x in (m.get('Kf') or [0.0] * DOF))
        out.append((mt, kf, r, jt_analyze.stalls(r)))
    return sorted(out, key=lambda x: x[0]), skipped


def groups(runs):
    """the steps in the order run: [(Kf, [runs])]"""
    out = []
    for mt, kf, r, st in runs:
        for g in out:
            if g[0] == kf:
                g[1].append((r, st))
                break
        else:
            out.append((kf, [(r, st)]))
    return out


def decide(entries, tau_s, step):
    """entries: [(k, stall, maxe)] of one joint in the order tried (k increasing; k = 0 is the record without the
    feedforward). Returns (kind, k, note, best): kind 'fixed' or 'next'; best is the value to take if the sweep ends
    now (--max-steps)"""
    if not any(e[0] > 0.0 for e in entries):         # only the record without the feedforward (or nothing)
        best = 0.0 if entries and not entries[-1][1] else None
        return 'next', min(0.5 * tau_s, tau_s), 'first step 0.5 tau_s', best
    cap = tau_s
    i = next((n for n, e in enumerate(entries) if not e[1]), None)
    if i is None:                                    # it stalled at every value tried
        k_last = entries[-1][0]
        if k_last >= cap - 1e-9:
            return 'fixed', k_last, 'WARNING: stalls even at tau_s (check tau_s, the stall in jt_analyze)', k_last
        return 'next', min(k_last + step, cap), 'stalled', None
    while True:
        k_i, _, e_i = entries[i]
        if i == len(entries) - 1:                    # no larger value tried yet
            if k_i >= cap - 1e-9:
                return 'fixed', k_i, 'no stall at tau_s', k_i
            return 'next', min(k_i + step, cap), 'no stall at %.3f: one step more to compare' % k_i, k_i
        k_n, st_n, e_n = entries[i + 1]
        if st_n:
            return 'fixed', k_i, 'no stall at %.3f, a stall at %.3f' % (k_i, k_n), k_i
        if e_n >= (1.0 - GAIN) * e_i:
            return 'fixed', k_i, '%.3f -> %.3f: largest |e| %.3f -> %.3f mrad (%+.0f %%)' % (
                k_i, k_n, 1e3 * e_i, 1e3 * e_n, 100.0 * (e_n - e_i) / e_i if e_i > 0 else 0.0), k_i
        i += 1                                       # the larger value is better by more than 10 %


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logdir')
    ap.add_argument('--start', action='store_true', help='the start of the sweep: K_f = 0, the state file')
    ap.add_argument('--max-steps', type=int, default=5, help='steps after the start (default 5)')
    ap.add_argument('--step', type=float, default=0.5, help='[N m] the increase of a joint that stalled')
    ap.add_argument('--min-runs', type=int, default=2, help='runs per step (default 2)')
    ap.add_argument('--cfg', default=cfgset.CFG_MAIN, help='the cfg (default indy7/INDY7.cfg)')
    a = ap.parse_args()
    if not os.path.exists(a.cfg):
        sys.exit('kf_step: %s missing' % a.cfg)
    state_path = os.path.join(a.logdir, STATE)
    names = ['KF_%d' % i for i in range(DOF)] + ['SIM_TAU_S', 'JOINT_TEST_SLOW_DQ']
    cur = cfgset.values(a.cfg, names)
    if any(cur[n] is None for n in names):
        sys.exit('kf_step: %s lacks %s' % (a.cfg, ', '.join(n for n in names if cur[n] is None)))
    kf_now = tuple(round(float(cur['KF_%d' % i]), 3) for i in range(DOF))
    if a.start:
        tau = [float(v) for v in cur['SIM_TAU_S'].split()][:DOF]
        tau += [0.0] * (DOF - len(tau))
        if not any(t > 0.0 for t in tau):
            sys.exit('kf_step: SIM_TAU_S of %s is all 0: jt_breakaway.py --apply first' % a.cfg)
        dq = [float(v) for v in cur['JOINT_TEST_SLOW_DQ'].split()][:DOF]
        os.makedirs(a.logdir, exist_ok=True)
        st = {'start': time.strftime('%F %T'), 'start_epoch': time.time(), 'tau_s': tau, 'slow_dq': dq,
              'step': a.step, 'max_steps': a.max_steps, 'min_runs': a.min_runs}
        with open(state_path, 'w') as f:
            json.dump(st, f, indent=1)
        print('kf_step: start of the sweep (%s): tau_s %s N m, step %.2f N m, at most %d steps'
              % (state_path, fmt_kf(tau), a.step, a.max_steps))
        for j in range(DOF):
            if dq[j] == 0.0:
                print('  note: J%d does not move in the slow test (JOINT_TEST_SLOW_DQ): its K_f stays 0' % (j + 1))
            elif tau[j] <= 0.0:
                print('  note: J%d has no tau_s: its K_f stays 0' % (j + 1))
        ok, res = cfgset.apply_files([a.cfg], kf_items([0.0] * DOF), tag='kf_step --start')
        if not ok:
            sys.exit('kf_step: ' + '; '.join(res) + ' - nothing written')
        print('next: restart, u 4 x 2 (K_f = 0: the record without the feedforward)')
        return 0

    try:
        with open(state_path) as f:
            st = json.load(f)
    except (OSError, ValueError):
        sys.exit('kf_step: no %s: kf_step.py %s --start first' % (state_path, a.logdir))
    tau, dq, step = st['tau_s'], st['slow_dq'], float(st['step'])
    max_steps, min_runs = int(st['max_steps']), int(st.get('min_runs', 2))
    runs, skipped = load_runs(a.logdir, float(st['start_epoch']))
    for s in skipped:
        print('kf_step: left out %s' % s)
    gs = groups(runs)
    if not gs:
        print('kf_step: no runs of u 4 since the start (%s): restart, u 4 x 2' % st['start'])
        return 1
    last = [g for g in gs if g[0] == kf_now]
    if not last:
        print('kf_step: no runs with the K_f of the cfg (%s) yet; the last runs used %s.\n'
              'next: restart (the controller reads the cfg at its start), u 4 x 2' % (fmt_kf(kf_now),
                                                                                       fmt_kf(gs[-1][0])))
        return 1
    if gs[-1][0] != kf_now:
        print('kf_step: note: the newest runs used %s, not the K_f of the cfg %s; the step %s is analyzed'
              % (fmt_kf(gs[-1][0]), fmt_kf(kf_now), fmt_kf(kf_now)))
    if len(last[0][1]) < min_runs:
        print('kf_step: K_f %s has %d run(s), %d needed: u 4 once more' % (fmt_kf(kf_now), len(last[0][1]), min_runs))
        return 1

    # per step: the stall and the largest |e| of every joint (mean over the runs)
    table = []
    for kf, rs in gs:
        stall = [any(s[j][0] > 0 for _, s in rs) for j in range(DOF)]
        maxe = [float(np.mean([np.abs(r['e'][:, j]).max() for r, _ in rs])) for j in range(DOF)]
        table.append((kf, stall, maxe, len(rs)))
    steps_done = sum(1 for kf, *_ in table if any(v > 0.0 for v in kf))
    print('kf_step: %d step(s) of K_f since %s (the record without the feedforward %s)'
          % (steps_done, st['start'], 'included' if any(not any(v > 0 for v in kf) for kf, *_ in table) else 'missing'))
    print('  %-5s %-8s %s' % ('', 'tau_s', '   '.join('step %d' % n for n in range(len(table)))))
    for j in range(DOF):
        cells = ['%.2f%s %.2f' % (kf[j], '*' if stall[j] else ' ', 1e3 * maxe[j]) for kf, stall, maxe, _ in table]
        print('  J%-4d %-8.3f %s' % (j + 1, tau[j], '   '.join(cells)))
    print('  (K_f [N m], * = a stall, largest |e| [mrad], the mean over %s runs)'
          % '/'.join(str(n) for *_, n in table))

    decisions = []
    for j in range(DOF):
        if dq[j] == 0.0 or tau[j] <= 0.0:
            decisions.append(('fixed', 0.0, 'does not move in the slow test' if dq[j] == 0.0 else 'no tau_s'))
            continue
        # the values tried for this joint in order, from the record without the feedforward (k = 0); a value
        # kept while others stepped counts with its latest step (closer to the final setting: a stall of the
        # others moves this joint too)
        entries = []
        for kf, stall, maxe, n in table:
            k = kf[j]
            if entries and abs(entries[-1][0] - k) < 1e-6:
                entries[-1] = (k, stall[j], maxe[j], n)
            elif entries and k < entries[-1][0] - 1e-6:
                continue            # set back to a chosen value while others stepped: no new information
            else:
                entries.append((k, stall[j], maxe[j], n))
        kind, k, note, best = decide([(k, s, e) for k, s, e, _ in entries], tau[j], step)
        if steps_done >= max_steps and kind == 'next':
            if best is not None:
                kind, k, note = 'fixed', best, 'at the last step: the best value so far (no stall)'
            else:
                kind, k, note = 'fixed', max(e[0] for e in entries), \
                    'WARNING: stalls at every step; the largest value tried'
        decisions.append((kind, k, note))
    for j, (kind, k, note) in enumerate(decisions):
        print('  J%d: %s %.3f N m (%s)' % (j + 1, 'fixed at' if kind == 'fixed' else 'next', k, note))
    new = [k for _, k, _ in decisions]
    if all(kind == 'fixed' for kind, _, _ in decisions):
        ok, res = cfgset.apply_files([a.cfg], kf_items(new), tag='kf_step done')
        if not ok:
            sys.exit('kf_step: ' + '; '.join(res) + ' - nothing written')
        print('done: K_f %s N m in %s; restart the controller' % (fmt_kf(new), a.cfg))
        if tuple(round(v, 3) for v in new) != kf_now:
            print('  note: these values have not run together (the last step was %s): u 4 x 2 once more and check '
                  'the stalls (jt_analyze.py)' % fmt_kf(kf_now))
        return 0
    ok, res = cfgset.apply_files([a.cfg], kf_items(new), tag='kf_step next')
    if not ok:
        sys.exit('kf_step: ' + '; '.join(res) + ' - nothing written')
    print('next: restart, u 4 x 2 (K_f %s N m; step %d of at most %d)' % (fmt_kf(new), steps_done + 1, max_steps))
    return 0


if __name__ == '__main__':
    sys.exit(main())
