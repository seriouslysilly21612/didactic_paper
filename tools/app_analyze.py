#!/usr/bin/env python3
"""app_analyze.py - runs of the camera application (paper, Sec. V-C, Fig. 9; started with 'p', or 'y' then 8).

Reads the logs of the controller (hl_app_*.csv with their .json, in the cfg JOINT_TEST_LOG_DIR); in every
period of TRACK they hold the goal of the slot with the hover height (goal_x..z: the detected hover point),
the accepted goal (acc_x..z), the tool (tool_x..z, by the forward kinematics of the measured joint angles),
the flags (32: at rest) and the age of the slot (goal_age, periods since the bridge last wrote it). Prints
for every run
  rest       every interval at rest of at least --min-rest s: the distance of the tool to the detected
             hover point (mean over the interval, and at its end) and to the accepted goal
  stale      every interval in which the slot was not written for more than --stale s (object hidden or
             its samples rejected): its length, the motion of the tool from its start to its end and its
             largest motion after the arm came to rest in it (the last accepted goal stays: the tool should
             not move)
  timing     the timing faults of the run (from the .json)
and, with --csv OUT, the curves of Fig. 9: t, distance of the tool to the detected hover point [mm], age of
the slot [s], at rest, every --every periods.
The reference contract in every period of these runs is checked by tools/hl_analyze.py (section 2).
Usage:
  python3 tools/app_analyze.py [LOG_OR_DIR ...] [--min-rest 0.5] [--stale 0.2] [--csv OUT] [--every 10]
  (default indy7/jt_logs)
"""
import argparse
import csv
import glob
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DIR = os.path.join(HERE, '..', 'indy7', 'jt_logs')
F_REST = 32


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    meta = {}
    js = os.path.splitext(path)[0] + '.json'
    if os.path.exists(js):
        with open(js) as f:
            meta = json.load(f)
    t = [num(r['t']) for r in rows]
    goal = [(num(r['goal_x']), num(r['goal_y']), num(r['goal_z'])) for r in rows]
    acc = [(num(r['acc_x']), num(r['acc_y']), num(r['acc_z'])) for r in rows]
    tool = [(num(r['tool_x']), num(r['tool_y']), num(r['tool_z'])) for r in rows]
    rest = [(int(r['flags']) & F_REST) != 0 for r in rows]
    age = [int(r['goal_age']) for r in rows]
    return meta, t, goal, acc, tool, rest, age


def intervals(flags):
    """[start, end) index pairs of the runs of True"""
    out, s = [], None
    for i, f in enumerate(flags):
        if f and s is None:
            s = i
        elif not f and s is not None:
            out.append((s, i))
            s = None
    if s is not None:
        out.append((s, len(flags)))
    return out


def dist(a, b):
    return math.dist(a, b) if all(math.isfinite(x) for x in a + b) else float('nan')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='*', default=[DEFAULT_DIR])
    ap.add_argument('--min-rest', type=float, default=0.5, help='[s] shortest interval at rest that counts')
    ap.add_argument('--stale', type=float, default=0.2, help='[s] slot not written for longer: object lost')
    ap.add_argument('--csv', help='write the curves of the figure (one file per run: OUT_<run>.csv)')
    ap.add_argument('--every', type=int, default=10, help='periods per row of --csv')
    a = ap.parse_args()

    paths = []
    for item in a.logs:
        if os.path.isdir(item):
            paths += sorted(glob.glob(os.path.join(item, 'hl_app_*.csv')))
        else:
            paths.append(item)
    if not paths:
        raise SystemExit('app_analyze: no hl_app_*.csv in %s' % ', '.join(a.logs))
    all_rest, all_stale = [], []
    for p in paths:
        meta, t, goal, acc, tool, rest, age = load(p)
        if len(t) < 2:
            continue
        dt = meta.get('period') or (t[1] - t[0]) or 1e-3
        off = meta.get('goal_offset')
        print('%s  (%s, %d periods, %.1f s; hover %s m)' % (os.path.basename(p), meta.get('result', '?'), len(t),
                                                          len(t) * dt, off[2] if isinstance(off, list) else '?'))
        print('  rest (>= %.1f s)        dist. to the hover point: mean   at end   | to the accepted goal' % a.min_rest)
        for s, e in intervals(rest):
            if (e - s) * dt < a.min_rest:
                continue
            d = [dist(tool[i], goal[i]) for i in range(s, e)]
            d = [x for x in d if math.isfinite(x)]
            if not d:
                continue
            dm, de = sum(d) / len(d), d[-1]
            da = dist(tool[e - 1], acc[e - 1])
            all_rest.append((dm, de))
            print('    t %7.2f-%7.2f s %6.1f s %26.2f mm %6.2f mm | %6.2f mm' % (t[s], t[e - 1], (e - s) * dt,
                                                                             dm * 1e3, de * 1e3, da * 1e3))
        n_stale = int(round(a.stale / dt))
        print('  stale slot (> %.1f s)   length   tool motion: start to end   after rest' % a.stale)
        for s, e in intervals([g > n_stale for g in age]):
            s0 = max(0, s - n_stale)          # the slot last changed n_stale periods before s
            move = dist(tool[e - 1], tool[s0])
            r = next((i for i in range(s0, e) if rest[i]), None)
            after = max((dist(tool[i], tool[r]) for i in range(r, e)), default=float('nan')) if r is not None \
                else float('nan')
            all_stale.append((((e - s0) * dt), move, after))
            print('    t %7.2f-%7.2f s %6.2f s %18.2f mm %12s' % (t[s0], t[e - 1], (e - s0) * dt, move * 1e3,
                                                                 ('%.3f mm' % (after * 1e3)) if math.isfinite(after)
                                                                 else 'not at rest'))
        if meta:
            print('  timing faults %s (late releases %s, overruns %s, missed send deadlines %s, working-counter '
                  'errors %s); goals accepted %s, rejected by the IK %s' % tuple(
                      meta.get(k, '?') for k in ('timing_faults', 'late_releases', 'overruns', 'missed_deadlines',
                                                 'wkc_errors', 'accepted', 'rejected')))
            if 'controller_faults' in meta:
                print('  controller faults %s%s' % (meta['controller_faults'],
                                                    ' - the run ENDED there, the drives went off'
                                                    if meta['controller_faults'] > 0 else ''))
            if 'drive_dropouts' in meta:
                print('  drive dropouts %s%s' % (meta['drive_dropouts'],
                                                 ' - the run ENDED there, every axis went off'
                                                 if meta['drive_dropouts'] > 0 else ''))
        if a.csv:
            out = '%s_%s.csv' % (a.csv, os.path.splitext(os.path.basename(p))[0])
            with open(out, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t_s', 'dist_tool_hover_mm', 'slot_age_s', 'at_rest'])
                for i in range(0, len(t), max(1, a.every)):
                    w.writerow(['%.3f' % t[i], '%.3f' % (dist(tool[i], goal[i]) * 1e3), '%.3f' % (age[i] * dt),
                                int(rest[i])])
            print('  curves -> %s' % out)
    if all_rest:
        dm = [x[0] for x in all_rest]
        print('all runs: %d intervals at rest, distance of the tool to the detected hover point: mean %.2f mm, '
              'largest %.2f mm' % (len(dm), 1e3 * sum(dm) / len(dm), 1e3 * max(dm)))
    if all_stale:
        aft = [x[2] for x in all_stale if math.isfinite(x[2])]
        print('all runs: %d stale intervals (%.1f to %.1f s); largest motion of the tool after rest in them: %s' % (
            len(all_stale), min(x[0] for x in all_stale), max(x[0] for x in all_stale),
            ('%.3f mm' % (1e3 * max(aft))) if aft else '-'))


if __name__ == '__main__':
    main()
