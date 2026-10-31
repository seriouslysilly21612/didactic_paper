#!/usr/bin/env python3
"""goal_noise_file.py - the noise of the camera goals, from recordings of an object at rest at several positions
(paper, Table 9: the goal threshold delta above the stationary scatter of the vision goals): sigma_g, the
threshold delta of the goal admission (Sec. IV-C, Step 1 of indy7/src/HighLevelRT.h), and the file that the
high-level tests 3 and 4 replay (cfg HL_TEST_NOISE_FILE), in the simulation and on the robot.

  python3 tools/goal_noise_file.py p1_target.csv p2_target.csv ... [--out indy7/goal_noise.csv]
          [--seconds 20] [--kappa 0.4] [--gap 1.0] [--raw]

One recording per position. Accepted inputs:
  - the target CSV of tools/vision_probe.py (t, x, y, z, class, target_valid, depth_valid, ...);
  - the output of  ros2 topic echo --csv /pick_target_base  (my_interfaces/PickTarget3D: header stamp sec,
    nanosec, frame_id, capture stamp sec, nanosec, target_valid, depth_valid, class_id, class_name,
    confidence, center_x, center_y, width, height, x, y, z);
  - a CSV whose header names t, x, y, z ([s], [m]), all rows valid.
Only valid samples count (target_valid and depth_valid, where the input has them).

sigma_g is the noise of the goal that Step 1 compares, i.e. of the goal that the bridge writes into the goal
slot: the samples are passed through the average of the bridge (indy7/src/GoalFilter.h: p_k = p_{k-1} +
kappa (z_k - p_{k-1}), restarted after a gap of more than --gap s between samples; cfg APP_EMA_KAPPA,
APP_GAP_S), unless --raw. For white noise the average halves the standard deviation at kappa = 0.4.

Per position: the number of samples and their rate, the valid fraction, the standard deviation per axis, and
the largest distance from the first goal (the threshold below which Step 1 would accept a goal while the
object rests there) and between any two goals (the same from any start). Then
    delta = max(6 sigma_g, the smallest threshold with which the replay of the whole recording, as tests 3 and
            4 replay it, accepts no goal after the first, and with which no position accepts one),
rounded up to 1 mm, where sigma_g is the largest standard deviation of any axis at any position (delta >=
6 sigma_g: for normal noise, two goals differ by more than 6 sigma_g in fewer than 0.05 % of the pairs, but a
recording of thousands of samples may still contain such a pair, hence the second bound).

--out writes the replay file: the goals of each position minus their mean, the positions one after the other
(times continued), as t,dx,dy,dz [s, m]. Set in the cfg: HL_TEST_NOISE_FILE (that file), HL_TEST_NOISE_S
(its length, so that one run of test 3 or 4 replays all of it; it is evaluated in windows), HL_GOAL_DELTA_MM.
Standard library only.
"""
import argparse
import csv
import math
import os
import sys


def read(path):
    """[(t, x, y, z)] of the valid samples, sorted by t; and the number of rows read"""
    rows, total = [], 0
    with open(path, newline='') as f:
        first = f.readline()
        f.seek(0)
        head = [h.strip().lower() for h in first.strip().split(',')]
        if {'t', 'x', 'y', 'z'} <= set(head):
            for r in csv.DictReader(f):
                r = {(k or '').strip().lower(): (v or '').strip() for k, v in r.items()}
                total += 1
                ok = all(r.get(k, '1').lower() in ('1', 'true') for k in ('target_valid', 'depth_valid'))
                try:
                    t, x, y, z = float(r['t']), float(r['x']), float(r['y']), float(r['z'])
                except ValueError:
                    continue
                if ok and all(math.isfinite(v) for v in (t, x, y, z)):
                    rows.append((t, x, y, z))
        else:
            for r in csv.reader(f):
                if len(r) < 17:
                    continue
                try:
                    t = int(r[0]) + 1e-9 * int(r[1])
                    valid = r[5].strip().lower() in ('true', '1') and r[6].strip().lower() in ('true', '1')
                    x, y, z = float(r[-3]), float(r[-2]), float(r[-1])
                except ValueError:
                    continue
                total += 1
                if valid and all(math.isfinite(v) for v in (x, y, z)):
                    rows.append((t, x, y, z))
    rows.sort()
    return rows, total


def bridge_average(rows, kappa, gap):
    """the goals that the bridge writes into the slot (GoalFilter::Push for valid samples)"""
    out, avg, last = [], None, None
    for t, x, y, z in rows:
        if avg is None or t - last > gap:
            avg = [x, y, z]
        else:
            avg = [a + kappa * (p - a) for a, p in zip(avg, (x, y, z))]
        last = t
        out.append((t, avg[0], avg[1], avg[2]))
    return out


def dist(a, b):
    return math.sqrt((a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2 + (a[3] - b[3]) ** 2)


def stats(rows):
    n = len(rows)
    mean = [sum(r[k] for r in rows) / n for k in (1, 2, 3)]
    sd = [math.sqrt(sum((r[k + 1] - mean[k]) ** 2 for r in rows) / (n - 1)) for k in range(3)]
    from_first = max(dist(r, rows[0]) for r in rows)
    diameter = 0.0
    for i in range(n):
        ri = rows[i]
        for j in range(i + 1, n):
            d = dist(ri, rows[j])
            if d > diameter:
                diameter = d
    span = rows[-1][0] - rows[0][0]
    return {'n': n, 'mean': mean, 'sd': sd, 'from_first': from_first, 'diameter': diameter, 'span': span,
            'rate': (n - 1) / span if span > 0 else 0.0}


def step1_accepts(goals, delta):
    """the goals that Step 1 accepts after the first, for a threshold delta [m]"""
    acc, n = goals[0], 0
    for g in goals[1:]:
        if dist(g, acc) > delta:
            acc, n = g, n + 1
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('rec', nargs='+', help='one recording per position')
    ap.add_argument('--out', default=None, help='the replay file t,dx,dy,dz for HL_TEST_NOISE_FILE')
    ap.add_argument('--seconds', type=float, default=0.0, help='only the first S seconds of each recording (0: all)')
    ap.add_argument('--kappa', type=float, default=0.4, help='weight of the newest sample (cfg APP_EMA_KAPPA)')
    ap.add_argument('--gap', type=float, default=1.0, help='restart of the average after a gap [s] (APP_GAP_S)')
    ap.add_argument('--raw', action='store_true', help='the samples of the pipeline, without the average')
    a = ap.parse_args()
    if not (0.0 < a.kappa <= 1.0):
        sys.exit('goal_noise_file: --kappa in (0, 1]')

    pos = []
    for path in a.rec:
        rows, total = read(path)
        valid_frac = len(rows) / total if total else 0.0       # of the whole recording
        if a.seconds > 0 and rows:
            rows = [r for r in rows if r[0] - rows[0][0] <= a.seconds]
        if len(rows) < 10:
            sys.exit('goal_noise_file: %d valid samples in %s, need at least 10' % (len(rows), path))
        goals = rows if a.raw else bridge_average(rows, a.kappa, a.gap)
        s = stats(goals)
        s.update(path=path, valid_frac=valid_frac, goals=goals)
        pos.append(s)

    what = 'the samples of the pipeline (--raw)' if a.raw else \
        'the goals of the slot (average of the bridge, kappa %.2f, restart after %.1f s)' % (a.kappa, a.gap)
    print('goal_noise_file: %d position(s), %s' % (len(pos), what))
    print('  %-28s %6s %6s %6s %21s %8s %8s %8s' % ('recording', 'goals', 'Hz', 'valid', 'mean x y z (m)',
                                                    'sd x', 'sd y', 'sd z (mm)'))
    for s in pos:
        print('  %-28s %6d %6.1f %5.1f%% %6.3f %6.3f %6.3f %8.2f %8.2f %8.2f' % (
            os.path.basename(s['path'])[:28], s['n'], s['rate'], 100.0 * s['valid_frac'],
            s['mean'][0], s['mean'][1], s['mean'][2], 1e3 * s['sd'][0], 1e3 * s['sd'][1], 1e3 * s['sd'][2]))
    sigma = max(max(s['sd']) for s in pos)
    worst = max(pos, key=lambda s: max(s['sd']))
    print('  sigma_g = %.2f mm (largest axis, %s)' % (1e3 * sigma, os.path.basename(worst['path'])))
    for s in pos:
        print('  %-28s largest distance from the first goal %.2f mm, between any two goals %.2f mm' % (
            os.path.basename(s['path'])[:28], 1e3 * s['from_first'], 1e3 * s['diameter']))

    # the replay: each position minus its mean, one after the other
    replay, t_next = [], 0.0
    for s in pos:
        g = s['goals']
        t0 = g[0][0]
        dt = 1.0 / s['rate'] if s['rate'] > 0 else 1.0 / 30.0
        for t, x, y, z in g:
            replay.append((t_next + (t - t0), x - s['mean'][0], y - s['mean'][1], z - s['mean'][2]))
        t_next = replay[-1][0] + dt
    replay_first = max(dist(r, replay[0]) for r in replay)
    zero_accept = max([replay_first] + [s['from_first'] for s in pos])
    six = 6.0 * sigma
    delta_mm = math.ceil(max(six, zero_accept) * 1e3 - 1e-9)
    print('  6 sigma_g = %.2f mm; no accepted goal in the replay or at any position needs delta > %.2f mm'
          % (1e3 * six, 1e3 * zero_accept))
    for d_mm in sorted({math.ceil(six * 1e3 - 1e-9), delta_mm}):
        n_rep = step1_accepts(replay, d_mm * 1e-3)
        n_pos = sum(step1_accepts(s['goals'], d_mm * 1e-3) for s in pos)
        print('  delta %d mm: accepted goals after the first: replay %d, positions %d' % (d_mm, n_rep, n_pos))
    print('  delta = %d mm (HL_GOAL_DELTA_MM)%s' % (
        delta_mm, '' if delta_mm <= math.ceil(six * 1e3 - 1e-9) else
        ' - set by the recording, above 6 sigma_g: look for an outlier (a jump of the detection) before using it'))
    span = replay[-1][0] + (replay[-1][0] - replay[-2][0] if len(replay) > 1 else 0.0)
    if a.out:
        with open(a.out, 'w') as f:
            f.write('t,dx,dy,dz\n')
            for t, x, y, z in replay:
                f.write('%.6f,%.6f,%.6f,%.6f\n' % (t, x, y, z))
        rate = (len(replay) - 1) / replay[-1][0] if replay[-1][0] > 0 else 30.0
        print('  wrote %s: %d goals over %.1f s. cfg: HL_TEST_NOISE_FILE=%s, HL_TEST_NOISE_S=%d, '
              'HL_TEST_NOISE_HZ=%.0f, HL_GOAL_DELTA_MM=%d' % (a.out, len(replay), span, os.path.abspath(a.out),
                                                              math.ceil(span), rate, delta_mm))
    return 0


if __name__ == '__main__':
    sys.exit(main())
