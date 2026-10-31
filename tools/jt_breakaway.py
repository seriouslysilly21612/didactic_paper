#!/usr/bin/env python3
"""jt_breakaway.py - the breakaway test (JOINT TEST 6): the breakaway torque tau_s of a joint in both
directions (paper, Sec. IV-A, Table 7), and whether the joint moves in the direction of the torque (a check
of the sign s_i of eq. (2)).

Reads jt_brk_*.csv (with their .json). In the test the controller commands g(q) plus a torque ramp on one
joint until it moves (|qd| above the threshold of the cfg, or 5 mrad from the start of the ramp) or the
ramp reaches its maximum, then removes the extra torque and damps the joint to rest, and ramps again in
the other direction. Per ramp: tau_s = the extra torque (tau_ff) of the last period of the ramp, i.e. the
torque at which the motion was detected (one period after it began at most: the ramp adds rate * T_s per
period, 0.002 N m at 2 N m/s); the report of the drive in that period (tau_act - tau_g) as a cross-check;
the sign test passes if the joint moved in the direction of the extra torque (the position 50 ms after the
end of the ramp against its start). A ramp that reached the maximum without motion gives no tau_s.

  python3 tools/jt_breakaway.py LOG_OR_DIR ... [--apply] [--cfg FILE] [--platform robot|simulation]
--apply writes tau_s of every joint measured in both directions (the mean of the two directions) into
SIM_TAU_S of the cfg (default indy7/INDY7.cfg; tools/cfgset.py, the changed lines are printed): the
breakaway torques of the simulated arm (SIM_FRICTION_ON=1) and the start of tools/kf_step.py and
tools/sine_cfg.py. Restart the controller afterwards. A joint without a ramp that moved in one direction
keeps its value (rerun it with a larger BREAKAWAY_MAX_NM). Nothing is written if a sign test fails, if the
logs are of more than one platform (--platform), or if the cfg is missing the key.
Standard library and numpy.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cfgset  # noqa: E402


def load(path):
    with open(os.path.splitext(path)[0] + '.json') as f:
        meta = json.load(f)
    with open(path) as f:
        head = f.readline().strip().split(',')
    data = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    return meta, {h: data[:, i] for i, h in enumerate(head)}


def segments(phase):
    """[(first, last)] of the runs of phase 0"""
    out, a = [], None
    for k, p in enumerate(phase):
        if p == 0 and a is None:
            a = k
        elif p != 0 and a is not None:
            out.append((a, k - 1))
            a = None
    if a is not None:
        out.append((a, len(phase) - 1))
    return out


def analyze(path):
    meta, c = load(path)
    j = int(meta.get('breakaway_joint', 0)) - 1
    if j < 0:
        return None
    tmax = float(meta['breakaway_max'])
    rate = float(meta['breakaway_rate'])
    T = float(meta.get('period', 0.001))
    ff, q, ph = c['tau_ff%d' % j], c['q%d' % j], c['phase'].astype(int)
    act = c.get('tau_act%d' % j)
    g = c.get('tau_g%d' % j)
    ramps = []
    for a, b in segments(ph):
        if b + 1 >= len(ph) or b - a < 2:
            continue                        # the log ended in the ramp, or nothing to measure
        tau = ff[b]
        sign = 1.0 if tau > 0 else -1.0
        at_max = abs(tau) + rate * T * 1.5 >= tmax
        k1 = min(b + int(round(0.05 / T)), len(q) - 1)
        moved = q[k1] - q[a]
        rep = (act[b] - g[b]) if (act is not None and g is not None) else None
        ramps.append({'sign': sign, 'tau': abs(tau), 'at_max': at_max, 'moved': moved,
                      'sign_ok': (moved * sign > 0) if not at_max else None, 'reported': rep})
    return {'log': os.path.basename(path), 'platform': meta.get('platform', '?'), 'joint': j + 1, 'max': tmax,
            'rate': rate, 'ramps': ramps}


def apply_tau_s(by_joint, a):
    """SIM_TAU_S of the cfg from the ramps; 0 written, 1 nothing to write, 2 refused"""
    plats = sorted({plat for (_, plat) in by_joint})
    want = {'sim': 'simulation'}.get(a.platform, a.platform)
    if want:
        plats = [p for p in plats if p == want]
    if len(plats) != 1:
        print('\n--apply: logs of %s: give --platform robot|simulation - nothing written'
              % (', '.join(plats) if plats else 'no platform %s' % want))
        return 2
    plat = plats[0]
    rows = {jn: rs for (jn, p), rs in by_joint.items() if p == plat}
    bad = [jn for jn, rs in sorted(rows.items())
           if any(x['sign_ok'] is False for x in rs)]
    print('\n--apply (%s):' % plat)
    if bad:
        print('  sign test FAIL on %s: the torque does not move the joint in its direction (sign s_i of the drive?) '
              '- nothing written; stop and check' % ', '.join('J%d' % j for j in bad))
        return 2
    paths = [a.cfg]
    old = cfgset.values(a.cfg, ['SIM_TAU_S'])['SIM_TAU_S']
    if old is None:
        print('  no SIM_TAU_S in %s - nothing written' % a.cfg)
        return 2
    tau = [float(v) for v in old.split()]
    if len(tau) < 6:
        tau += [0.0] * (6 - len(tau))
    new = list(tau)
    for jn, rs in sorted(rows.items()):
        means = []
        for s in (1.0, -1.0):
            v = [x['tau'] for x in rs if x['sign'] == s and not x['at_max']]
            means.append(float(np.mean(v)) if v else None)
        if None in means:
            print('  J%d: no ramp moved in the %s direction (all at the maximum): not written; rerun with a larger '
                  'BREAKAWAY_MAX_NM' % (jn, '+' if means[0] is None else '-'))
            continue
        new[jn - 1] = 0.5 * (means[0] + means[1])
        print('  J%d: tau_s = (%.3f + %.3f) / 2 = %.3f N m' % (jn, means[0], means[1], new[jn - 1]))
    if new == tau:
        print('  nothing to write')
        return 1
    ok, res = cfgset.apply_files(paths, [('SIM_TAU_S', ' '.join('%.3f' % v for v in new))],
                                 tag='jt_breakaway --apply')
    if not ok:
        print('  ' + '\n  '.join(res) + '\n  nothing written')
        return 2
    print('  SIM_TAU_S written; restart the controller')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--apply', action='store_true', help='write tau_s into SIM_TAU_S of the cfg')
    ap.add_argument('--cfg', default=cfgset.CFG_MAIN, help='the cfg (default indy7/INDY7.cfg)')
    ap.add_argument('--platform', default=None, help='with --apply: the platform of the logs to use (robot, simulation)')
    a = ap.parse_args()
    paths = []
    for item in a.logs:
        paths += sorted(glob.glob(os.path.join(item, 'jt_brk_*.csv'))) if os.path.isdir(item) else [item]
    res = [r for r in (analyze(p) for p in paths) if r]
    if not res:
        sys.exit('jt_breakaway: no breakaway logs')
    by_joint = {}
    for r in res:
        print('%s (%s): J%d, ramp %.2f N m/s up to %.1f N m, %d ramps' % (r['log'], r['platform'], r['joint'],
                                                                        r['rate'], r['max'], len(r['ramps'])))
        for k, x in enumerate(r['ramps']):
            print('   ramp %2d %s  tau_s %s N m%s  moved %+.2f mrad  sign test %s' % (
                k + 1, '+' if x['sign'] > 0 else '-', '%.2f' % x['tau'] if not x['at_max'] else '>= max',
                ('  (drive: %.2f)' % abs(x['reported'])) if x['reported'] is not None else '', 1e3 * x['moved'],
                'n/a' if x['sign_ok'] is None else ('PASS' if x['sign_ok'] else 'FAIL')))
        by_joint.setdefault((r['joint'], r['platform']), []).extend(r['ramps'])
    print('\nbreakaway torque tau_s (N m), mean +- sd (min..max) over the ramps that moved; sign test')
    for (jn, plat), rs in sorted(by_joint.items()):
        cells = []
        for s, lab in ((1.0, '+'), (-1.0, '-')):
            v = [x['tau'] for x in rs if x['sign'] == s and not x['at_max']]
            nmax = sum(1 for x in rs if x['sign'] == s and x['at_max'])
            cells.append('%s: %s%s' % (lab, ('%.2f +- %.2f (%.2f..%.2f), n=%d' % (np.mean(v), np.std(v, ddof=1) if len(v) > 1
                                                                                    else 0.0, min(v), max(v), len(v)))
                                       if v else '-', (', %d at max' % nmax) if nmax else ''))
        sg = [x['sign_ok'] for x in rs if x['sign_ok'] is not None]
        print('   J%d (%s)  %s;  sign test %s (%d of %d ramps)' % (jn, plat, ';  '.join(cells),
                                                                  'PASS' if sg and all(sg) else ('FAIL' if sg else 'n/a'),
                                                                  sum(sg), len(sg)))
    if a.apply:
        return apply_tau_s(by_joint, a)
    return 0


if __name__ == '__main__':
    sys.exit(main())
