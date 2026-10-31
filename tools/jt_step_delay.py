#!/usr/bin/env python3
"""jt_step_delay.py - the torque-step test (JOINT TEST 5): in how many periods does the torque that the drive
reports follow a step of the commanded torque?

Reads jt_step_*.csv (with their .json) of the controller. In the test the controller commands g(q) plus a
small torque step on one joint (cfg TORQUE_STEP_*: on, off, sign alternating), and logs in every period the
step (tau_ff), the commanded torque (tau, N m; tau_cmd_raw, the drive units of the target torque 0x6071 of
that period) and the torque that the drive reported in that period (tau_act, N m; tau_act_raw, 0x6077). For
every edge of the step (period k0: the first period that computes the new value) the delay is the number of
periods until the reported torque has moved by half the commanded change from its mean over the 20 periods
before the edge; the 10-90 % rise is given as well. The Dynamics task computes the step in period k0 and the
EtherCAT task sends it in the next one (indy7/src/TaskSlots.h), so the written units change one period after
k0 ("cmd" 1). On the robot the delay then counts the frame to the drive, the internal application and
acquisition phases of the drive (a and b of Fig. 2, not measured in the paper, Sec. III-B) and the response
of its current loop, and the frame that carries the report back; in the simulation it is the pipeline
period plus the torque and the state delay of the simulator (cfg SIM_TORQUE_DELAY_STEPS,
SIM_STATE_DELAY_STEPS).

  python3 tools/jt_step_delay.py LOG_OR_DIR ... [--signal tau_act|tau_act_raw] [--csv OUT.csv]
--csv writes the mean normalized response around the edges (periods -5..30) for a figure.
Standard library and numpy.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

PRE = 20          # periods before the edge for the baseline
WIN = 30          # periods after the edge searched


def load(path):
    with open(os.path.splitext(path)[0] + '.json') as f:
        meta = json.load(f)
    with open(path) as f:
        head = f.readline().strip().split(',')
    data = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    return meta, {h: data[:, i] for i, h in enumerate(head)}


def edges(step, tau):
    """periods k0 at which the commanded step changes by more than half its size"""
    d = np.diff(step)
    return [int(k) + 1 for k in np.where(np.abs(d) > 0.5 * tau)[0]]


def analyze(path, signal):
    meta, c = load(path)
    j = int(meta.get('step_joint', 0)) - 1
    if j < 0:
        return None
    tau = float(meta['step_tau'])
    step = c['tau_ff%d' % j]
    sig = c['%s%d' % (signal, j)]
    raw_cmd = c.get('tau_cmd_raw%d' % j)
    out = {'log': os.path.basename(path), 'joint': j + 1, 'tau': tau, 'platform': meta.get('platform', '?'),
           'delays': [], 'rise': [], 'cmd_delay': [], 'resp': [], 'skipped': 0}
    if out['platform'] == 'simulation':
        out['sim'] = (meta.get('sim_torque_delay_steps'), meta.get('sim_state_delay_steps'),
                      int(meta.get('pipeline_periods', 1)))
    for k0 in edges(step, tau):
        if k0 < PRE or k0 + WIN >= len(sig):
            out['skipped'] += 1
            continue
        dcmd = step[k0] - step[k0 - 1]
        if signal == 'tau_act_raw':      # the change in drive units: from the units written
            if raw_cmd is None:
                out['skipped'] += 1
                continue
            dcmd = raw_cmd[k0] - raw_cmd[k0 - 1]
            if dcmd == 0:
                out['skipped'] += 1
                continue
        b = sig[k0 - PRE:k0].mean()
        r = (sig[k0 - 5:k0 + WIN] - b) / dcmd            # normalized response, 1 = the commanded change
        out['resp'].append(r)
        rel = r[5:]                                       # from k0 on
        half = np.where(rel >= 0.5)[0]
        if not len(half):
            out['skipped'] += 1
            continue
        out['delays'].append(int(half[0]))
        k10 = np.where(rel >= 0.1)[0]
        k90 = np.where(rel >= 0.9)[0]
        if len(k10) and len(k90):
            out['rise'].append(int(k90[0] - k10[0]))
        if raw_cmd is not None and signal == 'tau_act':
            # the period in which the written units change (1: the torque computed in period k0 is sent in the
            # next one)
            dk = np.where(np.abs(np.diff(raw_cmd[k0 - 2:k0 + 5])) > 0)[0]
            if len(dk):
                out['cmd_delay'].append(int(dk[0]) - 1)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--signal', choices=('tau_act', 'tau_act_raw'), default='tau_act')
    ap.add_argument('--csv', help='mean normalized response around the edges')
    a = ap.parse_args()
    paths = []
    for item in a.logs:
        paths += sorted(glob.glob(os.path.join(item, 'jt_step_*.csv'))) if os.path.isdir(item) else [item]
    res = [r for r in (analyze(p, a.signal) for p in paths) if r]
    if not res:
        sys.exit('jt_step_delay: no torque-step logs')
    alld, allr, resp = [], [], []
    print('torque-step test: periods from the edge of the commanded step until the reported torque (%s) '
          'has moved by half the commanded change' % a.signal)
    print('%-44s %-10s %5s %6s %6s %7s %9s %8s %9s' % ('log', 'platform', 'joint', 'step', 'edges', 'skipped',
                                                    'delay', 'rise', 'cmd'))
    for r in res:
        d = r['delays']
        hist = ' '.join('%d:%d' % (v, d.count(v)) for v in sorted(set(d))) if d else '-'
        print('%-44s %-10s %5d %6.2f %6d %7d %9s %8s %9s%s' % (
            r['log'][:44], r['platform'][:10], r['joint'], r['tau'], len(d), r['skipped'], hist,
            '%.1f' % np.mean(r['rise']) if r['rise'] else '-',
            ('%d' % max(set(r['cmd_delay']), key=r['cmd_delay'].count)) if r['cmd_delay'] else '-',
            ('   (simulation: torque delay %s steps, state delay %s steps, pipeline %s period)' % r['sim'])
            if 'sim' in r else ''))
        alld += d
        allr += r['rise']
        resp += r['resp']
    if alld:
        print('all: %d edges; delay %s periods (counts), mean %.2f, min %d, max %d; 10-90 %% rise %s periods'
              % (len(alld), ' '.join('%d:%d' % (v, alld.count(v)) for v in sorted(set(alld))), np.mean(alld),
                 min(alld), max(alld), '%.1f' % np.mean(allr) if allr else '-'))
    if a.csv and resp:
        m = np.mean(np.array(resp), axis=0)
        s = np.std(np.array(resp), axis=0)
        with open(a.csv, 'w') as f:
            f.write('k mean std\n')
            for i, (x, y) in enumerate(zip(m, s)):
                f.write('%d %.4f %.4f\n' % (i - 5, x, y))
        print('wrote %s (%d edges)' % (a.csv, len(resp)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
