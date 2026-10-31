#!/usr/bin/env python3
"""sine_cfg.py - the sine test of one joint (JOINT TEST 7, 'u' '7') in the cfg, one command per joint: the
excitation of the identification of the rotor inertia (paper, Sec. IV-A; tools/identify_rotor.py).

  python3 tools/sine_cfg.py J [--cfg indy7/INDY7.cfg] [--kf NM]

  J1-J3: SINE_TEST_JOINT=J, SINE_TEST_SWEEP_RADS=0, SINE_TEST_AMP_RAD=0.03, SINE_TEST_FREQ_HZ=2.0,
         SINE_TEST_PERIODS=40 (20 s)
  J4-J6: SINE_TEST_JOINT=J, SINE_TEST_SWEEP_RADS=0.2, SINE_TEST_AMP_RAD=0.006, SINE_TEST_FREQ_HZ=4.0,
         SINE_TEST_PERIODS=32 (8 s), and KF_<J-1> = tau_s of J (SIM_TAU_S, written by jt_breakaway.py
         --apply; --kf gives another value), so that the wrist keeps sliding on the slow sweep
Restart the controller, then 'u' '7' three times. The start of the test prints its reach: check the space
around the arm. J6 needs its B first (ROTOR_INERTIA_5 > 0: identify_rotor.py --apply with the J5 runs).
The sine test changes KF_<J-1> of J4-J6: set the K_f of the controller afterwards (tools/kf_step.py).
Standard library only.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cfgset  # noqa: E402

BIG = dict(SINE_TEST_SWEEP_RADS='0', SINE_TEST_AMP_RAD='0.03', SINE_TEST_FREQ_HZ='2.0', SINE_TEST_PERIODS='40')
WRIST = dict(SINE_TEST_SWEEP_RADS='0.2', SINE_TEST_AMP_RAD='0.006', SINE_TEST_FREQ_HZ='4.0', SINE_TEST_PERIODS='32')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('joint', type=int, choices=range(1, 7), metavar='J', help='1..6')
    ap.add_argument('--cfg', default=cfgset.CFG_MAIN, help='the cfg (default indy7/INDY7.cfg)')
    ap.add_argument('--kf', type=float, default=None, help='J4-J6: K_f [N m] instead of tau_s')
    a = ap.parse_args()
    j = a.joint
    if not os.path.exists(a.cfg):
        sys.exit('sine_cfg: %s missing' % a.cfg)
    vals = BIG if j <= 3 else WRIST
    items = [('SINE_TEST_JOINT', str(j))] + sorted(vals.items())
    cur = cfgset.values(a.cfg, ['SIM_TAU_S', 'ROTOR_INERTIA_5', 'SINE_TEST_FF', 'KF_%d' % (j - 1)])
    if j >= 4:
        if a.kf is not None:
            kf = a.kf
        else:
            tau = [float(v) for v in (cur['SIM_TAU_S'] or '').split()]
            kf = tau[j - 1] if len(tau) >= j else 0.0
            if kf <= 0.0:
                sys.exit('sine_cfg: no tau_s of J%d in SIM_TAU_S of %s (%s): jt_breakaway.py --apply first, or --kf'
                         % (j, a.cfg, cur['SIM_TAU_S']))
        items.append(('KF_%d' % (j - 1), '%.3f' % kf))
        if (cur['SINE_TEST_FF'] or '1').strip() == '0':
            print('sine_cfg: note: SINE_TEST_FF=0 in %s: the test runs without the friction feedforward' % a.cfg)
    if j == 6 and float(cur['ROTOR_INERTIA_5'] or 0.0) <= 0.0:
        sys.exit('sine_cfg: ROTOR_INERTIA_5 is 0: J6 cannot follow the sine without its B - identify_rotor.py '
                 '--apply with the J5 runs first (it sets B6 = B5)')
    ok, res = cfgset.apply_files([a.cfg], items, tag='sine_cfg %d' % j, prefix='')
    if not ok:
        sys.exit('sine_cfg: ' + '; '.join(res) + ' - nothing written')
    per = int(vals['SINE_TEST_PERIODS']) / float(vals['SINE_TEST_FREQ_HZ'])
    print('sine_cfg: J%d, A %s rad, %s Hz, %s periods (%.0f s), sweep %s rad/s%s; restart the controller, then '
          'u 7 x 3 (the start prints the reach)' % (j, vals['SINE_TEST_AMP_RAD'], vals['SINE_TEST_FREQ_HZ'], vals['SINE_TEST_PERIODS'],
                                 per, vals['SINE_TEST_SWEEP_RADS'],
                                 (', K_f %.3f N m' % kf) if j >= 4 else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
