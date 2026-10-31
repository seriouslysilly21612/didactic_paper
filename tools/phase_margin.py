#!/usr/bin/env python3
"""phase_margin.py - the model-predicted phase margin of the joint loops (paper, Sec. IV-B: Table 8, and the rows
"PM lost / left" of Table 7).

Single-joint approximation of the loop of eq. (5) with the delay tau_d between the measurement and the torque:

    L_i(s) = (Kd_i s + Kp_i) e^(-s tau_d) / s^2

At the gain crossover |L(j w_c)| = 1, i.e. w_c^4 = Kd^2 w_c^2 + Kp^2, the phase margin without the delay is
atan(Kd w_c / Kp); the delay takes w_c tau_d of it ("lost"), and what remains is the margin ("left").

  python3 tools/phase_margin.py [--cfg indy7/INDY7.cfg] [--tau-d 3.5 4.5 5.5]

The gains are KP_n and KD_n of the cfg; tau_d in ms (Table 8: 3.5, 4.5 and 5.5 ms; 3.5 ms: the nominal delay
3 T_s plus half a period of hold). Standard library only.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jt_predict as jp  # noqa: E402


def margin(kp, kd, tau_d):
    """(w_c [rad/s], lost [deg], left [deg]) of L(s) = (kd s + kp) e^(-s tau_d) / s^2"""
    wc = math.sqrt((kd * kd + math.sqrt(kd ** 4 + 4.0 * kp * kp)) / 2.0)
    pm0 = math.degrees(math.atan2(kd * wc, kp))
    lost = math.degrees(wc * tau_d)
    return wc, lost, pm0 - lost


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cfg', default=jp.DEFAULT_CFG)
    ap.add_argument('--tau-d', type=float, nargs='+', default=[3.5, 4.5, 5.5], help='[ms]')
    a = ap.parse_args()
    p = jp.cfg_params(a.cfg)
    kp, kd = p['Kp'], p['Kd']
    n = len(kp)
    print('gains of %s: Kp %s, Kd %s' % (os.path.relpath(a.cfg), kp, kd))
    print('%-22s' % 'omega_n (rad/s), zeta' + ''.join('%14s' % ('%.1f, %.2f' % (math.sqrt(kp[i]), kd[i] / (2 * math.sqrt(kp[i])))
                                                               if kp[i] > 0 else '-') for i in range(n)))
    for t in a.tau_d:
        cells = []
        for i in range(n):
            if kp[i] <= 0.0:
                cells.append('%14s' % '-')
                continue
            wc, lost, left = margin(kp[i], kd[i], 1e-3 * t)
            cells.append('%14s' % ('%.0f/%.1f' % (lost, left)))
        print('%-22s' % ('tau_d %.1f ms: lost/left' % t) + ''.join(cells))
    print('(degrees; J1 .. J%d; Table 8: the "left" values, Table 7: lost/left at 3.5 ms)' % n)
    return 0


if __name__ == '__main__':
    sys.exit(main())
