#!/usr/bin/env python3
"""hl_run_sim.py - run the high-level tests in the simulation with the keys of the operator (paper, Sec. V-B,
Fig. 6).

Like tools/jt_run_sim.py: starts the controller in simulation mode (-s), presses r, g, k (hold OFF), and for
every run of every test h (servo on), y and the menu digit, waits for '[HL] saved', then j (servo off: the
brakes hold the arm between tests). Every test moves to q_r first (HOME), then runs TRACK with goals that the
test program of the controller writes into the goal slot:
  hl-fixed (1)     random goals in a box (cfg HL_RAND_*, default 50, each from q_r), one key press runs
                   them all; with HL_RAND_GOALS=0 the fixed goal 0.3 m away, default 5 runs
  hl-jump (2)      random goal jumps (cfg HL_RAND_JUMP*, default 50, each from q_r), one key press runs them
                   all; with HL_RAND_JUMPS=0 the goal of (1) moved 0.15 m sideways during the motion
  hl-noise (3)     goal with noise (synthetic, or replayed from HL_TEST_NOISE_FILE), threshold delta
  hl-noise0 (4)    the same with delta = 0
  hl-refcheck (5)  reference check: goal changes, goals out of reach, repeats, gaps (HL_TEST_REF_S); the
                   goal stream of the host-timing test (Sec. III-C)
The script drives nothing but the simulation.

Usage (in the repository):
  python3 tools/hl_run_sim.py [-n N] [--tests hl-fixed,hl-jump,...] [--analyze]
  --cfg FILE, --direct: as for jt_run_sim.py
"""
import argparse
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jt_predict as jp  # noqa: E402
from jt_run_sim import APP, Console, FAIL_START, SIM_START, SIM_READY, CTRL_ON, GRAV_COMP  # noqa: E402

MENU = {'hl-fixed': '1', 'hl-jump': '2', 'hl-noise': '3', 'hl-noise0': '4', 'hl-refcheck': '5'}
DEFAULT_TESTS = ['hl-fixed', 'hl-jump', 'hl-noise', 'hl-noise0', 'hl-refcheck']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-n', type=int, default=None,
                    help='runs of hl-fixed: sequences of the random goals (default 1), or runs of the fixed goal '
                         '(HL_RAND_GOALS=0, default 5); the other tests run --runs-all times')
    ap.add_argument('--runs-all', type=int, default=1, help='runs of every other test')
    ap.add_argument('--tests', help='comma separated, from: ' + ', '.join(MENU))
    ap.add_argument('--cfg', default=os.path.join(APP, 'INDY7.cfg'))
    ap.add_argument('--direct', action='store_true', help='start bin/indy7_ctrl instead of run.sh')
    ap.add_argument('--settle', type=float, default=0.3, help='[s] between servo on and the test')
    ap.add_argument('--analyze', action='store_true', help='run tools/hl_analyze.py on the new logs')
    a = ap.parse_args()

    cfg = os.path.abspath(a.cfg)
    tests = a.tests.split(',') if a.tests else DEFAULT_TESTS
    for t in tests:
        if t not in MENU:
            sys.exit('hl_run_sim: unknown test %s (%s)' % (t, ', '.join(MENU)))
    num = lambda k, d: float((jp.cfg_get(cfg, 'SYSTEM', k) or str(d)).split()[0])
    longest = max(num('HL_TEST_REF_S', 60), num('HL_TEST_NOISE_S', 10), 20.0) + 15.0
    log_dir = jp.cfg_get(cfg, 'SYSTEM', 'JOINT_TEST_LOG_DIR', 'jt_logs') or 'jt_logs'
    if not os.path.isabs(log_dir):
        log_dir = os.path.join(os.path.dirname(cfg), log_dir)
    os.makedirs(log_dir, exist_ok=True)
    console_log = os.path.join(log_dir, 'run_hl_%s.txt' % time.strftime('%Y%m%d_%H%M%S'))

    n_rand = int(num('HL_RAND_GOALS', 50))
    n_fixed = a.n if a.n is not None else (1 if n_rand > 0 else 5)
    plan = []          # run-major: run 1 of every test, then run 2, ...
    for run in range(1, max(n_fixed, a.runs_all) + 1):
        for t in tests:
            if run <= (n_fixed if t == 'hl-fixed' else a.runs_all):
                plan.append((run, t))
    env = dict(os.environ, MALLOC_ARENA_MAX='2')
    cmd = ['./bin/indy7_ctrl', '-f', cfg, '-s'] if a.direct else ['./run.sh', '-s', '-f', cfg]
    print('hl_run_sim: %s (in %s); %d test runs' % (' '.join(cmd), APP, len(plan)))
    print('hl_run_sim: console -> %s' % console_log)
    c = Console(cmd, APP, env, console_log)
    saved = []
    try:
        c.expect(SIM_START, 30, fail=FAIL_START)
        c.expect(SIM_READY, 30, fail=FAIL_START)
        time.sleep(0.5)
        c.send('r')
        c.expect(CTRL_ON, 5)
        c.send('g')
        c.expect(GRAV_COMP, 5)
        for _ in range(2):
            c.send('k')
            if c.expect(r'grav-comp hold (ON|OFF)', 5).group(1) == 'OFF':
                break
        else:
            raise RuntimeError('cannot switch the hold off')
        with c.lock:
            mg = re.search(r'\[HL\] test 1: (\d+) random goals', c.text)
        goals = int(mg.group(1)) if mg else 0          # what the controller found, 0: the fixed goal
        if goals:
            print('hl_run_sim: hl-fixed runs %d random goals per key press' % goals)
        with c.lock:
            mj = re.search(r'\[HL\] test 2: (\d+) random goal jumps', c.text)
        jumps = int(mj.group(1)) if mj else 0          # 0: the fixed jump
        if jumps:
            print('hl_run_sim: hl-jump runs %d random goal jumps per key press' % jumps)
        seq = {'hl-fixed': (goals, 'goal', r'\[HL\] random goals: all \d+ done'),
               'hl-jump': (jumps, 'jump', r'\[HL\] random goal jumps: all \d+ done')}
        for run, t in plan:
            c.send('h')
            c.expect(r'SERVO ON', 5)
            time.sleep(a.settle)
            c.send('y')
            c.expect(r'\[HL\] high-level tests', 5)
            with c.lock:
                pos0 = len(c.text)
            c.send(MENU[t])
            n_seq, what, all_done = seq.get(t, (0, '', ''))
            for k in range(n_seq if n_seq else 1):
                c.expect(r'\[HL\] [^\n]*: HOME to q_r', 15,
                         fail=r'\[HL\] (refused|random goal(s| jumps) stopped)[^\n]*')
                c.expect(r'\[HL\] TRACK:', 30, fail=r'\[HL\] (HOME stopped|TRACK refused)[^\n]*')
                m = c.expect(r'\[HL\] saved (\S+) \(([^)]*)\)', longest)
                m2 = c.expect(r'\[HL\] tool ([^\n]*)', 5)
                saved.append(m.group(1))
                print('hl_run_sim: run %d %-12s %s%s' % (run, t, m.group(2),
                                                          ' (%s %d of %d)' % (what, k + 1, n_seq) if n_seq else ''))
                print('            %s' % m2.group(1))
            if n_seq:
                # printed by the Trajectory task when the last run ends, i.e. before the log writer's lines
                t0 = time.time()
                while True:
                    with c.lock:
                        done = re.search(all_done, c.text[pos0:])
                    if done or time.time() - t0 > 15:
                        break
                    time.sleep(0.05)
                if not done:
                    raise RuntimeError('the random %ss did not end as expected (see the console log)' % what)
            c.send('j')
            c.expect(r'SERVO OFF', 5)
            time.sleep(0.2)
        c.send('q')
    except (RuntimeError, KeyboardInterrupt) as ex:
        print('hl_run_sim: stopped: %s' % ex)
        try:
            c.send('j')
            time.sleep(0.3)
            c.send('q')
        except OSError:
            pass
        c.close()
        return 1
    c.close()
    with c.lock:
        for line in c.text.splitlines():
            if line.startswith(('[SIM]', '[RT]', '[MALLOC]')):
                print('hl_run_sim: ' + line)
    print('hl_run_sim: %d logs in %s' % (len(saved), log_dir))
    if a.analyze and saved:
        import hl_analyze
        sys.argv = ['hl_analyze.py'] + saved
        return hl_analyze.main()
    return 0


if __name__ == '__main__':
    sys.exit(main())
