#!/usr/bin/env python3
"""jt_run_sim.py - run the JOINT TESTs in the simulation, n times each, with the keys of the operator.

Starts the controller in simulation mode (-s) and presses its keys through a pseudo-terminal, waiting for the
answer of the controller to each:
  r (controller on), g (gravity compensation), k until the hold is OFF (in the simulation without the rotor
  inertia of J6, SIM_ARMATURE 0, the hold is unstable at J6), then for every run of every test
    h            servo on: the brakes release
    u, 1..7      the test; then wait for '[JT] saved'
    j            servo off: the brakes hold the arm (without friction the arm in gravity compensation
                 drifts away within minutes, because the gravity torque is computed from a posture 3.5 ms
                 old; the brakes keep every run starting from the same posture)
  and finally q. Runs are interleaved (run 1 of every test, then run 2, ...), as on the robot.

Tests (menu entries of 'u'): fast-pdg (1), fast-ctc (2), slow-ctc (3), slow-ctcff (4), step (5), breakaway (6),
sine (7). Default: fast-pdg, fast-ctc, slow-ctc, and slow-ctcff only if the simulation has friction
(SIM_FRICTION_ON=1). The sine test identifies the rotor inertia of SINE_TEST_JOINT (--analyze:
tools/identify_rotor.py; the simulated arm has the known armature SIM_ARMATURE, default ROTOR_INERTIA_n, which
the identification must recover).
The script drives nothing but the simulation: on the robot, the tests are started by hand, next to the
emergency stop.

Usage (in the repository; run.sh checks the limits: rtprio >= 97, memlock unlimited):
  python3 tools/jt_run_sim.py [-n 5] [--tests fast-pdg,fast-ctc,slow-ctc] [--analyze]
  --cfg FILE    another cfg (default indy7/INDY7.cfg, through run.sh)
  --direct      start bin/indy7_ctrl directly instead of run.sh (no check of the limits)
The console of the controller is saved next to the logs: <JOINT_TEST_LOG_DIR>/run_sim_<time>.txt.
"""
import argparse
import os
import pty
import re
import select
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.normpath(os.path.join(HERE, '..', 'indy7'))
sys.path.insert(0, HERE)
import jt_predict as jp  # noqa: E402

MENU = {'fast-pdg': '1', 'fast-ctc': '2', 'slow-ctc': '3', 'slow-ctcff': '4', 'step': '5', 'breakaway': '6',
        'sine': '7'}
ANSI = re.compile(rb'\x1b\[[0-9;]*[A-Za-z]')
# the console of the controller (indy7/src/main.cpp, Indy7Ctrl.cpp)
FAIL_START = r'the controller cannot start|mlockall failed|usage:'
SIM_START = r'simulation, MuJoCo in place of the bus'
SIM_READY = r'\(CRobotIndy7\) \S+: simulation \(MuJoCo in place of the bus\)'
CTRL_ON = r'\(CRobotIndy7\) controller ENABLED'
GRAV_COMP = r'\(CRobotIndy7\) mode: Gravity Compensation'


class Console:
    """The controller on a pseudo-terminal: send keys, wait for lines."""

    def __init__(self, cmd, cwd, env, log_path):
        self.master, slave = pty.openpty()
        self.proc = subprocess.Popen(cmd, cwd=cwd, env=env, stdin=slave, stdout=slave, stderr=slave,
                                     start_new_session=True, close_fds=True)
        os.close(slave)
        self.text = ''
        self.pos = 0
        self.lock = threading.Lock()
        self.log = open(log_path, 'w')
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while True:
            try:
                r, _, _ = select.select([self.master], [], [], 0.2)
                if not r:
                    if self.proc.poll() is not None:
                        break
                    continue
                chunk = os.read(self.master, 65536)
            except OSError:
                break
            if not chunk:
                break
            s = ANSI.sub(b'', chunk).decode('utf-8', 'replace').replace('\r', '')
            with self.lock:
                self.text += s
            self.log.write(s)
            self.log.flush()
        self.alive = False

    def send(self, key):
        os.write(self.master, key.encode())

    def expect_all(self, patterns, timeout, fail=None):
        """wait until every regex has appeared after the last match, in any order"""
        rxs = [re.compile(x) for x in patterns]
        rf = re.compile(fail) if fail else None
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                tail = self.text[self.pos:]
            ms = [rx.search(tail) for rx in rxs]
            if all(ms):
                self.pos += max(m.end() for m in ms)
                return ms
            if rf is not None:
                mf = rf.search(tail)
                if mf:
                    raise RuntimeError('controller: %s' % tail[mf.start():].splitlines()[0])
            if not self.alive and self.proc.poll() is not None:
                raise RuntimeError('the controller has exited (see the console log)')
            time.sleep(0.02)
        missing = [x for x, m in zip(patterns, [rx.search(self.text[self.pos:]) for rx in rxs]) if not m]
        raise RuntimeError('no "%s" within %.0f s' % ('", "'.join(missing), timeout))

    def expect(self, pattern, timeout, fail=None):
        """wait for the regex in the output after the last match; fail: a regex that aborts"""
        rx = re.compile(pattern)
        rf = re.compile(fail) if fail else None
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                tail = self.text[self.pos:]
            m = rx.search(tail)
            if m:
                self.pos += m.end()
                return m
            if rf is not None:
                mf = rf.search(tail)
                if mf:
                    raise RuntimeError('controller: %s' % tail[mf.start():].splitlines()[0])
            if not self.alive and self.proc.poll() is not None:
                raise RuntimeError('the controller has exited (see the console log)')
            time.sleep(0.02)
        raise RuntimeError('no "%s" within %.0f s' % (pattern, timeout))

    def close(self, timeout=20):
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
        time.sleep(0.3)
        self.log.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-n', type=int, default=5, help='runs of every test')
    ap.add_argument('--tests', help='comma separated, from: ' + ', '.join(MENU))
    ap.add_argument('--cfg', default=os.path.join(APP, 'INDY7.cfg'))
    ap.add_argument('--direct', action='store_true', help='start bin/indy7_ctrl instead of run.sh')
    ap.add_argument('--settle', type=float, default=0.3, help='[s] between servo on and the test')
    ap.add_argument('--analyze', action='store_true', help='run tools/jt_analyze.py on the new logs')
    a = ap.parse_args()

    cfg = os.path.abspath(a.cfg)
    friction = (jp.cfg_get(cfg, 'SYSTEM', 'SIM_FRICTION_ON', '0') or '0').strip() == '1'
    tests = a.tests.split(',') if a.tests else ['fast-pdg', 'fast-ctc', 'slow-ctc'] + (['slow-ctcff'] if friction else [])
    for t in tests:
        if t not in MENU:
            sys.exit('jt_run_sim: unknown test %s (%s)' % (t, ', '.join(MENU)))
    p = jp.cfg_params(cfg)
    def duration(t):
        if t == 'step':     # the torque-step test (JOINT TEST 5)
            num = lambda k, d: float((jp.cfg_get(cfg, 'SYSTEM', k, str(d)) or str(d)).split(';')[0])
            return num('TORQUE_STEP_REPEAT', 20) * (num('TORQUE_STEP_ON_S', 0.2) + num('TORQUE_STEP_OFF_S', 0.3)) + 5.0
        if t == 'breakaway':  # the breakaway test (JOINT TEST 6): the longest ramp and the damping
            num = lambda k, d: float((jp.cfg_get(cfg, 'SYSTEM', k, str(d)) or str(d)).split(';')[0])
            return 2 * num('BREAKAWAY_REPEAT', 5) * (num('BREAKAWAY_MAX_NM', 15.0) / num('BREAKAWAY_RATE_NMS', 2.0)
                                                     + 3.0) + 5.0
        if t == 'sine':     # the sine test (JOINT TEST 7)
            num = lambda k, d: float((jp.cfg_get(cfg, 'SYSTEM', k, str(d)) or str(d)).split(';')[0])
            return num('SINE_TEST_PERIODS', 60) / num('SINE_TEST_FREQ_HZ', 3.0) + 5.0
        return jp.test_duration(p['tests'][t.split('-')[0]]['T'], p['dwell'])
    longest = max(duration(t) for t in tests)
    log_dir = jp.cfg_get(cfg, 'SYSTEM', 'JOINT_TEST_LOG_DIR', 'jt_logs') or 'jt_logs'
    if not os.path.isabs(log_dir):
        log_dir = os.path.join(os.path.dirname(cfg), log_dir)
    os.makedirs(log_dir, exist_ok=True)
    console_log = os.path.join(log_dir, 'run_sim_%s.txt' % time.strftime('%Y%m%d_%H%M%S'))

    env = dict(os.environ, MALLOC_ARENA_MAX='2')
    if a.direct:
        cmd = ['./bin/indy7_ctrl', '-f', cfg, '-s']
    else:
        cmd = ['./run.sh', '-s', '-f', cfg]
    print('jt_run_sim: %s (in %s); tests %s x %d; friction %s' % (' '.join(cmd), APP, ', '.join(tests), a.n,
                                                                  'on' if friction else 'off'))
    print('jt_run_sim: console -> %s' % console_log)
    c = Console(cmd, APP, env, console_log)
    saved = []
    try:
        c.expect(SIM_START, 30, fail=FAIL_START)
        # the simulated arm, then the three real-time tasks and the keyboard task (verified at their start)
        ms = c.expect_all([r'\(CSimArm\) .*friction (on|off)', SIM_READY], 30, fail=FAIL_START)
        if (ms[0].group(1) == 'on') != friction:
            raise RuntimeError('the simulation reports friction %s, the cfg says %s' % (ms[0].group(1), friction))
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
        for run in range(1, a.n + 1):
            for t in tests:
                c.send('h')
                c.expect(r'SERVO ON', 5)
                time.sleep(a.settle)
                c.send('u')
                c.expect(r'\[JT\] JOINT TEST', 5)
                c.send(MENU[t])
                c.expect(r'\[JT\] start', 5, fail=r'\[JT\] refused[^\n]*')
                m = c.expect(r'\[JT\] saved (\S+) \(([^)]*)\)', longest + 20)
                saved.append(m.group(1))
                c.expect(r'\[JT\] max \|e\| \[mrad\]:([^\n]*)', 5)
                print('jt_run_sim: run %d/%d %-10s %s' % (run, a.n, t, m.group(2)))
                c.send('j')
                c.expect(r'SERVO OFF', 5)
                time.sleep(0.2)
        c.send('q')
    except (RuntimeError, KeyboardInterrupt) as ex:
        print('jt_run_sim: stopped: %s' % ex)
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
                print('jt_run_sim: ' + line)
    print('jt_run_sim: %d logs in %s' % (len(saved), log_dir))
    steps = [f for f in saved if os.path.basename(f).startswith('jt_step_')]
    brks = [f for f in saved if os.path.basename(f).startswith('jt_brk_')]
    sines = [f for f in saved if os.path.basename(f).startswith('jt_sine_')]
    saved = [f for f in saved if f not in steps and f not in brks and f not in sines]
    if a.analyze and sines:
        import identify_rotor
        sys.argv = ['identify_rotor.py'] + sines
        identify_rotor.main()
    if a.analyze and steps:
        import jt_step_delay
        sys.argv = ['jt_step_delay.py'] + steps
        jt_step_delay.main()
    if a.analyze and brks:
        import jt_breakaway
        sys.argv = ['jt_breakaway.py'] + brks
        jt_breakaway.main()
    if a.analyze and saved:
        import jt_analyze
        sys.argv = ['jt_analyze.py'] + saved
        return jt_analyze.main()
    return 0


if __name__ == '__main__':
    sys.exit(main())
