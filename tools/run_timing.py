#!/usr/bin/env python3
"""run_timing.py - one condition of the host-timing test (paper, Sec. III-C, Table 6, Fig. 3).

The bus of the robot in OP with its drives off, beside the simulated arm (-b), the reference-check goal
stream (high-level test 5: newly sampled reachable goals, unreachable goals, repeated goals, goal-free
intervals, 1-mm perturbations every 67 periods) for --duration s, the timing of every period (-t), and the
summary of tools/timing_summary.py (the row of Table 6).

  python3 tools/run_timing.py --condition idle|stress|perception [--duration 600] [--cfg indy7/INDY7.cfg]
                              [--out DIR] [--settle 30] [--direct]

  idle        nothing else runs on the host
  stress      stress-ng --taskset OTHERS --cpu 14 --vm 2 --vm-bytes 35% (Table 6), started 120 s before the
              controller; OTHERS = the CPUs besides the real-time core and its sibling (default 0-5,8-15)
  perception  the camera pipeline runs (started by the operator beforehand: ros2 launch system_bringup_pkg
              pick_place_openvino.launch.py; checked here by its processes), and the controller runs its
              bridge (a build with ROS 2, APP_BRIDGE=1; the environments of ROS 2 sourced)

In order:
  1. checks, warnings only: the host as in Table 3 (PREEMPT_RT, the core isolated, kernel.timer_migration 0,
     NTP not synchronizing the clock), the bus (the IgH master running), the controller built
  2. a copy of the cfg in OUT: HL_TEST_REF_S = the duration, VIEW_PORT = 0, JOINT_TEST_LOG_DIR = OUT/hl_logs,
     its relative paths made absolute
  3. the load of the condition
  4. the controller through indy7/run.sh (the limits, EtherCAT-OP at SCHED_FIFO 40) with -b -t OUT; --settle s
     in OP; then the keys r, g, k (the hold off), h (servo on of the simulated arm; the drives of the bus stay
     off), y 5 (HOME to q_r, then TRACK with the stream); its end; j, q
  5. the load stopped; tools/timing_summary.py on the timing log: OUT/timing_summary.txt, timing_table.csv
The console of the controller is saved as OUT/console.txt. The high-level log of the run takes about 0.7 kB
per period (OUT/hl_logs), the timing log 60 bytes per period. Standard library only.
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import jt_predict as jp  # noqa: E402
from jt_run_sim import APP, Console, FAIL_START, CTRL_ON, GRAV_COMP  # noqa: E402

PATH_KEYS = ('URDF_PATH', 'SIM_MJCF_PATH', 'JOINT_TEST_LOG_DIR', 'CALIB_DIR', 'HL_TEST_NOISE_FILE')
STRESS_ARGS = ['--cpu', '14', '--vm', '2', '--vm-bytes', '35%', '--timeout', '0', '--quiet']
PERCEPTION = ('realsense2_camera_node', 'openvino_detector_node', 'pick_target_base_node')
B_READY = r'\(CRobotIndy7\) \S+: the bus of the robot \(drives off\) beside the simulated arm \(-b\)'


def warn(text):
    print('run_timing: WARNING: %s' % text)


def cpu_list(text):
    """'domain,managed_irq,6-7' or '0-5,8-15' -> the set of CPU numbers"""
    cpus = set()
    for part in text.split(','):
        if re.fullmatch(r'\d+-\d+', part):
            lo, hi = part.split('-')
            cpus.update(range(int(lo), int(hi) + 1))
        elif part.isdigit():
            cpus.add(int(part))
    return cpus


def checks(a, rt_cpu):
    try:
        rt = open('/sys/kernel/realtime').read().strip() == '1'
    except OSError:
        rt = False
    if not rt:
        warn('not a PREEMPT_RT kernel (Table 3)')
    m = re.search(r'\bisolcpus=(\S+)', open('/proc/cmdline').read())
    if not m or rt_cpu not in cpu_list(m.group(1)):
        warn('CPU %d of the real-time tasks is not in isolcpus= of the kernel command line (Table 3)' % rt_cpu)
    try:
        if open('/proc/sys/kernel/timer_migration').read().strip() != '0':
            warn('kernel.timer_migration is not 0 (Table 3: sudo sysctl kernel.timer_migration=0)')
    except OSError:
        pass
    try:
        out = subprocess.run(['timedatectl', 'show', '-p', 'NTP', '--value'], capture_output=True, text=True,
                             timeout=5).stdout.strip()
        if out == 'yes':
            warn('NTP is active (Table 3: stopped during the measurements; sudo timedatectl set-ntp false)')
    except (OSError, subprocess.SubprocessError):
        pass
    if subprocess.run(['pgrep', '-x', 'EtherCAT-OP'], capture_output=True).returncode != 0:
        warn('no EtherCAT-OP thread: is the IgH master running (ethercat.service)?')
    if not os.path.exists(os.path.join(APP, 'bin', 'indy7_ctrl')):
        sys.exit('run_timing: indy7/bin/indy7_ctrl is missing: make')


def cfg_copy(src, dst, out, duration):
    """the cfg of the run: the test values set, the relative paths of the source made absolute"""
    base = os.path.dirname(os.path.abspath(src))
    sets = {'HL_TEST_REF_S': '%g' % duration, 'VIEW_PORT': '0', 'JOINT_TEST_LOG_DIR': os.path.join(out, 'hl_logs')}
    lines, done = [], set()
    for line in open(src).read().split('\n'):
        m = re.match(r'^([A-Za-z0-9_]+)(\s*=\s*)(.*)$', line)
        if m and m.group(1) in sets and m.group(1) not in done:
            line = '%s=%s' % (m.group(1), sets[m.group(1)])
            done.add(m.group(1))
        elif m and m.group(1) in PATH_KEYS:
            v = m.group(3).split(';', 1)[0].strip()
            if v and not os.path.isabs(v):
                line = '%s=%s' % (m.group(1), os.path.normpath(os.path.join(base, v)))
        lines.append(line)
    missing = [k for k in sets if k not in done]
    if missing:
        sys.exit('run_timing: %s lacks %s' % (src, ', '.join(missing)))
    with open(dst, 'w') as f:
        f.write('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--condition', required=True, choices=('idle', 'stress', 'perception'))
    ap.add_argument('--duration', type=float, default=600.0, help='[s] of the goal stream (Table 6: 600)')
    ap.add_argument('--cfg', default=os.path.join(APP, 'INDY7.cfg'))
    ap.add_argument('--out', help='results (default ./timing_<condition>_<date>)')
    ap.add_argument('--settle', type=float, default=30.0, help='[s] in OP before the keys')
    ap.add_argument('--others', default=os.environ.get('OTHERS', '0-5,8-15'), help='CPUs of the stress-ng load')
    ap.add_argument('--direct', action='store_true', help='start bin/indy7_ctrl instead of run.sh (no limit check)')
    a = ap.parse_args()

    src = os.path.abspath(a.cfg)
    rt_cpu = int(float(jp.cfg_get(src, 'TASK0', 'CPU', '6') or 6))
    out = os.path.abspath(a.out or 'timing_%s_%s' % (a.condition, time.strftime('%Y%m%d_%H%M')))
    os.makedirs(os.path.join(out, 'hl_logs'), exist_ok=True)
    checks(a, rt_cpu)
    cfg = os.path.join(out, 'INDY7.cfg')
    cfg_copy(src, cfg, out, a.duration)

    load = None
    if a.condition == 'stress':
        load = subprocess.Popen(['stress-ng', '--taskset', a.others] + STRESS_ARGS)
        print('run_timing: stress-ng --taskset %s %s; 120 s warm-up' % (a.others, ' '.join(STRESS_ARGS)))
        time.sleep(120)
        if load.poll() is not None:
            sys.exit('run_timing: stress-ng did not start')
    elif a.condition == 'perception':
        missing = [p for p in PERCEPTION if subprocess.run(['pgrep', '-f', p], capture_output=True).returncode]
        if missing:
            sys.exit('run_timing: the camera pipeline is not running (no %s): start it first' % ', '.join(missing))

    cmd = (['./bin/indy7_ctrl'] if a.direct else ['./run.sh']) + ['-f', cfg, '-b', '-t', out]
    print('run_timing: %s (in %s)' % (' '.join(cmd), APP))
    c = Console(cmd, APP, dict(os.environ, MALLOC_ARENA_MAX='2'), os.path.join(out, 'console.txt'))
    code = 0
    try:
        c.expect(B_READY, 60, fail=FAIL_START)
        c.expect(r'\[BUS\] OP: \d+ slaves', 60, fail=FAIL_START)
        if a.condition == 'perception':
            c.expect(r'\[bridge\] up', 10, fail=r'\[bridge\] cannot start|built without ROS 2')
        print('run_timing: the bus in OP; %.0f s settle' % a.settle)
        time.sleep(a.settle)
        c.send('r')
        c.expect(CTRL_ON, 10)
        c.send('g')
        c.expect(GRAV_COMP, 10)
        for _ in range(2):
            c.send('k')
            if c.expect(r'grav-comp hold (ON|OFF)', 10).group(1) == 'OFF':
                break
        c.send('h')
        c.expect(r'\[ARM\] servo on: every joint in Operation enabled', 10)
        c.send('y')
        c.expect(r'\[HL\] high-level tests', 10)
        c.send('5')
        c.expect(r'\[HL\] reference check: HOME to q_r', 10, fail=r'\[HL\] refused[^\n]*')
        c.expect(r'\[HL\] TRACK: reference check', 120, fail=r'\[HL\] (HOME stopped|TRACK refused)[^\n]*')
        print('run_timing: the goal stream runs for %.0f s' % a.duration)
        m = c.expect(r'\[HL\] reference check (done|ended|stopped)[^\n]*', a.duration + 300)
        print('run_timing: %s' % m.group(0).strip())
        time.sleep(1.0)
        c.send('j')
        c.expect(r'SERVO OFF', 10)
        time.sleep(0.5)
        c.send('q')
    except (RuntimeError, KeyboardInterrupt) as ex:
        print('run_timing: stopped: %s' % ex)
        code = 1
        try:
            c.send('j')
            time.sleep(0.3)
            c.send('q')
        except OSError:
            pass
    c.close()
    if load is not None:
        load.terminate()
        try:
            load.wait(timeout=10)
        except subprocess.TimeoutExpired:
            load.kill()
        subprocess.run(['pkill', '-f', '^stress-ng'], capture_output=True)
    with c.lock:
        for line in c.text.splitlines():
            if line.startswith(('[RT]', '[BUS]', '[SIM]')):
                print('run_timing: ' + line)
    logs = sorted(glob.glob(os.path.join(out, 'timing_*.bin')))
    if not logs:
        print('run_timing: no timing log in %s' % out)
        return 1
    r = subprocess.run([sys.executable, os.path.join(HERE, 'timing_summary.py'), logs[-1], '--out', out])
    return code or r.returncode


if __name__ == '__main__':
    sys.exit(main())
