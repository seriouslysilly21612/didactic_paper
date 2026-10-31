#!/usr/bin/env python3
"""place_check.py - put the object on the positions P1-P5 of the camera-guided reaching test (paper, Sec. V-C)
without measuring them: the position of the object that the camera pipeline publishes (/pick_target_base,
base frame) against the nearest P, twice a second, until Ctrl-C.

    python3 tools/place_check.py [--p x,y/x,y/...] [--tol 0.02] [--once]

The P of the test: P1 (0.40, -0.25), P2 (0.70, -0.25), P3 (0.65, 0.35), P4 (0.40, 0.45), P5 (0.55, 0.10) m;
--p replaces them, e.g. --p 0.40,-0.25/0.70,-0.25 . Shown per line: the mean of the valid samples of the last
second (x, y, z, and their largest standard deviation), the nearest P and the move that brings the object
onto it: +x is away from the base of the robot, +y to the left as seen from the base. 'OK' when the object
is within --tol [m] of the P horizontally. The positions are those of the current calibration (the transform
of the launch file); after a new calibration they move with it, which keeps the same places in the base
frame. --once: one line after 2 s of samples, exit code 0 if OK. Needs the ROS 2 environment (source
/opt/ros/jazzy/setup.bash and ros2/install/setup.bash) and the pipeline running.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from my_interfaces.msg import PickTarget3D

PLACES = [('P1', 0.40, -0.25), ('P2', 0.70, -0.25), ('P3', 0.65, 0.35), ('P4', 0.40, 0.45), ('P5', 0.55, 0.10)]


def parse_places(text):
    out = []
    for i, item in enumerate(text.split('/')):
        x, y = (float(v) for v in item.split(','))
        out.append(('P%d' % (i + 1), x, y))
    return out


def advice(name, px, py, x, y, tol):
    dx, dy = px - x, py - y
    d = math.hypot(dx, dy)
    if d <= tol:
        return '%s OK (%.1f cm)' % (name, 100.0 * d)
    parts = []
    if abs(dx) > 0.005:
        parts.append('%s %.1f cm' % ('away from the robot' if dx > 0 else 'toward the robot', 100.0 * abs(dx)))
    if abs(dy) > 0.005:
        parts.append('%s %.1f cm' % ('left' if dy > 0 else 'right', 100.0 * abs(dy)))
    return '%s: move %s (%.1f cm off)' % (name, ', '.join(parts) or 'a little', 100.0 * d)


class Check(Node):
    def __init__(self):
        super().__init__('place_check')
        self.samples = []          # (t, x, y, z) of valid samples
        self.msgs = 0
        self.create_subscription(PickTarget3D, '/pick_target_base', self.on_target, 10)

    def on_target(self, m):
        self.msgs += 1
        if bool(m.target_valid) and bool(m.depth_valid) and all(math.isfinite(v) for v in (m.x, m.y, m.z)):
            self.samples.append((time.monotonic(), m.x, m.y, m.z))

    def window(self, secs=1.0):
        now = time.monotonic()
        self.samples = [s for s in self.samples if now - s[0] <= secs]
        return self.samples


def line(win, places, tol):
    n = len(win)
    if n == 0:
        return None, 'no valid target in the last second (object of the selected class in view? pipeline up?)'
    mean = [sum(s[k] for s in win) / n for k in (1, 2, 3)]
    sd = max(math.sqrt(sum((s[k + 1] - mean[k]) ** 2 for s in win) / (n - 1)) for k in range(3)) if n > 1 else 0.0
    name, px, py = min(places, key=lambda p: math.hypot(p[1] - mean[0], p[2] - mean[1]))
    ok = math.hypot(px - mean[0], py - mean[1]) <= tol
    return ok, 'object x %.3f y %.3f z %.3f m (n %d, sd %.1f mm) | %s' % (
        mean[0], mean[1], mean[2], n, 1e3 * sd, advice(name, px, py, mean[0], mean[1], tol))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--p', default='', help='positions x,y/x,y/... [m] in place of P1-P5')
    ap.add_argument('--tol', type=float, default=0.02, help='horizontal tolerance [m] (default 0.02)')
    ap.add_argument('--once', action='store_true', help='one line after 2 s, exit 0 if OK')
    a = ap.parse_args()
    places = parse_places(a.p) if a.p else PLACES
    print('place_check: %s, tolerance %.0f mm; Ctrl-C ends' % (
        ', '.join('%s (%.2f, %.2f)' % p for p in places), 1e3 * a.tol))
    rclpy.init()
    node = Check()
    ok = None
    try:
        t_start = time.monotonic()
        t_next = t_start + (2.0 if a.once else 0.5)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= t_next:
                ok, text = line(node.window(), places, a.tol)
                print(text, flush=True)
                if a.once:
                    break
                if node.msgs == 0 and time.monotonic() - t_start > 5.0:
                    print('  (no message at all on /pick_target_base: pipeline running? same ROS_DOMAIN_ID?)')
                t_next += 0.5
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    try:
        node.destroy_node()
        rclpy.try_shutdown()
    except Exception:
        pass
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
