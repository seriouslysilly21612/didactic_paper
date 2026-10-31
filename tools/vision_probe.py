#!/usr/bin/env python3
"""vision_probe.py - record /pick_target_base and /detections of the camera pipeline for a fixed time and print
a summary (paper, Sec. V-C): the noise of the goals of an object at rest (one recording per position, then
tools/goal_noise_file.py), the rate and latency of the pipeline, and how often the detector finds the class.

    python3 tools/vision_probe.py OUT [--secs 60] [--cls orange]
    e.g. python3 tools/vision_probe.py noise/p1 --secs 20 --cls orange

OUT_target.csv  t, x, y, z, class, target_valid, depth_valid, lat_ms
OUT_det.csv     t, frame, class, conf, lat_ms     (frame without detections: class "")
t counts from the first message; lat_ms = arrival - header.stamp (empty if no header).
Needs the ROS 2 environment (source /opt/ros/jazzy/setup.bash and ros2/install/setup.bash) and the pipeline.
"""
import argparse, csv, math, os, statistics as st, time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from my_interfaces.msg import PickTarget3D, DetectionArray

NAN = float('nan')


class Probe(Node):
    def __init__(self, out, secs):
        super().__init__('vision_probe')
        os.makedirs(os.path.dirname(os.path.abspath(out)) or '.', exist_ok=True)
        self.ft = open(out + '_target.csv', 'w', newline='')
        self.fd = open(out + '_det.csv', 'w', newline='')
        self.wt, self.wd = csv.writer(self.ft), csv.writer(self.fd)
        self.wt.writerow(['t', 'x', 'y', 'z', 'class', 'target_valid', 'depth_valid', 'lat_ms'])
        self.wd.writerow(['t', 'frame', 'class', 'conf', 'lat_ms'])
        self.secs, self.t0, self.tstart = secs, None, time.monotonic()
        self.tg, self.fr, self.lat_t, self.lat_d = [], [], [], []
        # QoS(10), reliable: the same as the bridge (ROS2PickBridge.cpp)
        self.create_subscription(PickTarget3D, '/pick_target_base', self.on_t, 10)
        self.create_subscription(DetectionArray, '/detections', self.on_d, 10)

    def _t(self):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    def _lat(self, m):
        h = getattr(m, 'header', None)
        if h is None or (h.stamp.sec == 0 and h.stamp.nanosec == 0):
            return NAN
        return (self.get_clock().now().nanoseconds - (h.stamp.sec * 10**9 + h.stamp.nanosec)) * 1e-6

    def done(self):
        if self.t0 is None:   # nothing yet: give up after 15 s
            return time.monotonic() - self.tstart > 15.0
        return time.monotonic() - self.t0 >= self.secs

    def on_t(self, m):
        t, lat = self._t(), self._lat(m)
        r = (t, m.x, m.y, m.z, m.class_name, bool(m.target_valid), bool(m.depth_valid))
        self.tg.append(r)
        if math.isfinite(lat):
            self.lat_t.append(lat)
        self.wt.writerow(['%.4f' % t, '%.6f' % m.x, '%.6f' % m.y, '%.6f' % m.z, m.class_name,
                          int(r[5]), int(r[6]), '' if math.isnan(lat) else '%.2f' % lat])

    def on_d(self, m):
        t, lat, k, best = self._t(), self._lat(m), len(self.fr), {}
        if math.isfinite(lat):
            self.lat_d.append(lat)
        ls = '' if math.isnan(lat) else '%.2f' % lat
        for d in m.detections:
            c = next((float(getattr(d, a)) for a in ('confidence', 'score', 'conf') if hasattr(d, a)), NAN)
            p = best.get(d.class_name)
            best[d.class_name] = c if p is None or not (p >= c) else p   # max, NaN-safe
            self.wd.writerow(['%.4f' % t, k, d.class_name, '' if math.isnan(c) else '%.3f' % c, ls])
        if not m.detections:
            self.wd.writerow(['%.4f' % t, k, '', '', ls])
        self.fr.append((t, best))


def _rate(rows):
    return (len(rows) - 1) / (rows[-1][0] - rows[0][0]) if len(rows) > 1 and rows[-1][0] > rows[0][0] else 0.0


def _lat_line(v):
    if not v:
        return ''
    v = sorted(v)
    return ', latency median %.1f / p99 %.1f / max %.1f ms' % (
        st.median(v), v[min(len(v) - 1, int(0.99 * len(v)))], v[-1])


def summary(p, cls):
    T, F = p.tg, p.fr
    if T:
        v = [r for r in T if r[5] and r[6] and all(math.isfinite(q) for q in r[1:4])]
        print('target: %d msgs, %.1f Hz, valid %d/%d%s' % (len(T), _rate(T), len(v), len(T), _lat_line(p.lat_t)))
        for c in sorted({r[4] for r in v}):
            print('  class %s: %d valid samples' % (c, sum(1 for r in v if r[4] == c)))
        if len(v) > 2:
            for i, a in enumerate('xyz'):
                q = [r[1 + i] * 1e3 for r in v]
                mu, sd = st.fmean(q), st.stdev(q)
                print('  %s: mean %8.1f mm  std %5.2f mm  max|dev| %6.2f mm' % (a, mu, sd, max(abs(u - mu) for u in q)))
    else:
        print('target: no message on /pick_target_base (pipeline running? same ROS_DOMAIN_ID?)')
    if F:
        print('detections: %d frames, %.1f Hz%s' % (len(F), _rate(F), _lat_line(p.lat_d)))
        for c in sorted({c for _, b in F for c in b}):
            cf = [b[c] for _, b in F if c in b]
            ok = [x for x in cf if math.isfinite(x)]
            print('  %-15s in %5d/%d frames (%5.1f %%), conf median %s' % (
                c, len(cf), len(F), 100.0 * len(cf) / len(F), '%.2f' % st.median(ok) if ok else 'n/a'))
        if cls:
            hit = sum(1 for _, b in F if cls in b)
            oth = sum(1 for _, b in F if any(c != cls for c in b))
            print('  RESULT %s: detected in %.1f %% of frames, another class in %.1f %%' % (
                cls, 100.0 * hit / len(F), 100.0 * oth / len(F)))
    else:
        print('detections: no message on /detections (pipeline running? same ROS_DOMAIN_ID?)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', help='prefix of the two CSV files, e.g. noise/p1')
    ap.add_argument('--secs', type=float, default=60.0, help='recording time from the first message [s]')
    ap.add_argument('--cls', default='', help='class that is in view (detection rate)')
    a = ap.parse_args()
    rclpy.init()
    p = Probe(os.path.expanduser(a.out), a.secs)
    try:
        while rclpy.ok() and not p.done():
            rclpy.spin_once(p, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    p.ft.close()
    p.fd.close()
    summary(p, a.cls)
    try:
        p.destroy_node()
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
