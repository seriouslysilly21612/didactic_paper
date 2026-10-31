#!/usr/bin/env python3
"""openvino_detector_node - YOLOv5n with OpenVINO on the host GPU (paper, Sec. V-C).

The detector of the camera pipeline, with in-process OpenVINO inference (yolo_openvino.py).
Interface: sensor_msgs/Image in, my_interfaces/DetectionArray out, with the header (capture stamp) of
the image, so that pick_logic, pick_target_3d and pick_target_base work unchanged.

Threads: the subscription keeps only the newest image (a slot); one worker thread takes it, detects,
and publishes, so a frame that arrives while the detector is busy replaces the waiting one (counted as
dropped) and the detector always works on the newest frame. OpenVINO releases the GIL during inference.

Parameters (config/openvino_detector.yaml): model_path (.onnx or .xml), device (GPU = the integrated
GPU, CPU), class_thresholds ("name:thr,name:thr"), default_threshold, nms_iou, input_topic,
output_topic, process_period_sec (minimum interval between the starts of two detections, 0 = none),
publish_empty_detections, log_period_sec, metrics_csv_path, metrics_duration_sec.

Metrics: with metrics_csv_path, one row per processed frame:
capture stamp, the time the image arrived, the start and the end of the detection and its phases, the
time of the publication, the number of detections, and the frames dropped since the last one; the
summary (rate, inference and latency, median / 95th percentile / max) is logged at the end.
"""
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

from my_interfaces.msg import Detection, DetectionArray

from openvino_detector_pkg.yolo_openvino import YoloOpenVino


def ros_image_to_bgr(msg):
    """sensor_msgs/Image -> BGR uint8 (H, W, 3), padding rows handled"""
    import cv2
    enc = msg.encoding.lower()
    ch = {'bgr8': 3, 'rgb8': 3, '8uc3': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1, '8uc1': 1}.get(enc)
    if ch is None:
        raise RuntimeError('unsupported image encoding %s' % msg.encoding)
    data = np.frombuffer(msg.data, dtype=np.uint8)
    need = msg.height * msg.step
    if data.size < need:
        raise RuntimeError('image data too small: %d < %d' % (data.size, need))
    pix = data[:need].reshape((msg.height, msg.step))[:, :msg.width * ch]
    if ch == 1:
        return cv2.cvtColor(pix.reshape((msg.height, msg.width)), cv2.COLOR_GRAY2BGR)
    img = pix.reshape((msg.height, msg.width, ch))
    if enc in ('bgr8', '8uc3'):
        return np.ascontiguousarray(img)
    if enc == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)


def stamp_s(t):
    return t.sec + 1e-9 * t.nanosec


class OpenVinoDetectorNode(Node):
    def __init__(self):
        super().__init__('openvino_detector_node')
        p = self.declare_parameter
        p('model_path', '')
        p('device', 'GPU')
        p('class_thresholds', '')
        p('default_threshold', 0.5)
        p('nms_iou', 0.45)
        p('input_topic', '/camera/camera/color/image_raw')
        p('output_topic', '/detections')
        p('process_period_sec', 0.0)
        p('publish_empty_detections', True)
        p('log_period_sec', 5.0)
        p('metrics_csv_path', '')
        p('metrics_duration_sec', 0.0)
        g = lambda k: self.get_parameter(k).value
        th = {}
        for item in str(g('class_thresholds')).split(','):
            if ':' in item:
                k, v = item.split(':', 1)
                th[k.strip()] = float(v)
        self.det = YoloOpenVino(g('model_path'), device=str(g('device')), class_thresholds=th,
                                default_threshold=float(g('default_threshold')), nms_iou=float(g('nms_iou')),
                                log=lambda s: self.get_logger().info(s))
        self.period = float(g('process_period_sec'))
        self.publish_empty = bool(g('publish_empty_detections'))
        self.log_period = float(g('log_period_sec'))
        self.metrics_path = str(g('metrics_csv_path'))
        self.metrics_duration = float(g('metrics_duration_sec'))
        self.rows = []
        self.t_first = None
        img_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        out_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(DetectionArray, str(g('output_topic')), out_qos)
        self.sub = self.create_subscription(Image, str(g('input_topic')), self.on_image, img_qos)
        self.lock = threading.Lock()
        self.event = threading.Event()
        self.slot = None              # (msg, t_arrival)
        self.dropped = 0
        self.running = True
        self.n_done = 0
        self.t_log = time.monotonic()
        self.n_log = 0
        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.worker.start()
        self.get_logger().info('openvino_detector: %s -> %s, device %s, period %.3f s' % (
            g('input_topic'), g('output_topic'), self.det.device, self.period))

    def on_image(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            if self.slot is not None:
                self.dropped += 1
            self.slot = (msg, now)
        self.event.set()

    def loop(self):
        t_last = 0.0
        while self.running:
            if not self.event.wait(timeout=0.5):
                continue
            if self.period > 0.0:
                wait = t_last + self.period - time.monotonic()
                if wait > 0.0:
                    time.sleep(wait)
            with self.lock:
                item, self.slot = self.slot, None
                dropped, self.dropped = self.dropped, 0
                self.event.clear()
            if item is None:
                continue
            msg, t_arr = item
            t_last = time.monotonic()
            t0 = self.get_clock().now().nanoseconds * 1e-9
            try:
                bgr = ros_image_to_bgr(msg)
                dets, tm = self.det.detect(bgr)
            except Exception as ex:      # noqa: BLE001 - a bad frame must not end the node
                self.get_logger().error('detection failed: %s' % ex)
                continue
            out = DetectionArray()
            out.header = msg.header        # the capture stamp travels on (latency of the chain)
            for d in dets:
                m = Detection()
                m.class_id = int(d['class_id'])
                m.class_name = d['class_name']
                m.confidence = float(d['confidence'])
                m.center_x = float(d['center_x'])
                m.center_y = float(d['center_y'])
                m.width = float(d['width'])
                m.height = float(d['height'])
                out.detections.append(m)
            if dets or self.publish_empty:
                self.pub.publish(out)
            t_pub = self.get_clock().now().nanoseconds * 1e-9
            self.n_done += 1
            self.n_log += 1
            if self.metrics_path:
                if self.t_first is None:
                    self.t_first = t_pub
                if self.metrics_duration <= 0.0 or t_pub - self.t_first <= self.metrics_duration:
                    self.rows.append((stamp_s(msg.header.stamp), t_arr, t0, tm['pre_ms'], tm['infer_ms'],
                                      tm['post_ms'], t_pub, len(dets), dropped))
                elif self.rows and not getattr(self, '_saved', False):
                    self.save_metrics()
            if time.monotonic() - self.t_log >= self.log_period:
                dt = time.monotonic() - self.t_log
                self.get_logger().info('%.1f Hz, inference %.1f ms, %d detections%s' % (
                    self.n_log / dt, tm['infer_ms'], len(dets),
                    (', best %s %.2f' % (dets[0]['class_name'], dets[0]['confidence'])) if dets else ''))
                self.t_log, self.n_log = time.monotonic(), 0

    def save_metrics(self):
        if not self.metrics_path or not self.rows:
            return
        self._saved = True
        os.makedirs(os.path.dirname(os.path.abspath(self.metrics_path)), exist_ok=True)
        with open(self.metrics_path, 'w') as f:
            f.write('capture_s,arrival_s,start_s,pre_ms,infer_ms,post_ms,publish_s,detections,dropped\n')
            for r in self.rows:
                f.write('%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f,%d,%d\n' % r)
        a = np.array([r[:7] for r in self.rows])
        span = a[-1, 6] - a[0, 6]
        lat = 1e3 * (a[:, 6] - a[:, 0])
        inf = a[:, 4]
        q = lambda v: '%.1f / %.1f / %.1f' % (np.median(v), np.percentile(v, 95), v.max())
        self.get_logger().info('metrics: %d frames in %.1f s (%.2f Hz), %d dropped; inference %s ms; '
                               'capture to publication %s ms (median / p95 / max) -> %s' % (
                                   len(self.rows), span, (len(self.rows) - 1) / span if span > 0 else 0.0,
                                   sum(r[8] for r in self.rows), q(inf), q(lat), self.metrics_path))

    def destroy_node(self):
        self.running = False
        self.event.set()
        self.worker.join(timeout=2.0)
        if not getattr(self, '_saved', False):
            self.save_metrics()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenVinoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
