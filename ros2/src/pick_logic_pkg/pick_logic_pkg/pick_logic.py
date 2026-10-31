import math
import signal
import time

import rclpy
from rclpy.node import Node

from my_interfaces.msg import Detection, DetectionArray, PickTarget
from pick_logic_pkg.metrics_recorder import (
    MetricsRecorder,
    age_ms,
    write_summary_csv,
    _round,
)


class PickLogicNode(Node):
    """The target of the camera application (paper, Sec. V-C): one instance of the class that the operator
    selects.

      L1 gate       confidence / class / bbox size / distance from the image edges / area.
      L2 selection  `desired_class` is required. It is a LIVE parameter that the object menu of the
                    controller sets (`ros2 param set /pick_logic_node desired_class apple`); with ''
                    every target is published invalid (reason no_class_selected). The candidates are
                    the detections of that class that pass L1. Without a track, the candidate of the
                    highest confidence is taken.
      L3 stability  the instance taken must be seen in stability_min_frames consecutive frames before
                    target_valid; after that it is kept (sticky): no hysteresis and no switch to a
                    "better" candidate. The target changes in two cases only: (a) it has been missing
                    for more than lost_timeout_frames consecutive frames (released in the N+1th frame
                    without it, then a new choice and a new warm-up), (b) desired_class changes (the
                    track is reset). Continuation: the nearest candidate. The rule for two or more
                    instances of the same class, and why it exists, is described in _update_track.

    Output: one PickTarget per DetectionArray, target_valid true or false (whether invalid targets are
    published: publish_invalid_targets). Reject reasons: those of L1, no_class_selected,
    desired_class_not_visible, target_unstable.
    """

    def __init__(self):
        super().__init__('pick_logic_node')

        self.declare_parameter('input_topic', '/detections')
        self.declare_parameter('output_topic', '/pick_target')

        self.declare_parameter('min_confidence', 0.3)
        self.declare_parameter('allowed_classes',
                               ['apple', 'orange', 'banana',
                                'tennis_ball', 'mustard_bottle'])

        self.declare_parameter('image_width', 848)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('edge_margin_px', 30)

        self.declare_parameter('min_bbox_area_px', 400.0)
        self.declare_parameter('max_bbox_area_ratio', 0.5)

        # --- selection / stability ---
        self.declare_parameter('desired_class', '')       # LIVE: '' = nothing selected

        self.declare_parameter('stability_min_frames', 3)
        self.declare_parameter('stability_match_px', 60.0)
        self.declare_parameter('lost_timeout_frames', 5)

        self.declare_parameter('publish_invalid_targets', True)
        self.declare_parameter('log_period_sec', 1.0)

        # Metrics (off by default). performance CSV: one row per DetectionArray; yield CSV: one row with
        # the reject counts at the end. Both are written once, at the end, with no I/O per frame, as in
        # the detector node.
        self.declare_parameter('metrics_csv_path', '')
        self.declare_parameter('metrics_duration_sec', 0.0)
        self.declare_parameter('yield_csv_path', '')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.allowed_classes = list(self.get_parameter('allowed_classes').value)

        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.edge_margin_px = int(self.get_parameter('edge_margin_px').value)

        self.min_bbox_area_px = float(self.get_parameter('min_bbox_area_px').value)
        self.max_bbox_area_ratio = float(self.get_parameter('max_bbox_area_ratio').value)

        self.stability_min_frames = int(self.get_parameter('stability_min_frames').value)
        self.stability_match_px = float(self.get_parameter('stability_match_px').value)
        self.lost_timeout_frames = int(self.get_parameter('lost_timeout_frames').value)

        self.publish_invalid_targets = bool(
            self.get_parameter('publish_invalid_targets').value
        )
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)

        self.last_accept_log_time_ns = None
        self.last_reject_log_time_ns = None

        # --- state ---
        self._desired_class_cached = str(self.get_parameter('desired_class').value)
        # the track of the current target: dict(cx, cy, stable, missing) or None; its class is always
        # desired_class (reset when that changes)
        self._track = None

        self.metrics = MetricsRecorder(
            self.get_logger(),
            str(self.get_parameter('metrics_csv_path').value),
            float(self.get_parameter('metrics_duration_sec').value),
        )
        self.yield_csv_path = str(self.get_parameter('yield_csv_path').value)
        self.frames_total = 0
        self.frames_accepted = 0
        self.frames_empty = 0
        self.reject_tally = {}

        self.subscription = self.create_subscription(
            DetectionArray,
            input_topic,
            self.detections_callback,
            10
        )

        self.publisher = self.create_publisher(
            PickTarget,
            output_topic,
            10
        )

        self.get_logger().info(
            f'Pick logic node started: {input_topic} -> {output_topic}'
        )

    # ------------------------------------------------------------------ #
    # main callback
    # ------------------------------------------------------------------ #

    def detections_callback(self, msg: DetectionArray):
        compute_start_ns = time.perf_counter_ns()
        # DetectionArray.header.stamp is the capture time of the color frame (propagated by the
        # detector); every PickTarget carries it, so that the nodes downstream can compute the latency
        # from the capture.
        capture_stamp = msg.header.stamp
        in_age = (age_ms(self.get_clock().now().nanoseconds, capture_stamp)
                  if self.metrics.enabled else None)
        self.frames_total += 1

        # desired_class is a LIVE parameter (set by the object menu of the controller): read in every
        # frame, and the track is reset when it changes
        desired = str(self.get_parameter('desired_class').value)
        if desired != self._desired_class_cached:
            self.get_logger().info(
                f"desired_class: '{self._desired_class_cached}' -> '{desired}'"
                f' - tracking state reset')
            self._desired_class_cached = desired
            self._reset_track()

        if len(msg.detections) == 0:
            self.frames_empty += 1
            self._step_track_missing()
            self._finish_invalid('empty_detection_array', capture_stamp,
                                 compute_start_ns, in_age, 0)
            return

        # no class selected: nothing to follow. The operator selects one with the menu 'p' of the
        # controller; without the controller, launch with desired_class:=<class>.
        if not desired:
            self._finish_invalid('no_class_selected', capture_stamp,
                                 compute_start_ns, in_age, len(msg.detections))
            return

        # L1 gate -> the candidates (only desired_class remains)
        candidates = []
        first_reject_reason = None
        first_rejected_detection = None
        for det in msg.detections:
            accepted, reason = self.is_detection_acceptable(det)
            if accepted and det.class_name != desired:
                accepted, reason = False, 'desired_class_not_visible'
            if accepted:
                candidates.append(det)
            elif first_reject_reason is None:
                first_reject_reason = reason  # the reason of the first rejected detection (the highest confidence)
                first_rejected_detection = det

        if not candidates:
            self._step_track_missing()
            reason = first_reject_reason or 'no_acceptable_detection'
            self._finish_invalid(reason, capture_stamp, compute_start_ns,
                                 in_age, len(msg.detections),
                                 first_rejected_detection)
            return

        # L2 + L3: take or continue one instance (sticky track). None: the tracked instance is not in
        # this frame (still within lost_timeout); stable < stability_min_frames: warming up
        det = self._update_track(candidates)
        if det is None or self._track['stable'] < self.stability_min_frames:
            self._finish_invalid('target_unstable', capture_stamp,
                                 compute_start_ns, in_age,
                                 len(msg.detections),
                                 det if det is not None else candidates[0])
            return

        target = self.make_pick_target(det)
        target.target_valid = True
        target.capture_stamp = capture_stamp
        self.publisher.publish(target)
        self.frames_accepted += 1

        if self.should_log_accept():
            self.get_logger().info(
                f'Accepted target: class={target.class_name}, '
                f'conf={target.confidence:.2f}, '
                f'cx={target.center_x:.1f}, cy={target.center_y:.1f}, '
                f'stable={self._track["stable"]}, '
                f'candidates={len(msg.detections)}, desired={desired}'
            )
        self._record_metrics(capture_stamp, compute_start_ns, in_age,
                             n_candidates=len(msg.detections),
                             target_valid=True)

    # ------------------------------------------------------------------ #
    # the track
    # ------------------------------------------------------------------ #

    def _reset_track(self):
        self._track = None

    # kept up to lost_timeout_frames, released in the next (N+1th) consecutive frame without it
    def _step_track_missing(self):
        if self._track is None:
            return
        self._track['missing'] += 1
        if self._track['missing'] > self.lost_timeout_frames:
            self._reset_track()

    def _update_track(self, candidates):
        """take or continue one instance of desired_class in this frame.

        candidates: the detections of desired_class in this frame that pass L1 (not empty). Returns the
        Detection to publish, or None when the tracked instance is not in this frame (still within
        lost_timeout).

        The behavior with two or more instances of the same class is defined here. The experiments put
        one object per class on the table, but that is an assumption about the scene, not a guarantee
        of the detector: two boxes of the same class reach /detections regardless of the scene, e.g.
          - a long object such as a banana split into two boxes that both pass the NMS (IoU 0.45),
          - a similar object in the background detected as the same class.
        Following the order of the decoder would let the center of the target alternate between two
        boxes from frame to frame, and the bridge has no jump gate, so the tool would go back and forth.
        With a person next to the robot, no behavior is left undefined.

        Rule: take = the highest confidence / continue = the nearest (the distance gate match_px only
        when two or more are in view) / no switch before the target is lost. No re-identification by
        appearance: one object per class is enough for the experiments, and more is beyond this node.
        """
        # 1. no track: take the highest confidence
        if self._track is None:
            best = max(candidates, key=lambda d: float(d.confidence))
            self._track = {'cx': float(best.center_x), 'cy': float(best.center_y),
                           'stable': 1, 'missing': 0}
            return best

        # 2. the distance from the box of the target in the last frame to the boxes of the candidates:
        # one candidate is taken as it is; with two or more, the nearest within stability_match_px
        # dist: the distance, cont: the Detection
        dist, cont = min(
            ((math.hypot(float(d.center_x) - self._track['cx'],
                         float(d.center_y) - self._track['cy']), d)
             for d in candidates),
            key=lambda t: t[0])
        
        if len(candidates) > 1 and dist > self.stability_match_px:
            cont = None

        # 3. the target is not in this frame: beyond lost_timeout, start again
        if cont is None:
            self._step_track_missing()
            if self._track is None:      # timed out: take a new one
                return self._update_track(candidates)
            return None                  # wait (this frame is invalid)

        # 4. the track goes on (sticky: no switch to a better candidate)
        self._track.update({'cx': float(cont.center_x),
                            'cy': float(cont.center_y),
                            'stable': self._track['stable'] + 1,
                            'missing': 0})
        return cont

    def _finish_invalid(self, reason, capture_stamp, compute_start_ns,
                        in_age, n_candidates, det=None):
        self._tally(reason)
        self.publish_invalid_target(reason, capture_stamp, det)
        self._record_metrics(capture_stamp, compute_start_ns, in_age,
                             n_candidates=n_candidates, target_valid=False)

    # ------------------------------------------------------------------ #
    # metrics / publishing / L1 gate / logging / yield
    # ------------------------------------------------------------------ #

    # counts of the reject reasons
    def _tally(self, reason):
        self.reject_tally[reason] = self.reject_tally.get(reason, 0) + 1

    def _record_metrics(self, capture_stamp, compute_start_ns, in_age,
                        n_candidates, target_valid):
        if not self.metrics.enabled:
            return
        now_ns = self.get_clock().now().nanoseconds
        compute_ms = (time.perf_counter_ns() - compute_start_ns) / 1e6
        self.metrics.add({
            'capture_sec': capture_stamp.sec,
            'capture_nanosec': capture_stamp.nanosec,
            'pick_logic_in_age_ms': _round(in_age),
            'pick_logic_compute_ms': _round(compute_ms),
            'pick_logic_out_age_ms': _round(age_ms(now_ns, capture_stamp)),
            'n_candidates': n_candidates,
            'target_valid': int(target_valid),
        })

    def publish_invalid_target(self, reason, capture_stamp, det=None):
        target = PickTarget()
        target.target_valid = False
        target.capture_stamp = capture_stamp

        if det is not None:
            target.class_id = det.class_id
            target.class_name = det.class_name
            target.confidence = det.confidence
            target.center_x = det.center_x
            target.center_y = det.center_y
            target.width = det.width
            target.height = det.height

        if self.should_log_reject():
            self.get_logger().warn(
                f'Rejected detection array: reason={reason}'
            )

        if self.publish_invalid_targets:
            self.publisher.publish(target)

    def make_pick_target(self, msg: Detection):
        target = PickTarget()
        target.class_id = msg.class_id
        target.class_name = msg.class_name
        target.confidence = msg.confidence
        target.center_x = msg.center_x
        target.center_y = msg.center_y
        target.width = msg.width
        target.height = msg.height
        return target

    def is_detection_acceptable(self, msg: Detection):
        # min_confidence = 0.3 (yaml). The detector already filters at its own thresholds, so this gate
        # only matters when another source (a synthetic test, a replay) publishes /detections
        if msg.confidence < self.min_confidence:
            return False, 'confidence_below_threshold'

        # classes outside allowed_classes (the five classes of the experiments) are rejected
        if self.allowed_classes and msg.class_name not in self.allowed_classes:
            return False, 'class_not_allowed'

        if msg.width <= 0.0 or msg.height <= 0.0:
            return False, 'invalid_bbox_size'

        left = msg.center_x - msg.width / 2.0
        right = msg.center_x + msg.width / 2.0
        top = msg.center_y - msg.height / 2.0
        bottom = msg.center_y + msg.height / 2.0

        # edge_margin_px = 30px
        if left < self.edge_margin_px:
            return False, 'bbox_too_close_to_left_edge'

        if right > self.image_width - self.edge_margin_px:
            return False, 'bbox_too_close_to_right_edge'

        if top < self.edge_margin_px:
            return False, 'bbox_too_close_to_top_edge'

        if bottom > self.image_height - self.edge_margin_px:
            return False, 'bbox_too_close_to_bottom_edge'

        bbox_area = float(msg.width * msg.height)
        image_area = float(self.image_width * self.image_height)
        max_bbox_area = image_area * self.max_bbox_area_ratio

        # min_bbox_area_px = 400
        if bbox_area < self.min_bbox_area_px:
            return False, 'bbox_area_too_small'

        # max_bbox_area = 0.5 * image_area
        if bbox_area > max_bbox_area:
            return False, 'bbox_area_too_large'

        return True, 'accepted'

    def should_log_accept(self):
        should_log, now_ns = self.should_log(self.last_accept_log_time_ns)
        if should_log:
            self.last_accept_log_time_ns = now_ns
        return should_log

    def should_log_reject(self):
        should_log, now_ns = self.should_log(self.last_reject_log_time_ns)
        if should_log:
            self.last_reject_log_time_ns = now_ns
        return should_log

    def should_log(self, last_log_time_ns):
        now_ns = self.get_clock().now().nanoseconds

        if last_log_time_ns is None:
            return True, now_ns

        elapsed_sec = (now_ns - last_log_time_ns) / 1e9
        return elapsed_sec >= self.log_period_sec, now_ns

    def write_yield_summary(self):
        if not self.yield_csv_path:
            return
        total = max(self.frames_total, 1)
        row = {
            'node': 'pick_logic',
            'frames_total': self.frames_total,
            'frames_accepted': self.frames_accepted,
            'target_valid_rate': round(self.frames_accepted / total, 4),
            'frames_empty': self.frames_empty,
        }
        for reason, count in sorted(self.reject_tally.items()):
            row[f'reject_{reason}'] = count
        write_summary_csv(self.get_logger(), self.yield_csv_path, [row])


def main(args=None):
    rclpy.init(args=args)
    node = PickLogicNode()

    # SIGTERM (the escalation of launch, pkill) acts as Ctrl-C, so that the finally block writes the
    # metrics and yield CSVs at the end
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # block a second signal: Ctrl-C of the process group plus the propagation by launch can deliver
        # SIGINT twice, which would cut the writing of the CSVs in this block
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.metrics.save()
        node.write_yield_summary()
        node.destroy_node()
        # on SIGINT the signal handler of rclpy has already shut the context down; another shutdown
        # would raise "rcl_shutdown already called", hence the guard
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
