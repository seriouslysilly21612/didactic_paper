import signal
import time

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point

from my_interfaces.msg import PickTarget3D
from target_3d_pkg.metrics_recorder import (
    MetricsRecorder,
    age_ms,
    write_summary_csv,
    _round,
)

# the unit quaternion of a TF transform (the calibration) as a rotation matrix
def quat_to_rotation_matrix(qx, qy, qz, qw):
    """Unit quaternion -> 3x3 rotation matrix (same math as tf2)."""
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


class PickTargetBaseNode(Node):
    def __init__(self):
        super().__init__('pick_target_base_node')

        self.declare_parameter('input_topic', '/pick_target_3d')
        self.declare_parameter('output_topic', '/pick_target_base')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('transform_timeout_sec', 0.2)  # a target that arrives before the TF waits up to this

        self.declare_parameter('require_depth_valid', True)
        self.declare_parameter('min_camera_z_m', 0.20)
        self.declare_parameter('max_camera_z_m', 0.95)   # see target_base.yaml

        self.declare_parameter('publish_invalid_targets', True)
        self.declare_parameter('log_period_sec', 1.0)
        # the TF base_link -> camera is static (the camera is mounted fixed), so a tf2 lookup and
        # do_transform_point per message would be waste: (R, t) of the first lookup is cached and every
        # later point is R @ p + t. Set False if the TF moves (e.g. a camera on the arm).
        self.declare_parameter('static_tf_cache', True)

        # Metrics (off by default): performance CSV (incl. true_e2e) + yield.
        self.declare_parameter('metrics_csv_path', '')
        self.declare_parameter('metrics_duration_sec', 0.0)
        self.declare_parameter('yield_csv_path', '')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.target_frame = self.get_parameter('target_frame').value
        self.transform_timeout_sec = float(
            self.get_parameter('transform_timeout_sec').value
        )

        self.require_depth_valid = bool(
            self.get_parameter('require_depth_valid').value
        )
        self.min_camera_z_m = float(
            self.get_parameter('min_camera_z_m').value
        )
        self.max_camera_z_m = float(
            self.get_parameter('max_camera_z_m').value
        )

        self.publish_invalid_targets = bool(
            self.get_parameter('publish_invalid_targets').value
        )
        self.log_period_sec = float(
            self.get_parameter('log_period_sec').value
        )
        self.static_tf_cache = bool(
            self.get_parameter('static_tf_cache').value
        )
        self._tf_cache = {}   # source frame_id -> (R 3x3, t 3)
        self.last_warn_log_time_ns = {}

        self.metrics = MetricsRecorder(
            self.get_logger(),
            str(self.get_parameter('metrics_csv_path').value),
            float(self.get_parameter('metrics_duration_sec').value),
        )
        self.yield_csv_path = str(self.get_parameter('yield_csv_path').value)
        self.frames_total = 0
        self.published_valid = 0
        self.fail_tally = {}
        self._last_pub_ns = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PickTarget3D,
            input_topic,
            self.target_callback,
            10
        )

        self.pub = self.create_publisher(
            PickTarget3D,
            output_topic,
            10
        )

        self.get_logger().info(
            f'pick_target_base_node started: {input_topic} -> {output_topic}, '
            f'target_frame={self.target_frame}'
        )
        self.get_logger().info(
            f'Filters: require_depth_valid={self.require_depth_valid}, '
            f'camera_z_range=[{self.min_camera_z_m:.2f}, '
            f'{self.max_camera_z_m:.2f}]'
        )

    def target_callback(self, msg: PickTarget3D):
        compute_start_ns = time.perf_counter_ns()
        capture_stamp = msg.capture_stamp
        in_age = (age_ms(self.get_clock().now().nanoseconds, capture_stamp)
                  if self.metrics.enabled else None)
        self.frames_total += 1

        out = self.make_output_from_input(msg)
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.target_frame

        accepted, reason = self.is_input_acceptable(msg)

        if not accepted:
            out.target_valid = False
            out.depth_valid = False
            self.log_warn_throttled(
                reason,
                f'Rejected 3D target before TF: reason={reason}, '
                f'input_frame={msg.header.frame_id}, '
                f'target_valid={msg.target_valid}, '
                f'depth_valid={msg.depth_valid}, z={msg.z:.3f}'
            )
            self._emit(out, capture_stamp, compute_start_ns, in_age,
                       publish=self.publish_invalid_targets)
            return

        # cached: transform with R, t and return
        cached = (
            self._tf_cache.get(msg.header.frame_id)
            if self.static_tf_cache else None
        )
        if cached is not None:
            R, t = cached
            p = R @ np.array(
                [msg.x, msg.y, msg.z], dtype=np.float64) + t
            out.target_valid = True
            out.depth_valid = True
            out.x = float(p[0])
            out.y = float(p[1])
            out.z = float(p[2])
            self._emit(out, capture_stamp, compute_start_ns, in_age)
            return

        # the point as tf2 takes it
        point_in = PointStamped()
        point_in.header = msg.header
        point_in.point.x = float(msg.x)
        point_in.point.y = float(msg.y)
        point_in.point.z = float(msg.z)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,  # base_link (target)
                msg.header.frame_id,  # the frame of the depth camera (source)
                rclpy.time.Time(),  # the newest transform
                timeout=Duration(seconds=self.transform_timeout_sec)
            )

            point_out = do_transform_point(point_in, transform)

            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.target_frame
            out.target_valid = True
            out.depth_valid = True
            out.x = float(point_out.point.x)
            out.y = float(point_out.point.y)
            out.z = float(point_out.point.z)

            self._emit(out, capture_stamp, compute_start_ns, in_age)

            # cache R, t
            if self.static_tf_cache:
                q = transform.transform.rotation
                tr = transform.transform.translation
                R = quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
                t = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
                p = R @ np.array(
                    [msg.x, msg.y, msg.z], dtype=np.float64) + t
                diff = max(
                    abs(p[0] - point_out.point.x),
                    abs(p[1] - point_out.point.y),
                    abs(p[2] - point_out.point.z),
                )
                # the difference is logged, not checked: a large one (an error of the formula) would be
                # cached all the same. A stricter version would not cache when diff > 1e-6 and keep the
                # tf2 path (slower, exact).
                self._tf_cache[msg.header.frame_id] = (R, t)
                self.get_logger().info(
                    f'Static TF cached ({msg.header.frame_id} -> '
                    f'{self.target_frame}), matmul vs tf2 max diff '
                    f'{diff:.3e} m'
                )

        except TransformException as e:
            out.target_valid = False
            out.depth_valid = False

            self.log_warn_throttled(
                'tf_transform_failed',
                f'Failed to transform target from {msg.header.frame_id} '
                f'to {self.target_frame}: {e}'
            )

            self._emit(out, capture_stamp, compute_start_ns, in_age,
                       publish=self.publish_invalid_targets)

    def make_output_from_input(self, msg: PickTarget3D):
        out = PickTarget3D()
        out.capture_stamp = msg.capture_stamp

        out.target_valid = msg.target_valid
        out.depth_valid = msg.depth_valid

        out.class_id = msg.class_id
        out.class_name = msg.class_name
        out.confidence = msg.confidence

        out.center_x = msg.center_x
        out.center_y = msg.center_y
        out.width = msg.width
        out.height = msg.height

        out.x = 0.0
        out.y = 0.0
        out.z = 0.0

        return out

    def is_input_acceptable(self, msg: PickTarget3D):
        if not msg.target_valid:
            return False, 'target_not_valid'  # from pick_logic or the detector

        if self.require_depth_valid and not msg.depth_valid:
            return False, 'depth_not_valid'  # from pick_target_3d

        if msg.header.frame_id == '':
            return False, 'empty_input_frame'  # no frame id from the camera

        if msg.z < self.min_camera_z_m:
            return False, 'camera_z_too_close'  # the range check of this node

        if msg.z > self.max_camera_z_m:
            return False, 'camera_z_too_far'  # the range check of this node

        return True, 'accepted'

    def _emit(self, out, capture_stamp, compute_start_ns, in_age, publish=True):
        """Record one performance row (incl. true_e2e for valid targets) and
        optionally publish. Every exit routes through here -> one CSV row per
        input target. output_period_ms tracks the interval between actual
        published outputs (the cadence the robot consumer sees)."""
        if out.target_valid:
            self.published_valid += 1
        if self.metrics.enabled:
            now_perf = time.perf_counter_ns()
            now_ns = self.get_clock().now().nanoseconds
            compute_ms = (now_perf - compute_start_ns) / 1e6
            out_period = None
            if publish:
                if self._last_pub_ns is not None:
                    out_period = (now_perf - self._last_pub_ns) / 1e6
                self._last_pub_ns = now_perf
            # true E2E is only meaningful for a valid target (capture -> here).
            true_e2e = age_ms(now_ns, capture_stamp) if out.target_valid else None
            self.metrics.add({
                'capture_sec': capture_stamp.sec,
                'capture_nanosec': capture_stamp.nanosec,
                'base_in_age_ms': _round(in_age),
                'base_compute_ms': _round(compute_ms),
                'true_e2e_ms': _round(true_e2e),
                'output_period_ms': _round(out_period),
                'target_valid': int(out.target_valid),
            })
        if publish:
            self.pub.publish(out)

    def write_yield_summary(self):
        if not self.yield_csv_path:
            return
        total = max(self.frames_total, 1)
        row = {
            'node': 'pick_target_base',
            'frames_total': self.frames_total,
            'published_valid': self.published_valid,
            'final_target_valid_rate': round(self.published_valid / total, 4),
        }
        for reason, count in sorted(self.fail_tally.items()):
            row[f'reject_{reason}'] = count
        write_summary_csv(self.get_logger(), self.yield_csv_path, [row])

    def log_warn_throttled(self, key, message):
        self.fail_tally[key] = self.fail_tally.get(key, 0) + 1
        now_ns = self.get_clock().now().nanoseconds
        last_log_time_ns = self.last_warn_log_time_ns.get(key)

        if last_log_time_ns is not None:
            elapsed_sec = (now_ns - last_log_time_ns) / 1e9
            if elapsed_sec < self.log_period_sec:
                return

        self.last_warn_log_time_ns[key] = now_ns
        self.get_logger().warn(message)


def main(args=None):
    rclpy.init(args=args)
    node = PickTargetBaseNode()

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
