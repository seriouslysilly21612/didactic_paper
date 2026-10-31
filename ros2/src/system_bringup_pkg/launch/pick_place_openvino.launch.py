"""The camera pipeline of the camera application (paper, Sec. V-C).

    camera -> openvino_detector -> pick_logic -> pick_target_3d -> (static TF) -> pick_target_base

ROS 2 Jazzy. Every process runs on the non-isolated cores (CPU 6 is the real-time core, CPU 7 its
sibling, offline; paper, Table 3): each node is started with the prefix `taskset -c <cpus>` (launch
argument cpus, default 0-5,8-15), and so is the camera driver.

    ros2 launch system_bringup_pkg pick_place_openvino.launch.py model:=/path/to/yolov5n_416.onnx \
        desired_class:=tennis_ball [device:=GPU] [metrics:=true metrics_dir:=~/vision_metrics]

The controller receives the targets of /pick_target_base through its bridge (indy7/src/ROS2PickBridge.h).
The static transform base_link -> camera_link is the hand-eye calibration of the setup
(calib/calib_to_tf.py --write puts a new one here); repeat the calibration if the camera has been moved.
"""
import os
from datetime import datetime

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    share = lambda pkg, *p: PathJoinSubstitution([FindPackageShare(pkg)] + list(p))
    cfg = lambda name: share('system_bringup_pkg', 'config', name)
    cpus = LaunchConfiguration('cpus').perform(context)
    prefix = 'taskset -c %s' % cpus if cpus else ''
    model = os.path.expanduser(LaunchConfiguration('model').perform(context))
    metrics_on = LaunchConfiguration('metrics').perform(context).lower() in ('true', '1', 'yes', 'on')
    run_dir = ''
    if metrics_on:
        run_dir = os.path.join(os.path.expanduser(LaunchConfiguration('metrics_dir').perform(context)),
                               datetime.now().strftime('%Y%m%d_%H%M%S'))
        os.makedirs(run_dir, exist_ok=True)
        print('[metrics] this run -> %s' % run_dir)
    duration = float(LaunchConfiguration('metrics_duration').perform(context))

    def metrics(key):
        if not metrics_on:
            return []
        return [{'metrics_csv_path': os.path.join(run_dir, 'performance', '%s.csv' % key),
                 'metrics_duration_sec': duration}]

    # the camera driver as a node of its own (as rs_launch.py starts it, camera/camera), so that it gets
    # the prefix too; realsense_pick_place.yaml is the flat parameter file of rs_launch.py
    with open(cfg('realsense_pick_place.yaml').perform(context)) as f:
        cam_params = yaml.safe_load(f) or {}
    det_params = [cfg('openvino_detector.yaml'),
                  {'device': LaunchConfiguration('device').perform(context)}]
    if model:
        det_params.append({'model_path': model})
    return [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', cfg('fastdds_shm_profile.xml')),
        Node(package='realsense2_camera', executable='realsense2_camera_node', namespace='camera',
             name='camera', parameters=[cam_params], prefix=prefix, output='screen'),
        Node(package='openvino_detector_pkg', executable='openvino_detector_node',
             name='openvino_detector_node', parameters=det_params + metrics('detector'),
             prefix=prefix, output='screen'),
        Node(package='pick_logic_pkg', executable='pick_logic', name='pick_logic_node',
             parameters=[cfg('pick_logic.yaml'), {'desired_class': LaunchConfiguration('desired_class')}]
             + metrics('pick_logic'), prefix=prefix, output='screen'),
        Node(package='target_3d_pkg', executable='pick_target_3d_node', name='pick_target_3d_node',
             parameters=[cfg('target_3d.yaml')] + metrics('target_3d'), prefix=prefix, output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_camera_tf',
             arguments=['--x', '1.160640', '--y', '0.087206', '--z', '0.496697',
                        '--qx', '0.31667766', '--qy', '-0.01284202', '--qz', '-0.94771778', '--qw', '0.03716651',
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
             prefix=prefix, output='screen'),
        Node(package='target_3d_pkg', executable='pick_target_base_node', name='pick_target_base_node',
             parameters=[cfg('target_base.yaml')] + metrics('base'), prefix=prefix, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='', description='YOLOv5n ONNX (or OpenVINO .xml) file'),
        DeclareLaunchArgument('device', default_value='GPU', description='OpenVINO device: GPU or CPU'),
        DeclareLaunchArgument('desired_class', default_value='',
                              description='class that pick_logic follows (the operator sets it live)'),
        DeclareLaunchArgument('cpus', default_value='0-5,8-15',
                              description='CPUs of every node (taskset -c); empty: no pinning'),
        DeclareLaunchArgument('metrics', default_value='false', description='true: per-frame CSVs of the nodes'),
        DeclareLaunchArgument('metrics_dir', default_value='~/vision_metrics'),
        DeclareLaunchArgument('metrics_duration', default_value='0.0',
                              description='[s] from the first frame, 0: until shutdown'),
        OpaqueFunction(function=launch_setup),
    ])
