# Camera pipeline (ROS 2 Jazzy)

The packages of the camera application (paper, Sec. V-C, Figs. 9 and 10), run on the non-isolated cores of
the host (paper, Table 3):

| Package | Node | Role |
|---|---|---|
| `my_interfaces` | - | `Detection`, `DetectionArray`, `PickTarget`, `PickTarget3D` |
| `openvino_detector_pkg` | `openvino_detector_node` | YOLOv5n with OpenVINO on the integrated GPU: `/camera/camera/color/image_raw` -> `/detections` |
| `pick_logic_pkg` | `pick_logic` | the object of the selected class: `/detections` -> `/pick_target` |
| `target_3d_pkg` | `pick_target_3d_node`, `pick_target_base_node` | depth at the object, camera -> base frame: `/pick_target_3d`, `/pick_target_base` |
| `system_bringup_pkg` | launch `pick_place_openvino.launch.py` | the camera (RealSense D435i, 848 x 480, 30 fps), the nodes above, the static transform base -> camera |

The controller takes `/pick_target_base` through its bridge (below).

## Build (once)

    sudo apt install ros-jazzy-realsense2-camera ros-jazzy-realsense2-camera-msgs ros-jazzy-cv-bridge \
                     ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs ros-jazzy-rmw-fastrtps-cpp python3-opencv
    python3 -m pip install --user --break-system-packages "openvino==2026.3.1" "numpy<2" onnx
    source /opt/ros/jazzy/setup.bash
    cd ros2 && colcon build --symlink-install && source install/setup.bash
    python3 -m pytest src/openvino_detector_pkg/test src/target_3d_pkg/test/test_depth_pick.py

The integrated GPU needs the compute runtime (`intel-opencl-icd`); without it the detector runs on the CPU
and says so in its first lines. `numpy<2`: `cv_bridge` of Jazzy is built against NumPy 1 (1.26 of Ubuntu
24.04); a NumPy 2 that pip pulls in with OpenVINO would shadow it in `~/.local` and break every node that
converts images (`_ARRAY_API not found`). Check with `python3 -c "import numpy; print(numpy.__version__)"`.

The detector needs a YOLOv5n model exported with a fixed input size (416 x 416) to ONNX or to the OpenVINO
IR; the class names are read from the metadata of the ONNX file. The model of the paper was fine-tuned on
five objects of the YCB set (tennis_ball, apple, orange, banana, mustard_bottle) and is not part of this
repository.

## Run

    taskset -c 0-5,8-15 ros2 launch system_bringup_pkg pick_place_openvino.launch.py \
        model:=/path/to/yolov5n_416.onnx desired_class:=tennis_ball
    ros2 topic hz /detections                  # the rate of the detector

Every node also gets `taskset -c 0-5,8-15` as its prefix (launch argument `cpus`). With `metrics:=true`
each node writes one row per frame to `~/vision_metrics/<time>/performance/` (launch argument
`metrics_dir`); the rows of the detector hold the capture stamp, the arrival, the inference time and the
publication, so that its rate and latency come out of the same run.

The static transform `base_link -> camera_link` of the launch file is the hand-eye calibration of the
setup of the paper; after moving the camera, calibrate again (`calib/`, see the main README) and write the
result with `calib/calib_to_tf.py --write`.

## The bridge of the controller

The control process takes `/pick_target_base` through its bridge (`indy7/src/ROS2PickBridge.h`; paper,
Sec. II and V-C): the goal checks and the smoothing (`indy7/src/GoalFilter.h`), the goal slot, `/joint_states`
at 20 Hz, and the class menu. Build the controller after this workspace, in a shell with both environments,
and start it from the same shell:

    source /opt/ros/jazzy/setup.bash && source ros2/install/setup.bash
    cd indy7 && make clean && make             # prints [indy7] ... ROS=1
    make tools && ./bin/goal_filter_check      # the checks and the smoothing: PASS
    ./run.sh                                   # APP_BRIDGE=1: prints [bridge] up

`ros2 topic hz /joint_states` then shows 20 Hz. In the controller: `r`, `g`, `h`, then `p` lists the
classes in view, and a digit sets `desired_class` of the selection node and starts the camera application
(HOME to q_r, then TRACK on the goals of the camera, the hover height added by the Trajectory task); `x`
stops. The bridge reports a lost target after `APP_LOSS_S` without an accepted sample, and every rejection
of a valid sample (goal below the floor, outside the box). Without ROS 2 (`make ROS=0`), or without the
environments, the controller runs without the bridge, and the camera application is refused. The same
works in simulation (`./run.sh -s`): the camera then moves the simulated arm.

## Goal noise (the threshold delta of Table 9)

With the object at rest, one recording per position:

    python3 tools/vision_probe.py noise/p1 --secs 60 --cls orange
    python3 tools/goal_noise_file.py noise/p1_target.csv ... --out indy7/goal_noise.csv

prints sigma_g and delta and writes the file that the high-level tests 3 and 4 replay
(`HL_TEST_NOISE_FILE=goal_noise.csv` in `indy7/INDY7.cfg`).
