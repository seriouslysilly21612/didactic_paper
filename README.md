# Goal-to-Torque

Reference software for torque-level robot control from asynchronous goals: a camera, a planner or an
operator gives Cartesian goals at its own rate, and a real-time controller on PREEMPT_RT Linux turns them
into joint torques that it sends to the drives over EtherCAT every millisecond.

This is the source code of the article

> J. H. Yoon, K. W. Kim, J. E. Kim, and R. Delgado, "From High-Level Goals to Joint Torques: A Reference
> Software Architecture for Real-Time Robot Control Using Open-Source Components," submitted to IEEE Access.

It contains the controller of the Neuromeka Indy7 used in the experiments of the paper, its simulation in
MuJoCo, the ROS 2 camera pipeline of the camera application, the computations reused on a humanoid model,
and the tools that produce the tables and figures of the paper from the logs of the controller.

## The architecture in brief (paper, Sec. II, Fig. 1)

The controller is one process with three periodic tasks on one isolated core, released on a common 1-ms
grid (`indy7/src/Indy7Ctrl.h`):

| Task | Priority | In every period |
|---|---|---|
| EtherCAT (`TaskEtherCAT.cpp`) | 97 | receive the frame, publish the state (State slot), send the torque of the last period (Torque slot) |
| Trajectory (`TaskTrajectory.cpp`) | 96 | take the newest goal (Goal slot), goal admission, bounded IK, jerk-limited reference (Ruckig), publish it (Reference slot) |
| Dynamics (`TaskDynamics.cpp`) | 95 | computed-torque law (RBDL) on the reference and the state, publish the torque (Torque slot) |

The tasks exchange data only through single-writer atomic slots (`indy7/src/base/AtomicSlot.h`, eq. (1));
every payload carries the period in which it was produced. The torque computed in period k is sent in
period k + 1. The ROS 2 bridge (`ROS2PickBridge.h`) writes camera goals into the Goal slot outside the
periodic path. With `-s` a MuJoCo model of the arm takes the place of the bus; with `-b` the bus of the robot
runs in OP with its drives kept off while the simulated arm is controlled (the host-timing test).

## Repository layout

| Directory | Content |
|---|---|
| `indy7/` | the controller: `src/` (tasks, control law, high-level controller, EtherCAT layer, simulation), `INDY7.cfg` (all parameters, with the values of the experiments), `models/` (URDF, MuJoCo model), `Makefile`, `run.sh` |
| `tools/` | offline checks, analysis of the logs (the tables and figures of the paper), identification, test runners, the viewer |
| `tools/host/` | host setup and measurements: cyclictest (Table 4), the operating point of the CPU, the priority of the IgH master thread, the bus inventory |
| `calib/` | the hand-eye calibration of the camera (eq. (10)) |
| `ros2/` | the camera pipeline (ROS 2 Jazzy): RealSense, YOLOv5n with OpenVINO, target selection, 3-D target in the base frame (`ros2/README.md`) |
| `humanoid/` | the computations of the controller on a fixed-base Unitree G1 in MuJoCo (Sec. VI) |
| `third_party/rt-posix/` | RT-POSIX, the real-time task library, used unmodified |

## Dependencies (paper, Table 1)

| Component | Version of the experiments | Used by |
|---|---|---|
| Linux with PREEMPT_RT | 6.12.66-rt15 (Ubuntu Server 24.04) | the controller (any Linux runs the simulation) |
| IgH EtherCAT master | 1.6.9, native `igc` driver | the controller (`IGH_DIR`, default `/opt/etherlab`) |
| RBDL with its URDF reader | 3.3.1 | the controller, the humanoid (`RBDL_DIR`, default `/usr/local`) |
| Ruckig | 0.19.4 | the controller, the humanoid (`RUCKIG_DIR`) |
| Eigen 3 | of the distribution | (`EIGEN_DIR`, default `/usr/include/eigen3`) |
| MuJoCo (C library) | 3.14.0 | `-s`, `-b`, the humanoid, `model_check` (`MUJOCO_DIR`, default `/opt/mujoco`) |
| ROS 2 | Jazzy | the bridge and the camera pipeline (optional) |
| Python 3 | with numpy; mujoco, matplotlib for some tools | the tools |

## Build

    cd indy7
    make                       # bin/indy7_ctrl; prints [indy7] MUJOCO=.. ROS=.. MALLOC_COUNT=..
    make tools                 # bin/hl_check, bin/goal_filter_check, bin/slot_check, bin/model_check (MuJoCo)

`make MUJOCO=0` builds without the simulation, `make ROS=0` without the bridge (the default is ROS=1 when ROS 2
and the messages of `ros2/` are found: build `ros2/` first and source both environments, `ros2/README.md`).
`make MALLOC_COUNT=1` (after `make clean`) is the test build that counts the heap allocations of the real-time
tasks, which must stay at zero. The humanoid: `cd humanoid && make`.

## Host configuration (paper, Sec. III-A, Table 3)

The experiments ran on an Intel NUC 13 Pro (Core i7-1360P). Its configuration:

- kernel: PREEMPT_RT, high-resolution timers, `HZ_1000`, `NO_HZ_FULL`, `RCU_NOCB_CPU`; no CPU-frequency or
  idle-state driver;
- boot: `isolcpus=domain,managed_irq,6-7 nohz_full=6-7 rcu_nocbs=6-7 irqaffinity=0-5,8-15 idle=poll`; CPU 7 (the
  sibling of the real-time core 6) offline: `echo 0 | sudo tee /sys/devices/system/cpu/cpu7/online`;
- CPU 6 at its largest non-turbo frequency, turbo off: `tools/host/cpu-noturbo.sh` (and its systemd unit);
- runtime: `sudo sysctl kernel.timer_migration=0`; NTP stopped (`sudo timedatectl set-ntp false`); Wi-Fi off;
- limits of the user: rtprio 99 and memlock unlimited (`indy7/run.sh` checks them);
- the thread of the IgH master at SCHED_FIFO 40: `sudo bash tools/host/install_ecat_op_fifo.sh` once, then
  `indy7/run.sh` lifts it at every start.

`[RT_TASKS]` of `INDY7.cfg` places the three tasks on CPU 6; the controller reads back the policy, the priority
and the affinity of every task at its start and refuses to run otherwise.

## Running

    cd indy7
    ./run.sh -s                # simulation: MuJoCo in place of the bus
    ./run.sh                   # the robot (the bus must be up: ethercat.service)
    ./run.sh -b -t DIR         # the bus of the robot in OP, drives off, beside the simulated arm; timing into DIR

The keys of the operator are printed at the start and with `?`:

| Key | Action |
|---|---|
| `r` | controller on/off (it starts in gravity compensation; off also clears a latched fault) |
| `g` | gravity compensation |
| `h` / `j` | servo on (brakes released, gravity compensation holds) / servo off (brakes) |
| `x` / `e` | stop the motion and hold / e-stop: stop, servo off, brakes |
| `k` | the hold of the gravity compensation on/off |
| `m` | joint move: absolute joint angles in degrees (refused beyond the URDF limits minus the margin) |
| `t` | task move: tool position x y z in mm (refused when the IK cannot reach it) |
| `p` | camera application: the objects in view; a digit makes the arm follow one (needs the bridge) |
| `u` | joint tests and identification: 1 fast PD+g, 2 fast CTC, 3 slow CTC, 4 slow CTC+FF, 5 torque step, 6 breakaway, 7 sine |
| `y` | high-level tests: 1 random goals, 2 goal jumps, 3 goal noise with delta, 4 with delta = 0, 5 reference check, 6 HOME, 7 record q_r, 8 camera application |
| `s` / `w` | print the tool pose / arm the pose capture of the hand-eye calibration (then `s` writes the pose) |
| `q` | quit (the drives are switched off before the master is released) |

On the robot, start with `r`, `g`, `h` and keep the emergency stop at hand. The constants of `[AXIS0]`..`[AXIS5]`
(encoder offsets `HOME_POSITION_OFFSET`, directions `MOVE_CCW`) belong to the robot of the experiments, and the
drive scales of Table 5 were not measured independently: check them on another robot with the arm braked
before any motion. The software responses to faults (a latched controller fault, a drive dropout) are
supervisory behavior, not certified protective functions.

`SYNC_MODE` of `[ECAT_MASTER]` selects how the drives are synchronized: `FREE_RUN` (the experiments of the
paper; nothing is written to the drives), `SM` (synchronous with the frame) or `DC` (distributed clocks,
SYNC0 `DC_SYNC0_SHIFT` after the release). With `SM` or `DC` the controller checks that every drive supports
the mode and runs in it once the bus is in OP, and ends with a warning otherwise.

Logs: the joint and high-level tests write a CSV and a JSON per run into `JOINT_TEST_LOG_DIR` (default
`indy7/jt_logs`); `-t DIR` writes the timing of every period into `DIR/timing_<stamp>.bin`.

## The paper and the code

| Paper | Code | Reproduction |
|---|---|---|
| Fig. 1, Sec. II | `indy7/src/Indy7Ctrl.h`, `TaskEtherCAT.cpp`, `TaskTrajectory.cpp`, `TaskDynamics.cpp`, `TaskSlots.h` | |
| eq. (1), Sec. II-A | `indy7/src/base/AtomicSlot.h` | `indy7/bin/slot_check` (1 writer, 3 readers, 150,000 snapshots; `--unsafe` shows that the test sees torn copies) |
| Table 3 | `indy7/src/base/RtTask.h`, `INDY7.cfg` `[RT_TASKS]`, `indy7/run.sh` | `tools/host/` |
| Table 4 | | `tools/host/run_cyclictest.sh`, then `tools/host/cyclictest_summary.py` |
| eq. (2), Table 5 | `indy7/src/ecat/Indy7Joint.cpp`, `INDY7.cfg` `[AXISn]` | `tools/host/bus_inventory.sh` (PDO mapping, sync objects) |
| eq. (3), Fig. 2 | `indy7/src/TaskEtherCAT.cpp`, `SEND_DEADLINE` | |
| Table 6, Fig. 3 | `indy7/src/TimingLog.h` (`-t`) | `tools/run_timing.py --condition idle|stress|perception`, `tools/timing_summary.py` |
| eqs. (4), (5), Table 7 | `indy7/src/FullDynControllerRT.cpp` | `indy7/bin/model_check` (RBDL vs MuJoCo), `tools/phase_margin.py` (PM rows) |
| eq. (6), Table 7 (B) | JOINT TEST 7 | `tools/sine_cfg.py`, `tools/identify_rotor.py` |
| eq. (7), Table 7 (tau_s, K_f) | JOINT TESTs 6 and 4 | `tools/jt_breakaway.py`, `tools/kf_step.py` |
| eq. (8), Table 8 | | `tools/jt_predict.py`, `tools/phase_margin.py` |
| eq. (9), Table 9, Sec. IV-C | `indy7/src/HighLevelRT.h`, `TrajectoryRT.h` | `indy7/bin/hl_check` (the reference contract C1-C5); delta: `tools/vision_probe.py`, `tools/goal_noise_file.py` |
| Table 10, Figs. 4 and 5 | JOINT TESTs 3 and 4 | `tools/jt_run_sim.py` (simulation), `tools/jt_analyze.py` |
| Table 11, Figs. 6-8 | high-level tests 1 and 2 | `tools/hl_run_sim.py` (simulation), `tools/hl_analyze.py --static-err ...`, `tools/sim_view.py` |
| eq. (10), Figs. 9 and 10 | `calib/`, `ros2/`, `indy7/src/ROS2PickBridge.h`, `GoalFilter.h` | `indy7/bin/goal_filter_check`, `tools/app_analyze.py`, `tools/place_check.py`, `tools/snap_view.py` |
| Table 12, Fig. 11 | `humanoid/src/HumanoidSim.cpp` | `humanoid/bin/humanoid_sim`, `humanoid/tools/hum_analyze.py` |

### Checks without the robot

    cd indy7 && make && make tools
    ./bin/slot_check                                      # PASS: 0 inconsistent snapshots accepted
    ./bin/hl_check models/indy7.urdf                      # the reference contract, PASS
    ./bin/goal_filter_check                               # the goal checks of the bridge, PASS
    ./bin/model_check models/indy7.urdf models/indy7_mujoco.xml B=2.0,2.0,1.99649,0.87306,0.86394,0.84747
    python3 ../tools/phase_margin.py                      # Tables 7 (PM) and 8
    make clean && make MALLOC_COUNT=1 && ./run.sh -s      # 'r', 'g', 'h', 'u' 3: no allocation ([MALLOC] at the end)

The three tasks run on CPU 6 (`CPU` of `[TASK0]`..`[TASK2]`), and `run.sh` needs the limits of the host
configuration. On a host whose real-time core is another one, give the tasks that core in a copy of the cfg in
`indy7/` (its relative paths stay valid; the commands from here on run in the top directory):

    cp indy7/INDY7.cfg indy7/sim.cfg
    python3 tools/cfgset.py indy7/sim.cfg TASK0.CPU=2 TASK1.CPU=2 TASK2.CPU=2

and pass the copy to the tools with `--cfg indy7/sim.cfg` (by hand, in `indy7/`: `./run.sh -s -f sim.cfg`).

### Joint tracking (Sec. V-A)

On the robot: `r`, `g`, `h`, then `u` `4` five times (CTC + FF) and, for the comparison, the same with all
`KF_n` set to 0 (`python3 tools/cfgset.py indy7/INDY7.cfg KF_0=0 ... KF_5=0`, restart). Then

    python3 tools/jt_analyze.py indy7/jt_logs            # Table 10, e_bar for Table 11

The simulation of Fig. 4 runs without friction and rotor inertia, with J6 held, on a copy of the cfg (on
another host with the `TASKn.CPU` values above as well):

    cp indy7/INDY7.cfg indy7/fig4.cfg
    python3 tools/cfgset.py indy7/fig4.cfg ROTOR_INERTIA_0=0 ROTOR_INERTIA_1=0 ROTOR_INERTIA_2=0 ROTOR_INERTIA_3=0 \
        ROTOR_INERTIA_4=0 ROTOR_INERTIA_5=0 "JOINT_TEST_SLOW_DQ=0.9 0.6 0.9 0.9 0.9 0" JOINT_TEST_LOG_DIR=jt_fig4
    python3 tools/jt_run_sim.py --cfg indy7/fig4.cfg --tests slow-ctc -n 5 --analyze    # PASS: within 1.5 x eq. (8)

The simulator steps in the EtherCAT task, once per period, so the simulated loop has the timing of the host:
the values of the paper, and five identical runs, need a host configured as in Table 3. The summary at the end
of a run counts the late releases, overruns, and missed send deadlines; with any of them, the simulated loop
has seen the delay.

### Task-space goals (Sec. V-B)

    python3 tools/hl_run_sim.py --tests hl-fixed,hl-jump --analyze        # simulation, the goals of the robot (seed 1)
    python3 tools/hl_analyze.py indy7/jt_logs --static-err "2.75 0.83 1.52 7.16 6.42 0.66"   # Table 11 with Est.

On the robot: `y` `1` runs the ten goals of `HL_RAND_*` once (five rounds for Table 11), `y` `2` the goal jumps.

### Host timing (Sec. III-C)

With the bus of the robot (drives in OP, power stages off), 10 min per condition:

    python3 tools/run_timing.py --condition idle
    python3 tools/run_timing.py --condition stress
    python3 tools/run_timing.py --condition perception       # with the camera pipeline running (ros2/README.md)

Each run writes the row of Table 6 into `timing_summary.txt` and `timing_table.csv` of its result directory.

### Camera application (Sec. V-C)

Calibration (in `calib/`, the controller running with `r`, `g`, `h` and the capture armed with `w`):
`calib_grab.py` (images; `s` in the controller writes the matching pose), `calib_cpo.py --tag-mm <side>`
(tag poses), `eye_to_hand_calib.py` (rMc; move ambiguous or outlying pairs to `data/excluded/` and run it
again), `calib_to_tf.py --live --write` (the transform of the launch file). Then start the pipeline
(`ros2/README.md`), the controller with its bridge, and choose the object with `p`. `tools/app_analyze.py`
evaluates the runs (`hl_app_*` logs).

### Humanoid (Sec. VI)

    cd humanoid && make
    ./bin/humanoid_sim --models models --mode check                              # RBDL vs MuJoCo, PASS
    ./bin/humanoid_sim --models models --mode joint --cpu 6 --out /tmp/hum_joint  # 6000 periods, no load
    python3 tools/hum_analyze.py /tmp/hum_joint models/g1_mujoco.xml             # Fig. 11 (e)
    ./bin/humanoid_sim --models models --mode hl --goals 5000 --duration 3600 --cpu 6 --out /tmp/hum_hl
                                                                                 # under stress-ng: Table 12

The summary of each run (`summary_<mode>.txt`) holds the columns of Table 12; `hl_goals.csv` the final
distances of Fig. 11 (f).

## Third-party content

- `third_party/rt-posix/`: RT-POSIX, unmodified.
- `indy7/models/indy7.urdf`: from `neuromeka-robotics/indy-ros2` (variant indy7, commit ca0353f); the changes
  are described at its head. Its mesh paths refer to the ROS 2 package `indy_description` of that repository.
- `humanoid/models/`: derived from `unitreerobotics/unitree_ros` (`g1_29dof.urdf`), see `humanoid/models/README.md`.
- The YOLOv5n model of the camera pipeline is not included (`ros2/README.md`).
