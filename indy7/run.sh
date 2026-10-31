#!/bin/bash
# run.sh - start the controller with the limits and the environment it needs
# (paper, Table 3):
#
#     ./run.sh [-f FILE] [-s | -b] [-t DIR]        the options of bin/indy7_ctrl
#
# - The real-time priority limit must be at least 97 and the memlock limit
#   unlimited, e.g. in /etc/security/limits.d/99-realtime.conf
#       <user>  -  rtprio   99
#       <user>  -  memlock  unlimited
#   (log in again afterwards).
# - MALLOC_ARENA_MAX=2: the controller locks all of its memory (mlockall with
#   MCL_FUTURE), and the default arenas of glibc (up to 64 MB per thread)
#   would inflate the locked memory.
# - With the bus (the robot, or -b): the kernel thread of the IgH master in
#   its operation phase, EtherCAT-OP, is created SCHED_OTHER. It runs the
#   state machines of the master and the slaves, whose results the EtherCAT
#   task reads in every period, and under load it can wait for milliseconds.
#   It is lifted to SCHED_FIFO 40, above the load and below the real-time
#   tasks (95-97), by a fixed-command sudo wrapper that
#   tools/host/install_ecat_op_fifo.sh installs once.
# - A build with ROS 2 (the bridge of the camera application) loads the type
#   support of its messages at run time: source /opt/ros/jazzy/setup.bash and
#   ros2/install/setup.bash first.
set -e
cd "$(dirname "$0")"

RT=$(ulimit -r); ML=$(ulimit -l)
if [ "$RT" != "unlimited" ] && [ "$RT" -lt 97 ]; then
    echo "run.sh: the real-time priority limit is $RT, at least 97 is needed (ulimit -r; see the head of run.sh)" >&2
    exit 1
fi
if [ "$ML" != "unlimited" ]; then
    echo "run.sh: the memlock limit is $ML, unlimited is needed (ulimit -l; see the head of run.sh)" >&2
    exit 1
fi
if [ ! -x ./bin/indy7_ctrl ]; then
    echo "run.sh: bin/indy7_ctrl is missing: make" >&2
    exit 1
fi

export MALLOC_ARENA_MAX=2

if ldd ./bin/indy7_ctrl 2>/dev/null | grep -q librclcpp && \
   [[ "${AMENT_PREFIX_PATH:-}" != *my_interfaces* ]]; then
    echo "run.sh: note: built with ROS 2, but ros2/install is not sourced - the bridge will not start" \
         "(source /opt/ros/jazzy/setup.bash && source ../ros2/install/setup.bash)"
fi

SIM=0
for a in "$@"; do
    [ "$a" = "-s" ] && SIM=1
done
if [ "$SIM" = 0 ]; then
    (
        for _ in $(seq 1 180); do
            if pgrep -x EtherCAT-OP >/dev/null 2>&1; then
                if sudo -n /usr/local/sbin/ecat-op-fifo 2>/dev/null; then
                    echo "[run.sh] EtherCAT-OP -> SCHED_FIFO 40"
                else
                    echo "[run.sh] WARNING: EtherCAT-OP stays SCHED_OTHER: install the wrapper once with" \
                         "sudo bash ../tools/host/install_ecat_op_fifo.sh, or for this session:" \
                         "sudo chrt -f -p 40 \$(pgrep -x EtherCAT-OP)"
                fi
                break
            fi
            sleep 0.5
        done
    ) &
fi

exec ./bin/indy7_ctrl "$@"
