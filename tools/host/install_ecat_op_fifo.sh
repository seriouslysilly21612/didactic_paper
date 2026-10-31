#!/bin/bash
# install_ecat_op_fifo.sh - one-time root install: lift the kernel thread of the IgH master to SCHED_FIFO 40
# (paper, Table 3: "IgH master support thread: SCHED_FIFO priority 40").
#
# The kernel thread of the master in its operation phase, EtherCAT-OP, starts as SCHED_NORMAL. In IgH 1.6
# it shares no lock with the cyclic calls of the controller: send, receive, domain queue and clock
# synchronization take io_mutex, an rt_mutex with priority inheritance, while EtherCAT-OP takes only
# master_sem and hands its datagrams to the send of the controller without a lock. EtherCAT-OP runs the
# state machines of the master and the slaves: the AL states that the controller reads in every period, SDO
# requests, the reconfiguration of a slave. As SCHED_NORMAL under load (stress-ng, the camera pipeline) it
# can wait for milliseconds, and those results arrive late; at SCHED_FIFO 40 it runs above the load and
# below the interrupt threads (50) and the real-time tasks of the controller (95-97).
#
# This installs
#   /usr/local/sbin/ecat-op-fifo     a fixed-command wrapper: EtherCAT-OP to SCHED_FIFO 40
#   /etc/sudoers.d/ecat-op-fifo      lets USER run only that wrapper without a password, so that
#                                    indy7/run.sh calls it after every start of the controller
#
#   sudo bash tools/host/install_ecat_op_fifo.sh [USER]       (USER: default the user who runs sudo)
set -e
if [ "$(id -u)" != 0 ]; then
    echo "run with sudo: sudo bash $0" >&2
    exit 1
fi
U=${1:-${SUDO_USER:-}}
if [ -z "$U" ] || ! id "$U" >/dev/null 2>&1; then
    echo "usage: sudo bash $0 [USER] (the user who starts the controller)" >&2
    exit 2
fi

install -m 0755 /dev/stdin /usr/local/sbin/ecat-op-fifo <<'WRAPPER'
#!/bin/sh
# lift the IgH EtherCAT-OP kernel thread to SCHED_FIFO 40.
# A fixed command on purpose: it is referenced by a NOPASSWD sudoers rule.
pid=$(pgrep -x EtherCAT-OP) || exit 1
exec chrt -f -p 40 "$pid"
WRAPPER

echo "$U ALL=(root) NOPASSWD: /usr/local/sbin/ecat-op-fifo" > /etc/sudoers.d/ecat-op-fifo
chmod 0440 /etc/sudoers.d/ecat-op-fifo
visudo -cf /etc/sudoers.d/ecat-op-fifo

echo "installed: /usr/local/sbin/ecat-op-fifo and /etc/sudoers.d/ecat-op-fifo (user $U)"
echo "indy7/run.sh now lifts EtherCAT-OP to SCHED_FIFO 40 at every start."
