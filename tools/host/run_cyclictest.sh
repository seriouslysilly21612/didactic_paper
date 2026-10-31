#!/bin/bash
# run_cyclictest.sh - the scheduler wake-up latency of Table 4 (paper, Sec. III-A): cyclictest on the
# isolated core, idle and under stress-ng, and on a non-isolated core under stress-ng, one hour each, with
# the controller stopped. Run as root on the configured host (Table 3), inside tmux or screen, and leave
# the host alone until the end (about 3 h 10 min):
#
#     sudo bash tools/host/run_cyclictest.sh [--duration 1h] [--only iso_idle,iso_stress,noniso_stress]
#                                            [--out DIR] [--dry-run]
#     python3 tools/host/cyclictest_summary.py DIR          the rows of Table 4
#
# Conditions: iso_idle (cyclictest on RT_CPU, no load), iso_stress (RT_CPU, stress-ng on OTHERS),
# noniso_stress (NONRT_CPU, stress-ng on OTHERS). cyclictest: one thread, SCHED_FIFO 97, 1 ms interval,
# memory locked, a histogram of 1-us bins up to 10 ms, its main thread on OTHERS. stress-ng: --cpu 14
# --vm 2 --vm-bytes 35% on OTHERS, started WARMUP s before the run. RT_CPU (6), NONRT_CPU (4), OTHERS
# (0-5,8-15) and WARMUP (120) can be set in the environment. Before and after each run the state of the
# host is recorded (<id>.state_before.txt, <id>.state_after.txt): the kernel, its command line,
# kernel.timer_migration, the online CPUs, the interrupts of the measured CPU.
set -u
RT_CPU=${RT_CPU:-6}
NONRT_CPU=${NONRT_CPU:-4}
OTHERS=${OTHERS:-0-5,8-15}
WARMUP=${WARMUP:-120}
PRIO=97
HIST=10000
DUR=1h
CONDS="iso_idle iso_stress noniso_stress"
OUT=""
DRY=0
STRESS_ARGS="--cpu 14 --vm 2 --vm-bytes 35% --timeout 0 --quiet"
while [ $# -gt 0 ]; do
    case "$1" in
        --duration) DUR=$2; shift ;;
        --only) CONDS=$(echo "$2" | tr ',' ' '); shift ;;
        --out) OUT=$2; shift ;;
        --dry-run) DRY=1 ;;
        -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done
[ -n "$OUT" ] || OUT=./cyclictest_$(date +%Y%m%d_%H%M)

cond_cpu()  { case "$1" in noniso_*) echo "$NONRT_CPU" ;; *) echo "$RT_CPU" ;; esac; }
cond_load() { case "$1" in *_idle) echo 0 ;; *) echo 1 ;; esac; }
ct_cmd()    { echo "cyclictest -t1 -a $1 -p$PRIO -i1000 -m -h$HIST -D$DUR -q --mainaffinity=$OTHERS"; }

for c in $CONDS; do
    case "$c" in iso_idle|iso_stress|noniso_stress) ;; *) echo "unknown condition $c" >&2; exit 2 ;; esac
done
if [ "$DRY" = 1 ]; then
    for c in $CONDS; do
        echo "$c: $( [ "$(cond_load "$c")" = 1 ] && echo "stress-ng --taskset $OTHERS $STRESS_ARGS; " )$(ct_cmd "$(cond_cpu "$c")") --histfile=$OUT/$c.hist"
    done
    exit 0
fi
if [ "$(id -u)" != 0 ]; then echo "run with sudo: sudo bash $0" >&2; exit 1; fi
for t in cyclictest stress-ng; do
    command -v $t >/dev/null || { echo "$t is missing (apt install rt-tests stress-ng)" >&2; exit 1; }
done
if pgrep -x indy7_ctrl >/dev/null; then echo "the controller is running: stop it first" >&2; exit 1; fi
[ "$(cat /sys/kernel/realtime 2>/dev/null)" = 1 ] || echo "WARNING: not a PREEMPT_RT kernel (Table 3)"
grep -q "isolcpus=[^ ]*$RT_CPU" /proc/cmdline || echo "WARNING: CPU $RT_CPU is not in isolcpus= of /proc/cmdline (Table 3)"
[ "$(sysctl -n kernel.timer_migration 2>/dev/null)" = 0 ] || echo "WARNING: kernel.timer_migration is not 0 (Table 3)"
mkdir -p "$OUT" || exit 1

state() {   # state <file> <cpu>
    {
        echo "# $(date '+%F %T')"
        uname -a
        cat /proc/cmdline
        echo "kernel.timer_migration=$(sysctl -n kernel.timer_migration 2>/dev/null)"
        echo "online CPUs: $(cat /sys/devices/system/cpu/online)"
        echo "interrupts of CPU $2:"
        awk -v cpu="CPU$2" 'NR == 1 { for (i = 1; i <= NF; i++) if ($i == cpu) col = i + 1; next }
                            col && $col ~ /^[0-9]+$/ && $col > 0 { print $1, $col }' /proc/interrupts
    } > "$1"
}

STRESS_PID=""
stop_stress() {
    [ -n "$STRESS_PID" ] || return 0
    kill "$STRESS_PID" 2>/dev/null; wait "$STRESS_PID" 2>/dev/null
    STRESS_PID=""
    sleep 3
    pkill -f '^stress-ng' 2>/dev/null; sleep 1
}
trap 'stop_stress; exit 130' INT TERM

echo "results: $OUT"
for c in $CONDS; do
    cpu=$(cond_cpu "$c")
    if [ "$(cond_load "$c")" = 1 ]; then
        stress-ng --taskset "$OTHERS" $STRESS_ARGS &
        STRESS_PID=$!
        echo "$(date '+%T') $c: stress-ng started, $WARMUP s warm-up"
        sleep "$WARMUP"
    fi
    state "$OUT/$c.state_before.txt" "$cpu"
    echo "$(date '+%T') $c: $(ct_cmd "$cpu")"
    $(ct_cmd "$cpu") --histfile="$OUT/$c.hist" > "$OUT/$c.txt" 2>&1
    state "$OUT/$c.state_after.txt" "$cpu"
    stop_stress
    echo "$(date '+%T') $c: done"
done
echo "next: python3 $(dirname "$0")/cyclictest_summary.py $OUT"
