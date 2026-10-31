#!/bin/bash
# bus_inventory.sh - the EtherCAT bus of the robot, read with the command-line tool of the IgH master
# (paper, Sec. III-B, Table 5). Read-only: SDO uploads and listings, nothing is written to a slave.
# The control box powered, the master running (ethercat.service), the controller NOT running.
#
#   bash tools/host/bus_inventory.sh [OUTDIR]               everything below, into OUTDIR
#   bash tools/host/bus_inventory.sh --counters FILE        only the error counters of the sync managers,
#                                                           appended to FILE with a time stamp (before and
#                                                           after a run)
# OUTDIR defaults to ./bus_<date>. JOINTS="0 1 2 3 4 5" (the bus positions of the joint drives, PHYSICAL_POS
# of the [AXISn] sections of indy7/INDY7.cfg) can be set in the environment.
#
# Files: master.txt, slaves.txt, slaves_v.txt (names, vendor and product codes, DC delays), pdos_p<n>.txt
# (the PDO mapping of every slave, to compare with Table 5), sdo_p<n>.txt (identity, versions, limits),
# sync_objects.txt (0x1C32 / 0x1C33 of every joint drive: sync type, cycle, shift, supported types, calc
# and copy time, delay, the error counters), and summary.txt: the synchronization types the drives support
# and report (SYNC_MODE of indy7/INDY7.cfg: FREE_RUN, SM or DC), the largest calc and copy time 0x1C32:06
# (for DC_SYNC0_SHIFT with DC), and the propagation delays.
set -u
JOINTS=${JOINTS:-"0 1 2 3 4 5"}
ETHERCAT=${ETHERCAT:-ethercat}

up() {   # up <pos> <index> <sub> <type>: the value, or "-" with the error
    local out
    if out=$($ETHERCAT upload -p "$1" -t "$4" "$2" "$3" 2>&1); then
        echo "$out" | tr '\n' ' ' | sed 's/ *$//'
    else
        echo "- ($(echo "$out" | head -1))"
    fi
}

dec() {   # the decimal value of an upload ("0x0002 2" -> 2), "-" if it failed
    case "$1" in
        -*) echo "-" ;;
        *) echo "$1" | grep -oE '\b[0-9]+\b' | tail -1 ;;
    esac
}

counters() {   # the error counters of the sync managers of every joint drive, one line
    local line="" p
    for p in $JOINTS; do
        line="$line p$p 0x1C32:0B=$(dec "$(up "$p" 0x1C32 0x0B uint16)")"
        line="$line :0C=$(dec "$(up "$p" 0x1C32 0x0C uint16)")"
        line="$line :0D=$(dec "$(up "$p" 0x1C32 0x0D uint16)")"
        line="$line :20=$(dec "$(up "$p" 0x1C32 0x20 uint8)")"
    done
    echo "$(date '+%F %T')$line"
}

if [ "${1:-}" = "--counters" ]; then
    [ -n "${2:-}" ] || { echo "usage: $0 --counters FILE" >&2; exit 2; }
    mkdir -p "$(dirname "$2")"
    counters >> "$2"
    tail -1 "$2"
    exit 0
fi

OUT=${1:-./bus_$(date +%Y%m%d_%H%M)}
mkdir -p "$OUT" || exit 1
cd "$OUT" || exit 1
echo "bus inventory -> $OUT"
if pgrep -x indy7_ctrl >/dev/null; then
    echo "the controller is running: stop it first (SDO reads while it runs would load the master)" >&2
    exit 1
fi
$ETHERCAT master > master.txt 2>&1 || { echo "ethercat master failed (ethercat.service?)" >&2; cat master.txt; exit 1; }
$ETHERCAT slaves > slaves.txt 2>&1
$ETHERCAT slaves -v > slaves_v.txt 2>&1
cat slaves.txt
NSL=$(grep -cE '^ *[0-9]+ ' slaves.txt)
echo "$NSL slaves"
for p in $(seq 0 $((NSL - 1))); do
    $ETHERCAT pdos -p "$p" > "pdos_p$p.txt" 2>&1
    {
        echo "p$p 0x1018:01 vendor   $(up "$p" 0x1018 1 uint32)"
        echo "p$p 0x1018:02 product  $(up "$p" 0x1018 2 uint32)"
        echo "p$p 0x1018:03 revision $(up "$p" 0x1018 3 uint32)"
        echo "p$p 0x1018:04 serial   $(up "$p" 0x1018 4 uint32)"
        echo "p$p 0x1008 device name $(up "$p" 0x1008 0 string)"
        echo "p$p 0x1009 hardware    $(up "$p" 0x1009 0 string)"
        echo "p$p 0x100A software    $(up "$p" 0x100A 0 string)"
    } > "sdo_p$p.txt"
done
for p in $JOINTS; do
    {
        echo "p$p 0x6072 max torque        $(up "$p" 0x6072 0 uint16)"
        echo "p$p 0x6080 max motor speed   $(up "$p" 0x6080 0 uint32)"
        echo "p$p 0x6060 mode of operation $(up "$p" 0x6060 0 int8)"
    } >> "sdo_p$p.txt"
done
: > sync_objects.txt
for p in $JOINTS; do
    for idx in 0x1C32 0x1C33; do
        for s in "01 uint16 sync type" "02 uint32 cycle time" "03 uint32 shift time" "04 uint16 types supported" \
                 "05 uint32 min cycle time" "06 uint32 calc and copy time" "09 uint32 delay time" \
                 "0A uint32 SYNC0 cycle time" "0B uint16 SM event missed" "0C uint16 cycle time too small" \
                 "0D uint16 shift time too short" "20 uint8 sync error"; do
            set -- $s
            sub=$1; typ=$2; shift 2
            echo "p$p $idx:$sub $* = $(up "$p" "$idx" "0x$sub" "$typ")" >> sync_objects.txt
        done
    done
done
counters > counters_initial.txt
C_NS=0
for p in $JOINTS; do
    v=$(dec "$(up "$p" 0x1C32 0x06 uint32)")
    [ -n "$v" ] && [ "$v" != "-" ] && [ "$v" -gt "$C_NS" ] && C_NS=$v
done
{
    echo "date: $(date '+%F %T %Z')"
    echo "slaves: $NSL"
    sed -n '1,20p' slaves.txt
    echo "synchronization types supported, 0x1C32:04 (bit 0 free run, bit 1 synchronous with the SM2 event"
    echo "  (SYNC_MODE=SM), bits 2-4 the DC types, 001 = SYNC0 (SYNC_MODE=DC)):"
    grep -E ':04 types supported' sync_objects.txt
    echo "sync type 0x1C32:01 / 0x1C33:01 (0 = free run, 1 = synchronous with the SM2 event, 2 = DC SYNC0;"
    echo "  read before the controller has run, it may be the default of the drive; '-' = not readable):"
    grep -E ':01 sync type' sync_objects.txt
    if [ "$C_NS" -gt 0 ]; then
        echo "calc and copy time 0x1C32:06: largest $C_NS ns (with SYNC_MODE=DC, DC_SYNC0_SHIFT must leave the"
        echo "  frame and this time before SYNC0)"
    else
        echo "calc and copy time 0x1C32:06: not readable or 0"
    fi
    echo "propagation delays (slaves -v):"
    grep -iE 'transmission delay|delay to next dc' slaves_v.txt | head -10
} > summary.txt
cat summary.txt
echo "files: $(ls | tr '\n' ' ')"
