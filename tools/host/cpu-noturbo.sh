#!/bin/bash
# cpu-noturbo.sh - the operating point of the measurements (paper, Table 3: "CPU 6 fixed at 2.2 GHz; turbo
# disabled"): turbo off on every core (IA32_MISC_ENABLE bit 38), and the real-time core RT_CPU at the
# largest non-turbo ratio (MSR_PLATFORM_INFO 0xCE, bits 15:8) through IA32_PERF_CTL (0x199).
# For a PREEMPT_RT kernel built without a cpufreq driver (Table 3); with another kernel it does nothing (use
# intel_pstate/no_turbo there). Needs msr-tools (rdmsr, wrmsr) and root; cpu-noturbo.service runs it at boot:
#     sudo install -m 0755 tools/host/cpu-noturbo.sh /usr/local/sbin/cpu-noturbo.sh
#     sudo install -m 0644 tools/host/cpu-noturbo.service /etc/systemd/system/
#     sudo systemctl enable --now cpu-noturbo.service
set -eu
RT_CPU=${RT_CPU:-6}
uname -v | grep -q PREEMPT_RT || exit 0
modprobe msr
for d in /dev/cpu/[0-9]*; do
    [ -e "$d/msr" ] || continue
    c=${d##*/}
    v=$(rdmsr -p "$c" -c 0x1a0)
    wrmsr -p "$c" 0x1a0 $(( v | (1 << 38) ))
done
R=$(rdmsr -p "$RT_CPU" -d -f 15:8 0xce)
v=$(rdmsr -p "$RT_CPU" -c 0x199)
wrmsr -p "$RT_CPU" 0x199 $(( (v & ~0xFFFF) | (R << 8) ))
echo "cpu-noturbo: turbo off on all cores, CPU $RT_CPU at ratio $R"
