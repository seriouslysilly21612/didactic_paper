#!/usr/bin/env python3
"""cyclictest_summary.py - the rows of Table 4 (paper, Sec. III-A) from the histograms of
tools/host/run_cyclictest.sh.

  python3 tools/host/cyclictest_summary.py DIR [DIR ...]

For every <id>.hist (cyclictest --histfile, 1-us bins): the samples, Min, Mean, sigma, 99.9 %, 99.99 %,
99.999 % and Max in us. The latencies are whole microseconds, truncated, as cyclictest reports them; the mean
and the standard deviation are those of these values, and a percentile is the smallest latency L such that at
least that share of the samples is <= L. Samples beyond the last bin (histogram overflows) count in the
percentiles and in the maximum (cyclictest reports the maximum itself). Standard library only.
"""
import glob
import math
import os
import sys

ROWS = (('iso_idle', 'Isolated core, idle'), ('iso_stress', 'Isolated core, stress'),
        ('noniso_stress', 'Non-isolated core, stress'))
PCTS = (0.999, 0.9999, 0.99999)


def read_hist(path):
    """-> ({latency_us: count}, {'total', 'min', 'avg', 'max', 'overflows'} from the trailer)"""
    bins, info = {}, {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith('#'):
                key, _, val = line[1:].partition(':')
                vals = val.split()
                if vals and vals[0].lstrip('-').isdigit():
                    k = key.strip().lower()
                    name = {'total': 'total', 'min latencies': 'min', 'avg latencies': 'avg',
                            'max latencies': 'max', 'histogram overflows': 'overflows'}.get(k)
                    if name:
                        info[name] = int(vals[0])
                continue
            parts = line.split()
            if len(parts) >= 2 and parts[0].isdigit() and parts[1].isdigit():
                bins[int(parts[0])] = int(parts[1])
    return bins, info


def row(path):
    bins, info = read_hist(path)
    over = info.get('overflows', 0)
    n_in = sum(bins.values())
    n = n_in + over
    if n == 0:
        return None
    lat = sorted(k for k, v in bins.items() if v > 0)
    mean = sum(k * v for k, v in bins.items()) / n_in if n_in else float('nan')
    var = sum(v * (k - mean) ** 2 for k, v in bins.items()) / n_in if n_in else float('nan')
    out = {'n': n, 'min': info.get('min', lat[0] if lat else None), 'mean': mean, 'sd': math.sqrt(var),
           'max': info.get('max', lat[-1] if lat else None), 'over': over}
    top = max(bins) if bins else 0
    for p in PCTS:
        cum, out[p] = 0, None
        for k in range(0, top + 1):
            cum += bins.get(k, 0)
            if cum >= p * n - 1e-9:
                out[p] = k
                break
        if out[p] is None:
            out[p] = '>%d' % top            # in the overflows
    return out


def main(argv):
    if not argv:
        sys.exit(__doc__)
    print('%-28s %10s %5s %6s %6s %7s %8s %9s %5s %9s' % ('condition (us)', 'samples', 'Min', 'Mean', 'sigma',
                                                          '99.9%', '99.99%', '99.999%', 'Max', 'overflows'))
    for d in argv:
        files = {os.path.splitext(os.path.basename(p))[0]: p for p in glob.glob(os.path.join(d, '*.hist'))}
        order = [k for k, _ in ROWS if k in files] + sorted(k for k in files if k not in dict(ROWS))
        for k in order:
            r = row(files[k])
            if r is None:
                print('%-28s no samples' % k)
                continue
            print('%-28s %10d %5s %6.2f %6.2f %7s %8s %9s %5s %9d' % (
                dict(ROWS).get(k, k), r['n'], r['min'], r['mean'], r['sd'], r[0.999], r[0.9999], r[0.99999],
                r['max'], r['over']))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
