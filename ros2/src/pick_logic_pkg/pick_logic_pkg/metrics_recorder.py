"""In-memory metric buffering that writes ONE CSV at window-close or shutdown.

Mirrors the machinery of the detector node so every pipeline node records
the same way: cheap per-frame append, a single CSV write at the end, and a
disabled default (csv_path='') that costs nothing.

Clocks:
  - INTRA-node stage durations -> time.perf_counter_ns() (monotonic).
  - CROSS-node ages / E2E       -> get_clock().now() minus a propagated
                                   capture_stamp (shared wall clock). Use age_ms().
"""
import csv
import os
import time


def stamp_to_ns(stamp):
    """builtin_interfaces/Time -> int nanoseconds. Returns 0 if unset/invalid."""
    sec = int(stamp.sec)
    nanosec = int(stamp.nanosec)
    if sec <= 0 and nanosec <= 0:
        return 0
    return sec * 1_000_000_000 + nanosec


def age_ms(now_ns, stamp):
    """(now - capture_stamp) in ms on the shared wall clock. Returns None if the
    stamp is unset or the clock went backwards -- guarded like the detector's
    frame_age_ms(). now_ns must come from node.get_clock().now().nanoseconds."""
    sn = stamp_to_ns(stamp)
    if sn <= 0:
        return None
    age = now_ns - sn
    if age < 0:
        return None
    return age / 1e6


def _round(value, ndigits=3):
    return None if value is None else round(value, ndigits)


class MetricsRecorder:
    """Per-frame row buffer -> one CSV. Disabled (no buffering, no I/O) when
    csv_path is empty, exactly like the detector node's metrics path."""

    def __init__(self, logger, csv_path, duration_sec=0.0):
        self._log = logger
        self.csv_path = csv_path or ""
        self.duration_sec = float(duration_sec)
        self.rows = [] if self.csv_path else None
        self._start_ns = None
        self._closed = False
        self._fieldnames = None
        self._saved_rows = -1   # len(rows) at last successful save

    @property
    def enabled(self):
        return self.rows is not None

    def add(self, row):
        """Append one row (dict). No-op when disabled or the window is closed."""
        if self.rows is None or self._closed:
            return
        now = time.perf_counter_ns()  # monotonic: measures the window duration only
        if self._start_ns is None:
            self._start_ns = now
            if self.duration_sec > 0:
                self._log.info(
                    "Metrics window started: collecting "
                    f"{self.duration_sec:.0f}s from the first row -> {self.csv_path}")
        if self._fieldnames is None:
            self._fieldnames = list(row.keys())
        self.rows.append(row)
        if self.duration_sec > 0 and (now - self._start_ns) / 1e9 >= self.duration_sec:
            self.save()
            self._closed = True
            self._log.info(
                f"Metrics window {self.duration_sec:.0f}s complete: "
                f"{len(self.rows)} rows -> {self.csv_path}. You can stop now (Ctrl-C).")

    def save(self):
        """Write buffered rows to csv_path ATOMICALLY (temp file + os.replace).

        Why atomic: shutdown runs this inside `finally` while a SECOND signal can
        still arrive (e.g. Ctrl-C delivers SIGINT to the process group AND launch
        propagates its own SIGINT to each child). A plain open("w") rewrite that
        gets KeyboardInterrupt'ed mid-write TRUNCATES the file. With replace(),
        the reader only ever sees the previous complete file or the new
        complete file.

        Idempotent, and a no-op when the closed window was already saved (the
        shutdown re-save would rewrite identical bytes)."""
        if not self.csv_path or not self.rows:
            return
        if self._closed and self._saved_rows == len(self.rows):
            return  # window closed and already on disk -- nothing new to write
        directory = os.path.dirname(self.csv_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        fieldnames = self._fieldnames or list(self.rows[0].keys())
        tmp_path = self.csv_path + ".tmp"
        with open(tmp_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(self.rows)
        os.replace(tmp_path, self.csv_path)   # atomic on POSIX
        self._saved_rows = len(self.rows)
        self._log.info(f"Saved {len(self.rows)} metric rows to {self.csv_path}")


def write_summary_csv(logger, path, rows):
    """Write a small summary CSV (list of dicts) once -- e.g. a yield/reliability
    tally at shutdown. No-op when path or rows is empty. Column set is the union
    of the rows' keys, first-seen order preserved. Atomic (temp + replace) for
    the same second-signal reason as MetricsRecorder.save()."""
    if not path or not rows:
        return
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    fieldnames = []
    for r in rows:
        for k in r.keys():
            if k not in fieldnames:
                fieldnames.append(k)
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    os.replace(tmp_path, path)
    logger.info(f"Saved yield summary ({len(rows)} rows) to {path}")
