"""nearest_by_stamp: pick the depth frame closest to the color capture instant.

Guards the choice of the nearest depth frame instead of the newest one: the
newest frame is the WRONG one, so a regression that quietly reverts to buf[-1]
must fail here. Pure function, no ROS needed:
    python3 -m pytest src/target_3d_pkg/test/test_depth_pick.py
"""
from collections import deque

from target_3d_pkg.pick_target_3d_node import nearest_by_stamp

MS = 1_000_000
PERIOD = 66_666_667   # 15 fps depth period, ns


def _buf(n=6, t0=1_000_000_000):
    """n depth frames at 15 fps, oldest -> newest, msg == its index."""
    return deque(((t0 + i * PERIOD, i) for i in range(n)), maxlen=n)


def test_picks_nearest_not_newest():
    buf = _buf()
    # A color frame captured at frame 2's instant: the detection arrives ~106 ms
    # later (1.6 periods), so frames 3 and 4 exist by now. Must still pick 2.
    assert nearest_by_stamp(buf, buf[2][0])[1] == 2
    assert nearest_by_stamp(buf, buf[2][0])[1] != buf[-1][1]


def test_rounds_to_the_closer_side():
    buf = _buf()
    assert nearest_by_stamp(buf, buf[2][0] + 20 * MS)[1] == 2   # < half period
    assert nearest_by_stamp(buf, buf[2][0] + 50 * MS)[1] == 3   # > half period


def test_capture_older_than_buffer_gives_oldest():
    buf = _buf()
    assert nearest_by_stamp(buf, buf[0][0] - 10 * PERIOD)[1] == 0


def test_no_usable_capture_stamp_falls_back_to_newest():
    buf = _buf()
    assert nearest_by_stamp(buf, 0)[1] == buf[-1][1]


def test_empty_buffer():
    assert nearest_by_stamp(deque(), 1_000_000_000) is None
