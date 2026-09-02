"""Tests for the sample-rate measurement in pico_monitor.py.

The point of the board-side timestamp is telling two failure modes apart, both
of which look identical from the host (fewer lines per second than expected):

  * the board could not sustain the rate  -> every interval is long
  * lines were lost in transport          -> most intervals are nominal, a few
                                             are clean multiples of it

Getting this wrong is easy and quiet, which is why it is tested: an earlier
version derived the rate from the *mean* delta, and a dropped line inflates the
mean by exactly enough to make the derived rate match the host's line count. The
loss became arithmetically invisible. The cadence must come from the median.

    python3 -m pytest tests/test_rate_report.py -v
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from manta_common import PICO_TICKS_MODULO
from pico_monitor import TickTracker


def tracker_from_deltas(deltas, start=1000):
    """Feed a tracker with timestamps that produce the given deltas."""
    tracker = TickTracker()
    t = start
    tracker.feed(t)

    for delta in deltas:
        t = (t + delta) % PICO_TICKS_MODULO
        tracker.feed(t)

    return tracker
# def


def test_no_timestamps_reports_nothing():
    assert TickTracker().report() is None
# def


def test_single_sample_has_no_intervals():
    tracker = TickTracker()
    tracker.feed(1234)

    assert tracker.report() is None
# def


def test_clean_stream():
    report = tracker_from_deltas([2000] * 100).report(nominal_us=2000.0)

    assert report["count"] == 100
    assert report["median_us"] == 2000
    assert report["sustained_hz"] == pytest.approx(500.0)
    assert report["gaps"] == 0
    assert report["dropped"] == 0
# def


def test_dropped_line_does_not_move_the_cadence():
    """The regression this file exists for.

    99 intervals at 2 ms plus one doubled interval: one line was lost. The board
    never changed cadence, and the report must still say 500 Hz.
    """
    deltas = [2000] * 99 + [4000]
    report = tracker_from_deltas(deltas).report(nominal_us=2000.0)

    assert report["sustained_hz"] == pytest.approx(500.0)
    assert report["dropped"] == 1
    assert report["gaps"] == 1

    # The mean is dragged up, which is exactly why it cannot be trusted here:
    # 1e6 / mean would report ~499 Hz and agree with the host's line count,
    # making the lost line invisible.
    assert report["mean_us"] > report["median_us"]
# def


def test_consecutive_drops_counted_individually():
    """One gap can represent more than one lost line."""
    report = tracker_from_deltas([2000] * 50 + [8000]).report(nominal_us=2000.0)

    assert report["gaps"] == 1
    assert report["dropped"] == 3
# def


def test_slow_board_is_not_reported_as_drops():
    """Uniformly long intervals mean the board is slow, not that lines vanished."""
    report = tracker_from_deltas([3333] * 100).report(nominal_us=2000.0)

    assert report["sustained_hz"] == pytest.approx(300.0, rel=1e-3)
    assert report["gaps"] == 0
    assert report["dropped"] == 0

    # This is the condition print_rate_report() warns on.
    assert report["median_us"] > report["nominal_us"] * 1.10
# def


def test_jitter_alone_is_not_a_drop():
    """Normal scheduling jitter must not be mistaken for missing samples."""
    deltas = [1900, 2100, 1950, 2050, 2000] * 20
    report = tracker_from_deltas(deltas).report(nominal_us=2000.0)

    assert report["gaps"] == 0
    assert report["dropped"] == 0
    assert report["sustained_hz"] == pytest.approx(500.0, rel=0.05)
# def


def test_counter_wrap_is_absorbed():
    """ticks_us() wraps at 2^30; a wrap must not read as a huge backwards jump."""
    tracker = TickTracker()

    start = PICO_TICKS_MODULO - 4000
    tracker.feed(start)
    tracker.feed((start + 2000) % PICO_TICKS_MODULO)
    tracker.feed((start + 4000) % PICO_TICKS_MODULO)   # wraps to 0
    tracker.feed((start + 6000) % PICO_TICKS_MODULO)

    report = tracker.report(nominal_us=2000.0)

    assert tracker.deltas == [2000, 2000, 2000]
    assert report["gaps"] == 0
    assert report["sustained_hz"] == pytest.approx(500.0)
# def


def test_cadence_is_inferred_when_no_rate_was_requested():
    """Without --rate there is no nominal, so the median has to stand in."""
    report = tracker_from_deltas([2000] * 30 + [4000]).report(nominal_us=None)

    assert report["nominal_us"] is None
    assert report["sustained_hz"] == pytest.approx(500.0)
    assert report["dropped"] == 1
# def


def test_degenerate_stream_does_not_divide_by_zero():
    """A board emitting identical timestamps must not crash the report."""
    assert tracker_from_deltas([0] * 10).report() is None
# def


def test_transport_loss_is_a_whole_multiple_of_the_period():
    """A lost line leaves a hole of exactly N periods: the board never paused."""
    report = tracker_from_deltas([2000] * 50 + [4010] + [2000] * 50).report(2000.0)

    assert TickTracker.gap_shape(report["gap_residual"]) == "transport"
# def


def test_board_stall_is_not_a_whole_multiple():
    """A collector stall leaves an arbitrary hole - measured at ~3.84 periods."""
    report = tracker_from_deltas([2002] * 50 + [7689] + [2002] * 50).report(2000.0)

    assert TickTracker.gap_shape(report["gap_residual"]) == "stall"
# def


def test_gap_shape_is_undefined_without_gaps():
    report = tracker_from_deltas([2000] * 50).report(2000.0)

    assert report["gap_residual"] is None
    assert TickTracker.gap_shape(report["gap_residual"]) is None
# def
