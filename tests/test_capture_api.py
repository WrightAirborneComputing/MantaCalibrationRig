"""Tests for PositionReader's rate negotiation and raw capture.

Run against a fake Pico on a pty (tests/fake_pico.py) so the reader thread, the
reply routing and the capture buffer are exercised over a real serial port
without hardware. These are the parts with genuine failure modes - threading and
a shared port - so they are worth testing for real rather than by stubbing.

    python3 -m pytest tests/test_capture_api.py -v
"""

import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fake_pico import FakePico

import MantaTrimmer as MT


def wait_until(predicate, timeout=5.0, interval=0.02):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)

    return False
# def


@pytest.fixture
def rig():
    """A PositionReader streaming from a fake Pico, torn down cleanly."""
    pico = FakePico().start()
    reader = MT.PositionReader()
    reader.set_port(pico.device)

    assert wait_until(reader.is_streaming), "reader never started streaming"

    try:
        yield reader, pico
    finally:
        reader.stop()
        pico.stop()
# def


def test_history_depth_follows_the_rate():
    """The 10 Hz case must not change; 1000 Hz must outlast the averaging window."""
    assert MT.position_history_for(MT.SLOW_RATE_HZ) == 200

    fast = MT.position_history_for(MT.FAST_RATE_HZ)
    assert fast == 4000
    assert fast > MT.POSITION_WINDOW_S * MT.FAST_RATE_HZ
# def


@pytest.mark.parametrize("hz", [10, 100, 500, 1000, 1500, 2000])
def test_history_always_outlasts_the_averaging_window(hz):
    """The invariant that a fixed 200-sample backlog quietly broke at 500 Hz."""
    assert MT.position_history_for(hz) > MT.POSITION_WINDOW_S * hz
# def


def test_starts_slow_and_readable(rig):
    reader, _ = rig

    assert reader.sample_hz == MT.SLOW_RATE_HZ
    assert reader._queue_left.maxlen == 200
# def


def test_status_command_round_trips(rig):
    reader, _ = rig

    assert reader.send_command("?").startswith("# STATUS")
# def


def test_rate_negotiation_resizes_the_backlog(rig):
    reader, _ = rig

    assert reader.set_sample_rate(MT.FAST_RATE_HZ) == MT.FAST_RATE_HZ
    assert reader.sample_hz == MT.FAST_RATE_HZ
    assert reader._queue_left.maxlen == MT.position_history_for(MT.FAST_RATE_HZ)
# def


def test_rate_is_clamped_by_the_firmware(rig):
    reader, _ = rig

    assert reader.set_sample_rate(99999) == 2000
# def


def test_returning_to_slow_shrinks_the_backlog(rig):
    reader, _ = rig

    reader.set_sample_rate(MT.FAST_RATE_HZ)
    assert reader.set_sample_rate(MT.SLOW_RATE_HZ) == MT.SLOW_RATE_HZ
    assert reader._queue_left.maxlen == 200
# def


def test_resize_keeps_the_newest_samples():
    """Shrinking the backlog must drop history, never the latest reading."""
    reader = MT.PositionReader()

    for i in range(300):
        reader._queue_left.append((float(i), i))
        reader._queue_right.append((float(i), i))

    reader._resize_history(MT.SLOW_RATE_HZ)

    assert reader._queue_left.maxlen == 200
    assert reader._queue_left[-1] == (299.0, 299)
# def


def test_capture_collects_raw_timestamped_samples(rig):
    reader, _ = rig
    reader.set_sample_rate(MT.FAST_RATE_HZ)

    reader.start_capture()
    assert reader.is_capturing()
    time.sleep(0.6)
    samples, truncated = reader.stop_capture()

    assert not truncated
    assert not reader.is_capturing()

    # 0.6 s at the fast rate. Generous bounds - this is a real timed capture.
    expected = 0.6 * MT.FAST_RATE_HZ
    assert 0.5 * expected < len(samples) < 1.5 * expected

    host_t, pico_us, raw_l, raw_r = samples[0]
    assert pico_us is not None
    assert isinstance(raw_l, int) and isinstance(raw_r, int)

    stamps = [s[1] for s in samples]
    assert stamps == sorted(stamps), "board timestamps must be monotonic"

    hosts = [s[0] for s in samples]
    assert hosts == sorted(hosts)
# def


def test_capture_resolves_a_transit_far_better_than_the_averaged_getter(rig):
    """Why capture exists at all: the averaged getter cannot see 100 ms events."""
    reader, _ = rig
    reader.set_sample_rate(MT.FAST_RATE_HZ)

    reader.start_capture()
    time.sleep(0.4)
    samples, _ = reader.stop_capture()

    span_s = (samples[-1][1] - samples[0][1]) / 1000000.0
    per_100ms = len(samples) * (0.1 / span_s)

    # A 100 ms transit must be covered by many samples, not one or two.
    assert per_100ms > 30
# def


def test_capture_is_bounded(rig):
    reader, _ = rig
    reader.set_sample_rate(MT.FAST_RATE_HZ)

    reader.start_capture(limit=25)
    assert wait_until(lambda: reader._capture_truncated, timeout=3.0)

    samples, truncated = reader.stop_capture()

    assert truncated
    assert len(samples) == 25
# def


def test_capture_is_cleared_when_the_port_changes(rig):
    reader, pico = rig

    reader.start_capture()
    reader.set_port(pico.device + "-does-not-exist")

    assert not reader.is_capturing()

    samples, _ = reader.stop_capture()
    assert samples == []
# def


def test_averaging_still_works_at_the_fast_rate(rig):
    """The regression the backlog resize prevents: the window outrunning history."""
    reader, _ = rig
    reader.set_sample_rate(MT.FAST_RATE_HZ)

    assert wait_until(
        lambda: reader.get_average_position_nonblocking("LEFT") is not None)

    left = reader.get_average_position_nonblocking("LEFT")
    right = reader.get_average_position_nonblocking("RIGHT")

    assert 31000 < left < 32000
    assert 34000 < right < 35000
# def


def test_replies_never_reach_the_sample_queues(rig):
    """A "# ACK" line must not be parsed as a position."""
    reader, _ = rig

    reader.clear_queues()
    for _ in range(5):
        reader.send_command("?")

    time.sleep(0.3)

    with reader._lock:
        values = [v for _, v in reader._queue_left]

    assert all(v > 1000 for v in values), "a reply leaked into the sample queue"
# def


def test_stop_returns_the_board_to_slow_mode(rig):
    reader, pico = rig

    reader.set_sample_rate(MT.FAST_RATE_HZ)
    assert pico.state["hz"] == MT.FAST_RATE_HZ

    reader.stop()

    assert wait_until(lambda: pico.state["hz"] == MT.SLOW_RATE_HZ, timeout=3.0)
    assert reader.sample_hz == MT.SLOW_RATE_HZ
# def


def test_send_command_returns_none_without_a_port():
    reader = MT.PositionReader()

    assert reader.send_command("F") is None
    assert reader.set_sample_rate(MT.FAST_RATE_HZ) is None
# def


def test_legacy_firmware_is_reported_not_crashed():
    """A board that never answers must degrade, not raise."""
    pico = FakePico().start()
    reader = MT.PositionReader()

    try:
        reader.set_port(pico.device)
        assert wait_until(reader.is_streaming)

        # Stop the fake answering, exactly like the 10 Hz firmware.
        pico.sampler_apply = None
        pico._stop.set()
        pico._thread.join(2.0)

        assert reader.send_command("F", timeout=0.2, attempts=1) is None
    finally:
        reader.stop()
        pico.stop()
# def
