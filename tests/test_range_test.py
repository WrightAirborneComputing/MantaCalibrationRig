"""Tests for the elevon range/rate analysis in range_test.py.

Driven with synthetic traces whose answers are known exactly, so the metric
extraction is verified before any of it is pointed at a real control surface.
The servo model is the usual one: a dead time, then a constant-rate ramp to the
commanded endpoint.

    python3 -m pytest tests/test_range_test.py -v
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from manta_common import PICO_TICKS_MODULO
from range_test import (
    MIN_TRAVEL_DEG,
    analyse_leg,
    crossing_time,
    mean_sd,
    signed_tick_delta,
    to_series,
)

SAMPLE_HZ = 500
DT = 1.0 / SAMPLE_HZ


def ramp_series(start_deg, end_deg, rate_deg_s, dead_s=0.05,
                pre_roll_s=0.4, capture_s=2.0, noise=None):
    """A servo trace: `dead_s` of nothing, then a constant-rate ramp."""
    travel = end_deg - start_deg
    direction = 1.0 if travel > 0 else -1.0
    duration = abs(travel) / rate_deg_s

    series = []
    t = -pre_roll_s

    while t <= capture_s:
        if t < dead_s:
            angle = start_deg
        elif t < dead_s + duration:
            angle = start_deg + direction * rate_deg_s * (t - dead_s)
        else:
            angle = end_deg

        if noise is not None:
            # Deterministic wobble - no RNG, so failures reproduce exactly.
            angle += noise * ((int(t * 100000) % 7) - 3) / 3.0

        series.append((t, angle))
        t += DT

    return series
# def


def test_recovers_a_known_rate_exactly():
    """60 deg at 300 deg/s: 10-90% covers 48 deg, so 160 ms, and 0.8*60/0.16=300."""
    series = ramp_series(-30.0, +30.0, rate_deg_s=300.0, dead_s=0.05)
    result = analyse_leg(series)

    assert result["ok"]
    assert result["travel_deg"] == pytest.approx(60.0, abs=0.05)
    assert result["transit_s"] == pytest.approx(0.160, abs=2 * DT)
    assert result["rate_deg_s"] == pytest.approx(300.0, rel=0.03)
# def


def test_latency_is_the_10_percent_crossing():
    """Dead time 50 ms plus 20 ms to reach 10% of 60 deg at 300 deg/s."""
    series = ramp_series(-30.0, +30.0, rate_deg_s=300.0, dead_s=0.05)
    result = analyse_leg(series)

    assert result["latency_s"] == pytest.approx(0.07, abs=2 * DT)
# def


def test_endpoints_are_recovered():
    series = ramp_series(-33.0, +35.0, rate_deg_s=250.0)
    result = analyse_leg(series)

    assert result["baseline_deg"] == pytest.approx(-33.0, abs=0.05)
    assert result["final_deg"] == pytest.approx(+35.0, abs=0.05)
# def


def test_reverse_direction_reports_negative_travel_but_positive_rate():
    """Direction is carried by travel's sign; a rate is a magnitude."""
    series = ramp_series(+30.0, -30.0, rate_deg_s=300.0)
    result = analyse_leg(series)

    assert result["ok"]
    assert result["travel_deg"] == pytest.approx(-60.0, abs=0.05)
    assert result["rate_deg_s"] == pytest.approx(300.0, rel=0.03)
    assert result["transit_s"] > 0
# def


@pytest.mark.parametrize("rate", [100.0, 250.0, 500.0, 1000.0])
def test_rate_recovery_across_the_plausible_range(rate):
    series = ramp_series(-30.0, +30.0, rate_deg_s=rate)
    result = analyse_leg(series)

    assert result["rate_deg_s"] == pytest.approx(rate, rel=0.05)
# def


def test_adc_noise_does_not_break_the_metrics():
    """Endpoints are medians precisely so a few noisy samples cannot set them."""
    series = ramp_series(-30.0, +30.0, rate_deg_s=300.0, noise=0.4)
    result = analyse_leg(series)

    assert result["ok"]
    assert result["travel_deg"] == pytest.approx(60.0, abs=0.6)
    assert result["rate_deg_s"] == pytest.approx(300.0, rel=0.10)
# def


def test_unpowered_servo_is_reported_as_no_movement():
    """The dry-run and servos-off case: must not divide by noise."""
    series = [(t * DT - 0.4, -30.0) for t in range(1200)]
    result = analyse_leg(series)

    assert not result["ok"]
    assert "no movement" in result["reason"]
# def


def test_movement_below_the_threshold_is_rejected():
    series = ramp_series(0.0, MIN_TRAVEL_DEG - 0.5, rate_deg_s=100.0)
    result = analyse_leg(series)

    assert not result["ok"]
# def


def test_too_short_a_capture_is_rejected():
    series = [(i * DT, float(i)) for i in range(5)]
    result = analyse_leg(series)

    assert not result["ok"]
    assert "too few samples" in result["reason"]
# def


def test_crossing_ignores_the_pre_roll():
    """A pre-command sample must never satisfy a threshold and zero the latency."""
    series = [(-0.2, 100.0), (-0.1, 100.0)] + \
             [(i * DT, -30.0 + i * 0.12) for i in range(500)]

    t10 = crossing_time(series, baseline=-30.0, travel=60.0, fraction=0.10)

    assert t10 is not None
    assert t10 >= 0.0
# def


def test_signed_tick_delta_handles_wrap_both_ways():
    """Pre-roll samples sit before the anchor, so negatives must work too."""
    assert signed_tick_delta(5000, 3000) == 2000
    assert signed_tick_delta(3000, 5000) == -2000

    # Anchor just below the wrap, sample just after it.
    assert signed_tick_delta(1000, PICO_TICKS_MODULO - 1000) == 2000
    # And the reverse: sample before the wrap, anchor after.
    assert signed_tick_delta(PICO_TICKS_MODULO - 1000, 1000) == -2000
# def


def test_to_series_zeroes_on_the_command_and_uses_pico_time():
    """Host times are bursty; the trace shape must come from the Pico clock."""
    cal = {"LEFT": {"scaler": 1.0, "offset": 0.0},
           "RIGHT": {"scaler": 1.0, "offset": 0.0}}

    # Four samples 2 ms apart on the Pico, but delivered in one host-side burst.
    samples = [
        (10.000, 1000, 10, 20),
        (10.000, 3000, 11, 21),
        (10.010, 5000, 12, 22),   # first at/after the command
        (10.010, 7000, 13, 23),
    ]

    series = to_series(samples, t_cmd_host=10.005, cal=cal, side="LEFT")
    times = [round(t, 6) for t, _ in series]

    assert times == [-0.004, -0.002, 0.0, 0.002]
    assert [a for _, a in series] == [10.0, 11.0, 12.0, 13.0]
# def


def test_to_series_falls_back_to_host_time_without_stamps():
    """Legacy firmware: coarser, but it must still produce a usable trace."""
    cal = {"LEFT": {"scaler": 1.0, "offset": 0.0},
           "RIGHT": {"scaler": 1.0, "offset": 0.0}}

    samples = [(10.0, None, 10, 20), (10.1, None, 11, 21), (10.2, None, 12, 22)]
    series = to_series(samples, t_cmd_host=10.05, cal=cal, side="RIGHT")

    assert [round(t, 6) for t, _ in series] == [-0.1, 0.0, 0.1]
    # RIGHT is negated by convention: -(scaler * raw) + offset
    assert [a for _, a in series] == [-20.0, -21.0, -22.0]
# def


def test_right_side_uses_the_right_channel():
    cal = {"LEFT": {"scaler": 1.0, "offset": 0.0},
           "RIGHT": {"scaler": 2.0, "offset": 5.0}}

    samples = [(1.0, 0, 100, 200), (1.0, 2000, 101, 201)]

    left = to_series(samples, 1.0, cal, "LEFT")
    right = to_series(samples, 1.0, cal, "RIGHT")

    assert [a for _, a in left] == [100.0, 101.0]
    # RIGHT is negated by convention: -(scaler * raw) + offset
    assert [a for _, a in right] == [-395.0, -397.0]
# def


def test_mean_sd():
    assert mean_sd([]) == (None, None)
    assert mean_sd([4.0]) == (4.0, None)

    mean, sd = mean_sd([1.0, 2.0, 3.0])
    assert mean == pytest.approx(2.0)
    assert sd == pytest.approx(1.0)
# def
