"""Tests for the creep curve plot geometry. No display, no hardware."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import curve_plot as CP
import range_test as RT


def make_series(band=1.0, points=11, curvature=0.0):
    """Two directions separated by a known band, on a shared PWM grid."""
    up, down = [], []
    for index in range(points):
        pwm = 1200 + (index * 60)
        angle = -30.0 + (index * 6.0) + curvature * ((index - points / 2.0) ** 2)
        up.append((pwm, angle - band / 2.0, 0.05, 3))
        down.append((pwm, angle + band / 2.0, 0.05, 3))
    return {1.0: up, -1.0: down}


def test_plotbox_puts_higher_angles_higher_up():
    box = CP.PlotBox((1000, 2000), (-40, 40), 800, 500)
    assert box.y(40) < box.y(-40), "y grows downward in pixels"
    assert box.x(1000) < box.x(2000)
    assert box.data_y(box.y(12.5)) == pytest.approx(12.5)


def test_nothing_to_plot_returns_nothing():
    """An empty frame would read as a measurement of zero."""
    assert CP.build_plot({}) is None
    assert CP.build_plot({1.0: []}) is None
    assert CP.to_svg(None) is None


def test_deviation_mode_preserves_the_band():
    """The point of a common reference fit.

    Referencing each direction to its own fit would let each absorb its own
    offset, and the separation being measured would vanish.
    """
    plot = CP.build_plot(make_series(band=2.0), mode=CP.MODE_DEVIATION)
    box = plot["box"]

    ups = [box.data_y(y) for role, (x, y) in
           ((m["role"], m["point"]) for m in plot["markers"])
           if role == CP.ROLE_UP]
    downs = [box.data_y(y) for role, (x, y) in
             ((m["role"], m["point"]) for m in plot["markers"])
             if role == CP.ROLE_DOWN]

    assert len(ups) == len(downs)
    for up, down in zip(ups, downs):
        assert down - up == pytest.approx(2.0, abs=0.05)

    # And it straddles zero, so neither direction is privileged.
    assert sum(ups) / len(ups) == pytest.approx(-1.0, abs=0.05)
    assert sum(downs) / len(downs) == pytest.approx(1.0, abs=0.05)


def test_curve_mode_keeps_the_measured_angles():
    plot = CP.build_plot(make_series(band=2.0), mode=CP.MODE_CURVE)
    box = plot["box"]

    values = [box.data_y(m["point"][1]) for m in plot["markers"]]
    assert min(values) < -25.0 and max(values) > 25.0


def test_tramlines_are_flat_rails_about_zero_in_deviation_mode():
    plot = CP.build_plot(make_series(band=1.0), tolerance_deg=0.5,
                         mode=CP.MODE_DEVIATION)
    box = plot["box"]

    trams = [line for line in plot["polylines"] if line["role"] == CP.ROLE_TRAM]
    assert len(trams) == 2

    levels = sorted(round(box.data_y(line["points"][0][1]), 3) for line in trams)
    assert levels == pytest.approx([-0.5, 0.5], abs=0.01)
    for line in trams:
        assert line["points"][0][1] == pytest.approx(line["points"][-1][1])


def test_tramlines_stay_in_frame_when_nothing_reaches_them():
    """An in-spec result must look in-spec, not merely off the top."""
    plot = CP.build_plot(make_series(band=0.02), tolerance_deg=1.0,
                         mode=CP.MODE_DEVIATION)
    assert plot["box"].y_max >= 1.0
    assert plot["box"].y_min <= -1.0


def test_stats_report_the_band_and_the_fit_quality():
    plot = CP.build_plot(make_series(band=1.5, curvature=0.4), order=2)
    stats = plot["stats"]

    assert stats["band"]["mean_deg"] == pytest.approx(1.5, abs=0.05)
    # A quadratic through quadratic data leaves nothing behind.
    for direction in (1.0, -1.0):
        assert stats["fits"][direction]["rms_deg"] < 1e-6


def test_a_localised_defect_survives_the_fit():
    """The whole reason for plotting residuals rather than the curve."""
    series = make_series(band=1.0, points=11)
    bumped = list(series[1.0])
    pwm, angle, sd, n = bumped[5]
    bumped[5] = (pwm, angle + 3.0, sd, n)
    series[1.0] = bumped

    stats = CP.build_plot(series, order=2)["stats"]
    assert abs(stats["fits"][1.0]["max_deg"]) > 2.0
    assert stats["fits"][1.0]["max_pwm"] == pwm


def test_svg_is_well_formed_and_self_contained():
    svg = CP.to_svg(CP.build_plot(make_series()), "LEFT")

    assert svg.startswith("<?xml")
    assert svg.rstrip().endswith("</svg>")
    assert svg.count("<svg") == 1
    assert "polyline" in svg and "LEFT" in svg
    # No external references: it has to open anywhere, offline.
    assert "http://www.w3.org/2000/svg" in svg
    assert ".css" not in svg and "<image" not in svg


def test_svg_escapes_a_hostile_title():
    svg = CP.to_svg(CP.build_plot(make_series()), '<script>&"')
    assert "<script>" not in svg
    assert "&lt;script&gt;" in svg


def test_ticks_are_round_numbers_inside_the_range():
    ticks = CP.nice_ticks(1244, 1866)
    assert ticks
    assert all(1244 <= t <= 1866 for t in ticks)
    assert all(float(t).is_integer() for t in ticks)
    assert CP.nice_ticks(5, 5) == []


def test_plot_is_built_from_the_shared_analysis(monkeypatch):
    """curve_series feeds the plot directly - no second implementation."""
    points = [
        {"side": "LEFT", "direction": 1.0, "rep": 1, "cmd": -0.5,
         "pwm_us": 1400, "angle_deg": -10.0},
        {"side": "LEFT", "direction": 1.0, "rep": 2, "cmd": -0.5,
         "pwm_us": 1400, "angle_deg": -10.2},
        {"side": "LEFT", "direction": -1.0, "rep": 1, "cmd": -0.5,
         "pwm_us": 1400, "angle_deg": -9.0},
        {"side": "RIGHT", "direction": 1.0, "rep": 1, "cmd": -0.5,
         "pwm_us": 1400, "angle_deg": 99.0},
    ]
    series = RT.curve_series(points, "LEFT")
    assert series[1.0][0][1] == pytest.approx(-10.1)
    assert 99.0 not in [row[1] for rows in series.values() for row in rows]
