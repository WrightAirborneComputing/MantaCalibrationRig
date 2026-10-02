"""The angle maths in inclinometer_sweep.py. No hardware."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import inclinometer_sweep as sweep


def up_vector(hinge_deg, roll_deg=0.0):
    """Gravity as a module sees it, rotated about Y by hinge_deg."""
    h = math.radians(hinge_deg)
    r = math.radians(roll_deg)
    return (math.cos(h) * math.cos(r), math.sin(r), math.sin(h) * math.cos(r))


def test_hinge_angle_sign_follows_px4_positive():
    assert abs(sweep.hinge_angle(up_vector(+30.0)) - 30.0) < 1e-9
    assert abs(sweep.hinge_angle(up_vector(-45.0)) + 45.0) < 1e-9


def test_half_turn_about_z_restores_the_upright_angle():
    upright = up_vector(-5.9)
    mounted = (-upright[0], -upright[1], upright[2])
    assert abs(sweep.hinge_angle(sweep.half_turn_z(mounted)) + 5.9) < 1e-9


def test_out_of_plane_is_not_inflated_by_the_hinge_angle():
    # atan2(y, x) would read 2 deg here; the tilt is 1.
    assert abs(sweep.out_of_plane(up_vector(60.0, roll_deg=1.0)) - 1.0) < 1e-6


def test_hold_cancels_airframe_movement_sample_by_sample():
    frames = []
    for body in (-2.0, 0.0, 3.0):
        frames.append({"CENTRE": up_vector(body),
                       "LEFT": up_vector(body + 24.5),
                       "RIGHT": up_vector(body - 38.5)})
    result = sweep.summarise_hold(frames)
    for name, expected in (("LEFT", 24.5), ("RIGHT", -38.5)):
        mean, sd, lo, hi = result[name]["relative"]
        assert abs(mean - expected) < 1e-9
        assert hi - lo < 1e-9
    assert result["CENTRE"][3] - result["CENTRE"][2] > 4.9


def test_hold_skips_frames_missing_a_channel():
    frames = [{"CENTRE": up_vector(0.0), "LEFT": up_vector(10.0)}]
    assert sweep.summarise_hold(frames) is None
