"""Host-side tests for the end stop decision logic. No hardware, no imports."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import endpoint_logic


def test_endpoint_command_follows_the_reversal():
    assert endpoint_logic.endpoint_command("MAX", rev=False) == 1.0
    assert endpoint_logic.endpoint_command("MIN", rev=False) == -1.0
    assert endpoint_logic.endpoint_command("MAX", rev=True) == -1.0
    assert endpoint_logic.endpoint_command("MIN", rev=True) == 1.0
    with pytest.raises(ValueError):
        endpoint_logic.endpoint_command("MIDDLE", rev=False)


def test_backing_off_moves_the_pwm_into_the_range():
    """Inward is down from MAX and up from MIN, whichever way the channel runs."""
    assert endpoint_logic.inward_sign("MAX") == -1.0
    assert endpoint_logic.inward_sign("MIN") == 1.0

    # 10 us inward on an 810 us span, not reversed: PWM falls, so does the command.
    delta = endpoint_logic.command_delta_for_pwm(-10.0, 1116, 1926, rev=False)
    assert delta == pytest.approx(-0.0247, abs=1e-4)
    # Reversed, the same PWM change needs the opposite command change.
    assert endpoint_logic.command_delta_for_pwm(-10.0, 1116, 1926, rev=True) == -delta


def test_hard_stop_verdict_at_the_50_us_boundary():
    assert endpoint_logic.hard_stop_verdict(20.0) == (False, 0.0)
    assert endpoint_logic.hard_stop_verdict(50.0) == (False, 0.0)

    # Past 50 us it is jammed: pull in by the breakaway distance plus 10 us.
    is_hard, pull = endpoint_logic.hard_stop_verdict(60.0)
    assert is_hard and pull == 70.0

    # Never broke away at all - retreat by the whole search plus the margin.
    is_hard, pull = endpoint_logic.hard_stop_verdict(None)
    assert is_hard and pull == endpoint_logic.BACKOFF_CEILING_US + 10.0


def test_gain_sign_falls_out_of_the_probe():
    """On this linkage the angle rises as the PWM falls: the slope is negative.

    Backing off MAX lowers the PWM by 20 us and the elevon comes up 1.0 deg,
    so the slope is -0.05 deg/us. Backing off MIN raises the PWM instead, and
    the same physical linkage must yield the same sign.
    """
    assert endpoint_logic.angle_gain_per_us(-36.0, -35.0, 20.0, "MAX") == pytest.approx(-0.05)
    assert endpoint_logic.angle_gain_per_us(37.0, 35.9, 20.0, "MIN") == pytest.approx(-0.055)

    assert endpoint_logic.angle_gain_per_us(-36.0, -36.0, 20.0, "MAX") is None
    assert endpoint_logic.angle_gain_per_us(-36.0, -35.0, None, "MAX") is None


def test_correction_moves_the_endpoint_toward_the_target():
    """Overshooting the target pulls the endpoint in, which shrinks the span."""
    # 3 deg past a -33 target at -0.05 deg/us: the endpoint comes in 60 us.
    assert endpoint_logic.correction_us(-36.0, -33.0, -0.05) == pytest.approx(-60.0)
    # Falling short of it pushes the endpoint the other way.
    assert endpoint_logic.correction_us(-30.0, -33.0, -0.05) == pytest.approx(60.0)
    assert endpoint_logic.correction_us(-36.0, -33.0, None) is None


def test_acceptance_needs_both_conditions():
    assert endpoint_logic.endpoint_accepted(-33.4, -33.0, is_hard_stop=False)
    assert endpoint_logic.endpoint_accepted(-32.5, -33.0, is_hard_stop=False)
    # Within tolerance but jammed is not accepted.
    assert not endpoint_logic.endpoint_accepted(-33.4, -33.0, is_hard_stop=True)
    # Free but out of tolerance is not accepted.
    assert not endpoint_logic.endpoint_accepted(-34.0, -33.0, is_hard_stop=False)


def test_alternation_is_max_min_max_min():
    assert endpoint_logic.alternating_order(3) == ["MAX", "MIN"] * 3
    assert endpoint_logic.alternating_order(0) == []


def test_endpoints_are_clamped_to_a_sane_band():
    assert endpoint_logic.clamp_endpoint(1500.4) == 1500
    assert endpoint_logic.clamp_endpoint(50) == endpoint_logic.PWM_FLOOR
    assert endpoint_logic.clamp_endpoint(9000) == endpoint_logic.PWM_CEILING


def test_a_left_endpoint_converges_in_one_correction():
    """The worked example: LEFT MAX reads -36 against a -33 target.

    Backing off 20 us brought it up to -35, so the slope is -0.05 deg/us and
    the endpoint needs to come in 60 us - from 1926 to 1866, a tighter span,
    which is the behaviour the two-stage approach is supposed to produce.
    """
    angle, target, pwm = -36.0, -33.0, 1926
    breakaway, angle_backed = 20.0, -35.0

    is_hard, _ = endpoint_logic.hard_stop_verdict(breakaway)
    assert not is_hard
    assert not endpoint_logic.endpoint_accepted(angle, target, is_hard)

    gain = endpoint_logic.angle_gain_per_us(angle, angle_backed, breakaway, "MAX")
    shift = endpoint_logic.correction_us(angle, target, gain)
    # 1926 -> 1866: the endpoint comes inward and the span tightens.
    assert endpoint_logic.clamp_endpoint(pwm + shift) == 1866
    assert shift < 0

    # And the corrected endpoint would now be inside tolerance.
    assert endpoint_logic.endpoint_accepted(angle + shift * gain, target, False)
