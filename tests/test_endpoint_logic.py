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


def test_overshoot_is_measured_against_the_far_end_not_the_sign():
    """Which way is "past" comes from the two targets, so it holds for either
    wiring and for asymmetric travel."""
    # MAX at +30, MIN at -30: past means above +30.
    assert endpoint_logic.overshot_target(47.0, 30.0, -30.0)
    assert not endpoint_logic.overshot_target(25.0, 30.0, -30.0)
    assert not endpoint_logic.overshot_target(30.3, 30.0, -30.0), "inside tolerance"

    # MIN at -30: past means below -30.
    assert endpoint_logic.overshot_target(-47.0, -30.0, 30.0)
    assert not endpoint_logic.overshot_target(-25.0, -30.0, 30.0)

    # Reversed wiring puts the negative angle at the MAX end. Still works.
    assert endpoint_logic.overshot_target(-40.0, -33.0, 35.0)
    assert not endpoint_logic.overshot_target(-30.0, -33.0, 35.0)


def test_asymmetric_targets_do_not_confuse_the_direction():
    """-33/+35 is not symmetric, and neither is a trimmed airframe."""
    assert endpoint_logic.travel_direction(35.0, -33.0) == 1.0
    assert endpoint_logic.travel_direction(-33.0, 35.0) == -1.0


def test_nominal_gain_takes_its_sign_from_the_travel():
    """Reversed channels run the other way, and the gain has to say so."""
    plain = endpoint_logic.nominal_gain_deg_per_us(35.0, -33.0, 1116, 1926)
    assert plain == pytest.approx(68.0 / 810.0)

    reversed_channel = endpoint_logic.nominal_gain_deg_per_us(-33.0, 35.0, 1116, 1926)
    assert reversed_channel == pytest.approx(-68.0 / 810.0)

    assert endpoint_logic.nominal_gain_deg_per_us(35.0, -33.0, 1500, 1500) is None


def test_a_large_overshoot_corrects_toward_the_target():
    """The 17 deg case: a rough gain is enough to get near, and the attempt
    that follows lands in range and measures the local gain properly."""
    gain = endpoint_logic.nominal_gain_deg_per_us(30.0, -30.0, 1000, 2000)
    shift = endpoint_logic.correction_us(47.0, 30.0, gain)

    assert shift < 0, "pulled inward"
    assert endpoint_logic.clamp_endpoint(2000 + shift) == pytest.approx(1717, abs=1)


def test_a_probed_gain_that_disagrees_with_the_geometry_is_dropped():
    """The RIGHT run's failure, in numbers.

    MIN read -31.18 against a -33.00 target on a 1213-1893 span, so the gain
    must be about +0.10 deg/us. The probe returned -0.087: right magnitude,
    wrong sign, because the surface was still creeping back from the overshoot
    when the 10 us step was measured. Correcting on it pushed MIN the wrong way
    three attempts running.
    """
    nominal = endpoint_logic.nominal_gain_deg_per_us(35.0, -33.0, 1213, 1893)
    assert nominal == pytest.approx(0.1, abs=0.005)

    gain, source = endpoint_logic.usable_gain(-0.087, nominal)
    assert gain == nominal
    assert "direction" in source

    # And now the correction goes the way it should: MIN comes down, not up.
    shift = endpoint_logic.correction_us(-31.18, -33.0, gain)
    assert shift < 0
    assert endpoint_logic.clamp_endpoint(1213 + shift, "MIN", 1893, 1213) < 1213


def test_a_plausible_probed_gain_is_kept():
    """The probe exists because the local gain is not the average one - it was
    measured to vary about 2:1 end to end, and that has to survive the check."""
    nominal = endpoint_logic.nominal_gain_deg_per_us(35.0, -33.0, 1213, 1893)

    for probed in (nominal * 0.5, nominal, nominal * 2.0):
        gain, source = endpoint_logic.usable_gain(probed, nominal)
        assert gain == probed
        assert source == "probe"


def test_an_implausible_magnitude_is_dropped_even_with_the_right_sign():
    nominal = endpoint_logic.nominal_gain_deg_per_us(35.0, -33.0, 1213, 1893)

    gain, source = endpoint_logic.usable_gain(nominal * 10.0, nominal)
    assert gain == nominal and "10.0x" in source

    gain, source = endpoint_logic.usable_gain(nominal * 0.05, nominal)
    assert gain == nominal and "nominal" in source


def test_a_missing_gain_falls_back_to_whichever_one_exists():
    assert endpoint_logic.usable_gain(None, 0.1) == (0.1, "nominal")
    assert endpoint_logic.usable_gain(0.1, None) == (0.1, "probe")
    assert endpoint_logic.usable_gain(None, None) == (None, "none")


def test_one_attempt_cannot_move_an_endpoint_more_than_the_cap():
    limit = endpoint_logic.MAX_CORRECTION_US

    assert endpoint_logic.limit_correction(5154.0) == limit
    assert endpoint_logic.limit_correction(-1348.0) == -limit
    assert endpoint_logic.limit_correction(-60.0) == -60.0
    assert endpoint_logic.limit_correction(None) is None


def test_the_end_stops_cannot_cross():
    """PX4 swaps MIN and MAX at param load, so a crossed pair does not fail -
    it calibrates on against a range nobody chose. MIN=2200 against MAX=1893 is
    what the RIGHT run wrote, and every reading after it was meaningless."""
    span = endpoint_logic.MIN_ENDPOINT_SPAN_US

    assert endpoint_logic.clamp_endpoint(2200, "MIN", 1893) == 1893 - span
    assert endpoint_logic.clamp_endpoint(1100, "MAX", 1500) == 1500 + span

    # Well clear of the other end, nothing is imposed.
    assert endpoint_logic.clamp_endpoint(1213, "MIN", 1893) == 1213


def test_an_endpoint_stays_near_where_the_coarse_creep_found_it():
    drift = endpoint_logic.ENDPOINT_DRIFT_LIMIT_US

    assert endpoint_logic.clamp_endpoint(1674, "MIN", 1893, 1192) == int(1192 + drift)
    assert endpoint_logic.clamp_endpoint(1210, "MIN", 1893, 1192) == 1210

    # Below the anchor too, where the servo band is not already the tighter one.
    assert endpoint_logic.clamp_endpoint(1050, "MIN", 1893, 1400) == int(1400 - drift)


def test_the_servo_band_is_the_outer_bound():
    """800 us is a PX4 limit, not a servo one. Writing it drove the elevon so
    far past its stop that the flight controller browned out."""
    assert endpoint_logic.PWM_FLOOR == 1000
    assert endpoint_logic.PWM_CEILING == 2000
    assert endpoint_logic.clamp_endpoint(800, "MIN", 1893, 900) == 1000
    assert endpoint_logic.clamp_endpoint(2200, "MAX", 1000, 2100) == 2000


def test_a_growing_error_is_only_called_divergence_after_two_of_them():
    """One growth can be the hold noise on a surface that is nearly there."""
    assert not endpoint_logic.error_grew(None, 1.82)
    assert not endpoint_logic.error_grew(1.82, -1.9), "inside the noise floor"
    assert endpoint_logic.error_grew(1.82, 3.58)
    assert not endpoint_logic.error_grew(3.58, 1.82)

    assert not endpoint_logic.has_diverged(1)
    assert endpoint_logic.has_diverged(2)


def test_the_right_run_would_have_stopped_two_attempts_in():
    """Replaying the errors from the log: 1.82, 3.58, 15.82, 44.65."""
    growths, stopped_at = 0, None
    previous = None

    for attempt, error in enumerate((1.82, 3.58, 15.82, 44.65), start=1):
        growths = growths + 1 if endpoint_logic.error_grew(previous, error) else 0
        previous = error
        if endpoint_logic.has_diverged(growths):
            stopped_at = attempt
            break

    assert stopped_at == 3, "stopped while MIN was still 15 deg out, not 44"
