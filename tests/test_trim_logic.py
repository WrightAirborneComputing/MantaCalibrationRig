"""Host-side tests for the trim point's decision logic. No hardware."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import trim_logic


def test_the_estimate_comes_from_the_two_accepted_end_stops():
    """No creep needed: the end stops are two known (command, angle) pairs."""
    # -33 at cmd -1, +35 at cmd +1. -5 deg sits a little below the middle.
    cmd = trim_logic.trim_estimate(-5.0, -33.0, 35.0)
    assert cmd == pytest.approx(2.0 * (28.0 / 68.0) - 1.0)
    assert -0.2 < cmd < -0.1

    # Dead centre of the travel is cmd 0 whatever the numbers are.
    assert trim_logic.trim_estimate(1.0, -33.0, 35.0) == pytest.approx(0.0)
    assert trim_logic.trim_estimate(-5.0, 10.0, 10.0) is None


def test_the_estimate_holds_on_a_reversed_channel():
    """Reversed, cmd -1 is the positive angle. The caller passes the angles by
    command, so nothing here has to know about the reversal."""
    plain = trim_logic.trim_estimate(-5.0, -33.0, 35.0)
    flipped = trim_logic.trim_estimate(-5.0, 35.0, -33.0)

    assert flipped == pytest.approx(-plain)


def test_acceptance_is_one_direction_only():
    """These servos have more lash than any two-sided band could close, so the
    trim point is the angle reached arriving from one named direction."""
    assert trim_logic.trim_accepted(-5.2, -5.0)
    assert trim_logic.trim_accepted(-4.75, -5.0)
    assert not trim_logic.trim_accepted(-5.3, -5.0)

    # The other direction being 3 deg away does not enter into it.
    assert trim_logic.trim_accepted(-5.1, -5.0)


def test_backlash_is_reported_not_corrected():
    """The number that says where the surface sits arriving the other way."""
    assert trim_logic.backlash_deg(-5.1, -2.4) == pytest.approx(2.7)
    assert trim_logic.backlash_deg(-5.1, -7.8) == pytest.approx(-2.7)
    assert trim_logic.backlash_deg(-5.0, -5.0) == 0.0


def test_the_correction_uses_the_set_direction_reading():
    gain = trim_logic.command_gain_deg(-33.0, 35.0)
    assert gain == pytest.approx(34.0)

    # 2 deg high against a 34 deg/cmd gain: the command comes down 0.059.
    assert trim_logic.trim_correction(-3.0, -5.0, gain) == pytest.approx(-2.0 / 34.0)
    assert trim_logic.trim_correction(-7.0, -5.0, gain) == pytest.approx(2.0 / 34.0)
    assert trim_logic.trim_correction(-3.0, -5.0, None) is None


def test_the_secant_measures_the_gain_where_the_trim_sits():
    """Both readings come from the same approach, so the lash cancels."""
    gain = trim_logic.secant_gain_deg(-0.15, -8.0, -0.05, -4.6)
    assert gain == pytest.approx(34.0)

    assert trim_logic.secant_gain_deg(None, None, -0.05, -4.6) is None
    assert trim_logic.secant_gain_deg(-0.05, -4.6, -0.05, -4.6) is None
    assert trim_logic.secant_gain_deg(-0.15, -4.6, -0.05, -4.6) is None


def test_a_secant_that_disagrees_with_the_geometry_is_dropped():
    """Same guard the end stops use: a surface still settling between two
    attempts returns the settling, not the gain."""
    nominal = trim_logic.command_gain_deg(-33.0, 35.0)

    # Wrong sign - correcting on it would walk away from the target.
    assert trim_logic.trim_gain_deg(-0.15, -8.0, -0.05, -11.4, nominal) == nominal
    # Ten times too steep.
    assert trim_logic.trim_gain_deg(-0.15, -8.0, -0.05, 26.0, nominal) == nominal
    # Plausible, so it is kept.
    assert trim_logic.trim_gain_deg(-0.15, -8.0, -0.05, -4.6,
                                    nominal) == pytest.approx(34.0)
    # Nothing to draw a line through yet.
    assert trim_logic.trim_gain_deg(None, None, -0.15, -8.0, nominal) == nominal


def test_one_attempt_cannot_move_the_trim_more_than_the_cap():
    limit = trim_logic.TRIM_MAX_CORRECTION_CMD

    assert trim_logic.limit_trim_correction(0.9) == limit
    assert trim_logic.limit_trim_correction(-0.9) == -limit
    assert trim_logic.limit_trim_correction(-0.06) == pytest.approx(-0.06)
    assert trim_logic.limit_trim_correction(None) is None


def test_the_trim_command_stays_inside_its_limit():
    assert trim_logic.clamp_trim(-2.0) == -trim_logic.TRIM_COMMAND_LIMIT
    assert trim_logic.clamp_trim(2.0) == trim_logic.TRIM_COMMAND_LIMIT
    assert trim_logic.clamp_trim(-0.15) == pytest.approx(-0.15)


def test_a_growing_error_is_only_divergence_after_two_of_them():
    assert not trim_logic.trim_error_grew(None, 2.0)
    assert not trim_logic.trim_error_grew(2.0, -2.05), "inside the margin"
    assert trim_logic.trim_error_grew(2.0, 3.0)
    assert not trim_logic.trim_error_grew(3.0, 2.0)

    assert not trim_logic.trim_has_diverged(1)
    assert trim_logic.trim_has_diverged(2)


def test_what_the_trim_costs_the_travel():
    """PX4 computes clamp(cmd + trim, -1, +1), and the arithmetic is exact:
    a -0.29 trim gives up 117 us of an 810 us span."""
    assert trim_logic.travel_lost_us(-0.29, 810) == pytest.approx(117.45)
    assert trim_logic.travel_lost_us(0.29, 810) == pytest.approx(117.45)
    assert trim_logic.travel_lost_us(0.0, 810) == 0.0

    # And the other end is reached early, deaf over the last |trim| of command.
    assert trim_logic.dead_band_cmd(-0.29) == pytest.approx(0.29)


def test_which_end_the_trim_shortens_follows_the_command_not_the_pwm():
    """A negative trim shortens whichever end sits at cmd +1 - the MIN end when
    the channel is reversed."""
    assert trim_logic.shortened_endpoint(-0.29, rev=False) == "MAX"
    assert trim_logic.shortened_endpoint(0.29, rev=False) == "MIN"
    assert trim_logic.shortened_endpoint(-0.29, rev=True) == "MIN"
    assert trim_logic.shortened_endpoint(0.29, rev=True) == "MAX"
    assert trim_logic.shortened_endpoint(0.0, rev=False) is None
