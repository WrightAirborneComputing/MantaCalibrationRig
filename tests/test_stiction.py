"""Tests for the creep-versus-swing stiction analysis. No hardware, no display."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import range_test as RT


def test_creep_walks_to_the_target_and_lands_on_it_exactly():
    assert RT.creep_commands(0.0, 1.0, 0.25) == [0.25, 0.5, 0.75, 1.0]
    assert RT.creep_commands(0.0, -1.0, 0.5) == [-0.5, -1.0]


def test_creep_final_step_is_the_target_even_when_it_does_not_divide():
    """A creep that stopped short would measure a different PWM than the swing."""
    commands = RT.creep_commands(0.0, 1.0, 0.3)
    assert commands == [0.3, 0.6, 0.9, 1.0]
    assert commands[-1] == 1.0

    commands = RT.creep_commands(-1.0, 1.0, 0.75)
    assert commands[-1] == 1.0


def test_creep_from_an_offset_start_excludes_the_start():
    assert RT.creep_commands(0.5, 1.0, 0.25) == [0.75, 1.0]
    assert RT.creep_commands(-1.0, -1.0, 0.1) == [-1.0]


def test_creep_step_must_be_positive():
    with pytest.raises(ValueError):
        RT.creep_commands(0.0, 1.0, 0.0)


def test_the_number_of_creep_steps_sets_the_wall_clock():
    """100 steps from centre at the trim calibration's own 0.01 - the reason
    the repetition default is low."""
    assert len(RT.creep_commands(0.0, 1.0, 0.01)) == 100


def test_settled_angle_uses_the_last_fifth():
    # Ramp then hold: the settled value is the hold, not the mean of the ramp.
    values = list(range(0, 50)) + [100.0] * 50
    assert RT.settled_angle(values) == 100.0

    # Never fewer than 5 samples, and a lone outlier cannot define it.
    assert RT.settled_angle([1.0, 2.0, 3.0, 4.0, 99.0]) == 3.0
    assert RT.settled_angle([]) is None


def test_stiction_is_the_difference_of_the_means():
    stats = RT.stiction_stats([-33.5, -33.6, -33.4], [-37.7, -37.6, -37.8])

    assert stats["creep_mean"] == pytest.approx(-33.5)
    assert stats["swing_mean"] == pytest.approx(-37.7)
    assert stats["stiction"] == pytest.approx(-4.2)
    assert stats["creep_n"] == 3 and stats["swing_n"] == 3


def test_stiction_spread_adds_the_deviations_in_quadrature():
    """The scatter of a single paired comparison, not of the mean."""
    stats = RT.stiction_stats([0.0, 3.0, 6.0], [10.0, 14.0, 18.0])

    assert stats["creep_sd"] == pytest.approx(3.0)
    assert stats["swing_sd"] == pytest.approx(4.0)
    assert stats["stiction_sd"] == pytest.approx(5.0)
    # Standard error of the difference of the means is smaller by sqrt(n).
    assert stats["stiction_se"] == pytest.approx(5.0 / math.sqrt(3.0))


def test_stiction_sign_says_which_way_the_swing_overshot():
    """Positive end: the swing lands further positive, so the difference is +."""
    assert RT.stiction_stats([29.7, 29.7], [33.0, 33.0])["stiction"] == pytest.approx(3.3)
    assert RT.stiction_stats([-29.7, -29.7], [-33.0, -33.0])["stiction"] == pytest.approx(-3.3)


def test_a_single_repetition_leaves_the_spread_undefined():
    stats = RT.stiction_stats([-33.5], [-37.7])
    assert stats["stiction"] == pytest.approx(-4.2)
    assert stats["stiction_sd"] is None
    assert stats["stiction_se"] is None


def test_missing_measurements_do_not_invent_a_number():
    stats = RT.stiction_stats([], [-37.7, -37.6])
    assert stats["stiction"] is None
    assert stats["swing_mean"] is not None
