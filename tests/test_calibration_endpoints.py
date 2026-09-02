"""Tests for the auto calibration's PX4-facing decisions.

Two things the routine has to get right, both verified against PX4 v1.16
(`src/lib/mixer_module/mixer_module.cpp`):

  * PX4 has no inverted-range concept - `MixingOutput` swaps MIN and MAX at param
    load if MIN > MAX, and direction comes solely from PWM_MAIN_REV. So the
    endpoints must be written in numeric order for either wiring.
  * `Commander::handleCommandActuatorTest` refuses the command when armed, when
    the safety switch is on, or when COM_MOT_TEST_EN != 1. Without reading the
    ACK those look identical to a mechanically stuck surface.

Neither needs a window or a flight controller, so the GUI methods are called
against a stub rather than a real FourSliderGUI.

    python3 -m pytest tests/test_calibration_endpoints.py -v
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import MantaTrimmer as MT


class StubGUI:
    """Just enough of FourSliderGUI to exercise the calibration helpers."""

    def __init__(self, main_rev=0):
        self.main_rev = main_rev
        self.writes = []
        self.drone_interface = MT.DroneInterface()

    def set_side_param_and_refresh(self, side, param_name, py_type, value):
        self.writes.append((param_name, py_type(value)))
        return True
    # def

    is_main_channel_reversed = MT.FourSliderGUI.__dict__["is_main_channel_reversed"]
    load_endpoint_params = MT.FourSliderGUI.__dict__["load_endpoint_params"]
    actuator_test_rejected = MT.FourSliderGUI.__dict__["actuator_test_rejected"]
# class


def _load(gui, side, pwm_neg, pwm_pos):
    ok = gui.load_endpoint_params(side, "PWM_MAIN_MIN5", "PWM_MAIN_MAX5", pwm_neg, pwm_pos)
    return ok, dict(gui.writes)
# def


def test_endpoints_written_in_numeric_order_when_not_reversed():
    gui = StubGUI(main_rev=0)
    ok, written = _load(gui, "LEFT", pwm_neg=1260, pwm_pos=1770)

    assert ok
    assert written["PWM_MAIN_MIN5"] == 1260
    assert written["PWM_MAIN_MAX5"] == 1770
# def


def test_endpoints_written_in_numeric_order_when_reversed():
    """MAIN5 reversed: the positive command lands on the lower PWM.

    The old code hardcoded MIN=pwm_pos per side, which only held for this wiring
    and was silently swapped back by PX4 for the other.
    """
    gui = StubGUI(main_rev=(1 << 4))
    ok, written = _load(gui, "LEFT", pwm_neg=1770, pwm_pos=1260)

    assert ok
    assert written["PWM_MAIN_MIN5"] == 1260
    assert written["PWM_MAIN_MAX5"] == 1770
# def


def test_endpoint_order_disagreeing_with_rev_bit_warns_but_still_writes(capsys):
    """rev says the positive end should be lower, the rig measured it higher."""
    gui = StubGUI(main_rev=(1 << 4))
    ok, written = _load(gui, "LEFT", pwm_neg=1260, pwm_pos=1770)

    assert ok
    assert "WARNING" in capsys.readouterr().out
    assert written["PWM_MAIN_MIN5"] == 1260
    assert written["PWM_MAIN_MAX5"] == 1770
# def


def test_unconvertible_endpoint_aborts_without_writing():
    gui = StubGUI()
    ok, written = _load(gui, "LEFT", pwm_neg=None, pwm_pos=1770)

    assert not ok
    assert written == {}
# def


@pytest.mark.parametrize("result", [
    MT.DroneInterface.MAV_RESULT_ACCEPTED,
    MT.DroneInterface.ACK_NOT_RECEIVED,
    None,
])
def test_move_continues_when_px4_did_not_refuse(result):
    assert StubGUI().actuator_test_rejected("LEFT", result) is False
# def


@pytest.mark.parametrize("result", [1, 2, 3, 4])
def test_move_aborts_when_px4_refuses(result, capsys):
    gui = StubGUI()

    assert gui.actuator_test_rejected("LEFT", result) is True
    assert "COM_MOT_TEST_EN" in capsys.readouterr().out
# def


def test_refusal_names_the_mav_result():
    gui = StubGUI()
    gui.actuator_test_rejected("LEFT", 2)

    assert gui.drone_interface.describe_mav_result(2) == "DENIED"
    assert gui.drone_interface.describe_mav_result(MT.DroneInterface.ACK_NOT_RECEIVED) == "no ACK"
    assert gui.drone_interface.describe_mav_result(None) == "not sent"
# def


def test_no_actuate_reports_accepted_so_the_escape_hatch_still_calibrates():
    """MANTA_NO_ACTUATE must not read as a refusal, or every move would abort."""
    os.environ["MANTA_NO_ACTUATE"] = "1"
    drone = MT.DroneInterface()
    drone.master = object()   # is_connected() only checks for a link

    try:
        result = drone.command_elevon(1201, 0.0, wait_ack=True)
    finally:
        del os.environ["MANTA_NO_ACTUATE"]

    assert result == MT.DroneInterface.MAV_RESULT_ACCEPTED
    assert StubGUI().actuator_test_rejected("LEFT", result) is False
# def
