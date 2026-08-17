"""Host-side tests for the servo_probe bolt-on. No hardware."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import servo_probe


def test_expected_pwm_matches_the_trimmer_at_zero_trim():
    assert servo_probe.expected_pwm(-1.0, 1000, 2000, 0.0, False) == 1000
    assert servo_probe.expected_pwm(0.0, 1000, 2000, 0.0, False) == 1500
    assert servo_probe.expected_pwm(1.0, 1000, 2000, 0.0, False) == 2000


def test_reversed_channel_mirrors_the_travel():
    assert servo_probe.expected_pwm(-1.0, 1000, 2000, 0.0, True) == 2000
    assert servo_probe.expected_pwm(1.0, 1000, 2000, 0.0, True) == 1000


def test_negative_trim_eats_the_positive_end_and_clamps_the_negative():
    """The reported symptom, in arithmetic.

    A -0.147 trim (-5 deg over a 68 deg span) costs 7.4% of the full span at
    the positive end while the negative end saturates unchanged.
    """
    assert servo_probe.expected_pwm(-1.0, 1000, 2000, -0.147, False) == 1000
    assert servo_probe.expected_pwm(1.0, 1000, 2000, -0.147, False) == 1926
    assert servo_probe.expected_pwm_no_trim(1.0, 1000, 2000, False) == 2000


def test_parse_commands_clamps_and_ignores_blanks():
    assert servo_probe.parse_commands("-2, -0.5, ,0,1,3") == [-1.0, -0.5, 0.0, 1.0, 1.0]
    with pytest.raises(ValueError):
        servo_probe.parse_commands(" , ")


def test_side_table_matches_mantatrimmer():
    assert servo_probe.SIDES["LEFT"]["function"] == 1201
    assert servo_probe.SIDES["LEFT"]["trim_param"] == "CA_SV_CS0_TRIM"
    assert servo_probe.SIDES["RIGHT"]["function"] == 1202
    assert servo_probe.SIDES["RIGHT"]["max_param"] == "PWM_MAIN_MAX6"


def test_format_row_tolerates_missing_readback():
    row = {
        "side": "LEFT", "cmd": 1.0, "expected_pwm": 1927, "actual_pwm": None,
        "error_pwm": None, "expected_pwm_no_trim": 2000,
        "error_pwm_no_trim": None, "angle_deg": None,
    }
    text = servo_probe.format_row(row)
    assert "LEFT" in text and "--" in text


def test_csv_columns_cover_every_row_key():
    row = {
        "side": "LEFT", "cmd": 0.0, "expected_pwm": 1500, "actual_pwm": 1500,
        "error_pwm": 0, "expected_pwm_no_trim": 1500, "error_pwm_no_trim": 0,
        "servo_output_raw": 0, "angle_deg": -0.2,
    }
    assert set(row) == set(servo_probe.CSV_COLUMNS)


LISTENER_TWO_INSTANCES = """listener actuator_outputs -n 1

TOPIC: actuator_outputs 2 instances

Instance 0:
 actuator_outputs
    timestamp: 7009131511 (0.000297 seconds ago)
    noutputs: 9
    output: [0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000]


Instance 1:
 actuator_outputs
    timestamp: 7009154355 (0.003213 seconds ago)
    noutputs: 9
    output: [0.00000, 0.00000, 0.00000, 0.00000, 1926.00000, 1150.00000, 0.00000, 0.00000]

nsh> """


def test_parse_listener_outputs_separates_the_instances():
    """Instance 0 reads zero on this board; instance 1 has the real PWM."""
    instances = servo_probe.parse_listener_outputs(LISTENER_TWO_INSTANCES)
    assert set(instances) == {0, 1}
    assert instances[0][4] == 0.0
    assert instances[1][4] == 1926.0
    assert instances[1][5] == 1150.0


def test_single_instance_listing_falls_back_to_zero():
    text = "TOPIC: actuator_outputs\n    output: [1500.00000, 1600.00000]\n"
    assert servo_probe.parse_listener_outputs(text) == {0: [1500.0, 1600.0]}


def test_parse_listener_outputs_on_junk_is_empty():
    assert servo_probe.parse_listener_outputs("nsh> command not found\n") == {}


# ---- settle mode ----


def test_command_increment_tracks_the_span():
    """0.01 command is 6 us on 900..2100 and 4.05 us on the narrowed span."""
    assert servo_probe.command_increment_for_step_us(6.0, 900, 2100) == pytest.approx(0.01)
    assert servo_probe.command_increment_for_step_us(4.05, 1116, 1926) == pytest.approx(0.01)
    # Same physical step costs more command on a narrow span.
    assert servo_probe.command_increment_for_step_us(6.0, 1116, 1926) > 0.01
    with pytest.raises(ValueError):
        servo_probe.command_increment_for_step_us(6.0, 1500, 1500)


def test_reached_target_matches_the_trimmer():
    assert servo_probe.reached_target(-33.5, -33.0)
    assert not servo_probe.reached_target(-32.5, -33.0)
    assert servo_probe.reached_target(35.2, 35.0)
    assert not servo_probe.reached_target(34.8, 35.0)
    # The zero target uses a 0.5 deg band, not equality.
    assert servo_probe.reached_target(0.4, 0.0)
    assert not servo_probe.reached_target(0.6, 0.0)


class _FakeFcu:
    """A servo whose *mechanics* lag the command. The PWM readback does not.

    The FC reports the output it is driving, immediately; only the surface takes
    time to get there. Conflating the two hides exactly the effect under test.
    Repeated sends of an unchanged value are keepalives, not steps, so they must
    not advance the lag.
    """

    def __init__(self, lag_steps=4):
        self.commands = []
        self.steps = [0.0]
        self.lag_steps = lag_steps
        self.output_instance = 1

    def actuator_test(self, function, value, wait_ack=False, ack_timeout=1.0):
        value = float(value)
        self.commands.append(value)
        if value != self.steps[-1]:
            self.steps.append(value)
        return servo_probe.MAV_RESULT_ACCEPTED if wait_ack else None

    def commanded(self):
        return self.steps[-1]

    def position(self):
        """Where the surface actually is: lag_steps behind the command."""
        return self.steps[max(0, len(self.steps) - 1 - self.lag_steps)]

    def read_actuator_outputs(self, instance=None):
        # 900..2100, not reversed: expected_pwm() is 1500 + 600 * cmd.
        return {1: [0.0, 0.0, 0.0, 0.0, 1500.0 + 600.0 * self.commanded(), 0.0]}


class _FakePico:
    """Angle is 40 deg per command unit, read off the lagged position."""

    def __init__(self, fcu):
        self.fcu = fcu

    def angle(self, side, window_s=0.5):
        return 40.0 * self.fcu.position()


def test_sweep_to_threshold_reproduces_the_lag_overshoot(monkeypatch):
    """The stop command reflects the *reported* angle, so the surface is past it."""
    monkeypatch.setattr(servo_probe.time, "sleep", lambda seconds: None)

    fcu = _FakeFcu(lag_steps=4)
    pico = _FakePico(fcu)
    params = {"pwm_min": 900, "pwm_max": 2100, "trim": 0.0, "rev": False}

    result = servo_probe.sweep_to_threshold(
        fcu, pico, "LEFT", params, -33.0, servo_probe.SWEEP_STEP_US,
        0.0, 0.5, timeout_s=60.0)

    assert result is not None
    assert result["reported_deg"] <= -33.0
    # The command it recorded is 4 ticks further along than the angle it saw.
    settled = 40.0 * result["stop_cmd"]
    assert settled < result["reported_deg"]
    assert settled - result["reported_deg"] == pytest.approx(-4 * 0.01 * 40.0, abs=0.5)


def test_sweep_to_threshold_gives_up_at_the_command_limit(monkeypatch):
    monkeypatch.setattr(servo_probe.time, "sleep", lambda seconds: None)
    fcu = _FakeFcu(lag_steps=0)
    pico = _FakePico(fcu)
    params = {"pwm_min": 900, "pwm_max": 2100, "trim": 0.0, "rev": False}
    # 40 deg per command unit cannot reach -60 deg before cmd hits -1.0.
    assert servo_probe.sweep_to_threshold(
        fcu, pico, "LEFT", params, -60.0, servo_probe.SWEEP_STEP_US,
        0.0, 0.5, timeout_s=60.0) is None


def test_keepalive_refreshes_without_advancing_the_step(monkeypatch):
    """A long step period must not let the override lapse mid-step."""
    monkeypatch.setattr(servo_probe.time, "sleep", lambda seconds: None)
    fcu = _FakeFcu(lag_steps=0)
    servo_probe.sleep_with_keepalive(fcu, 1201, -0.5, 2.0)
    assert len(fcu.commands) >= 3          # 2.0 s / 0.5 s refresh
    assert set(fcu.commands) == {-0.5}     # the value never changes
    assert fcu.steps == [0.0, -0.5]        # and it counts as one step


def test_check_authority_spots_a_lapsed_override():
    fcu = _FakeFcu(lag_steps=0)
    params = {"pwm_min": 900, "pwm_max": 2100, "trim": 0.0, "rev": False}
    fcu.actuator_test(1201, -0.5)
    ok, actual, expected = servo_probe.check_authority(fcu, "LEFT", params, -0.5)
    assert ok and actual == expected == 1200

    # The FC has taken the outputs back: readback no longer tracks the command.
    ok, actual, expected = servo_probe.check_authority(fcu, "LEFT", params, 0.5)
    assert not ok and actual == 1200 and expected == 1800
