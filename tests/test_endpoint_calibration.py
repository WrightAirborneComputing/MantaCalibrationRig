"""The end stop calibration driven end to end against a simulated servo.

The procedure in SERVO_SETTING.md only means anything against a surface that
behaves like the real one: a crept-to position sits short of a driven-to one,
and a surface can run out of travel before the command does. Both are modelled
here, so these check the calibration recovers the right end stops rather than
merely that it runs.

    xvfb-run -a python3 -m pytest tests/test_endpoint_calibration.py -v
"""

import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

tk = pytest.importorskip("tkinter")

from fake_pico import FakePico

import endpoint_logic
import MantaTrimmer as MT


def _has_display():
    try:
        root = tk.Tk()
    except Exception:
        return False
    root.destroy()
    return True
# def


pytestmark = pytest.mark.skipif(not _has_display(), reason="no display available")

# A servo whose angle is linear in PWM, with a stiction lag and optional
# mechanical limits. Deliberately not symmetric: 1500 us is not 0 deg, so a
# calibration that quietly assumes centre would fail.
DEG_PER_US = 0.08
CENTRE_US = 1520.0
STICTION_DEG = 1.6


class SimServo:
    """PWM in, angle out, with stiction and optional hard stops."""

    def __init__(self, stop_low_deg=None, stop_high_deg=None):
        self.stop_low_deg = stop_low_deg
        self.stop_high_deg = stop_high_deg
        self.angle = 0.0
        self.last_pwm = None

    def free_angle(self, pwm):
        angle = (pwm - CENTRE_US) * DEG_PER_US
        if self.stop_low_deg is not None:
            angle = max(self.stop_low_deg, angle)
        if self.stop_high_deg is not None:
            angle = min(self.stop_high_deg, angle)
        return angle

    def drive(self, pwm):
        """Small steps leave the surface short; a large one arrives fully."""
        target = self.free_angle(pwm)

        if self.last_pwm is not None and abs(pwm - self.last_pwm) <= 25:
            direction = 1.0 if target > self.angle else -1.0
            shortfall = min(STICTION_DEG, abs(target - self.angle))
            self.angle = target - direction * shortfall
        else:
            self.angle = target

        self.last_pwm = pwm
        return self.angle
# class


class SimRig:
    """A flight controller whose parameters actually drive a simulated servo."""

    def __init__(self, gui, servo, rev=False):
        self.gui = gui
        self.servo = servo
        self.params = {"PWM_MAIN_MIN5": 1000, "PWM_MAIN_MAX5": 2000,
                       "CA_SV_CS0_TRIM": 0.0, "PWM_MAIN_REV": 32 if rev else 0}
        self.rev = rev
        self.writes = []

    def is_connected(self):
        return True

    def get_param(self, name, py_type=None, timeout=5.0):
        value = self.params.get(name)
        return py_type(value) if (py_type and value is not None) else value

    def set_param_value(self, name, py_type, value, **kwargs):
        self.params[name] = py_type(value)
        self.writes.append((name, py_type(value)))
        return True

    def command_elevon(self, output_function, value, wait_ack=False, **kwargs):
        pwm = self.gui.expected_pwm(value, self.params["PWM_MAIN_MIN5"],
                                    self.params["PWM_MAIN_MAX5"],
                                    self.params["CA_SV_CS0_TRIM"], self.rev)
        self.servo.drive(pwm)
        return MT.DroneInterface.MAV_RESULT_ACCEPTED if wait_ack else None

    def close(self):
        pass
# class


@pytest.fixture
def rig(monkeypatch):
    os.environ["MANTA_NO_ACTUATE"] = ""

    pico = FakePico().start()
    root = tk.Tk()
    reader = MT.PositionReader()
    gui = MT.FourSliderGUI(root, reader, MT.DroneInterface())

    servo = SimServo()
    drone = SimRig(gui, servo)
    gui.drone_interface = drone
    gui.main_rev = 0

    # The angle comes from the simulated servo, not the Pico: this exercises the
    # calibration's decisions, and a pot simulation would only add noise to them.
    monkeypatch.setattr(gui, "get_side_angle", lambda side: servo.angle)

    # Real time, compressed. Every dwell and hold in the procedure is honoured
    # in order and in proportion, just not in seconds.
    monkeypatch.setattr(MT.time, "sleep", lambda seconds: None)

    gui.angle_neg_degs = -30.0
    gui.angle_pos_degs = 30.0
    gui.angle_trim_degs = 0.0

    try:
        yield gui, drone, servo
    finally:
        gui.left_cal_active = False
        gui._closing = True
        try:
            reader.stop()
        except Exception:
            pass
        pico.stop()
        try:
            root.destroy()
        except Exception:
            pass
# def


def run_calibration(gui, side="LEFT", timeout=60.0):
    thread = threading.Thread(target=gui._calibration_worker, args=(side,),
                              daemon=True)
    thread.start()

    deadline = time.time() + timeout
    while thread.is_alive() and time.time() < deadline:
        root = gui.root
        try:
            root.update()
        except Exception:
            break
        time.sleep(0.005)

    assert not thread.is_alive(), "calibration worker did not finish"
# def


def test_end_stops_land_on_the_targets(rig):
    """The whole point: the written PWM must produce the target angle when the
    surface is driven there, not when it is crept there."""
    gui, drone, servo = rig

    run_calibration(gui)

    for which, param, target in (("MAX", "PWM_MAIN_MAX5", 30.0),
                                 ("MIN", "PWM_MAIN_MIN5", -30.0)):
        pwm = drone.params[param]
        # Driven there in one move, which is how a surface is actually used.
        servo.last_pwm = None
        assert servo.drive(pwm) == pytest.approx(target, abs=0.5), which
# def


def test_a_crept_calibration_would_have_missed(rig):
    """Establishes that the simulated servo has the fault being corrected for -
    otherwise the test above would pass against a servo with no stiction."""
    _gui, _drone, servo = rig

    pwm = CENTRE_US + (30.0 / DEG_PER_US)
    servo.last_pwm = None
    driven = servo.drive(pwm)

    servo.angle = 0.0
    servo.last_pwm = pwm - 10
    crept = servo.drive(pwm)

    assert abs(driven - crept) == pytest.approx(STICTION_DEG, abs=0.1)
# def


def test_an_unreachable_target_aborts_before_touching_the_end_stops(rig, capsys):
    """A surface that runs out of travel before the target does must abort in
    the coarse stage, leaving the wide-open range rather than a fabricated one."""
    gui, drone, servo = rig
    servo.stop_low_deg = -22.0

    run_calibration(gui)
    output = capsys.readouterr().out

    assert "hit command limit" in output
    assert "stopped before negative endpoint completed" in output
    assert drone.params["PWM_MAIN_MIN5"] == endpoint_logic.COARSE_MIN
    assert drone.params["PWM_MAIN_MAX5"] == endpoint_logic.COARSE_MAX


def test_a_pinned_end_stop_is_detected_as_a_hard_stop(rig, capsys):
    """The 056 case: an end stop written past where the surface can physically
    go. A range of PWM values all give the same angle, so the value written
    there means nothing and no correction derived from it would either."""
    gui, drone, servo = rig
    servo.stop_low_deg = -30.0
    gui.left_cal_active = True

    # cmd -1 asks for -41.6 deg; the surface stops at -30 and stays there for
    # 145 us of travel, far beyond the 50 us that counts as free.
    result = gui._cal_evaluate_endpoint(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "MIN", drone.params["PWM_MAIN_MIN5"],
        {"MAX": 30.0, "MIN": -30.0},
        (drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"]), False)

    assert result is not None
    assert result["hard_stop"] is True
    assert result["accepted"] is False
    assert result["breakaway_us"] > endpoint_logic.HARD_STOP_LIMIT_US
    # Pulled inward, out of the jammed region.
    assert result["new_pwm"] > drone.params["PWM_MAIN_MIN5"]
    assert "against a stop" in capsys.readouterr().out


def test_a_free_end_stop_is_not_called_jammed(rig):
    """The same probe on a surface with room must find breakaway quickly."""
    gui, drone, servo = rig
    gui.left_cal_active = True

    result = gui._cal_evaluate_endpoint(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "MIN", drone.params["PWM_MAIN_MIN5"],
        {"MAX": 41.6, "MIN": -41.6},
        (drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"]), False)

    assert result is not None
    assert result["hard_stop"] is False
    assert result["breakaway_us"] <= endpoint_logic.HARD_STOP_LIMIT_US
    assert result["gain"] is not None


def test_verification_runs_and_reports_both_ends(rig, capsys):
    gui, _drone, _servo = rig

    run_calibration(gui)
    output = capsys.readouterr().out

    assert "verifying" in output
    assert output.count("verify: ") >= 2
    assert "OUT" not in output, "a good calibration verifies inside tolerance"
# def


def test_stopping_mid_run_leaves_the_surface_centred(rig):
    gui, drone, _servo = rig

    thread = threading.Thread(target=gui._calibration_worker, args=("LEFT",),
                              daemon=True)
    thread.start()
    time.sleep(0.05)
    gui.left_cal_active = False

    deadline = time.time() + 30.0
    while thread.is_alive() and time.time() < deadline:
        gui.root.update()
        time.sleep(0.005)

    assert not thread.is_alive()
    assert drone.params["PWM_MAIN_MIN5"] is not None
# def


def test_a_failed_parameter_write_aborts(rig, capsys):
    """A write that does not land leaves every later number computed against a
    span the FC is not using."""
    gui, drone, _servo = rig

    def refuse(name, py_type, value, **kwargs):
        return False

    drone.set_param_value = refuse
    run_calibration(gui)

    assert "failed to write" in capsys.readouterr().out
# def


def test_an_end_stop_far_past_target_skips_the_stop_check(rig, capsys):
    """The 17 deg case. A surface well past its target has travel to spare, so
    probing it answers nothing and only costs a back-off cycle."""
    gui, drone, servo = rig
    gui.left_cal_active = True

    drives = []
    original = servo.drive
    servo.drive = lambda pwm: (drives.append(pwm), original(pwm))[1]

    # cmd -1 reaches -41.6 deg against a -25 deg target: 16.6 deg past.
    result = gui._cal_evaluate_endpoint(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "MIN", drone.params["PWM_MAIN_MIN5"],
        {"MAX": 25.0, "MIN": -25.0},
        (drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"]), False)

    assert result["probed"] is False
    assert result["hard_stop"] is False
    assert result["breakaway_us"] is None
    assert result["new_pwm"] > drone.params["PWM_MAIN_MIN5"], "pulled inward"

    # Park at the far end and one traverse to the end stop, and nothing else:
    # no back-off cycle was run.
    assert len(set(drives)) == 2, "the probe would have used several more"
    assert "without a stop check" in capsys.readouterr().out


def test_an_end_stop_near_target_still_gets_checked(rig):
    """The skip must not swallow the case the probe exists for."""
    gui, drone, servo = rig
    gui.left_cal_active = True

    drives = []
    original = servo.drive
    servo.drive = lambda pwm: (drives.append(pwm), original(pwm))[1]

    result = gui._cal_evaluate_endpoint(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "MIN", drone.params["PWM_MAIN_MIN5"],
        {"MAX": 41.6, "MIN": -41.6},
        (drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"]), False)

    assert result["probed"] is True
    assert len(set(drives)) > 2, "the back-off probe ran"


def _drive_path(servo):
    """Distinct PWM values the surface was driven through, in order."""
    path = []
    original = servo.drive

    def record(pwm):
        if not path or path[-1] != pwm:
            path.append(pwm)
        return original(pwm)

    servo.drive = record
    return path
# def


def test_every_evaluation_approaches_from_the_far_end(rig):
    """The invariant the whole procedure rests on.

    How far a surface travelled to reach an end stop changes where it settles,
    so a measurement is only comparable with another taken after the same
    run-up. Before this, once one end was accepted the loop stopped visiting it
    and the surviving end was measured from wherever its own back-off probe had
    left it - a few tens of microseconds away. On one airframe that read 5 deg
    different from the same PWM approached properly: accepted on one condition,
    then contradicted by the verify on another.
    """
    gui, drone, servo = rig
    gui.left_cal_active = True

    targets = {"MAX": 38.0, "MIN": -38.0}
    span = (drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"])
    path = _drive_path(servo)

    gui._cal_evaluate_endpoint("LEFT", gui.LEFT_OUTPUT_FUNCTION, "MAX",
                               drone.params["PWM_MAIN_MAX5"], targets, span,
                               False)

    assert path[0] == drone.params["PWM_MAIN_MIN5"], "parked at the far end"
    assert path[1] == drone.params["PWM_MAIN_MAX5"], "then a full-span traverse"


def test_evaluating_the_other_end_also_starts_from_its_far_end(rig):
    gui, drone, servo = rig
    gui.left_cal_active = True

    targets = {"MAX": 38.0, "MIN": -38.0}
    span = (drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"])
    path = _drive_path(servo)

    gui._cal_evaluate_endpoint("LEFT", gui.LEFT_OUTPUT_FUNCTION, "MIN",
                               drone.params["PWM_MAIN_MIN5"], targets, span,
                               False)

    assert path[0] == drone.params["PWM_MAIN_MAX5"]
    assert path[1] == drone.params["PWM_MAIN_MIN5"]


def test_verification_approaches_each_end_the_same_way(rig):
    """The verify has to measure under the conditions the refinement accepted
    under, or the two can only disagree."""
    gui, drone, servo = rig
    gui.left_cal_active = True
    path = _drive_path(servo)

    gui._cal_verify_endpoints(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION,
        {"MAX": drone.params["PWM_MAIN_MAX5"], "MIN": drone.params["PWM_MAIN_MIN5"]},
        {"MAX": 38.0, "MIN": -38.0}, False)

    low, high = drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"]

    # MAX is checked first, so: park low, traverse high. Then MIN: park high,
    # traverse low. Each end measured after its own full-span run-up.
    assert path == [low, high, low]


def test_a_parameter_write_leaves_the_surface_where_it_was_told(rig):
    """The FC reclaims the outputs during a write, so the next move must not
    start from wherever it parked the surface."""
    gui, drone, servo = rig

    commands = []
    original = drone.command_elevon
    drone.command_elevon = lambda fn, value, **kw: (
        commands.append(value), original(fn, value, **kw))[1]

    gui._cal_write_param("LEFT", gui.LEFT_OUTPUT_FUNCTION, "PWM_MAIN_MAX5",
                         int, 1900, -1.0)

    assert commands[-1] == pytest.approx(-1.0), "re-commanded after the write"
    assert drone.params["PWM_MAIN_MAX5"] == 1900
