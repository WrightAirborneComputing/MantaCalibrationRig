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

# Captured before the fake clock replaces time.sleep, so the test's own polling
# loop still waits in real seconds while the code under test does not.
REAL_SLEEP = time.sleep

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

tk = pytest.importorskip("tkinter")

from fake_pico import FakePico

import cal_flow
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
    """PWM in, angle out, with stiction, lash and optional hard stops."""

    def __init__(self, stop_low_deg=None, stop_high_deg=None, lash_deg=0.0,
                 stiction_deg=STICTION_DEG):
        self.stop_low_deg = stop_low_deg
        self.stop_high_deg = stop_high_deg
        # Per instance so a test can wind it up. The module default is what
        # every test that predates this saw, so none of them change.
        self.stiction_deg = stiction_deg
        # Backlash: the surface lands short of where it was driven, on the side
        # it came from, so the same command reads 2 x lash_deg apart depending
        # on the approach. Zero by default, so every test that predates it sees
        # exactly the servo it saw before.
        self.lash_deg = lash_deg
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
            shortfall = min(self.stiction_deg, abs(target - self.angle))
            self.angle = target - direction * shortfall
        elif self.lash_deg:
            direction = 1.0 if target > self.angle else -1.0
            self.angle = target - direction * self.lash_deg
        else:
            self.angle = target

        self.last_pwm = pwm
        return self.angle
# class


class FakeClock:
    """A wall clock without the wall. Tests only.

    The dwells in the procedure are real waits: _hold() spins until
    monotonic() passes its deadline, so patching sleep() to a no-op does not
    shorten a 0.8 s dwell, it only turns it into a busy-wait. That cost the
    file about a minute, and it made the interleaving between the worker
    thread and the Tk main loop depend on how fast the machine happened to be.

    Advancing the clock from sleep() instead leaves every loop running the
    same number of iterations, issuing the same commands in the same order,
    and takes the seconds out. time() is deliberately left alone - the test's
    own timeout is measured with it, and a frozen one would never fire.
    """

    def __init__(self):
        self.now = 0.0

    def sleep(self, seconds):
        self.now += max(0.0, float(seconds))

    def monotonic(self):
        return self.now
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
    clock = FakeClock()
    monkeypatch.setattr(MT.time, "sleep", clock.sleep)
    monkeypatch.setattr(MT.time, "monotonic", clock.monotonic)

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
        REAL_SLEEP(0.005)

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


def test_a_sticky_surface_is_not_called_a_hard_stop(rig):
    """The verdict this whole change turns on.

    A free surface with no mechanical stop anywhere, stiff enough that no 10 us
    probe step moves it within the 200 us ceiling. Nothing was learned about
    whether it can move, and saying so is the point - the old two-way verdict
    called it jammed and pulled the end stop in by 210 us.

    Driven through the evaluation directly rather than a whole run: the coarse
    sweep steps 6-18 us, so every step of it takes the stiction branch too, and
    a servo stiff enough to fail the probe cannot complete the sweep. The
    window where both happen is about 0.4 deg wide.
    """
    gui, drone, servo = rig
    servo.stiction_deg = 20.0
    gui.left_cal_active = True

    result = gui._cal_evaluate_endpoint(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "MIN", drone.params["PWM_MAIN_MIN5"],
        {"MAX": 41.6, "MIN": -41.6},
        (drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"]), False)

    assert result["breakaway_us"] is None
    assert result["stop_kind"] == endpoint_logic.STOP_UNCONFIRMED
    assert result["stop_kind"] != endpoint_logic.STOP_HARD, \
        "nothing here proved there is a stop"


def test_moderate_stiction_still_reads_as_a_stop_but_says_so_in_the_chart(rig):
    """Honest about what this change does not fix.

    70 us of friction and 70 us of mechanical pin are the same measurement, so
    this stays a hard stop. What it gains is a visible amber node instead of
    one line among hundreds.
    """
    gui, drone, servo = rig
    servo.stiction_deg = 5.0
    gui.left_cal_active = True

    result = gui._cal_evaluate_endpoint(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "MIN", drone.params["PWM_MAIN_MIN5"],
        {"MAX": 41.6, "MIN": -41.6},
        (drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"]), False)

    assert result["stop_kind"] == endpoint_logic.STOP_HARD
    assert result["breakaway_us"] > endpoint_logic.HARD_STOP_LIMIT_US

    states = dict((key, state) for key, _l, _r, _c, state, _n
                  in gui.cal_flow["LEFT"].snapshot(0.0)[2])
    assert states["travel_min"] == cal_flow.WARNED
    assert gui.cal_flow["LEFT"].verdict != cal_flow.FAILED, "the run goes on"


def test_two_unconfirmed_probes_at_one_end_end_the_side_for_stiction(rig, capsys):
    """One is recoverable, two is friction.

    The first pull-in is the recovery for an end stop written past the
    surface's reach. If the surface still will not move after the endpoint has
    travelled the full ceiling inward, there was no stop there to come off.
    """
    gui, drone, servo = rig
    servo.stiction_deg = 20.0
    gui.left_cal_active = True
    before = drone.params["PWM_MAIN_MIN5"]

    endpoints = {"MAX": (drone.params["PWM_MAIN_MAX5"], 41.6),
                 "MIN": (before, -41.6)}
    result = gui._cal_refine_endpoints(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "PWM_MAIN_MIN5", "PWM_MAIN_MAX5",
        endpoints, False)

    assert result is None
    flow = gui.cal_flow["LEFT"]
    assert flow.verdict == cal_flow.FAILED
    assert flow.failure[3] == "free_travel_unconfirmed"
    assert "Stiction is likely too high" in flow.failure[2]

    # Pulled in once, for the attempt that could still have been a stop, and
    # not again. Pulling in twice is 420 us of travel thrown away on a surface
    # that was never against anything.
    pull_ins = [pwm for name, pwm in drone.writes if name == "PWM_MAIN_MIN5"]
    assert len(pull_ins) == 1, "pulled in %d times: %r" % (len(pull_ins), pull_ins)

    assert "stiction, not a stop" in capsys.readouterr().out


def test_an_overshooting_endpoint_is_never_blamed_for_stiction(rig):
    """It reports no breakaway because it never probed, not because nothing moved.

    Reading "unconfirmed" off that None is how a healthy end stop several
    corrections from target would be failed for friction it does not have.
    """
    gui, drone, servo = rig
    gui.left_cal_active = True

    # cmd -1 reaches -41.6 deg against a -25 deg target: well past it.
    result = gui._cal_evaluate_endpoint(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "MIN", drone.params["PWM_MAIN_MIN5"],
        {"MAX": 25.0, "MIN": -25.0},
        (drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"]), False)

    assert result["probed"] is False
    assert result["breakaway_us"] is None
    assert result["stop_kind"] == endpoint_logic.STOP_FREE


def test_a_lost_reading_at_the_end_stop_is_not_reported_as_stiction(rig):
    """The probe returns the same pair for a sensor dropout as for a ceiling.

    Blaming friction for a link fault sends the operator to the wrong end of
    the rig.
    """
    gui, drone, servo = rig
    gui.left_cal_active = True

    gui._cal_measure = lambda *a, **k: None

    breakaway, angle = gui._cal_probe_breakaway(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "MIN",
        (drone.params["PWM_MAIN_MIN5"], drone.params["PWM_MAIN_MAX5"]),
        False, -1.0, -30.0)

    assert breakaway is None and angle is None
    assert gui.cal_flow["LEFT"].failure[3] == "no_angle_at_stop"


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
    REAL_SLEEP(0.05)
    gui.left_cal_active = False

    deadline = time.time() + 30.0
    while thread.is_alive() and time.time() < deadline:
        gui.root.update()
        REAL_SLEEP(0.005)

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


def test_a_probe_poisoned_by_creep_does_not_send_the_endpoint_backwards(rig):
    """The RIGHT run's failure.

    The probe measures the local gain over one 10 us step taken 0.8 s after a
    full-span slam into the end stop, while the surface is still creeping back
    from the overshoot. On the rig the creep won, and the probe reported the
    right magnitude with the wrong sign: MIN was corrected away from its target
    three attempts running, 1.8 deg of error becoming 44 deg.

    Here the probe is poisoned outright. The geometry still says which way the
    surface moves, and that is what the correction has to follow.
    """
    gui, drone, _servo = rig
    gui.left_cal_active = True
    drone.params["PWM_MAIN_MIN5"] = 1100

    # Backing off inward moved the surface the *other* way: creep, not gain.
    monkey = lambda side, fn, which, span, rev, cmd, angle: (10.0, angle - 1.0)
    gui._cal_probe_breakaway = monkey

    result = gui._cal_evaluate_endpoint(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "MIN", 1100,
        {"MAX": 35.0, "MIN": -35.0}, (1100, 2000), False)

    assert result["probed"] is True
    assert result["gain"] > 0, "corrected on the geometry, not on the creep"
    assert result["new_pwm"] < 1100, "MIN moved toward its target"
    assert result["new_pwm"] >= endpoint_logic.PWM_FLOOR


def test_an_end_stop_that_keeps_getting_worse_ends_the_run(rig, capsys):
    """Ten attempts of a correction pushing the wrong way is how MIN reached
    2200 us against a MAX of 1893 - a crossed pair PX4 silently swaps, so every
    reading after it was taken against a range nobody chose."""
    gui, drone, _servo = rig
    gui.left_cal_active = True

    errors = iter((1.82, 3.58, 15.82, 44.65, 66.79))

    def worsening(side, fn, which, pwm_now, targets, span, rev, anchor=None):
        if which == "MAX":
            return {"which": which, "angle_deg": targets["MAX"],
                    "target_deg": targets["MAX"], "accepted": True,
                    "new_pwm": None, "gain": 0.1, "hard_stop": False,
                    "breakaway_us": 10.0, "probed": True}
        angle = targets["MIN"] + next(errors)
        return {"which": which, "angle_deg": angle, "target_deg": targets["MIN"],
                "accepted": False, "new_pwm": pwm_now + 100, "gain": -0.1,
                "hard_stop": False, "breakaway_us": 10.0, "probed": True}

    gui._cal_evaluate_endpoint = worsening
    gui._cal_measure = lambda *args, **kwargs: 0.0

    result = gui._cal_refine_endpoints(
        "LEFT", gui.LEFT_OUTPUT_FUNCTION, "PWM_MAIN_MIN5", "PWM_MAIN_MAX5",
        {"MAX": (1893, 35.0), "MIN": (1192, -33.0)}, False)

    assert result is None, "the run stops rather than writing ten bad values"
    assert "diverging" in capsys.readouterr().out
    # Two attempts' worth of damage, not ten, and nowhere near MAX.
    assert drone.params["PWM_MAIN_MIN5"] <= 1192 + 200
    assert drone.params["PWM_MAIN_MIN5"] < drone.params["PWM_MAIN_MAX5"]


def _trim_param(drone):
    return drone.params["CA_SV_CS0_TRIM"]
# def


def test_the_trim_is_set_from_one_approach_and_the_lash_is_reported(rig, capsys):
    """The trim point is the angle reached arriving from one direction.

    With 0.4 deg of lash each way the same command reads 0.8 deg apart
    depending on the approach, which is wider than any acceptance band worth
    having - so the procedure converges the approach it is set from and reports
    the other as backlash rather than trying to close it.
    """
    gui, drone, servo = rig
    servo.lash_deg = 0.4
    gui.angle_trim_degs = -5.0

    run_calibration(gui)
    output = capsys.readouterr().out

    assert "did not settle" not in output
    assert "backlash" in output

    # Driven to the written trim from the set direction, the surface must sit
    # on target. That is what the trim point means here.
    pwm = gui.expected_pwm(0.0, drone.params["PWM_MAIN_MIN5"],
                           drone.params["PWM_MAIN_MAX5"], _trim_param(drone),
                           False)
    servo.last_pwm = None
    servo.angle = 40.0                      # arriving from the positive side
    assert servo.drive(pwm) == pytest.approx(-5.0, abs=0.25)

    # And the lash is 2 x 0.4 deg, on the record rather than corrected away.
    assert gui.left_backlash_deg == pytest.approx(-0.8, abs=0.1)


def test_the_trim_approaches_from_the_set_direction_every_time(rig):
    """Every reading is taken after a full traverse out to the approach
    command - the same invariant the end stops rest on."""
    gui, drone, servo = rig
    gui.left_cal_active = True
    gui.angle_trim_degs = -5.0
    path = _drive_path(servo)

    found = gui._cal_trim_point("LEFT", gui.LEFT_OUTPUT_FUNCTION,
                                {"MAX": 30.0, "MIN": -30.0}, False)

    assert found is not None
    cmd_trim, _lash = found

    high = gui.expected_pwm(MT.TRIM_APPROACH_CMD, drone.params["PWM_MAIN_MIN5"],
                            drone.params["PWM_MAIN_MAX5"], 0.0, False)
    low = gui.expected_pwm(-MT.TRIM_APPROACH_CMD, drone.params["PWM_MAIN_MIN5"],
                           drone.params["PWM_MAIN_MAX5"], 0.0, False)

    assert path[0] == high, "parked at the approach command first"
    # The last two moves are the backlash measurement: out to the far side,
    # then back to the same command.
    assert path[-2] == low
    assert path[-1] == path[-3], "the same trim command, arrived at both ways"
    assert -1.0 < cmd_trim < 0.0


def test_a_trim_that_will_not_settle_writes_nothing(rig, capsys):
    """A wandering reading must leave the trim alone rather than write a
    number that means nothing."""
    gui, drone, _servo = rig
    gui.left_cal_active = True
    gui.angle_trim_degs = -5.0

    before = _trim_param(drone)
    angles = iter([-5.0 + 3.0 * n for n in range(1, 40)])
    gui.get_side_angle = lambda side: next(angles)

    found = gui._cal_trim_point("LEFT", gui.LEFT_OUTPUT_FUNCTION,
                                {"MAX": 30.0, "MIN": -30.0}, False)

    assert found is None
    assert "diverging" in capsys.readouterr().out
    assert _trim_param(drone) == before


def test_the_trim_cost_is_reported_before_the_second_verify(rig, capsys):
    """A trim gives up travel at one end and deafens the other. Both are
    expected; neither should have to be worked out from the verify."""
    gui, drone, _servo = rig

    span = drone.params["PWM_MAIN_MAX5"] - drone.params["PWM_MAIN_MIN5"]
    gui._cal_report_trim_cost("LEFT", -0.29,
                              {"MIN": drone.params["PWM_MAIN_MIN5"],
                               "MAX": drone.params["PWM_MAIN_MAX5"]}, False)

    output = capsys.readouterr().out
    assert "%.0f us" % (0.29 / 2.0 * span) in output
    assert "MAX end" in output, "a negative trim shortens the cmd +1 end"
    assert "29%" in output, "the dead band at the other end"


def test_the_run_verifies_again_with_the_trim_applied(rig, capsys):
    gui, _drone, _servo = rig
    gui.angle_trim_degs = -5.0

    run_calibration(gui)
    output = capsys.readouterr().out

    assert "verifying with the trim applied" in output
    assert output.count("verify: ") >= 4, "both ends, both passes"


def test_a_backlash_that_cannot_be_read_still_leaves_the_trim_set(rig, capsys):
    """It is measured after the fact, not a condition of the trim being right."""
    gui, drone, _servo = rig
    gui.left_cal_active = True
    gui.angle_trim_degs = 0.0
    gui._cal_measure_backlash = lambda *args: None

    found = gui._cal_trim_point("LEFT", gui.LEFT_OUTPUT_FUNCTION,
                                {"MAX": 30.0, "MIN": -30.0}, False)

    assert found is not None
    cmd_trim, lash = found
    assert lash is None

    # 1500 us is deliberately not 0 deg on this servo, so the trim lands a
    # little off centre - what matters is that it landed at all.
    servo = _servo
    servo.last_pwm = None
    servo.angle = 40.0
    pwm = gui.expected_pwm(cmd_trim, drone.params["PWM_MAIN_MIN5"],
                           drone.params["PWM_MAIN_MAX5"], 0.0, False)
    assert servo.drive(pwm) == pytest.approx(0.0, abs=0.25)

    gui.set_side_backlash("LEFT", lash)
    assert gui.backlash_for_log("LEFT") == "", "logged blank, not zero"
