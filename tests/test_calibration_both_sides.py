"""Both elevons calibrating at once, and the rig they share while they do it.

The rig is one frame. Sweeping one elevon shakes it, and the shake reaches the
opposite potentiometer as angle, so a reading taken on one side while the other
is moving is not a measurement of anything. Left and right run as two
independent worker threads, and they decorrelate inside the first end stop -
the breakaway probe takes anywhere from one to twenty dwells depending on
stiction - so the overlap is the normal case rather than a rare one.

Nothing in the suite ran two workers at once before this file, which is why the
overlap survived as long as it did. What is checked here is the property the
whole rendezvous exists for, stated as a scheduling invariant rather than as an
endpoint value:

    no angle is read unless the rig has been still for a whole settle window.

The endpoint numbers are left to test_endpoint_calibration.py, which owns them.

    xvfb-run -a python3 -m pytest tests/test_calibration_both_sides.py -v
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

import MantaTrimmer as MT
from test_endpoint_calibration import (
    REAL_SLEEP,
    FakeClock,
    SimServo,
    _has_display,
)


pytestmark = pytest.mark.skipif(not _has_display(), reason="no display available")

SIDES = ("LEFT", "RIGHT")

# How long a compressed dwell really lasts. FakeClock takes the seconds out of
# a dwell by advancing a number instead of waiting, which is what makes the
# endpoint tests quick - but it also means a dwell occupies no wall clock at
# all, so the two worker threads never get the chance to interleave and a test
# of what happens when they do would pass by never letting them.
#
# A couple of milliseconds of real sleep per compressed one is enough: it is a
# yield point the other thread can be scheduled on, and a run has hundreds of
# them. That makes the check below probabilistic in one direction only. An
# unsynchronised pair will interleave somewhere across a few hundred windows,
# so a regression is caught; a synchronised pair cannot interleave at all, so
# a pass is not luck.
THREAD_YIELD_S = 0.002


class YieldingClock(FakeClock):
    """FakeClock, but the other thread actually gets to run during a dwell."""

    def sleep(self, seconds):
        FakeClock.sleep(self, seconds)
        REAL_SLEEP(THREAD_YIELD_S)
# class


class TwoSidedRig:
    """A flight controller with a servo on each side, keeping a log of events.

    The single-sided SimRig in test_endpoint_calibration.py hardwires channel 5
    and one servo, which is the whole reason the interleaving was never
    testable. This one maps each output function to its own parameters and its
    own surface, and writes down what physically happened, in order:

        move        a surface changed position, and the frame with it
        hold_start  a settle window opened
        hold_end    it closed, and the angles in it are about to be read
        read        an angle was taken

    A repeated identical PWM does not move a surface and is therefore not a
    move. SimServo would creep on one, because it models a *small step* as
    landing short - but the re-send that holds the actuator override alive is
    not a step, it is the same command again, and treating it as motion would
    make the invariant below untestable by inventing movement no real surface
    performs.
    """

    CHANNELS = {
        1201: ("LEFT", "PWM_MAIN_MIN5", "PWM_MAIN_MAX5", "CA_SV_CS0_TRIM"),
        1202: ("RIGHT", "PWM_MAIN_MIN6", "PWM_MAIN_MAX6", "CA_SV_CS1_TRIM"),
    }

    def __init__(self, gui, servos, clock):
        self.gui = gui
        self.servos = servos
        self.clock = clock
        self.params = {"PWM_MAIN_MIN5": 1000, "PWM_MAIN_MAX5": 2000,
                       "CA_SV_CS0_TRIM": 0.0,
                       "PWM_MAIN_MIN6": 1000, "PWM_MAIN_MAX6": 2000,
                       "CA_SV_CS1_TRIM": 0.0,
                       "PWM_MAIN_REV": 0}
        self.timeline = []
        self.last_pwm = {}
        self._lock = threading.Lock()

    # ---- the log ---------------------------------------------------------

    def record(self, kind, side=None, detail=None):
        with self._lock:
            self.timeline.append((self.clock.monotonic(), kind, side, detail))
    # def

    def events(self, kind):
        return [e for e in self.timeline if e[1] == kind]
    # def

    # ---- the flight controller ------------------------------------------

    def is_connected(self):
        return True
    # def

    def get_param(self, name, py_type=None, timeout=5.0):
        value = self.params.get(name)
        return py_type(value) if (py_type and value is not None) else value
    # def

    def set_param_value(self, name, py_type, value, **kwargs):
        self.params[name] = py_type(value)

        # A written endpoint re-scales the command, so the surface can move
        # without being commanded at all. That is a rig disturbance like any
        # other and the log has to see it, which is why this re-drives rather
        # than waiting for the next command.
        for output_function, (side, _min, _max, _trim) in self.CHANNELS.items():
            held = self.gui.rr_last_command.get(side)
            if held is not None:
                self.command_elevon(output_function, held)
        return True
    # def

    def command_elevon(self, output_function, value, wait_ack=False, **kwargs):
        side, min_param, max_param, trim_param = self.CHANNELS[output_function]
        self.gui.rr_last_command[side] = value

        pwm = self.gui.expected_pwm(value, self.params[min_param],
                                    self.params[max_param],
                                    self.params[trim_param], False)

        with self._lock:
            unchanged = self.last_pwm.get(side) == pwm
            self.last_pwm[side] = pwm

        if not unchanged:
            self.servos[side].drive(pwm)
            self.record("move", side, value)

        return MT.DroneInterface.MAV_RESULT_ACCEPTED if wait_ack else None
    # def

    def close(self):
        pass
    # def
# class


def build_rig(mp, sides=SIDES):
    """A window, two servos, a compressed clock, and a rig that logs. Torn down by the caller."""
    pico = FakePico().start()
    root = tk.Tk()
    reader = MT.PositionReader()
    gui = MT.FourSliderGUI(root, reader, MT.DroneInterface())

    # Where set_param_value finds the command a surface is being held at, so a
    # re-scaled endpoint moves the right surface to the right place.
    gui.rr_last_command = {}

    servos = {side: SimServo() for side in SIDES}
    rig = TwoSidedRig(gui, servos, YieldingClock())
    gui.drone_interface = rig
    gui.main_rev = 0

    # Reading an angle is an event in its own right: the invariant is about
    # when a reading is taken, not what it says.
    def read_angle(side):
        rig.record("read", side)
        return servos[side].angle

    mp.setattr(gui, "get_side_angle", read_angle)

    # The settle windows, bracketed. The arbiter holds both surfaces through
    # one of these per round, and everything the invariant says is about what
    # may happen inside one.
    real_hold = gui._hold_commands

    def bracketed_hold(commands, dwell_s):
        rig.record("hold_start", "+".join(sorted(commands)), dwell_s)
        try:
            real_hold(commands, dwell_s)
        finally:
            rig.record("hold_end", "+".join(sorted(commands)), dwell_s)

    mp.setattr(gui, "_hold_commands", bracketed_hold)

    # Which phase each side is in, so alignment can be checked as well as
    # stillness. Every phase the worker enters goes through _cal_start_step.
    real_start_step = gui._cal_start_step

    def note_phase(side, key, note=""):
        rig.record("phase", side, key)
        return real_start_step(side, key, note)

    mp.setattr(gui, "_cal_start_step", note_phase)

    # And when a side stops being one. A finished or failed worker leaves the
    # arbiter, and from that moment the other side is meant to run on alone -
    # so alignment must stop being measured against it, not go on comparing
    # against whatever phase it got to.
    real_leave = gui.rig_arbiter.leave

    def note_leave(side):
        rig.record("left", side)
        return real_leave(side)

    mp.setattr(gui.rig_arbiter, "leave", note_leave)

    mp.setattr(MT.time, "sleep", rig.clock.sleep)
    mp.setattr(MT.time, "monotonic", rig.clock.monotonic)

    gui.angle_neg_degs = -30.0
    gui.angle_pos_degs = 30.0
    gui.angle_trim_degs = 0.0

    # Park both surfaces before anything is logged. update_actuators() commands
    # every side that is not calibrating, once every 100 ms on the Tk thread,
    # so a single-sided run has the *other* side commanded throughout - it has
    # to be, or the override lapses and the FC parks the surface mid-run. The
    # first of those commands would otherwise read as a surface arriving
    # somewhere from nowhere. It is not: the slider is at centre and the
    # surface is already there. Starting from a known parked state says so, and
    # leaves any genuine movement afterwards visible.
    for output_function in sorted(TwoSidedRig.CHANNELS):
        rig.command_elevon(output_function, 0.0)
    rig.timeline.clear()

    return gui, rig, root, reader, pico
# def


def teardown_rig(gui, root, reader, pico):
    for side in SIDES:
        gui.set_calibration_active(side, False)
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


def run_calibrations(gui, sides, timeout=120.0, while_running=None):
    """Start a worker per side and pump Tk until every one of them is done.

    Both threads have to be joined before the test returns, or their Tk
    garbage outlives the window and conftest's collection has nothing to
    collect it from - which aborts the process rather than failing a test.

    while_running is called on the pump thread between updates, which is the
    only place a test can act like a user: the sliders and everything else Tk
    owns belong to this thread, not to the workers.
    """
    # Every side is declared before any thread runs, which is what
    # start_both_calibration does. A side that joins after the other has
    # started is one the first did not wait for at the boundary it has already
    # gone through, and from there the two run a phase apart for ever.
    for side in sides:
        gui.rig_arbiter.join(side, at=0.0)

    threads = [threading.Thread(target=gui._calibration_worker, args=(side,),
                                daemon=True, name="cal-%s" % side)
               for side in sides]
    for thread in threads:
        thread.start()

    deadline = time.time() + timeout
    while any(t.is_alive() for t in threads) and time.time() < deadline:
        try:
            gui.root.update()
        except Exception:
            break
        if while_running is not None:
            while_running()
        REAL_SLEEP(0.005)

    for thread in threads:
        thread.join(5.0)
    alive = [t.name for t in threads if t.is_alive()]
    assert not alive, "workers did not finish: %s" % alive
# def


def settle_violations(timeline):
    """Every way the log says a reading was taken off a moving rig.

    Two independent faults, because they are two different bugs.

    A disturbed window is a surface driven while another was settling - the
    original complaint, and what an unsynchronised pair does constantly.

    An uncovered reading is an angle taken without a clean window behind it
    that actually held this side: settling LEFT does nothing for a RIGHT
    reading, and a window that ended before the last move has been overtaken by
    it.

    Windows are tracked as a stack rather than a flag, because an
    unsynchronised pair has two of them open at once - which is precisely the
    case being looked for, and a single flag loses it by letting one side's
    window close the other's.
    """
    faults = []
    open_windows = []          # [{"sides": set, "moved": [side, ...]}]
    settled = None             # sides covered by the last window that closed
    moved_since = ["<start of run>"]

    for _t, kind, side, _detail in timeline:
        if kind == "hold_start":
            open_windows.append({"sides": set(side.split("+")), "moved": []})

        elif kind == "hold_end":
            sides = set(side.split("+"))
            mine = next((w for w in reversed(open_windows)
                         if w["sides"] == sides), None)
            if mine is None:
                faults.append("a settle window closed that never opened: %s" % side)
                continue
            open_windows.remove(mine)

            if mine["moved"]:
                faults.append("%s moved while %s was settling"
                              % (", ".join(sorted(set(mine["moved"]))), side))
            else:
                settled = sides
                moved_since = []

        elif kind == "move":
            for window in open_windows:
                window["moved"].append(side)
            moved_since.append(side)

        elif kind == "read":
            if moved_since:
                faults.append("%s was read after %s moved, with no settle between"
                              % (side, ", ".join(sorted(set(moved_since)))))
            elif settled is None or side not in settled:
                faults.append("%s was read out of a window that settled %s"
                              % (side, "nothing" if settled is None
                                 else "+".join(sorted(settled))))

    return faults
# def


def assert_every_reading_came_from_a_still_rig(timeline):
    faults = settle_violations(timeline)
    assert not faults, "%d readings taken off a moving rig, first few:\n  %s" \
        % (len(faults), "\n  ".join(faults[:5]))
# def


# The phase order the barriers enforce. The four refinement nodes collapse
# into one stage on purpose: inside it the sides may be working on different
# ends, because one can need three attempts at MAX where the other needs eight,
# and holding them together per attempt would stall the quicker side for
# nothing. Every measurement in there is still a shared window.
STAGE_ORDER = ("open_range", "sweep", "load_coarse", "refine", "verify",
               "trim", "write_trim", "verify_trim")
STAGE_OF = {"find_max": "refine", "find_min": "refine",
            "travel_max": "refine", "travel_min": "refine"}


def stage_drift(timeline):
    """The furthest apart, in stages, the two sides ever got.

    0 means they were always in the same stage. 1 is the smallest value that
    can actually be achieved: one side has to cross a boundary before the other
    can be released from it.
    """
    at = {}
    worst = 0
    for _t, kind, side, key in timeline:
        if kind == "left":
            at.pop(side, None)
            continue
        if kind != "phase":
            continue
        stage = STAGE_OF.get(key, key)
        if stage not in STAGE_ORDER:
            continue
        at[side] = STAGE_ORDER.index(stage)
        if len(at) == len(SIDES):
            worst = max(worst, max(at.values()) - min(at.values()))
    return worst
# def


def test_the_sides_move_through_the_phases_together():
    """The alignment the barriers exist for.

    Without them the sides drift freely: one finishes its coarse sweep and
    starts refining while the other is still sweeping. That is not a
    correctness fault on its own - every measurement is still a shared window -
    but it is not what running two calibrations at once is supposed to look
    like, and the side left behind pays for it.
    """
    with pytest.MonkeyPatch.context() as mp:
        gui, rig, root, reader, pico = build_rig(mp)
        try:
            # Two surfaces that are not the same surface, so the sides have
            # something to drift apart over: different stiction, backlash and a
            # mechanical stop on one side only.
            rig.servos["RIGHT"] = SimServo(stiction_deg=3.0, lash_deg=0.4,
                                           stop_low_deg=-34.0)
            run_calibrations(gui, SIDES)

            phases = [e for e in rig.timeline if e[1] == "phase"]
            assert {e[2] for e in phases} == set(SIDES)

            # Both sides have to have got to the end, or the alignment above is
            # a statement about a run that stopped early.
            reached = {side: {k for _t, kind, s, k in rig.timeline
                              if kind == "phase" and s == side}
                       for side in SIDES}
            for side in SIDES:
                assert "verify_trim" in reached[side], \
                    "%s did not finish: got as far as %s" % (side, reached[side])

            assert stage_drift(rig.timeline) <= 1, \
                "the sides drifted more than a phase boundary apart"

            # And still nothing read off a moving rig, with the barriers in.
            assert_every_reading_came_from_a_still_rig(rig.timeline)
        finally:
            teardown_rig(gui, root, reader, pico)
# def


def test_neither_side_is_read_while_the_other_is_moving():
    """The point of the rendezvous, checked against a full two-sided run.

    This is the regression test for the fault: two workers, one frame, and
    nothing that used to stop one of them measuring mid-sweep of the other.
    """
    with pytest.MonkeyPatch.context() as mp:
        gui, rig, root, reader, pico = build_rig(mp)
        try:
            run_calibrations(gui, SIDES)

            assert_every_reading_came_from_a_still_rig(rig.timeline)

            # The run has to have been real, or the invariant above holds
            # vacuously over an empty log.
            assert len(rig.events("read")) > 100
            assert {e[2] for e in rig.events("read")} == set(SIDES)
        finally:
            teardown_rig(gui, root, reader, pico)
# def


def test_the_two_sides_settle_together_rather_than_in_turn():
    """Both surfaces in one window is what keeps the run from doubling in length.

    Serialising the sides would satisfy the invariant above just as well and
    cost the whole calibration twice over, so the sharing is worth asserting
    on its own.
    """
    with pytest.MonkeyPatch.context() as mp:
        gui, rig, root, reader, pico = build_rig(mp)
        try:
            run_calibrations(gui, SIDES)

            windows = rig.events("hold_start")
            shared = [w for w in windows if w[2] == "LEFT+RIGHT"]

            assert len(shared) > 0.5 * len(windows), \
                "only %d of %d settle windows held both sides" \
                % (len(shared), len(windows))
        finally:
            teardown_rig(gui, root, reader, pico)
# def


def test_calibrating_both_sides_does_not_take_two_runs_worth_of_dwells():
    """A shared dwell is one dwell. The two sides must not queue behind each other.

    Measured in settle windows rather than seconds, because a window is what a
    dwell costs and it does not depend on how fast the machine is. The margin
    is generous on purpose: the sides do drift apart - the breakaway probe
    runs a different number of steps per end - and a few unshared windows at
    the seams are the expected price, not a regression.
    """
    with pytest.MonkeyPatch.context() as mp:
        gui, rig, root, reader, pico = build_rig(mp)
        try:
            run_calibrations(gui, ("LEFT",))
            alone = len(rig.events("hold_start"))
        finally:
            teardown_rig(gui, root, reader, pico)

    with pytest.MonkeyPatch.context() as mp:
        gui, rig, root, reader, pico = build_rig(mp)
        try:
            run_calibrations(gui, SIDES)
            together = len(rig.events("hold_start"))
        finally:
            teardown_rig(gui, root, reader, pico)

    assert alone > 0
    assert together < 1.6 * alone, \
        "two sides took %d settle windows against %d for one - they are queueing" \
        % (together, alone)
# def


def test_a_single_side_still_runs_alone():
    """One side calibrating must not wait for a side that is not calibrating.

    The regression this guards is a rendezvous that waits for participants
    rather than for the ones that actually joined, which would hang every
    single-sided run in the suite.
    """
    with pytest.MonkeyPatch.context() as mp:
        gui, rig, root, reader, pico = build_rig(mp)
        try:
            run_calibrations(gui, ("LEFT",), timeout=60.0)

            assert gui.rig_arbiter.participants() == set()
            assert {e[2] for e in rig.events("read")} == {"LEFT"}
            assert all(w[2] == "LEFT" for w in rig.events("hold_start"))
            assert_every_reading_came_from_a_still_rig(rig.timeline)
        finally:
            teardown_rig(gui, root, reader, pico)
# def


def test_dragging_the_idle_slider_cannot_shake_a_running_calibration():
    """The timer that drives the sliders is a third mover on the same frame.

    It commands every side that is not calibrating, ten times a second, from
    the Tk thread - a path the rendezvous knows nothing about. While LEFT
    calibrates, RIGHT is still on its slider, and a slider dragged mid-run put
    a moving surface inside LEFT's settle windows.

    So this run drags it, continuously, for the whole calibration: the worst a
    user can do, rather than the once that would pass by luck. RIGHT must stay
    where it was parked, and must go on being commanded there - dropping it
    instead of holding it would let the actuator override lapse, and a surface
    the FC has taken back parks itself, which is the same disturbance arriving
    by the other route.
    """
    with pytest.MonkeyPatch.context() as mp:
        gui, rig, root, reader, pico = build_rig(mp)
        try:
            sent = {side: 0 for side in SIDES}
            real_command = rig.command_elevon

            def counting_command(output_function, value, *args, **kwargs):
                sent[TwoSidedRig.CHANNELS[output_function][0]] += 1
                return real_command(output_function, value, *args, **kwargs)

            mp.setattr(rig, "command_elevon", counting_command)

            # Both ends of the slider, alternating, so no single value can look
            # still by being the one the surface was already at.
            drags = [0.8, -0.8]

            def drag_the_idle_slider():
                drags.append(drags.pop(0))
                gui.right_pos.set(drags[0])

            run_calibrations(gui, ("LEFT",), timeout=60.0,
                             while_running=drag_the_idle_slider)

            assert_every_reading_came_from_a_still_rig(rig.timeline)

            moved = [e for e in rig.events("move") if e[2] == "RIGHT"]
            assert not moved, \
                "the idle surface moved %d times while LEFT was calibrating" \
                % len(moved)

            # Still held, not merely quiet: an idle side that stopped being
            # commanded would also record no moves here, and would be taken
            # over by the FC on the real rig.
            assert sent["RIGHT"] > 10, \
                "the idle side was commanded %d times - it is being left to lapse" \
                % sent["RIGHT"]

            # And the run has to have been real.
            assert len(rig.events("read")) > 100
        finally:
            teardown_rig(gui, root, reader, pico)
# def
