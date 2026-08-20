"""Tests for the range/rate tab.

Runs the real tab against a fake Pico, with a stub flight controller standing in
for the actuators, so a full test cycle is driven end to end without hardware.
The surfaces are simulated as a servo with a dead time and a constant-rate ramp,
which is what the analysis is built to measure - so these check the tab actually
recovers the right numbers, not merely that it runs.

Skipped when no display is available:

    xvfb-run -a python3 -m pytest tests/test_range_rate_tab.py -v
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


def _has_display():
    try:
        root = tk.Tk()
    except Exception:
        return False
    root.destroy()
    return True
# def


pytestmark = pytest.mark.skipif(not _has_display(), reason="no display available")

# Deliberately different per side, as the real rig is, so a left/right mix-up
# shows up as a wrong number rather than passing by symmetry.
SERVO = {
    "LEFT": {"rate_deg_s": 580.0, "dead_s": 0.04},
    "RIGHT": {"rate_deg_s": 628.0, "dead_s": 0.04},
}

TRAVEL_DEG = {"LEFT": 66.0, "RIGHT": 64.0}

# The rate these tests *drive* at, deliberately not FAST_RATE_HZ. The fake Pico
# is a Python thread writing a pty, and asking it for the real 1000 Hz makes it
# fall behind under full-suite load - a leg then captures nothing and the tab
# correctly reports "too few samples", failing the test for a reason that says
# nothing about the tab. 400 Hz still puts ~240 samples in a 0.6 s leg and ~45
# across the simulated transit, which is what the analysis needs. That the
# product's default is 1000 Hz is asserted in test_setup_defaults_and_estimate.
TAB_RATE_HZ = 400


class SimulatedRig(object):
    """A fake FC whose commands move a simulated servo the fake Pico reports."""

    def __init__(self, pico, reader):
        self.pico = pico
        self.reader = reader
        self.commands = []

        self._lock = threading.Lock()
        self._target = {"LEFT": -1.0, "RIGHT": -1.0}
        self._position = {"LEFT": -1.0, "RIGHT": -1.0}
        self._commanded_at = {"LEFT": 0.0, "RIGHT": 0.0}
        self._start = {"LEFT": -1.0, "RIGHT": -1.0}

        # Stiction, off by default so the range/rate tests see a clean servo.
        # Modelled as a shortfall that applies only when the commanded change is
        # small: a creep step leaves the surface lagging the command by this
        # much, while a hard-over arrives exactly on it. That is the effect the
        # rig actually shows, and differencing the two is what recovers it.
        self.stiction_deg = {"LEFT": 0.0, "RIGHT": 0.0}
        self.creep_change_max = 0.15

        self._last_change = {"LEFT": 2.0, "RIGHT": 2.0}

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._integrate, daemon=True)
        self._thread.start()
    # def

    def _effective_target(self, side):
        """Where the servo will actually stop, given how big the last step was."""
        target = self._target[side]
        stiction = self.stiction_deg[side] / (TRAVEL_DEG[side] / 2.0)

        if stiction and self._last_change[side] <= self.creep_change_max:
            direction = 1.0 if target >= self._start[side] else -1.0
            target -= direction * stiction

        return target
    # def

    def is_connected(self):
        return True
    # def

    def close(self):
        self._stop.set()
        self._thread.join(2.0)
    # def

    def command_elevon(self, output_function, value, wait_ack=False, ack_timeout=1.0):
        side = "LEFT" if output_function == 1201 else "RIGHT"

        with self._lock:
            self.commands.append((side, float(value)))
            if abs(self._target[side] - float(value)) > 1e-9:
                self._last_change[side] = abs(self._target[side] - float(value))
                self._target[side] = float(value)
                self._start[side] = self._position[side]
                self._commanded_at[side] = time.monotonic()

        return MT.DroneInterface.MAV_RESULT_ACCEPTED if wait_ack else None
    # def

    def _integrate(self):
        while not self._stop.is_set():
            now = time.monotonic()

            with self._lock:
                for side in ("LEFT", "RIGHT"):
                    spec = SERVO[side]
                    # Command units per second, from the servo's deg/s.
                    units_s = spec["rate_deg_s"] / (TRAVEL_DEG[side] / 2.0)
                    elapsed = now - self._commanded_at[side] - spec["dead_s"]
                    effective = self._effective_target(side)

                    if elapsed <= 0:
                        position = self._start[side]
                    else:
                        span = effective - self._start[side]
                        step = units_s * elapsed
                        if abs(span) <= step:
                            position = effective
                        else:
                            position = self._start[side] + (1 if span > 0 else -1) * step

                    self._position[side] = position

                left, right = self._position["LEFT"], self._position["RIGHT"]

            # Drive the fake Pico's raw counts from the simulated positions,
            # inverting the reader's own calibration so the tab reads back the
            # degrees we intended.
            self.pico.base_left = self._raw("LEFT", left * (TRAVEL_DEG["LEFT"] / 2.0))
            self.pico.base_right = self._raw("RIGHT", right * (TRAVEL_DEG["RIGHT"] / 2.0))

            time.sleep(0.001)
    # def

    def _raw(self, side, degrees):
        if side == "LEFT":
            return int((degrees - self.reader.left_offset) / self.reader.left_scaler)
        return int((self.reader.right_offset - degrees) / self.reader.right_scaler)
    # def
# class


@pytest.fixture
def rig():
    os.environ["MANTA_NO_ACTUATE"] = ""

    pico = FakePico().start()
    root = tk.Tk()
    reader = MT.PositionReader()
    gui = MT.FourSliderGUI(root, reader, MT.DroneInterface())

    drone = SimulatedRig(pico, reader)
    gui.drone_interface = drone

    reader.set_port(pico.device)

    def pump(seconds):
        end = time.time() + seconds
        while time.time() < end:
            root.update()
            time.sleep(0.01)

    gui.pump = pump
    pump(1.0)

    try:
        yield gui, drone, pico
    finally:
        gui.measure_active = False
        try:
            gui._closing = True
            reader.stop()
        except Exception:
            pass
        drone.close()
        pico.stop()
        try:
            root.destroy()
        except Exception:
            pass
# def


def run_test(gui, phases, cycles=1, settle=0.6, timeout=90.0):
    """Drive one full test synchronously, pumping Tk so callbacks land."""
    for phase, var in gui.rr_phase_vars.items():
        var.set(1 if phase in phases else 0)

    gui.rr_cycles_var.set(str(cycles))
    gui.rr_settle_var.set(str(settle))
    gui.m_range_rate_var.set(1)
    gui.m_stiction_var.set(0)

    settings = gui.read_measure_settings()
    settings.update({"phases": list(phases), "cycles": cycles,
                     "settle": settle, "rate": TAB_RATE_HZ})

    gui.measure_active = True
    gui.rr_run_btn.config(state="disabled")
    gui.rr_stop_btn.config(state="normal")

    thread = threading.Thread(
        target=gui._measure_worker, args=(settings,), daemon=True)
    thread.start()

    deadline = time.time() + timeout
    while thread.is_alive() and time.time() < deadline:
        gui.pump(0.05)

    gui.pump(0.4)
    assert not thread.is_alive(), "worker did not finish"
# def


def test_setup_defaults_and_estimate(rig):
    gui, _, _ = rig

    settings = gui.read_measure_settings()

    # Range and rate on, stiction off: the creeps cost minutes, so they are
    # opt-in. BOTH alone among the phases, since the single-servo phases were
    # measured to add nothing and dropping them is what makes 30 cycles affordable.
    assert settings["range_rate"] and not settings["stiction"]
    assert settings["phases"] == list(MT.DEFAULT_PHASES)
    assert settings["cycles"] == MT.DEFAULT_CYCLES
    assert settings["rate"] == MT.FAST_RATE_HZ

    gui.update_measure_estimate()
    assert "60 hard-overs" in gui.rr_estimate_label.cget("text")
# def


def test_estimate_tracks_the_selection(rig):
    gui, _, _ = rig

    gui.rr_phase_vars["LEFT"].set(1)
    gui.rr_phase_vars["RIGHT"].set(0)
    gui.rr_phase_vars["BOTH"].set(0)
    gui.rr_cycles_var.set("2")
    gui.update_measure_estimate()

    assert "4 hard-overs" in gui.rr_estimate_label.cget("text")
# def


def test_selecting_stiction_adds_creeps_but_not_hard_overs(rig):
    """Combining the two must not change what range and rate costs."""
    gui, _, _ = rig

    gui.rr_phase_vars["LEFT"].set(1)
    gui.rr_phase_vars["BOTH"].set(0)
    gui.rr_cycles_var.set("2")
    gui.st_reps_var.set("3")

    gui.m_stiction_var.set(0)
    swings_alone, creeps_alone = gui.measure_plan(gui.read_measure_settings())
    assert (swings_alone, creeps_alone) == (4, 0)

    gui.m_stiction_var.set(1)
    swings_both, creeps_both = gui.measure_plan(gui.read_measure_settings())
    assert swings_both == swings_alone, "the hard-overs are shared, not doubled"
    assert creeps_both == 6
# def


def test_stiction_alone_still_plans_the_swings(rig):
    gui, _, _ = rig

    gui.rr_phase_vars["LEFT"].set(1)
    gui.rr_phase_vars["BOTH"].set(0)
    gui.rr_cycles_var.set("2")
    gui.m_range_rate_var.set(0)
    gui.m_stiction_var.set(1)

    swings, creeps = gui.measure_plan(gui.read_measure_settings())
    assert swings == 4 and creeps == 6
# def


def test_nothing_selected_plans_nothing(rig):
    gui, _, _ = rig

    gui.m_range_rate_var.set(0)
    gui.m_stiction_var.set(0)
    assert gui.measure_plan(gui.read_measure_settings()) == (0, 0)

    gui.update_measure_estimate()
    assert "Nothing selected" in gui.rr_estimate_label.cget("text")
# def


def test_measures_a_simulated_servo(rig):
    """The whole point: the tab must recover the rate the servo actually has."""
    gui, _, _ = rig

    run_test(gui, ["LEFT"], cycles=2)

    ok = [r for r in gui.rr_results if r["ok"]]
    assert len(ok) == 4, [r.get("reason") for r in gui.rr_results]

    for result in ok:
        assert result["side"] == "LEFT"
        assert abs(result["travel_deg"]) == pytest.approx(TRAVEL_DEG["LEFT"], abs=3.0)
        assert result["rate_deg_s"] == pytest.approx(SERVO["LEFT"]["rate_deg_s"], rel=0.15)
# def


def test_left_and_right_are_not_swapped(rig):
    """Each side has a distinct rate, so a mix-up cannot pass by symmetry."""
    gui, _, _ = rig

    run_test(gui, ["BOTH"], cycles=1)

    for side in ("LEFT", "RIGHT"):
        legs = [r for r in gui.rr_results if r["side"] == side and r["ok"]]
        assert legs, "no %s measurements" % side

        mean_rate = sum(r["rate_deg_s"] for r in legs) / len(legs)
        assert mean_rate == pytest.approx(SERVO[side]["rate_deg_s"], rel=0.15)
# def


def test_summary_table_is_populated(rig):
    gui, _, _ = rig

    run_test(gui, ["LEFT"], cycles=2)

    rows = gui.rr_tree.get_children()
    assert rows

    columns = gui.rr_tree.cget("columns")
    values = dict(zip(columns, gui.rr_tree.item(rows[0], "values")))

    assert values["phase"] == "LEFT"
    assert values["side"] == "LEFT"
    assert values["direction"] in ("neg_to_pos", "pos_to_neg")
    assert "+/-" in values["travel"], "mean +/- sd expected with more than one cycle"
    assert int(values["n"]) == 2
# def


def test_elevons_are_centred_when_the_test_ends(rig):
    """MAV_CMD_ACTUATOR_TEST holds for 60 s - leaving them hard over is not an option."""
    gui, drone, _ = rig

    run_test(gui, ["LEFT"], cycles=1)

    assert drone.commands[-1][1] == 0.0
    assert ("LEFT", 0.0) in drone.commands
    assert ("RIGHT", 0.0) in drone.commands
# def


def test_board_is_returned_to_slow_mode(rig):
    gui, _, pico = rig

    run_test(gui, ["LEFT"], cycles=1)

    assert pico.state["hz"] == MT.SLOW_RATE_HZ
    assert gui.position_reader.sample_hz == MT.SLOW_RATE_HZ
# def


def test_stopping_mid_run_still_centres(rig):
    gui, drone, _ = rig

    for phase, var in gui.rr_phase_vars.items():
        var.set(1 if phase == "LEFT" else 0)
    gui.rr_cycles_var.set("5")
    gui.rr_settle_var.set("0.6")

    gui.m_range_rate_var.set(1)
    gui.m_stiction_var.set(0)
    settings = gui.read_measure_settings()
    settings.update({"phases": ["LEFT"], "cycles": 5, "settle": 0.6,
                     "rate": TAB_RATE_HZ})

    gui.measure_active = True
    thread = threading.Thread(
        target=gui._measure_worker, args=(settings,), daemon=True)
    thread.start()

    gui.pump(1.5)
    gui.stop_measure()

    deadline = time.time() + 30.0
    while thread.is_alive() and time.time() < deadline:
        gui.pump(0.05)

    assert not thread.is_alive()
    assert drone.commands[-1][1] == 0.0
    assert not gui.measure_active
# def


def test_actuator_tick_yields_while_the_test_runs(rig):
    """The 100 ms slider tick must not fight the worker for the actuators.

    Not by going silent - a surface nobody commands is taken back by the FC
    after about 2 s - but by refusing to follow the slider while a run is on.
    A side the worker is not driving goes on being held where it already is.
    """
    gui, drone, _ = rig

    gui.left_pos.set(0.75)
    gui.measure_active = True
    gui._measure_sides = frozenset(("LEFT",))

    before = len(drone.commands)
    gui.update_actuators()
    gui.pump(0.35)
    during = [c for c in drone.commands[before:] if abs(c[1] - 0.75) < 1e-9]

    gui.measure_active = False
    gui._measure_sides = frozenset()

    assert during == [], "slider tick commanded actuators during the test"
# def


def test_trace_is_drawn(rig):
    gui, _, _ = rig

    run_test(gui, ["LEFT"], cycles=1)

    assert gui.rr_last_trace is not None

    # Stated as separate preconditions so a failure says which one broke. This
    # test used to assert only the canvas, and an empty canvas cannot tell a
    # layout problem from a leg that never moved.
    series, metrics = gui.rr_last_trace
    assert metrics["ok"], metrics.get("reason")

    angles = [a for _, a in series]
    assert max(angles) - min(angles) > 1.0, "the simulated surface did not move"

    # Tk assigns the canvas its real size on idle; redrawing before that has
    # happened plots into a 1-pixel box.
    gui.pump(0.2)
    gui.redraw_range_rate_trace()

    assert gui.rr_canvas.find_all(), "nothing drawn on the trace canvas"
# def


def test_export_writes_a_csv(rig, tmp_path, monkeypatch):
    gui, _, _ = rig

    run_test(gui, ["LEFT"], cycles=1)
    monkeypatch.setattr(MT, "APP_DIR", str(tmp_path))

    gui.drone_name_var.set("testbird")
    gui.export_range_rate_csv()

    written = list((tmp_path / "reports").glob("testbird_*_rangerate.csv"))
    assert len(written) == 1

    text = written[0].read_text()
    assert "phase,side,direction" in text
    assert "LEFT" in text
# def


def test_endpoints_are_reported_with_their_spread(rig, tmp_path, monkeypatch):
    """Max/min settled angle, each with its own sd - range alone hides a shift."""
    gui, _, _ = rig

    run_test(gui, ["LEFT"], cycles=2)

    row = gui.rr_summary_rows[0]
    angle_max, angle_min = row[3], row[4]

    # "+/-" present because 2 cycles give a defined sample stdev.
    assert "+/-" in angle_max and "+/-" in angle_min

    hi = float(angle_max.split()[0])
    lo = float(angle_min.split()[0])

    assert hi > lo
    assert hi - lo == pytest.approx(TRAVEL_DEG["LEFT"], abs=3.0)

    # The endpoints must bracket the reported range, not restate it.
    assert float(row[5]) == pytest.approx(hi - lo, abs=1.0)

    monkeypatch.setattr(MT, "APP_DIR", str(tmp_path))
    gui.drone_name_var.set("testbird")
    gui.export_range_rate_csv()

    written = list((tmp_path / "reports").glob("testbird_*_rangerate.csv"))
    header = written[0].read_text().splitlines()[0]

    assert header.split(",")[3:6] == ["angle_max_deg", "angle_min_deg", "range_deg"]
# def


def test_sample_export_writes_every_sample(rig, tmp_path, monkeypatch):
    """The raw material behind the summary, in the console tool's schema."""
    gui, _, _ = rig

    run_test(gui, ["LEFT"], cycles=2)
    monkeypatch.setattr(MT, "APP_DIR", str(tmp_path))

    assert gui.rr_samples_btn.cget("state") == "normal"

    gui.drone_name_var.set("testbird")
    gui.export_range_rate_samples_csv()

    written = list((tmp_path / "reports").glob("testbird_*_rangerate_samples.csv"))
    assert len(written) == 1

    lines = written[0].read_text().strip().splitlines()

    assert lines[0] == "phase,cycle,direction,side,t_rel_s,angle_deg"
    assert len(lines) - 1 == len(gui.rr_samples)

    # Two cycles x two legs, one side. Every leg must be represented.
    legs = {tuple(line.split(",")[1:4]) for line in lines[1:]}
    assert legs == {
        ("1", "neg_to_pos", "LEFT"), ("1", "pos_to_neg", "LEFT"),
        ("2", "neg_to_pos", "LEFT"), ("2", "pos_to_neg", "LEFT"),
    }

    # Pre-roll is captured before the command, so t_rel must go negative - that
    # is what gives analyse_leg a baseline to measure travel from.
    times = [float(line.split(",")[4]) for line in lines[1:]]
    assert min(times) < 0.0
    assert max(times) > 0.0
# def


def test_both_exports_share_one_filename_stem(rig, tmp_path, monkeypatch):
    """Summary and samples must pair up on disk however late either is saved."""
    gui, _, _ = rig

    run_test(gui, ["LEFT"], cycles=1)
    monkeypatch.setattr(MT, "APP_DIR", str(tmp_path))

    gui.drone_name_var.set("testbird")
    gui.rr_run_stamp = "20260101_120000"

    gui.export_range_rate_csv()
    gui.export_range_rate_samples_csv()

    names = sorted(p.name for p in (tmp_path / "reports").glob("*.csv"))

    assert names == ["testbird_20260101_120000_rangerate.csv",
                     "testbird_20260101_120000_rangerate_samples.csv"]
# def


def test_sample_retention_is_bounded(rig, monkeypatch):
    """A pathological run must not grow the buffer without limit."""
    monkeypatch.setattr(MT, "MAX_EXPORT_SAMPLES", 50)

    gui, _, _ = rig
    run_test(gui, ["LEFT"], cycles=1)

    assert len(gui.rr_samples) == 50
    assert gui.rr_samples_truncated
    assert "truncated" in gui.rr_note_label.cget("text")
# def


# ---- stiction ----

# Distinct per side, so a left/right mix-up in the new code cannot pass by
# symmetry. Both are far above the analysis noise and far below full travel.
STICTION_DEG = {"LEFT": 3.3, "RIGHT": 4.8}


def run_stiction(gui, phases, reps=2, cycles=2, step=0.1, period=0.02, points=5,
                 settle=0.4, range_rate=True, timeout=180.0):
    """Drive one stiction run synchronously, pumping Tk so callbacks land."""
    for phase, var in gui.rr_phase_vars.items():
        var.set(1 if phase in phases else 0)

    gui.rr_settle_var.set(str(settle))
    gui.st_reps_var.set(str(reps))
    gui.st_step_var.set(str(step))
    gui.st_period_var.set(str(period))
    gui.st_points_var.set(str(points))
    gui.m_range_rate_var.set(1 if range_rate else 0)
    gui.m_stiction_var.set(1)

    settings = gui.read_measure_settings()
    settings.update({"phases": list(phases), "creep_reps": reps,
                     "cycles": cycles, "settle": settle, "rate": TAB_RATE_HZ,
                     "creep_step": step, "creep_period": period,
                     "creep_points": points})

    gui.measure_active = True
    gui.rr_results = []
    gui.stiction_results = []
    gui.creep_points = []

    thread = threading.Thread(
        target=gui._measure_worker, args=(settings,), daemon=True)
    thread.start()

    deadline = time.time() + timeout
    while thread.is_alive() and time.time() < deadline:
        gui.pump(0.05)

    gui.pump(0.4)
    assert not thread.is_alive(), "measure worker did not finish"
# def


def test_stiction_recovers_a_known_creep_swing_gap(rig):
    """The point of the whole feature: a servo with 3.3 deg of stiction must
    read as 3.3 deg, not as noise and not as travel."""
    gui, drone, _ = rig
    drone.stiction_deg["LEFT"] = STICTION_DEG["LEFT"]

    run_stiction(gui, ["LEFT"], reps=2)

    creeps = [r for r in gui.stiction_results if r["kind"] == "creep" and r["ok"]]
    swings = [r for r in gui.rr_results if r["ok"]]
    assert len(creeps) == 4, "two reps at each end"
    assert len(swings) == 4, "the range/rate legs are the swings"

    arrival = {"neg_to_pos": 1.0, "pos_to_neg": -1.0}

    for target in (1.0, -1.0):
        stats = MT.stiction_stats(
            [r["final_deg"] for r in creeps if r["target"] == target],
            [r["final_deg"] for r in swings if arrival[r["direction"]] == target])
        assert abs(stats["stiction"]) == pytest.approx(STICTION_DEG["LEFT"], abs=1.2)
        # The swing always lands further out than the creep, whichever end.
        assert (stats["stiction"] > 0) == (target > 0)
# def


def test_a_servo_without_stiction_reads_near_zero(rig):
    """A measurement that invents stiction where there is none is worthless."""
    gui, drone, _ = rig
    assert drone.stiction_deg["LEFT"] == 0.0

    run_stiction(gui, ["LEFT"], reps=2)

    creeps = [r["final_deg"] for r in gui.stiction_results
              if r["kind"] == "creep" and r["ok"] and r["target"] == 1.0]
    swings = [r["final_deg"] for r in gui.rr_results
              if r["ok"] and r["direction"] == "neg_to_pos"]

    stats = MT.stiction_stats(creeps, swings)
    assert abs(stats["stiction"]) < 1.0
# def


def test_stiction_left_and_right_are_not_swapped(rig):
    """Each side has its own stiction, so a mix-up shows as a wrong number."""
    gui, drone, _ = rig
    drone.stiction_deg["LEFT"] = STICTION_DEG["LEFT"]
    drone.stiction_deg["RIGHT"] = STICTION_DEG["RIGHT"]

    run_stiction(gui, ["BOTH"], reps=2)

    for side in ("LEFT", "RIGHT"):
        creeps = [r["final_deg"] for r in gui.stiction_results
                  if r["kind"] == "creep" and r["ok"]
                  and r["side"] == side and r["target"] == 1.0]
        swings = [r["final_deg"] for r in gui.rr_results
                  if r["ok"] and r["side"] == side
                  and r["direction"] == "neg_to_pos"]

        stats = MT.stiction_stats(creeps, swings)
        assert stats["stiction"] == pytest.approx(STICTION_DEG[side], abs=1.2)
# def


def test_swings_report_a_rate_and_creeps_do_not(rig):
    """Rate only means something for a hard-over; a creep has no transit."""
    gui, drone, _ = rig
    drone.stiction_deg["LEFT"] = STICTION_DEG["LEFT"]

    run_stiction(gui, ["LEFT"], reps=2)

    for record in gui.stiction_results:
        assert record["kind"] == "creep"
        assert record["rate_deg_s"] is None

    for leg in gui.rr_results:
        if leg["ok"]:
            assert leg["rate_deg_s"] == pytest.approx(
                SERVO["LEFT"]["rate_deg_s"], rel=0.2)
# def


def test_the_two_measurements_share_one_set_of_hard_overs(rig):
    """The whole reason they were combined: selecting both must not double the
    swinging, and both summaries must quote the same legs."""
    gui, drone, _ = rig
    drone.stiction_deg["LEFT"] = STICTION_DEG["LEFT"]

    run_stiction(gui, ["LEFT"], reps=2, cycles=3, range_rate=True)

    # 3 cycles x 2 directions, and not one leg more.
    assert len([r for r in gui.rr_results if r["ok"]]) == 6
    assert all(r["kind"] == "creep" for r in gui.stiction_results)

    swing_cell = gui.st_tree.item(gui.st_tree.get_children()[0])["values"][4]
    assert swing_cell != "-"
# def


def test_stiction_alone_still_runs_the_swings_it_needs(rig):
    """Deselecting range and rate hides its table but cannot skip the swings -
    the comparison has no second half without them."""
    gui, drone, _ = rig
    drone.stiction_deg["LEFT"] = STICTION_DEG["LEFT"]

    run_stiction(gui, ["LEFT"], reps=2, cycles=2, range_rate=False)

    assert len([r for r in gui.rr_results if r["ok"]]) == 4
    assert gui.st_tree.get_children(), "stiction table populated"
    assert not gui.rr_tree.get_children(), "range/rate table left empty"
# def


def test_stiction_summary_table_is_populated(rig):
    gui, drone, _ = rig
    drone.stiction_deg["LEFT"] = STICTION_DEG["LEFT"]

    # The worker posts _measure_finish() itself; run_stiction pumps it through.
    run_stiction(gui, ["LEFT"], reps=2)

    rows = gui.st_tree.get_children()
    assert len(rows) == 2, "one row per end stop"

    values = gui.st_tree.item(rows[0])["values"]
    assert values[0] == "LEFT" and values[1] == "LEFT"
    # creep, swing and stiction all carry a spread from 2 repetitions.
    assert "+/-" in str(values[3]) and "+/-" in str(values[5])
# def


def test_stiction_run_centres_the_elevons(rig):
    gui, drone, _ = rig

    run_stiction(gui, ["LEFT"], reps=2)

    assert drone.commands[-1][1] == pytest.approx(0.0)
    assert not gui.measure_active
# def


def test_actuator_tick_yields_while_the_stiction_test_runs(rig):
    """The 10 Hz slider tick must not fight the test for the same actuators.

    A BOTH phase leaves the tick nothing to command, so it says nothing at all.
    Which sides are claimed is the worker's to publish, and it publishes the
    phase it is in - hence the claim set here rather than the flag alone.
    """
    gui, drone, _ = rig

    gui.measure_active = True
    gui._measure_sides = frozenset(("LEFT", "RIGHT"))
    try:
        before = len(drone.commands)
        gui.update_actuators()
        gui.pump(0.3)
        assert len(drone.commands) == before
    finally:
        gui.measure_active = False
        gui._measure_sides = frozenset()
# def


def test_a_single_sided_phase_leaves_the_other_surface_held_not_dropped(rig):
    """The tick owns whatever the phase does not, and must keep it alive.

    A LEFT phase runs for minutes. Dropping RIGHT for that long is not leaving
    it alone: the actuator override lapses after about 2 s and the FC parks the
    surface itself, which is motion, on the one frame both potentiometers are
    bolted to - arriving in the middle of LEFT's settle windows.

    So the tick has to go on commanding RIGHT the whole way through, and always
    at the value RIGHT is already at, which moves nothing.
    """
    gui, drone, _ = rig

    stamps = []
    real_command = drone.command_elevon

    def stamping_command(output_function, value, *args, **kwargs):
        stamps.append((time.monotonic(),
                       "LEFT" if output_function == 1201 else "RIGHT",
                       float(value)))
        return real_command(output_function, value, *args, **kwargs)

    drone.command_elevon = stamping_command

    # Where the tick will hold it: the slider is at centre and so is the
    # surface, which is what makes every refresh below a no-op physically.
    gui.right_pos.set(0.0)
    gui.pump(0.3)

    started = time.monotonic()
    run_test(gui, ["LEFT"], cycles=1)
    ended = time.monotonic()

    right = [(t, value) for t, side, value in stamps if side == "RIGHT"]
    assert right, "the idle side was never commanded at all"

    values = sorted({value for _t, value in right})
    assert values == [0.0], \
        "the idle side was commanded away from where it sat: %s" % values

    # The gap that matters is the physical one: MEASURE_REFRESH_S exists
    # because the override lapses somewhere around 2 s.
    times = [started] + [t for t, _value in right] + [ended]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert max(gaps) < 2.0, \
        "the idle side went uncommanded for %.2f s - the FC would have it" \
        % max(gaps)
# def


def test_creeps_run_the_full_span_from_the_opposite_end(rig):
    """Both directions must cover the same ground, or there is nothing to
    difference - creeping outward from centre overlaps nowhere."""
    gui, drone, _ = rig

    run_stiction(gui, ["LEFT"], reps=1, cycles=1, points=5)

    commands = [value for side, value in drone.commands if side == "LEFT"]
    assert min(commands) == pytest.approx(-1.0)
    assert max(commands) == pytest.approx(1.0)

    up = [p for p in gui.creep_points if p["direction"] > 0]
    down = [p for p in gui.creep_points if p["direction"] < 0]
    assert up and down

    # Each direction spans the travel, and they overlap across it.
    assert min(p["cmd"] for p in up) < 0 < max(p["cmd"] for p in up)
    assert min(p["cmd"] for p in down) < 0 < max(p["cmd"] for p in down)
# def


def test_curve_samples_land_on_the_grid_excluding_both_ends(rig):
    """The start was arrived at by a jump and the target by the arrival, so
    neither is a creep sample."""
    gui, _, _ = rig

    run_stiction(gui, ["LEFT"], reps=1, cycles=1, points=5)

    up = sorted(p["cmd"] for p in gui.creep_points if p["direction"] > 0)
    assert up == pytest.approx([-0.5, 0.0, 0.5])
    assert len(up) == len(MT.creep_grid(-1.0, 1.0, 5))
# def


def test_curve_samples_carry_a_pwm_axis(rig):
    """A command only means something against the min/max in force at the time,
    so the PWM is stored rather than reconstructed later."""
    gui, _, _ = rig

    run_stiction(gui, ["LEFT"], reps=1, cycles=1, points=5)

    pwm_min, pwm_max, trim = gui.read_side_param_snapshot("LEFT")
    rev = gui.is_main_channel_reversed(5)

    for point in gui.creep_points:
        assert point["pwm_us"] == gui.expected_pwm(
            point["cmd"], pwm_min, pwm_max, trim, rev)
        assert min(pwm_min, pwm_max) <= point["pwm_us"] <= max(pwm_min, pwm_max)
# def


def test_curve_samples_dwell_longer_than_the_averaging_window(rig):
    """Sampling on the move biases up and down sweeps opposite ways, which adds
    into the apparent band. The dwell is what prevents it."""
    assert MT.CREEP_SAMPLE_DWELL_S > MT.POSITION_WINDOW_S

    gui, _, _ = rig
    started = time.time()
    run_stiction(gui, ["LEFT"], reps=1, cycles=1, points=5, period=0.0)
    elapsed = time.time() - started

    samples = len([p for p in gui.creep_points if p["side"] == "LEFT"])
    assert samples == 6, "3 grid points each way"
    assert elapsed > samples * MT.CREEP_SAMPLE_DWELL_S
# def


def test_the_end_stop_comparison_is_unchanged_by_the_curve(rig):
    """Curve points are stored apart from the stiction results: a mid-travel
    position must never enter an end stop comparison."""
    gui, drone, _ = rig
    drone.stiction_deg["LEFT"] = STICTION_DEG["LEFT"]

    run_stiction(gui, ["LEFT"], reps=2, cycles=2, points=5)

    assert all(abs(r["target"]) == 1.0 for r in gui.stiction_results)
    assert all(r["kind"] == "creep" for r in gui.stiction_results)
    assert gui.creep_points, "and the curve was still captured"

    rows = gui.st_tree.get_children()
    assert len(rows) == 2
# def


def test_creep_curve_exports_every_point(rig, tmp_path, monkeypatch):
    gui, _, _ = rig
    monkeypatch.setattr(MT, "APP_DIR", str(tmp_path))

    run_stiction(gui, ["LEFT"], reps=1, cycles=1, points=5)
    assert gui.st_curve_btn.cget("state") == "normal"
    gui.export_creep_curve_csv()

    written = list((tmp_path / "reports").glob("*_creepcurve.csv"))
    assert len(written) == 1

    import csv as csv_module
    with open(written[0]) as handle:
        rows = list(csv_module.reader(handle))

    assert tuple(rows[0]) == MT.CREEP_CURVE_COLUMNS
    assert len(rows) - 1 == len(gui.creep_points)
    assert all(len(row) == len(MT.CREEP_CURVE_COLUMNS) for row in rows)
# def


def test_long_waits_keep_the_actuator_override_alive(rig):
    """The FC drops the override after ~2 s. A dwell that does not re-send is
    not a dwell - it is the FC quietly taking the surface back mid-measurement."""
    gui, drone, _ = rig

    before = len(drone.commands)
    gui._hold(["LEFT"], -1.0, MT.MEASURE_REFRESH_S * 4)
    sent = drone.commands[before:]

    assert len(sent) >= 3, "refreshed several times across the wait"
    assert all(side == "LEFT" and value == pytest.approx(-1.0)
               for side, value in sent), "and never moved the surface"
# def


def test_no_gap_longer_than_the_override_during_a_creep(rig):
    """The whole run, checked end to end: no silence long enough to lose it."""
    gui, drone, _ = rig

    stamps = []
    original = drone.command_elevon

    def timestamped(function, value, **kwargs):
        stamps.append(time.monotonic())
        return original(function, value, **kwargs)

    drone.command_elevon = timestamped
    try:
        run_stiction(gui, ["LEFT"], reps=1, cycles=1, points=5)
    finally:
        drone.command_elevon = original

    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    assert max(gaps) < 2.0, "longest silence was %.2f s" % max(gaps)
# def


def test_curve_plot_window_opens_and_draws(rig):
    gui, _, _ = rig

    run_stiction(gui, ["LEFT"], reps=1, cycles=1, points=5)
    assert gui.st_plot_btn.cget("state") == "normal"

    gui.show_creep_curve()
    gui.pump(0.4)

    assert gui.curve_window is not None and gui.curve_window.winfo_exists()
    assert gui.curve_canvas.find_all(), "something was drawn"
    assert "mean" in gui.curve_caption.cget("text")

    # Re-opening raises the existing window rather than stacking another.
    existing = gui.curve_window
    gui.show_creep_curve()
    gui.pump(0.2)
    assert gui.curve_window is existing

    gui.curve_window.destroy()
# def


def test_curve_plot_modes_label_their_axis(rig):
    gui, _, _ = rig
    run_stiction(gui, ["LEFT"], reps=1, cycles=1, points=5)
    gui.show_creep_curve()
    gui.pump(0.3)

    def axis_label():
        plot = gui.build_creep_plot(700, 400)
        return [t["text"] for t in plot["texts"]]

    assert "deviation deg" in axis_label(), "deviation is the default"

    gui.curve_mode_var.set(MT.curve_plot.MODE_CURVE)
    assert "angle deg" in axis_label()

    gui.curve_window.destroy()
# def


def test_curve_plot_exports_an_svg(rig, tmp_path, monkeypatch):
    gui, _, _ = rig
    monkeypatch.setattr(MT, "APP_DIR", str(tmp_path))

    run_stiction(gui, ["LEFT"], reps=1, cycles=1, points=5)
    gui.show_creep_curve()
    gui.pump(0.3)
    gui.export_creep_curve_svg()

    written = list((tmp_path / "reports").glob("*_creepcurve.svg"))
    assert len(written) == 1
    body = written[0].read_text()
    assert body.startswith("<?xml") and body.rstrip().endswith("</svg>")

    gui.curve_window.destroy()
# def
