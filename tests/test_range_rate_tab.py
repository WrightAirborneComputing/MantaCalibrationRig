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

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._integrate, daemon=True)
        self._thread.start()
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

                    if elapsed <= 0:
                        position = self._start[side]
                    else:
                        span = self._target[side] - self._start[side]
                        step = units_s * elapsed
                        if abs(span) <= step:
                            position = self._target[side]
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
        gui.range_test_active = False
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

    gui.range_test_active = True
    gui.rr_run_btn.config(state="disabled")
    gui.rr_stop_btn.config(state="normal")

    thread = threading.Thread(
        target=gui._range_rate_worker,
        args=(list(phases), cycles, settle, TAB_RATE_HZ),
        daemon=True,
    )
    thread.start()

    deadline = time.time() + timeout
    while thread.is_alive() and time.time() < deadline:
        gui.pump(0.05)

    gui.pump(0.4)
    assert not thread.is_alive(), "worker did not finish"
# def


def test_setup_defaults_and_estimate(rig):
    gui, _, _ = rig

    phases, cycles, settle, rate = gui.read_range_rate_settings()

    # BOTH alone: the single-servo phases were measured to add nothing, and
    # dropping them is what makes 30 cycles affordable.
    assert phases == list(MT.DEFAULT_PHASES)
    assert cycles == MT.DEFAULT_CYCLES
    assert rate == MT.FAST_RATE_HZ

    gui.update_range_rate_estimate()
    assert "60 hard-overs" in gui.rr_estimate_label.cget("text")
# def


def test_estimate_tracks_the_selection(rig):
    gui, _, _ = rig

    gui.rr_phase_vars["LEFT"].set(1)
    gui.rr_phase_vars["RIGHT"].set(0)
    gui.rr_phase_vars["BOTH"].set(0)
    gui.rr_cycles_var.set("2")
    gui.update_range_rate_estimate()

    assert "4 hard-overs" in gui.rr_estimate_label.cget("text")
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

    gui.range_test_active = True
    thread = threading.Thread(
        target=gui._range_rate_worker, args=(["LEFT"], 5, 0.6, TAB_RATE_HZ),
        daemon=True)
    thread.start()

    gui.pump(1.5)
    gui.stop_range_rate_test()

    deadline = time.time() + 30.0
    while thread.is_alive() and time.time() < deadline:
        gui.pump(0.05)

    assert not thread.is_alive()
    assert drone.commands[-1][1] == 0.0
    assert not gui.range_test_active
# def


def test_actuator_tick_yields_while_the_test_runs(rig):
    """The 100 ms slider tick must not fight the worker for the actuators."""
    gui, drone, _ = rig

    gui.left_pos.set(0.75)
    gui.range_test_active = True

    before = len(drone.commands)
    gui.update_actuators()
    gui.pump(0.35)
    during = [c for c in drone.commands[before:] if abs(c[1] - 0.75) < 1e-9]

    gui.range_test_active = False

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
