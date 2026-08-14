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

    def command_elevon(self, output_function, value):
        side = "LEFT" if output_function == 1201 else "RIGHT"

        with self._lock:
            self.commands.append((side, float(value)))
            if abs(self._target[side] - float(value)) > 1e-9:
                self._target[side] = float(value)
                self._start[side] = self._position[side]
                self._commanded_at[side] = time.monotonic()
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
        args=(list(phases), cycles, settle, MT.FAST_RATE_HZ),
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

    assert phases == ["LEFT", "RIGHT", "BOTH"]
    assert cycles == 3
    assert rate == MT.FAST_RATE_HZ

    gui.update_range_rate_estimate()
    assert "18 hard-overs" in gui.rr_estimate_label.cget("text")
# def


def test_estimate_tracks_the_selection(rig):
    gui, _, _ = rig

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

    values = gui.rr_tree.item(rows[0], "values")
    assert values[0] == "LEFT"
    assert values[1] == "LEFT"
    assert values[2] in ("neg_to_pos", "pos_to_neg")
    assert "+/-" in values[4], "mean +/- sd expected with more than one cycle"
    assert int(values[7]) == 2
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
        target=gui._range_rate_worker, args=(["LEFT"], 5, 0.6, MT.FAST_RATE_HZ),
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
