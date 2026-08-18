"""Tests for the notebook shell and the persistent status strip.

Builds the real window against a fake Pico, so the wiring between the GUI, the
reader thread and the board is exercised rather than mocked. Skipped when no
display is available - these need a real Tk window, so run them under Xvfb:

    xvfb-run -a python3 -m pytest tests/test_gui_shell.py -v
"""

import csv
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

tk = pytest.importorskip("tkinter")

from fake_pico import FakePico

import cal_flow
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


@pytest.fixture
def app():
    """The real window, streaming from a fake Pico."""
    os.environ["MANTA_NO_ACTUATE"] = "1"

    pico = FakePico().start()
    root = tk.Tk()
    gui = MT.FourSliderGUI(root, MT.PositionReader(), MT.DroneInterface())
    gui.position_reader.set_port(pico.device)

    def pump(seconds):
        end = time.time() + seconds
        while time.time() < end:
            root.update()
            time.sleep(0.02)

    gui.pump = pump
    pump(1.2)

    try:
        yield gui, pico
    finally:
        try:
            gui.on_close()
        except Exception:
            pass
        pico.stop()
# def


def test_tabs(app):
    gui, _ = app

    tabs = [gui.notebook.tab(t, "text").strip() for t in gui.notebook.tabs()]

    assert tabs == ["Trim", "Measure"]
# def


def test_connection_widgets_live_outside_the_notebook(app):
    """Link state must stay visible whichever tab is selected."""
    gui, _ = app

    notebook_path = str(gui.notebook)

    for widget in (gui.drone_combo, gui.pico_combo, gui.pico_rate_label,
                   gui.connect_btn, gui.refresh_btn):
        assert not str(widget).startswith(notebook_path + "."), \
            "%s is inside the notebook" % widget
# def


def test_instrumentation_lives_outside_the_notebook(app):
    """Per-tab logs would put the interesting line on the tab you are not on."""
    gui, _ = app

    assert not str(gui.log_text).startswith(str(gui.notebook) + ".")
# def


def test_the_flow_charts_live_outside_the_notebook_and_above_the_log(app):
    """A run you cannot see because you are on the other tab is one you cannot
    follow, which is the same argument the log itself is placed on."""
    gui, _ = app

    notebook_path = str(gui.notebook)
    for side in ("LEFT", "RIGHT"):
        assert not str(gui.cal_canvas[side]).startswith(notebook_path + ".")

    gui.pump(0.2)
    assert gui.cal_canvas["LEFT"].winfo_rooty() < gui.log_text.winfo_rooty()


def test_each_side_gets_its_own_canvas(app):
    """Both run at once. One shared chart could not show two runs."""
    gui, _ = app

    assert set(gui.cal_canvas) == {"LEFT", "RIGHT"}
    assert str(gui.cal_canvas["LEFT"]) != str(gui.cal_canvas["RIGHT"])
    assert set(gui.cal_detail) == {"LEFT", "RIGHT"}


def test_the_charts_keep_their_width_on_a_smaller_screen(app):
    """pack hands out space in pack order, so whoever is packed first takes its
    full request. With the notebook first this column was left 152 px of the
    650 it asks for, and a Canvas clips without saying anything."""
    gui, _ = app

    gui.root.geometry("1280x900")
    gui.pump(0.3)

    for side in ("LEFT", "RIGHT"):
        width = gui.cal_canvas[side].winfo_width()
        assert width >= gui.CAL_CANVAS_W, \
            "%s chart clipped to %d px" % (side, width)


def test_a_failed_side_draws_red_and_leaves_the_other_alone(app):
    gui, _ = app

    gui.cal_flow["RIGHT"].reset(0.0)
    gui.cal_flow["RIGHT"].fail("travel_min", "free_travel_unconfirmed", 5.0,
                               ceiling=200.0, threshold=0.25)
    gui.draw_calibration_flow("RIGHT", gui.cal_flow["RIGHT"].snapshot(5.0))
    gui.pump(0.2)

    assert gui.cal_detail["RIGHT"].cget("fg") == MT.PALETTE["bad"]
    assert "Stiction" in gui.cal_detail["RIGHT"].cget("text")
    assert "Min free travel" in gui.cal_detail["RIGHT"].cget("text"), \
        "which end failed is not in the chart column any more"

    assert gui.cal_detail["LEFT"].cget("text") == ""


def test_a_stopped_side_says_so_without_going_red(app):
    gui, _ = app

    flow = gui.cal_flow["LEFT"]
    flow.reset(0.0)
    flow.start("sweep")
    flow.stop(3.0)
    gui.draw_calibration_flow("LEFT", flow.snapshot(3.0))
    gui.pump(0.2)

    assert gui.cal_detail["LEFT"].cget("text") == ""
    assert flow.verdict == cal_flow.STOPPED


def test_drawing_after_the_window_closes_is_a_no_op(app):
    """Workers outlive on_close - its join gives up after 2 s and a probe takes
    about 16. Anything they post must find a shut door, not a dead widget."""
    gui, _ = app
    snapshot = gui.cal_flow["LEFT"].snapshot(0.0)

    gui._closing = True
    gui.draw_calibration_flow("LEFT", snapshot)


def test_rate_indicator_tracks_the_board(app):
    """Fast mode is flagged, because a test can leave the board in it."""
    gui, _ = app

    assert gui.pico_rate_label.cget("text") == "10 Hz"
    assert gui.pico_rate_label.cget("fg") == MT.PALETTE["ok"]

    gui.position_reader.set_sample_rate(MT.FAST_RATE_HZ)
    gui.pump(0.5)

    assert gui.pico_rate_label.cget("text") == "%d Hz" % MT.FAST_RATE_HZ
    assert gui.pico_rate_label.cget("fg") == MT.PALETTE["warn"]
# def


def test_angle_readout_survives_the_switch_to_fast(app):
    """The regression the rate-aware backlog prevents."""
    gui, _ = app

    assert gui.left_label.cget("text") != "Left: --"

    gui.position_reader.set_sample_rate(MT.FAST_RATE_HZ)
    gui.pump(0.8)

    assert gui.left_label.cget("text") != "Left: --"
    assert gui.right_label.cget("text") != "Right: --"
# def


def test_closing_hands_the_board_back_in_slow_mode(app):
    gui, pico = app

    gui.position_reader.set_sample_rate(MT.FAST_RATE_HZ)
    gui.pump(0.3)
    assert pico.state["hz"] == MT.FAST_RATE_HZ

    gui.on_close()

    deadline = time.time() + 3.0
    while time.time() < deadline and pico.state["hz"] != MT.SLOW_RATE_HZ:
        time.sleep(0.05)

    assert pico.state["hz"] == MT.SLOW_RATE_HZ
# def


def test_status_labels_are_not_doubly_prefixed(app):
    """The row already says "Drone" - the status must not repeat it."""
    gui, _ = app

    gui.set_conn_status("drone", "connected (system 1)", True)
    gui.set_conn_status("pico", "streaming on /dev/ttyACM1", True)

    assert not gui.drone_status_label.cget("text").startswith("Drone:")
    assert not gui.pico_status_label.cget("text").startswith("Pico:")
# def


def test_status_labels_fit_the_longest_message(app):
    """Fixed-width labels stop the strip reflowing; they must not clip either."""
    gui, _ = app

    longest = {
        gui.drone_status_label: "no heartbeat on /dev/ttyACM0",
        gui.pico_status_label: "streaming on /dev/ttyACM0",
    }

    for label, text in longest.items():
        assert len(text) <= label.cget("width"), \
            "%r (%d chars) will clip at width %d" % (text, len(text), label.cget("width"))
# def


def _write_log(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
# def


OLD_COLUMNS = MT.CAL_LOG_COLUMNS[:-2]
OLD_ROW = ["03/04/2026", "11:57:54", "SN020", "3.69E+18", "-33", "35", "-5",
           "1170", "1902", "-0.25", "1092", "1848", "-0.22", "y"]


def test_an_older_log_is_widened_to_the_current_columns(app, tmp_path, capsys):
    """The backlash columns were added after this file had real rows in it.
    Appending wider rows under a narrower header is how a log stops being
    readable, so the file is brought up to date once, keeping the original."""
    gui, _pico = app
    log = tmp_path / "calibration_log.csv"
    gui.calibration_log_file = str(log)
    _write_log(log, OLD_COLUMNS, [OLD_ROW, OLD_ROW])

    assert gui.migrate_calibration_log() is True

    with open(log, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))

    assert rows[0] == MT.CAL_LOG_COLUMNS
    assert len(rows) == 3
    assert rows[1] == OLD_ROW + ["", ""], "padded, not guessed at"
    assert os.path.exists(str(log) + ".bak"), "the original is kept"
    assert "Added 2 columns" in capsys.readouterr().out


def test_a_current_log_is_left_alone(app, tmp_path):
    gui, _pico = app
    log = tmp_path / "calibration_log.csv"
    gui.calibration_log_file = str(log)
    _write_log(log, MT.CAL_LOG_COLUMNS, [OLD_ROW + ["-0.80", "-0.74"]])
    before = log.read_text()

    assert gui.migrate_calibration_log() is True
    assert log.read_text() == before
    assert not os.path.exists(str(log) + ".bak"), "nothing to migrate, nothing copied"


def test_a_log_this_version_does_not_recognise_is_refused(app, tmp_path, capsys):
    """Only ever widen a header this file is already a prefix of. Anything else
    is a file this code did not write, and guessing at its columns would
    corrupt it."""
    gui, _pico = app
    log = tmp_path / "calibration_log.csv"
    gui.calibration_log_file = str(log)
    _write_log(log, ["when", "what", "who"], [["a", "b", "c"]])
    before = log.read_text()

    assert gui.migrate_calibration_log() is False
    assert log.read_text() == before
    assert "not one this version recognises" in capsys.readouterr().out


def test_an_unmeasured_backlash_is_logged_blank(app):
    """Blank rather than 0.0: never measured and measured as zero are not the
    same claim."""
    gui, _pico = app

    assert gui.backlash_for_log("LEFT") == ""
    gui.set_side_backlash("LEFT", -0.8)
    assert gui.backlash_for_log("LEFT") == "-0.80"
    assert gui.backlash_for_log("RIGHT") == ""


@pytest.fixture
def gated(app, tmp_path):
    """The window, with its settings file redirected - zeroing writes to disk."""
    gui, pico = app
    gui.position_reader.calibration_file = str(tmp_path / "settings.json")
    gui.position_reader.backup_file = gui.position_reader.calibration_file + ".bak"
    return gui, pico


def _gated_buttons(gui):
    return (gui.auto_both_btn, gui.left_cal_btn, gui.right_cal_btn,
            gui.sweep_btn, gui.rr_run_btn)


def _strip(gui):
    return [str(gui.setup_item_labels[key]["text"]) for key, _ in MT.SETUP_ITEMS]


def _complete_setup(gui):
    gui.drone_name_var.set("manta-07")
    gui.apply_drone_name()
    gui.zero_both_angles()


def test_the_gate_starts_closed(gated):
    """Nothing that drives the surfaces is reachable on an unset-up drone."""
    gui, _pico = gated

    assert gui.setup_blockers() == [
        "SET DRONE NAME", "ZERO LEFT ELEVON", "ZERO RIGHT ELEVON"]
    assert all(str(b["state"]) == "disabled" for b in _gated_buttons(gui))


def test_naming_and_zeroing_opens_the_gate(gated):
    gui, _pico = gated

    _complete_setup(gui)

    assert gui.is_setup_complete()
    assert all(str(b["state"]) == "normal" for b in _gated_buttons(gui))


def test_a_name_alone_is_not_enough(gated):
    gui, _pico = gated

    gui.drone_name_var.set("manta-07")
    gui.apply_drone_name()

    assert gui.setup_blockers() == ["ZERO LEFT ELEVON", "ZERO RIGHT ELEVON"]
    assert all(str(b["state"]) == "disabled" for b in _gated_buttons(gui))


def test_zeroing_alone_is_not_enough(gated):
    gui, _pico = gated

    gui.zero_both_angles()

    assert gui.setup_blockers() == ["SET DRONE NAME"]
    assert all(str(b["state"]) == "disabled" for b in _gated_buttons(gui))


def test_a_blank_name_does_not_count_as_set(gated):
    """It is the state the connect reset leaves behind."""
    gui, _pico = gated

    gui.drone_name_var.set("   ")
    gui.apply_drone_name()

    assert not gui.name_is_set


def test_connecting_a_drone_closes_the_gate_again(gated):
    gui, _pico = gated

    _complete_setup(gui)
    gui.reset_setup_state()

    assert gui.drone_name_var.get() == ""
    assert gui.position_reader.drone_name == ""
    assert gui.setup_blockers() == [
        "SET DRONE NAME", "ZERO LEFT ELEVON", "ZERO RIGHT ELEVON"]
    assert all(str(b["state"]) == "disabled" for b in _gated_buttons(gui))
    assert _strip(gui) == ["[ ] SET DRONE NAME", "[ ] ZERO LEFT ELEVON",
                           "[ ] ZERO RIGHT ELEVON"]


def test_connecting_a_drone_drops_the_previous_centring(gated):
    """The offsets on file were measured against whatever flew here last."""
    gui, _pico = gated

    gui.zero_both_angles()
    gui.position_reader.clear_center()

    assert gui.position_reader.left_offset == 0.0
    assert gui.position_reader.right_offset == 0.0


def test_a_running_side_cannot_be_started_again(gated):
    """refresh_setup_gate re-enables everything unconditionally and is called
    from apply_drone_name and reset_setup_state, either of which can fire in
    the middle of a run - so the starter cannot own the button state."""
    gui, _pico = gated
    _complete_setup(gui)

    gui.left_cal_active = True
    gui.refresh_setup_gate()

    assert str(gui.left_cal_btn["state"]) == "disabled"
    assert str(gui.auto_both_btn["state"]) == "disabled"
    assert str(gui.right_cal_btn["state"]) == "normal", "the other side is free"

    gui.left_cal_active = False
    gui.refresh_setup_gate()
    assert str(gui.left_cal_btn["state"]) == "normal"


def test_naming_the_drone_mid_run_does_not_reopen_the_button(gated):
    gui, _pico = gated
    _complete_setup(gui)
    gui.right_cal_active = True

    gui.apply_drone_name()

    assert str(gui.right_cal_btn["state"]) == "disabled"
    gui.right_cal_active = False


def test_a_closed_gate_refuses_to_run(gated, monkeypatch):
    gui, _pico = gated

    warnings = []
    monkeypatch.setattr(
        MT.messagebox, "showwarning", lambda title, msg: warnings.append(msg))

    gui.start_both_calibration()
    gui.start_left_calibration()
    gui.start_right_calibration()
    gui.start_sweep_to_csv()
    gui.start_measure()

    assert len(warnings) == 5
    assert all("SET DRONE NAME" in w for w in warnings)
    assert not gui.left_cal_active
    assert not gui.right_cal_active
    assert not gui.sweep_active
    assert not gui.measure_active


def test_the_strip_starts_with_all_three_flagged(gated):
    gui, _pico = gated

    assert str(gui.setup_state_label["text"]) == "Not ready"
    assert _strip(gui) == ["[ ] SET DRONE NAME", "[ ] ZERO LEFT ELEVON",
                           "[ ] ZERO RIGHT ELEVON"]
    assert all(str(gui.setup_item_labels[key]["fg"]) == MT.PALETTE["bad"]
               for key, _ in MT.SETUP_ITEMS)


def test_the_strip_ticks_one_item_at_a_time(gated):
    gui, _pico = gated

    gui.drone_name_var.set("manta-07")
    gui.apply_drone_name()

    assert _strip(gui) == ["[x] SET DRONE NAME", "[ ] ZERO LEFT ELEVON",
                           "[ ] ZERO RIGHT ELEVON"]

    gui.centre_left()

    assert _strip(gui) == ["[x] SET DRONE NAME", "[x] ZERO LEFT ELEVON",
                           "[ ] ZERO RIGHT ELEVON"]
    assert str(gui.setup_state_label["text"]) == "Not ready"


def test_the_strip_reads_ready_once_all_three_are_done(gated):
    gui, _pico = gated

    _complete_setup(gui)

    assert _strip(gui) == ["[x] SET DRONE NAME", "[x] ZERO LEFT ELEVON",
                           "[x] ZERO RIGHT ELEVON"]
    assert str(gui.setup_state_label["text"]) == "Ready"
    assert str(gui.setup_state_label["fg"]) == MT.PALETTE["ok"]
    assert all(str(gui.setup_item_labels[key]["fg"]) == MT.PALETTE["ok"]
               for key, _ in MT.SETUP_ITEMS)


def test_the_strip_cannot_jitter_as_items_tick(gated):
    """Fixed widths, or ticking one item shunts the rest along the strip."""
    gui, _pico = gated

    longest = max(len("[x] %s" % text) for _key, text in MT.SETUP_ITEMS)

    for key, _text in MT.SETUP_ITEMS:
        assert int(gui.setup_item_labels[key]["width"]) == longest

    assert int(gui.setup_state_label["width"]) >= len("Not ready")
