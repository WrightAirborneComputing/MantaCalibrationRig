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
