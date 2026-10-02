"""Can the real window be driven by a sensor set that is not the Pico?

This is the gate SENSORS.md puts before any Teensy work, and it is the only
test here that needs a display. Every previous test that faked a sensor did it
by monkeypatching `get_side_angle()`, which proves things about the code above
the seam and nothing about the seam itself. These build the actual window on a
`FakeDriver` - no serial port, no counts, no scaler - and drive it.

What is really being asserted is a negative: that nothing above the driver had
to be told which driver it has. If any of these ever needs a special case to
pass, the contract is wrong, and finding that out here costs an afternoon where
finding it out with a Teensy on the bench costs a week.

    xvfb-run -a python3 -m pytest tests/test_driver_seam.py -v
"""

import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

tk = pytest.importorskip("tkinter")

import sensor_driver as SD
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
    """The real window on a fake sensor set. No pty, no board, no counts.

    The PositionReader is still handed over because it owns settings.json,
    which is a separate concern from reading angles and a separate step of the
    migration. It is given no port, so if anything in here were still reading
    through it the angles would come back None and these tests would fail -
    which makes its presence a control rather than a loophole.
    """
    os.environ["MANTA_NO_ACTUATE"] = "1"

    driver = SD.FakeDriver()
    driver.start()

    root = tk.Tk()
    gui = MT.FourSliderGUI(root, MT.PositionReader(), MT.DroneInterface(),
                           sensor=driver)

    def pump(seconds):
        end = time.time() + seconds
        while time.time() < end:
            root.update()
            time.sleep(0.02)
    # def

    gui.pump = pump

    try:
        yield gui, driver
    finally:
        try:
            gui.on_close()
        except Exception:
            pass
        driver.stop()
# def


def test_the_window_reads_angles_from_a_driver_that_is_not_the_pico(app):
    gui, driver = app

    driver.set_angle("LEFT", 7.25)
    driver.set_angle("RIGHT", -4.75)

    assert gui.get_side_angle("LEFT") == pytest.approx(7.25)
    assert gui.get_side_angle("RIGHT") == pytest.approx(-4.75)
# def


def test_get_side_angle_is_still_an_ordinary_method(app):
    """The single constraint the whole migration is verified against.

    The existing suite fakes sensors by assigning over this on the instance,
    which only works while it is a plain method - not a property, not an
    attribute set in __init__. If this ever fails, dozens of calibration tests
    stop testing what they claim to.
    """
    gui, _ = app

    assert "get_side_angle" not in gui.__dict__
    assert callable(gui.get_side_angle)
    assert not isinstance(type(gui).get_side_angle, property)

    gui.get_side_angle = lambda side: 11.0
    assert gui.get_side_angle("LEFT") == 11.0
# def


def test_the_arbiters_read_callback_follows_the_driver(app):
    """rig_sync reads through the seam, so it must move with the driver too.

    Reaching for the private _read is deliberate: the callback the arbiter was
    constructed with is the thing under test, not the locking around it.
    """
    gui, driver = app

    driver.set_angle("LEFT", 2.0)
    assert gui.rig_arbiter._read("LEFT") == pytest.approx(2.0)

    driver.set_angle("LEFT", -8.5)
    assert gui.rig_arbiter._read("LEFT") == pytest.approx(-8.5)
# def


def test_a_stale_driver_reads_none_rather_than_a_stale_number(app):
    """A dead link must not present the last good angle as a current one."""
    gui, driver = app

    driver.set_angle("LEFT", 3.0)
    assert gui.get_side_angle("LEFT") is not None

    driver.go_stale()

    assert gui.get_side_angle("LEFT") is None
    assert gui.get_side_angle("RIGHT") is None
# def


def test_the_label_tick_survives_a_driver_with_nothing_to_say(app):
    """The GUI polls on a timer, so None has to be ordinary rather than fatal."""
    gui, driver = app

    driver.go_stale()
    gui.pump(0.4)

    driver.set_angle("LEFT", 1.5)
    driver.set_angle("RIGHT", 1.5)
    gui.pump(0.4)

    assert gui.get_side_angle("LEFT") == pytest.approx(1.5)
# def


def test_a_third_channel_changes_nothing_above_the_seam(app):
    """CENTRE is a channel and never a side.

    The fake offers LEFT, CENTRE and RIGHT, as the inclinometer rig will. The
    elevon code above must keep asking about two sides and must never be handed
    the third - which is the claim that lets the dozens of `if side == "LEFT"`
    branches stay exactly where they are.
    """
    gui, driver = app

    assert "CENTRE" in driver.channels()

    driver.set_angle("CENTRE", 30.0)
    driver.set_angle("LEFT", 1.0)
    driver.set_angle("RIGHT", -1.0)

    assert gui.get_side_angle("LEFT") == pytest.approx(1.0)
    assert gui.get_side_angle("RIGHT") == pytest.approx(-1.0)
    assert gui.get_side_angle("CENTRE") is None, "CENTRE is not a side"
# def


def test_nothing_above_the_seam_reaches_for_the_reader_to_get_an_angle(app):
    """The reader has no port, so any surviving path through it reads None.

    A structural check rather than a behavioural one: it fails if someone
    reintroduces a direct `position_reader.get_average_position_nonblocking()`
    call on the angle path, which is the exact regression this layer exists to
    prevent.
    """
    gui, driver = app

    assert not gui.position_reader.connected
    assert gui.position_reader.get_average_position_nonblocking("LEFT") is None

    driver.set_angle("LEFT", 9.0)
    assert gui.get_side_angle("LEFT") == pytest.approx(9.0)
# def
