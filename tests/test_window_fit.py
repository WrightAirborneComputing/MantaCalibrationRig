"""Tests for making the window fit the screen it is on.

The window is laid out in points and the screen is not, so a 1080p laptop at
125% display scaling was showing a window a good hundred pixels taller than it
had room for. The bottom of the Trim tab - the angle readout and the "Zero
angle" button - was simply not on the screen, and nothing scrolled, so there
was no way to reach it.

Two mechanisms, and both are pinned here:

  * the position sliders are measured in pixels, so they are the give in the
    layout - shrunk only as far as the screen demands, never past the floor;
  * whatever is still over, scrolls.

Needs a real Tk window, so run under Xvfb:

    xvfb-run -a python3 -m pytest tests/test_window_fit.py -v
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

tk = pytest.importorskip("tkinter")

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
    """The window, with no hardware behind it. Nothing here needs the rig."""
    os.environ["MANTA_NO_ACTUATE"] = "1"

    root = tk.Tk()
    gui = MT.FourSliderGUI(root, MT.PositionReader(), MT.DroneInterface())

    def pump(times=20):
        for _ in range(times):
            root.update()
            root.update_idletasks()
    # def

    gui.pump = pump
    pump()

    try:
        yield gui
    finally:
        gui.position_reader.stop()
        root.destroy()
# def


def refit(gui, avail_w, avail_h):
    """Re-run the fit as though the screen allowed exactly this much."""
    gui.available_work_area = lambda: (avail_w, avail_h)
    length = gui.fit_to_screen()
    gui.pump()
    return length
# def


def widget_by_text(widget, text):
    for child in widget.winfo_children():
        try:
            if child.cget("text") == text:
                return child
        except tk.TclError:
            pass
        found = widget_by_text(child, text)
        if found is not None:
            return found
    return None
# def


def bottom_of(gui, widget):
    """Distance from the top of the window to the bottom of the widget."""
    offset = widget.winfo_rooty() - gui.root.winfo_rooty()
    return offset + widget.winfo_height()
# def


def test_room_to_spare_leaves_the_sliders_alone(app):
    """The bench machine must get the window it has always had."""
    length = refit(app, 3000, 3000)

    assert length == MT.SLIDER_LENGTH_MAX
    assert app.left_pos.cget("length") == MT.SLIDER_LENGTH_MAX
    assert app._scroll_bars_shown == (False, False)
# def


def test_the_zero_angle_button_is_on_a_1080p_screen(app):
    """The bug, in one assertion. 1050 is a 1080p work area, taskbar deducted."""
    refit(app, 1905, 1050)

    button = widget_by_text(app.root, "Zero angle")
    assert button is not None
    assert bottom_of(app, button) <= app.root.winfo_height()
    assert bottom_of(app, app.left_label) <= app.root.winfo_height()
# def


def test_sliders_give_up_only_what_the_screen_demands(app):
    """As large as will fit: a tighter screen may take more, never less."""
    roomy = refit(app, 1905, 1050)
    tight = refit(app, 1905, 900)

    assert tight < roomy <= MT.SLIDER_LENGTH_MAX
    assert tight >= MT.SLIDER_LENGTH_MIN
# def


def test_the_slider_has_a_floor_and_the_window_scrolls_instead(app):
    """Below the floor a slider is not a control, so the body scrolls."""
    refit(app, 1000, 600)

    assert app.left_pos.cget("length") == MT.SLIDER_LENGTH_MIN
    assert app._scroll_bars_shown[1] is True
# def


def test_scrolling_reaches_the_bottom_of_the_tab(app):
    refit(app, 1000, 600)

    button = widget_by_text(app.root, "Zero angle")
    assert bottom_of(app, button) > app.root.winfo_height(), "not clipped to begin with"

    app._scroll_canvas.yview_moveto(1.0)
    app.pump()

    offset = button.winfo_rooty() - app.root.winfo_rooty()
    assert 0 <= offset <= app.root.winfo_height()
# def


def test_bars_go_away_again_when_the_window_grows(app):
    """The deadlock this had first time: a bar takes width out of the canvas,
    which is then narrow enough to justify the bar that caused it."""
    refit(app, 1905, 1050)
    width, height = app.root.winfo_width(), app.root.winfo_height()

    app.root.geometry("1100x620")
    app.pump()
    assert app._scroll_bars_shown == (True, True)

    app.root.geometry("%dx%d" % (width, height))
    app.pump(40)
    assert app._scroll_bars_shown == (False, False)
# def


def test_geometry_never_exceeds_the_work_area(app):
    refit(app, 1265, 770)

    assert app.root.winfo_width() <= 1265
    assert app.root.winfo_height() <= 770
# def


class FakeWheel:
    def __init__(self, widget, num=5, delta=0):
        self.widget = widget
        self.num = num
        self.delta = delta
    # def
# class


def test_the_wheel_leaves_the_log_and_the_tables_alone(app):
    """Both scroll themselves. Taking the wheel off them to move the window
    would make either one unreadable."""
    refit(app, 1000, 600)
    app._scroll_canvas.yview_moveto(0.0)
    app.pump()

    for widget in (app.log_text, app.rr_tree, app.st_tree):
        before = app._scroll_canvas.yview()[0]
        app._on_wheel(FakeWheel(widget))
        app.pump(2)
        assert app._scroll_canvas.yview()[0] == before, widget.winfo_class()
# def


def test_the_wheel_scrolls_the_body_everywhere_else(app):
    refit(app, 1000, 600)
    app._scroll_canvas.yview_moveto(0.0)
    app.pump()

    # Over the canvas itself, and over an ordinary widget deep inside it.
    for widget in (app._scroll_canvas, app.left_pwm_label):
        app._scroll_canvas.yview_moveto(0.0)
        app.pump(2)
        before = app._scroll_canvas.yview()[0]

        app._on_wheel(FakeWheel(widget))
        app.pump(2)

        assert app._scroll_canvas.yview()[0] > before, widget.winfo_class()
# def


def test_the_wheel_is_actually_wired_up(app):
    """Not _on_wheel() called by hand: a real event, through the real binding.

    Tk delivers a wheel event to the widget under the pointer and does not pass
    it up to that widget's ancestors, so a binding on the canvas would never
    see one. This is the test that would have caught that.
    """
    refit(app, 1000, 600)
    app._scroll_canvas.yview_moveto(0.0)
    app.pump()

    before = app._scroll_canvas.yview()[0]
    app.left_pwm_label.event_generate("<Button-5>", when="now")
    app.pump(2)

    assert app._scroll_canvas.yview()[0] > before
# def


def test_the_wheel_in_another_window_is_not_ours(app):
    """The binding is global, so the plot window's wheel arrives here too."""
    refit(app, 1000, 600)
    app._scroll_canvas.yview_moveto(0.0)
    app.pump()

    other = tk.Toplevel(app.root)
    label = tk.Label(other, text="a plot would go here")
    label.pack()
    app.pump(2)

    before = app._scroll_canvas.yview()[0]
    app._on_wheel(FakeWheel(label))
    app.pump(2)

    assert app._scroll_canvas.yview()[0] == before
    other.destroy()
# def


def test_the_status_strip_does_not_scroll_away(app):
    """The port pickers are what you reach for when the rig is misbehaving."""
    refit(app, 1000, 600)
    app._scroll_canvas.yview_moveto(1.0)
    app.pump()

    top = app.pico_combo.winfo_rooty() - app.root.winfo_rooty()
    assert 0 <= top <= app.root.winfo_height()
# def
