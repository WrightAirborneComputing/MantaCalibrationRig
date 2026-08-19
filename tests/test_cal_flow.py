"""Tests for the calibration flow model. No hardware, no display.

Everything here is about what the chart says, which is why none of it needs a
window: cal_flow decides node states and wording, and the drawing is somebody
else's problem.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cal_flow as F


def _states(flow, now=0.0):
    _verdict, _elapsed, rows, _failure = flow.snapshot(now)
    return {key: state for key, _l, _r, _c, state, _n in rows}


def test_a_fresh_side_is_idle_with_everything_pending():
    flow = F.SideFlow()

    verdict, elapsed, rows, failure = flow.snapshot(12.0)

    assert verdict == F.IDLE
    assert elapsed is None, "an idle side has not been running for 12 seconds"
    assert failure is None
    assert len(rows) == len(F.NODES)
    assert all(state == F.PENDING for _k, _l, _r, _c, state, _n in rows)


def test_the_graph_forks_at_the_ends_and_rejoins():
    """The two ends are worked at once, which a single column cannot show."""
    columns = {key: column for key, _l, _r, column in F.NODES}

    assert columns["find_max"] == "left" and columns["find_min"] == "right"
    assert columns["travel_max"] == "left" and columns["travel_min"] == "right"
    assert columns["load_coarse"] == "full" and columns["verify"] == "full"

    assert ("load_coarse", "find_max") in F.EDGES
    assert ("load_coarse", "find_min") in F.EDGES
    assert ("travel_max", "verify") in F.EDGES
    assert ("travel_min", "verify") in F.EDGES


def test_every_node_is_reachable_from_the_first_one():
    """An edge list and a node list that disagree draw a chart with holes."""
    keys = set(F.NODE_KEYS)
    for source, target in F.EDGES:
        assert source in keys and target in keys

    reached = {"open_range"}
    for _ in range(len(F.NODES)):
        for source, target in F.EDGES:
            if source in reached:
                reached.add(target)
    assert reached == keys


def test_a_failure_skips_what_the_run_never_reached():
    """Left pending, a later node reads "not yet" when it means "never"."""
    flow = F.SideFlow()
    flow.reset(0.0)
    flow.start("open_range")
    flow.finish("open_range")
    flow.start("sweep")

    flow.fail("sweep", "no_angle", 9.0)

    states = _states(flow)
    assert states["open_range"] == F.DONE
    assert states["sweep"] == F.FAILED
    assert all(states[key] == F.SKIPPED
               for key in F.NODE_KEYS if key not in ("open_range", "sweep"))


def test_a_failure_stops_the_clock_where_it_failed():
    flow = F.SideFlow()
    flow.reset(0.0)
    flow.start("open_range")
    flow.fail("open_range", "no_angle", 9.0)

    assert flow.elapsed(500.0) == pytest.approx(9.0), "not still counting"
    assert F.header_text(*flow.snapshot(500.0)[:2]) == "failed · 0:09"


def test_finished_work_survives_a_later_failure():
    """A warned node is settled: the failure must not repaint it as skipped."""
    flow = F.SideFlow()
    flow.reset(0.0)
    flow.finish("find_max", "1902 us")
    flow.warn("travel_max", "against a stop at 145 us")
    flow.fail("travel_min", "free_travel_unconfirmed", 30.0,
              ceiling=200.0, threshold=0.25)

    states = _states(flow)
    assert states["find_max"] == F.DONE
    assert states["travel_max"] == F.WARNED
    assert states["travel_min"] == F.FAILED


def test_stopping_is_not_a_failure_and_colours_nothing_red():
    flow = F.SideFlow()
    flow.reset(0.0)
    flow.finish("open_range")
    flow.start("sweep")

    flow.stop(20.0)

    verdict, _elapsed, rows, failure = flow.snapshot(20.0)
    states = {key: state for key, _l, _r, _c, state, _n in rows}
    notes = {key: note for key, _l, _r, _c, _s, note in rows}

    assert verdict == F.STOPPED
    assert failure is None, "nothing failed - the operator asked it to stop"
    assert F.FAILED not in states.values()
    assert states["sweep"] == F.SKIPPED
    assert notes["sweep"] == F.STOPPED_NOTE


def test_a_note_moves_without_changing_the_state():
    """The free travel probe reports every step, and none of them end it."""
    flow = F.SideFlow()
    flow.reset(0.0)
    flow.start("travel_min", "approaching")

    flow.note("travel_min", "backed off 120 us")

    assert _states(flow)["travel_min"] == F.RUNNING
    assert dict((k, n) for k, _l, _r, _c, _s, n
                in flow.snapshot(0.0)[2])["travel_min"] == "backed off 120 us"


def test_the_stiction_failure_says_stiction_and_what_to_do():
    """The wording this whole module exists to make possible."""
    headline, detail = F.render_failure("free_travel_unconfirmed",
                                        ceiling=200.0, threshold=0.25)

    assert headline == "stiction suspected"
    assert "Could not confirm the elevon moved off the end stop" in detail
    assert "200 us of inward travel" in detail
    assert "less than 0.25 deg" in detail
    assert "Stiction is likely too high" in detail
    assert "Free the surface" in detail


def test_a_range_the_servo_cannot_reach_blames_the_horn_not_a_linkage():
    """Direct drive: there is no linkage to be at fault."""
    headline, detail = F.render_failure("command_exhausted", target=-33.0,
                                        command=-1.0, angle=-28.4)

    assert headline == "ran out of command"
    assert "Ran out of command before -33.0 deg" in detail
    assert "-28.40 deg" in detail
    assert "horn" in detail
    assert "linkage" not in detail.lower()


def test_no_failure_wording_mentions_a_linkage():
    for reason in F.FAILURES:
        _headline, detail, advice = F.FAILURES[reason]
        assert "linkage" not in (detail + advice).lower(), reason


def test_every_failure_renders_from_the_fields_its_callers_pass():
    """A template that raises while reporting a fault reports nothing."""
    fields = dict(param="PWM_MAIN_MIN5", value="900", target=-33.0,
                  command=-1.0, angle=-28.4, refusal="Denied", which="Min",
                  ceiling=200.0, threshold=0.25, error=1.2, previous=0.4,
                  pwm=900, attempts=10, tolerance=0.5)

    for reason in F.FAILURES:
        headline, detail = F.render_failure(reason, **fields)
        assert headline and detail
        assert "%(" not in headline and "%(" not in detail, reason


def test_a_headline_fits_the_node_it_is_drawn_in():
    """Headlines are notes on a node, and a forked node is narrow. Anything
    longer belongs in the detail, which is what the box shows."""
    fields = dict(param="CA_SV_CS0_TRIM", value="900", target=-33.0,
                  command=-1.0, angle=-28.4, refusal="Denied", which="Min",
                  ceiling=200.0, threshold=0.25, error=1.2, previous=0.4,
                  pwm=900, attempts=10, tolerance=0.5)

    for reason in F.FAILURES:
        headline, _detail = F.render_failure(reason, **fields)
        assert len(headline) <= 26, "%s: %r is %d chars" % (reason, headline,
                                                            len(headline))


def test_a_note_too_long_for_its_node_is_cut_rather_than_drawn_over_the_edge():
    # One "character" six units wide, so the arithmetic is checkable by eye.
    measure = lambda text: 6 * len(text)

    assert F.fit("short", measure, 600) == "short", "untouched when it fits"
    assert F.fit("", measure, 60) == ""
    assert F.fit("abcdefghij", measure, 60) == "abcdefghij", "exactly fits"
    assert F.fit("abcdefghij", measure, 54) == "abcdef..."
    assert measure(F.fit("abcdefghij", measure, 54)) <= 54

    # Narrower than the ellipsis itself: nothing can be drawn honestly.
    assert F.fit("abcdefghij", measure, 10) == ""


def test_no_template_asks_for_a_field_name_the_call_already_uses():
    """A collision here raises inside the failure reporting itself."""
    for reason, (headline, detail, advice) in F.FAILURES.items():
        for name in F.RESERVED_FIELDS:
            assert "%%(%s)" % name not in headline + detail + advice, \
                "%s uses the reserved field %r" % (reason, name)


def test_a_missing_field_is_reported_rather_than_raised():
    """Failing to describe a failure must not become a second failure."""
    headline, detail = F.render_failure("at_limit")

    assert "Could not describe" in detail
    assert headline


def test_an_unknown_reason_still_produces_something_drawable():
    headline, detail = F.render_failure("no_such_reason")

    assert headline == "Failed"
    assert "no_such_reason" in detail


def test_the_two_free_travel_nodes_are_named_apart_in_the_detail_box():
    """In the chart the column says which end; below both columns it cannot."""
    assert F.LABELS["travel_max"] == F.LABELS["travel_min"] == "Free travel"
    assert F.DETAIL_LABELS["travel_max"] == "Max free travel"
    assert F.DETAIL_LABELS["travel_min"] == "Min free travel"

    flow = F.SideFlow()
    flow.reset(0.0)
    flow.fail("travel_min", "free_travel_unconfirmed", 1.0,
              ceiling=200.0, threshold=0.25)

    assert flow.snapshot(1.0)[3][0] == "Min free travel"


def test_node_for_maps_each_end_to_its_two_nodes():
    assert F.node_for("MAX", "find") == "find_max"
    assert F.node_for("MIN", "travel") == "travel_min"
    with pytest.raises(ValueError):
        F.node_for("MIDDLE", "find")


def test_an_unknown_node_is_refused_rather_than_silently_created():
    flow = F.SideFlow()
    with pytest.raises(ValueError):
        flow.start("no_such_node")


def test_every_state_has_a_style_and_they_are_all_palette_names():
    assert set(F.STYLE) == {F.PENDING, F.RUNNING, F.DONE, F.WARNED,
                            F.FAILED, F.SKIPPED}
    for outline, fill, text, width in F.STYLE.values():
        assert isinstance(outline, str) and isinstance(fill, str)
        assert isinstance(text, str) and width in (1, 2)


def test_elapsed_formats_as_minutes_and_seconds():
    assert F.format_elapsed(None) == "--"
    assert F.format_elapsed(0.0) == "0:00"
    assert F.format_elapsed(134.0) == "2:14"
    assert F.format_elapsed(3599.0) == "59:59"


def test_a_snapshot_does_not_change_under_the_renderer():
    """It is taken on the worker thread and drawn on the Tk one."""
    flow = F.SideFlow()
    flow.reset(0.0)
    flow.start("open_range", "opening")

    snapshot = flow.snapshot(1.0)
    flow.finish("open_range", "done now")
    flow.fail("sweep", "no_angle", 2.0)

    states = {key: state for key, _l, _r, _c, state, _n in snapshot[2]}
    assert states["open_range"] == F.RUNNING, "the snapshot moved"
    assert snapshot[3] is None
