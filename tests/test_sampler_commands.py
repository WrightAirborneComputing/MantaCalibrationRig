"""Host-side tests for the Pico firmware's command protocol.

`pico/sampler.py` is MicroPython, but its command parser is deliberately pure -
no `machine`, no `utime`, no I/O - so the protocol can be regression-tested here
under CPython with no board on the desk. Only `apply_command`, `clamp_hz` and
`derive_timing` are exercised; the sampling loop needs hardware and is covered by
the console verification in the README.

    python3 -m pytest tests/test_sampler_commands.py -v
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fake_pico import load_sampler

sampler = load_sampler()


@pytest.fixture
def state():
    return sampler.initial_state()
# def


def test_boots_slow_and_readable(state):
    """The boot default must stay drop-in compatible with the legacy firmware."""
    assert state == {"mode": "slow", "hz": 10, "ts": False}
# def


def test_status_reports_without_changing_state(state):
    new_state, reply, action = sampler.apply_command(state, "?")

    assert reply == "# STATUS mode=slow hz=10 ts=0"
    assert new_state is state
    assert action is None
# def


def test_collect_requests_the_side_effect_without_performing_it(state):
    """`G` must name the action, not do it - that is what keeps this testable.

    The host sends G before a capture so MicroPython's ~7 ms collector stall
    happens on demand rather than landing mid-measurement.
    """
    new_state, reply, action = sampler.apply_command(state, "G")

    assert reply == "# ACK G"
    assert action == "collect"
    assert new_state is state
# def


def test_collect_does_not_disturb_the_rate(state):
    """G is orthogonal to mode: collecting must not knock the board out of fast."""
    fast, _, _ = sampler.apply_command(state, "F1000")
    after, reply, action = sampler.apply_command(fast, "g")

    assert action == "collect"
    assert after is fast
    assert after["hz"] == 1000
    assert after["ts"] is True
# def


@pytest.mark.parametrize("text", ["?", "S", "F", "F250", "", "junk"])
def test_only_g_requests_an_action(state, text):
    _, _, action = sampler.apply_command(state, text)

    assert action is None
# def


def test_fast_preset(state):
    new_state, reply, action = sampler.apply_command(state, "F")

    assert reply == "# ACK F 500"
    assert new_state == {"mode": "fast", "hz": 500, "ts": True}
# def


def test_slow_preset_returns_to_untimestamped_10hz(state):
    fast, _, _ = sampler.apply_command(state, "F")
    slow, reply, action = sampler.apply_command(fast, "S")

    assert reply == "# ACK S 10"
    assert slow == {"mode": "slow", "hz": 10, "ts": False}
# def


def test_explicit_rate(state):
    new_state, reply, action = sampler.apply_command(state, "F1000")

    assert reply == "# ACK F 1000"
    assert new_state["hz"] == 1000
    assert new_state["ts"] is True
# def


@pytest.mark.parametrize("text,expected_hz", [
    ("F0", sampler.MIN_HZ),
    ("F1", sampler.MIN_HZ),
    ("F2000", sampler.MAX_HZ),
    ("F99999", sampler.MAX_HZ),
])
def test_rate_is_clamped_at_both_ends(state, text, expected_hz):
    new_state, reply, action = sampler.apply_command(state, text)

    assert new_state["hz"] == expected_hz
    assert reply == "# ACK F %d" % expected_hz
# def


@pytest.mark.parametrize("text", ["f", "F\r\n", " f \n", "f\r"])
def test_lowercase_and_line_endings_are_accepted(state, text):
    """Terminals send CR, CRLF or LF depending on the tool; all must work."""
    new_state, reply, action = sampler.apply_command(state, text)

    assert reply == "# ACK F 500"
    assert new_state["mode"] == "fast"
# def


def test_status_is_case_insensitive_too(state):
    fast, _, _ = sampler.apply_command(state, "f1000")
    _, reply, _ = sampler.apply_command(fast, "?")

    assert reply == "# STATUS mode=fast hz=1000 ts=1"
# def


@pytest.mark.parametrize("text", ["X", "FF", "F12A", "F-1", "SLOW", "!!"])
def test_junk_errors_without_mutating_state(state, text):
    new_state, reply, action = sampler.apply_command(state, text)

    assert reply.startswith("# ERR")
    assert new_state is state
# def


def test_blank_line_is_silently_ignored(state):
    """Bare Enter in a terminal must not produce an error reply."""
    new_state, reply, action = sampler.apply_command(state, "  \r\n")

    assert reply is None
    assert new_state is state
# def


def test_every_reply_is_comment_prefixed(state):
    """Replies must never match the host's sample regex."""
    import re

    position_regex = re.compile(
        r"\[(?:(?P<t_us>\d+):)?(?P<position1>-?\d+)\s*/\s*(?P<position2>-?\d+)\]"
    )

    for text in ("?", "S", "F", "F250", "G", "garbage"):
        _, reply, _ = sampler.apply_command(state, text)
        assert reply.startswith("# ")
        assert position_regex.search(reply) is None
# def


@pytest.mark.parametrize("hz,period_us", [(10, 100000), (500, 2000), (1000, 1000)])
def test_period_matches_rate(hz, period_us):
    assert sampler.derive_timing(hz)[0] == period_us
# def


def test_command_polling_stays_responsive_across_the_range():
    """Switch latency = cmd_interval * period. Must stay well under a second."""
    for hz in (sampler.MIN_HZ, 10, 100, 500, 1000, sampler.MAX_HZ):
        period_us, cmd_interval, blink_reload = sampler.derive_timing(hz)

        assert cmd_interval >= 1
        assert blink_reload >= 1
        assert cmd_interval * period_us <= 1000000
# def


def test_overlong_command_is_bounded():
    """Line noise must not grow the buffer unboundedly; 16 chars then # ERR."""
    assert sampler.MAX_COMMAND_LEN == 16

    state = sampler.initial_state()
    new_state, reply, action = sampler.apply_command(state, "F" * 32)

    assert reply.startswith("# ERR")
    assert new_state is state
# def
