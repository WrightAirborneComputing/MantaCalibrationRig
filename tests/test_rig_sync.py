"""The rendezvous: what it guarantees, and what it does when a side misbehaves.

The guarantee under test is one sentence - no side reads an angle while another
side is moving - and it is bought with a blocking wait between two real
threads. So these are threaded tests, with real time compressed into
milliseconds rather than a fake clock: the thing being verified is whether the
waiting actually happens, and a clock the code cannot block on would verify
nothing.

Every test has a timeout via THREAD_TIMEOUT_S. A rendezvous bug is a deadlock,
and a deadlocked test that hangs the suite tells you less than one that fails.
"""

import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rig_sync


THREAD_TIMEOUT_S = 5.0


class Recorder:
    """A rig that does nothing but write down what it was asked to do."""

    def __init__(self, angles=None, on_hold=None):
        self.angles = angles or {"LEFT": -1.5, "RIGHT": 2.5}
        self.commands = []          # (side, cmd)
        self.holds = []             # (dict(commands), dwell_s)
        self.reads = []             # side
        self.contended = []         # side
        self._on_hold = on_hold
        self._lock = threading.Lock()

    def command(self, side, cmd):
        with self._lock:
            self.commands.append((side, cmd))

    def hold(self, commands, dwell_s):
        with self._lock:
            self.holds.append((dict(commands), dwell_s))
        if self._on_hold is not None:
            self._on_hold(commands, dwell_s)

    def read(self, side):
        with self._lock:
            self.reads.append(side)
        return self.angles[side]

    def note_contended(self, side):
        with self._lock:
            self.contended.append(side)
# class


def arbiter(rig, **kwargs):
    kwargs.setdefault("refresh_s", 0.01)
    kwargs.setdefault("timeout_s", 1.0)
    return rig_sync.RigArbiter(command=rig.command, hold=rig.hold,
                               read=rig.read, now=time.monotonic,
                               on_contended=rig.note_contended, **kwargs)
# def


def measure_in_thread(arb, side, command, dwell_s=0.8, into=None):
    """Start a measure() on its own thread and hand back (thread, results)."""
    results = {} if into is None else into

    def run():
        results[side] = arb.measure(side, command, dwell_s)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread, results
# def


def test_one_side_measures_without_waiting_for_anyone():
    """A single-side calibration must be what it always was: command, hold, read.

    This is the case that has to stay byte-for-byte, because it is every
    existing calibration run and every existing test of one.
    """
    rig = Recorder()
    arb = arbiter(rig)
    arb.join("LEFT")

    assert arb.measure("LEFT", 0.4, 0.8) == -1.5
    assert rig.commands == [("LEFT", 0.4)]
    assert rig.holds == [({"LEFT": 0.4}, 0.8)]
    assert rig.reads == ["LEFT"]
    assert rig.contended == []
# def


def test_two_sides_settle_in_one_shared_window():
    """The whole point. Two commands, one uninterrupted window, two reads.

    Nothing may be commanded between the first command and the last read: the
    window is only still if nothing moves anywhere in it. It is allowed to be
    held in segments, because the two sides do not always want the same dwell.
    """
    rig = Recorder()
    arb = arbiter(rig)
    arb.join("LEFT")
    arb.join("RIGHT")

    results = {}
    left, _ = measure_in_thread(arb, "LEFT", -0.9, 0.8, results)
    right, _ = measure_in_thread(arb, "RIGHT", 0.6, 0.5, results)

    left.join(THREAD_TIMEOUT_S)
    right.join(THREAD_TIMEOUT_S)
    assert not left.is_alive() and not right.is_alive()

    # Both commanded once, up front, and never again inside the window.
    assert sorted(rig.commands) == [("LEFT", -0.9), ("RIGHT", 0.6)]
    assert all(held == {"LEFT": -0.9, "RIGHT": 0.6} for held, _d in rig.holds)

    assert results == {"LEFT": -1.5, "RIGHT": 2.5}
    assert sorted(rig.reads) == ["LEFT", "RIGHT"]
# def


def test_each_side_is_read_at_the_dwell_it_asked_for():
    """A short dwell paired with a long one must not become the long one.

    This is what stops a 0.25 s creep step being held for a partner's 0.8 s
    endpoint measurement. The creep decides it has arrived from what it reads,
    and a step that settles three times longer than the creep intends reads
    somewhere the creep was not going to stop - and, because the side that has
    fallen behind is the one being stretched, it falls further behind.
    """
    read_at = {}

    def clock_the_reads(side):
        read_at[side] = sum(d for _c, d in rig.holds)
        return rig.angles[side]

    rig = Recorder()
    rig.read = clock_the_reads

    arb = arbiter(rig)
    arb.join("LEFT")
    arb.join("RIGHT")

    results = {}
    slow, _ = measure_in_thread(arb, "LEFT", -0.9, 0.8, results)
    quick, _ = measure_in_thread(arb, "RIGHT", 0.6, 0.25, results)
    slow.join(THREAD_TIMEOUT_S)
    quick.join(THREAD_TIMEOUT_S)

    assert read_at == {"RIGHT": pytest.approx(0.25), "LEFT": pytest.approx(0.8)}

    # Held in two segments adding up to the longer dwell, not one of each.
    assert sum(d for _c, d in rig.holds) == pytest.approx(0.8)
    assert results == {"LEFT": -1.5, "RIGHT": 2.5}
# def


def test_nothing_moves_while_a_side_is_still_busy():
    """A side that has not arrived holds the round - that is the interlock."""
    rig = Recorder()
    arb = arbiter(rig)
    arb.join("LEFT", at=0.0)
    arb.join("RIGHT", at=0.0)

    left, results = measure_in_thread(arb, "LEFT", 0.2, 0.8)

    # LEFT is parked. RIGHT is off writing a parameter.
    time.sleep(0.15)
    assert rig.holds == [], "a round fired while RIGHT was still busy"
    assert rig.reads == [], "an angle was read while RIGHT was still busy"

    right, _ = measure_in_thread(arb, "RIGHT", -0.3, 0.8, results)
    left.join(THREAD_TIMEOUT_S)
    right.join(THREAD_TIMEOUT_S)

    assert len(rig.holds) == 1
    assert rig.holds[0][0] == {"LEFT": 0.2, "RIGHT": -0.3}
    assert results == {"LEFT": -1.5, "RIGHT": 2.5}
    assert rig.contended == []
# def


def test_a_waiting_side_is_held_where_it_is_not_where_it_is_going():
    """The wait keeps the surface alive without starting its traverse early.

    Both halves matter. The override lapses after about 2 s and a lapsed
    surface parks itself, which is motion in someone else's window - so the
    command has to be re-sent. But sending the command being waited on would
    drive the surface to the endpoint before the window meant to contain that
    move had opened, and leave it settling against a stop for the length of the
    wait plus the dwell. How long a surface sits against a stop changes where
    it comes to rest, so that is a measurement of something else.
    """
    rig = Recorder()
    arb = arbiter(rig)
    arb.join("LEFT", at=0.0)
    arb.join("RIGHT", at=0.0)

    left, results = measure_in_thread(arb, "LEFT", 0.9, 0.8)
    time.sleep(0.15)

    assert rig.commands, "the waiting side was left to lapse"
    assert set(rig.commands) == {("LEFT", 0.0)}, \
        "the waiting side was driven to where it had not been told to go yet"

    right, _ = measure_in_thread(arb, "RIGHT", -0.9, 0.8, results)
    left.join(THREAD_TIMEOUT_S)
    right.join(THREAD_TIMEOUT_S)

    # And 0.9 is only ever commanded once the window opens.
    assert ("LEFT", 0.9) in rig.commands
    assert rig.holds[0][0] == {"LEFT": 0.9, "RIGHT": -0.9}
# def


def test_a_finished_side_does_not_strand_the_other():
    """LEFT finishes its run; RIGHT must carry on, not wait out a timeout."""
    rig = Recorder()
    arb = arbiter(rig, timeout_s=30.0)
    arb.join("LEFT")
    arb.join("RIGHT")

    right, results = measure_in_thread(arb, "RIGHT", 0.5, 0.8)
    time.sleep(0.05)
    assert rig.holds == []

    arb.leave("LEFT")

    right.join(THREAD_TIMEOUT_S)
    assert not right.is_alive(), "RIGHT waited for a side that had gone"
    assert rig.holds == [({"RIGHT": 0.5}, 0.8)]
    assert results == {"RIGHT": 2.5}

    # Not contention: LEFT left, which is an answer. Flagging it would put a
    # warning on every reading after the first side finishes.
    assert rig.contended == []
# def


def test_a_stopped_side_stops_waiting():
    """Stop must reach a worker parked at the rendezvous.

    Without this the stop button does nothing until the other side arrives, and
    a stop pressed because the rig is misbehaving is exactly when the other
    side is least likely to.
    """
    rig = Recorder()
    arb = arbiter(rig, timeout_s=30.0)
    arb.join("LEFT")
    arb.join("RIGHT")

    left, results = measure_in_thread(arb, "LEFT", 0.1, 0.8)
    time.sleep(0.05)

    arb.leave("LEFT")

    left.join(THREAD_TIMEOUT_S)
    assert not left.is_alive()
    assert results == {"LEFT": None}
    assert rig.holds == [], "a dropped side still drove the rig"
# def


def test_a_wedged_partner_is_measured_around_and_reported():
    """A stuck side must not stall the run for ever - but must not be hidden.

    The round goes ahead, because a calibration that hangs on a partner is
    worse than one reading taken unguarded. It is reported, because a reading
    that may have been taken while the rig moved must not be indistinguishable
    from one that was not.
    """
    rig = Recorder()
    arb = arbiter(rig, timeout_s=0.2)
    arb.join("LEFT")
    arb.join("RIGHT")          # joins, then never arrives

    started = time.monotonic()
    assert arb.measure("LEFT", 0.7, 0.8) == -1.5
    waited = time.monotonic() - started

    assert waited >= 0.2, "gave up before the timeout"
    assert rig.holds == [({"LEFT": 0.7}, 0.8)]
    assert rig.contended == ["LEFT"]
# def


def test_a_failed_round_wakes_its_partner_instead_of_hanging_it():
    """If the leader's hardware call raises, the other side must not be left parked.

    The exception belongs to the leader's own worker, which already knows how
    to record a failed run. What must not happen is the partner waiting out a
    timeout for a leader that is gone.
    """
    def explode(commands, dwell_s):
        raise IOError("serial link went away")

    rig = Recorder(on_hold=explode)
    arb = arbiter(rig, timeout_s=30.0)
    arb.join("LEFT")
    arb.join("RIGHT")

    failures = {}

    def run(side, command):
        try:
            failures[side] = arb.measure(side, command, 0.8)
        except IOError as e:
            failures[side] = str(e)

    threads = [threading.Thread(target=run, args=args, daemon=True)
               for args in (("LEFT", 0.1), ("RIGHT", -0.1))]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(THREAD_TIMEOUT_S)
        assert not thread.is_alive(), "a side was left parked on a dead leader"

    # Whichever one led raised; the other was told there is no angle.
    assert sorted(failures) == ["LEFT", "RIGHT"]
    assert "serial link went away" in [v for v in failures.values() if v is not None]
    assert None in failures.values()
# def


def test_rounds_run_one_at_a_time():
    """Back-to-back rounds must not overlap, or the windows are not quiet.

    The probe steps twenty times in a row; if a second round could start while
    the first was still holding, every one of those steps would be read against
    a moving rig.
    """
    overlaps = []
    running = threading.Lock()

    def watch(commands, dwell_s):
        if not running.acquire(blocking=False):
            overlaps.append(dict(commands))
            return
        time.sleep(0.02)
        running.release()

    rig = Recorder(on_hold=watch)
    arb = arbiter(rig)
    arb.join("LEFT")
    arb.join("RIGHT")

    results = {}
    for step in range(5):
        # Both sides in flight together, then both collected before the next
        # step - which is what two worker threads stepping a probe look like.
        # A side is only ever in one measurement at a time.
        pair = [measure_in_thread(arb, side, sign * step * 0.1, 0.8, results)[0]
                for side, sign in (("LEFT", 1.0), ("RIGHT", -1.0))]
        for thread in pair:
            thread.join(THREAD_TIMEOUT_S)
            assert not thread.is_alive()

    # Five rounds, not ten: each pair settled together.
    assert len(rig.holds) == 5
    assert overlaps == []
    assert rig.contended == []
# def


def test_a_caller_that_never_joined_is_not_made_to_wait():
    """Measuring outside a coordinated run must not block on one.

    Nothing in the calibration reaches here today, but the alternative to
    saying so is a rendezvous that hangs whatever calls it from outside - and
    the first thing to do that would be a diagnostic run at three in the
    morning with the rig half apart.
    """
    rig = Recorder()
    arb = arbiter(rig, timeout_s=30.0)
    arb.join("RIGHT")           # a real run, in progress, on the other side

    started = time.monotonic()
    assert arb.measure("LEFT", 0.3, 0.8) == -1.5
    assert time.monotonic() - started < 1.0, "waited on a run it was not part of"

    assert rig.holds == [({"LEFT": 0.3}, 0.8)]
    assert rig.contended == []

    # And it did not join by measuring: RIGHT is still the only participant,
    # so the next round is RIGHT's alone rather than one waiting for a caller
    # that has already gone.
    assert arb.participants() == {"RIGHT"}
# def


# ---- Barriers ------------------------------------------------------------


def barrier_in_thread(arb, side, name, into=None):
    results = {} if into is None else into

    def run():
        results[side] = arb.barrier(side, name)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread, results
# def


def test_a_barrier_holds_until_every_side_reaches_it():
    """Phase alignment: the side that finishes first waits for the other."""
    rig = Recorder()
    arb = arbiter(rig)
    arb.join("LEFT", at=0.0)
    arb.join("RIGHT", at=0.0)

    left, results = barrier_in_thread(arb, "LEFT", "sweep")
    time.sleep(0.1)
    assert left.is_alive(), "LEFT walked through a barrier on its own"

    right, _ = barrier_in_thread(arb, "RIGHT", "sweep", results)
    left.join(THREAD_TIMEOUT_S)
    right.join(THREAD_TIMEOUT_S)

    assert not left.is_alive() and not right.is_alive()
    assert results == {"LEFT": True, "RIGHT": True}

    # Held where it was while it waited, and nothing settled: a barrier is a
    # place to stand still, not a measurement.
    assert set(rig.commands) == {("LEFT", 0.0)}
    assert rig.holds == []
# def


def test_a_side_at_a_barrier_does_not_block_one_still_working():
    """The deadlock the two mechanisms would otherwise make together.

    RIGHT finishes its phase and waits at the barrier for LEFT. LEFT is still
    sweeping and asks to measure - and would wait for RIGHT to arrive at a
    rendezvous it is never going to reach, because it is waiting for LEFT.

    A side parked at a barrier is stationary by definition, so dropping it from
    the quorum and letting LEFT measure alone is not a compromise: the rig LEFT
    measures against really is still.
    """
    rig = Recorder()
    arb = arbiter(rig, timeout_s=30.0)
    arb.join("LEFT", at=0.0)
    arb.join("RIGHT", at=0.0)

    right, at_barrier = barrier_in_thread(arb, "RIGHT", "sweep")
    time.sleep(0.1)

    started = time.monotonic()
    assert arb.measure("LEFT", -0.4, 0.8) == -1.5
    assert time.monotonic() - started < 1.0, "LEFT waited on a side at a barrier"

    assert rig.holds == [({"LEFT": -0.4}, 0.8)]
    assert rig.reads == ["LEFT"]

    # Measuring alone here is not contention - RIGHT's whereabouts are known.
    assert rig.contended == []

    left, _ = barrier_in_thread(arb, "LEFT", "sweep", at_barrier)
    right.join(THREAD_TIMEOUT_S)
    left.join(THREAD_TIMEOUT_S)
    assert at_barrier == {"LEFT": True, "RIGHT": True}
# def


def test_a_side_that_leaves_releases_the_barrier():
    """A side that fails or is stopped must not strand the other at a boundary."""
    rig = Recorder()
    arb = arbiter(rig, barrier_timeout_s=30.0)
    arb.join("LEFT", at=0.0)
    arb.join("RIGHT", at=0.0)

    right, results = barrier_in_thread(arb, "RIGHT", "verify")
    time.sleep(0.1)
    assert right.is_alive()

    arb.leave("LEFT")

    right.join(THREAD_TIMEOUT_S)
    assert not right.is_alive(), "RIGHT waited at a barrier for a side that had gone"
    assert results == {"RIGHT": True}
# def


def test_a_barrier_gives_up_rather_than_hanging_the_run():
    """A wedged partner costs a wait and a note, not the whole calibration."""
    rig = Recorder()
    arb = arbiter(rig, barrier_timeout_s=0.2)
    arb.join("LEFT", at=0.0)
    arb.join("RIGHT", at=0.0)      # joins, and never reaches the barrier

    started = time.monotonic()
    assert arb.barrier("LEFT", "trim") is False
    assert time.monotonic() - started >= 0.2, "gave up before the timeout"

    # And having given up, it is out of the way: RIGHT can still measure.
    assert arb.measure("RIGHT", 0.5, 0.8) == 2.5
# def


# ---- Moving outside a rendezvous ----------------------------------------


def test_a_busy_side_stops_anything_settling_around_it():
    """A parameter write parks the surface, so nothing may be settling.

    The write itself cannot be a rendezvous - it blocks for seconds, and the
    flight controller takes the outputs back part way through whatever anyone
    wants. All that can be done is to keep that movement out of the other
    side's window.
    """
    rig = Recorder()
    arb = arbiter(rig, timeout_s=30.0)
    arb.join("LEFT", at=0.0)
    arb.join("RIGHT", at=0.0)

    with arb.busy("RIGHT", at=0.0):
        left, results = measure_in_thread(arb, "LEFT", 0.3, 0.8)
        time.sleep(0.15)
        assert rig.holds == [], "a window opened while RIGHT was writing a parameter"
        assert rig.reads == [], "an angle was read while RIGHT was writing a parameter"

    # And the moment it ends, LEFT can go - a single participant now, because
    # RIGHT is off doing the next thing rather than waiting to measure.
    right, _ = measure_in_thread(arb, "RIGHT", -0.3, 0.8, results)
    left.join(THREAD_TIMEOUT_S)
    right.join(THREAD_TIMEOUT_S)
    assert not left.is_alive() and not right.is_alive()
    assert results == {"LEFT": -1.5, "RIGHT": 2.5}
# def


def test_going_busy_waits_out_a_window_already_open():
    """Declaring busy is not enough on its own - the round in flight must end.

    A parameter write that starts while the other side is mid-dwell moves the
    surface inside a window that has already opened. Checking the flag when the
    round starts cannot catch that; waiting for the round to finish can.
    """
    opened = threading.Event()
    holding = threading.Event()

    def slow_hold(commands, dwell_s):
        opened.set()
        holding.set()
        time.sleep(0.2)
        holding.clear()

    rig = Recorder(on_hold=slow_hold)
    arb = arbiter(rig, timeout_s=30.0)
    arb.join("LEFT", at=0.0)

    left, _ = measure_in_thread(arb, "LEFT", 0.2, 0.8)
    assert opened.wait(THREAD_TIMEOUT_S)

    with arb.busy("RIGHT", at=0.0):
        assert not holding.is_set(), \
            "a write began inside a window that was already open"

    left.join(THREAD_TIMEOUT_S)
    assert not left.is_alive()
# def


def test_two_sides_at_different_boundaries_are_not_treated_as_aligned():
    """The failure that looks exactly like success.

    Releasing on "everyone is parked somewhere" passes every test that counts
    arrivals, and is the opposite of a barrier: a side that reached its second
    boundary while the other was still at its first finds everyone parked and
    goes straight through. From then on the two run one phase apart for ever,
    each releasing the other from the boundary behind - which is a pipeline,
    and precisely what barriers are for.

    It still releases rather than hanging, because two sides waiting at
    different boundaries have no way to break the tie between them. But it
    says so, and a caller that reports it will show it rather than run a
    calibration that only looks synchronised.
    """
    rig = Recorder()
    arb = arbiter(rig, barrier_timeout_s=30.0)
    arb.join("LEFT", at=0.0)
    arb.join("RIGHT", at=0.0)

    results = {}
    left, _ = barrier_in_thread(arb, "LEFT", "sweep", results)
    time.sleep(0.1)
    right, _ = barrier_in_thread(arb, "RIGHT", "open_range", results)

    left.join(THREAD_TIMEOUT_S)
    right.join(THREAD_TIMEOUT_S)
    assert not left.is_alive() and not right.is_alive()

    assert results == {"LEFT": False, "RIGHT": False}, \
        "two sides at different boundaries were let through as if in step"
# def


def test_matching_boundaries_are_aligned():
    """The same check the other way, so the one above cannot pass by always failing."""
    rig = Recorder()
    arb = arbiter(rig, barrier_timeout_s=30.0)
    arb.join("LEFT", at=0.0)
    arb.join("RIGHT", at=0.0)

    results = {}
    left, _ = barrier_in_thread(arb, "LEFT", "sweep", results)
    right, _ = barrier_in_thread(arb, "RIGHT", "sweep", results)
    left.join(THREAD_TIMEOUT_S)
    right.join(THREAD_TIMEOUT_S)

    assert results == {"LEFT": True, "RIGHT": True}
# def
