"""Keeping the two calibration sides out of each other's measurements.

The rig is one frame. Sweeping an elevon shakes it, and that shake arrives at
the opposite potentiometer as angle, so a reading taken on one side while the
other is in motion is not a measurement of anything. The two calibration
workers are independent threads, so without something like this they overlap
constantly rather than occasionally.

Two mechanisms, because the sides have to agree about two different things.

A **rendezvous** is agreement about motion. Both sides submit the command they
want, the arbiter drives both surfaces, holds one window, and reads each side's
angle out of it. A shared window is one window, so two sides cost the same wall
clock as one. It does not care what step either side is on: LEFT parking for a
run-up and RIGHT stepping its breakaway probe are still move-together,
settle-together, read-together.

A **barrier** is agreement about progress. Rendezvous alone leaves the sides
free to drift into different phases, which they do, because the refinement
takes a different number of attempts per side. Barriers put them back in step
at the phase boundaries.

The two would deadlock against each other if a side parked at a barrier still
counted towards a rendezvous: the side still sweeping would wait for a partner
that has stopped asking to measure. So a parked side drops out of the quorum,
and the side still working measures alone. That is sound rather than a
compromise - a side at a barrier is stationary by definition, so the rig it
leaves behind is still.

Each side is held at the position it is *at* while it waits, never at the one
it is about to move to. The actuator override lapses after about 2 s and a
lapsed surface parks itself, which is motion, so waiting quietly means waiting
while still commanding - but commanding the surface onward early would start
its traverse outside the window that is supposed to contain it, and hand it a
longer settle than the one the calibration asked for. Dwell length is not
cosmetic here: how long a surface sits against a stop changes where it comes to
rest.

For the same reason a round with two different dwells in it holds for the
longer and reads each side at its own. Stretching a 0.25 s creep step to the
0.8 s of a partner's endpoint measurement changes where that creep decides it
has arrived, and it makes the side that has fallen behind slower still.

A run with one side calibrating has one participant, so quorum is satisfied the
moment it asks and every round fires straight through. Single-side calibration
is unchanged, not merely equivalent.

Not everything a side does can be a rendezvous. A parameter write blocks for
seconds while the flight controller reclaims the outputs and parks the surface,
so it is declared busy instead: no round may be in flight when one starts, and
none may start until it ends. Joining does the same, because a side that
appears part way through someone else's window and then moves is the same
disturbance wearing a different hat.

Imports threading and contextlib, and nothing else. Commanding, holding and reading are
callbacks, so this is testable without a rig - the same split that keeps
endpoint_logic.py and trim_logic.py off the hardware.
"""

import contextlib
import threading

# How often a side waiting here re-sends the command it is being held at. The
# override lapses after about 2 s. The caller passes its own MEASURE_REFRESH_S;
# this default only matters to the tests.
HOLD_REFRESH_S = 0.5

# When to stop waiting for the other side and get on with it. Above the worst
# case a healthy partner can impose - a breakaway probe that runs to the
# ceiling is twenty dwells, about 16 s, and a phase can hold several of those
# plus its parameter writes - so reaching this means the partner is wedged
# rather than slow. Whatever was being waited for happens anyway, because a
# stuck side must not be able to stall a calibration for ever, and it is
# reported rather than passed off as having gone to plan.
RENDEZVOUS_TIMEOUT_S = 30.0
BARRIER_TIMEOUT_S = 180.0


class _Round:
    """One shared move-settle-read, and the angles it produced."""

    __slots__ = ("results", "done", "contended")

    def __init__(self):
        self.results = {}
        self.done = False
        self.contended = False
# class


class _Gate:
    """One phase boundary, and whether everyone has reached it yet."""

    __slots__ = ("open", "names", "aligned")

    def __init__(self):
        self.open = False
        self.names = {}
        self.aligned = True
# class


class RigArbiter:
    """The meeting point. One instance per rig, shared by both workers."""

    def __init__(self, command, hold, read, now,
                 refresh_s=HOLD_REFRESH_S,
                 timeout_s=RENDEZVOUS_TIMEOUT_S,
                 barrier_timeout_s=BARRIER_TIMEOUT_S,
                 on_contended=None):
        self._command = command          # command(side, cmd)
        self._hold = hold                # hold({side: cmd}, duration_s)
        self._read = read                # read(side) -> angle or None
        self._now = now
        self._refresh_s = refresh_s
        self._timeout_s = timeout_s
        self._barrier_timeout_s = barrier_timeout_s
        self._on_contended = on_contended

        self._cond = threading.Condition()
        self._joined = set()
        self._waiting = {}               # side -> (command, dwell_s)
        self._parked = {}                # side -> barrier name
        self._busy = set()               # sides moving outside a rendezvous
        self._at = {}                    # side -> the command it is held at
        self._round = _Round()
        self._gate = _Gate()
        self._leader = None

    # ---- Membership ------------------------------------------------------

    def join(self, side, at=None):
        """This side is now part of the run and must be waited for.

        `at` is where its surface already is, so that a side which has to wait
        before its first measurement can be held there rather than left to
        lapse.
        """
        with self._cond:
            # Not while a round is in flight. The joining worker's next act is
            # to write parameters, which moves its surface, and a surface that
            # moves inside a window someone else is already settling in is the
            # disturbance this exists to prevent.
            self._settle_out()
            self._joined.add(side)
            if at is not None:
                self._at[side] = at
            self._cond.notify_all()
    # def

    def leave(self, side):
        """This side is out: stop waiting for it, and stop it waiting.

        Idempotent, and called from the worker's finally as well as from the
        stop button, because both are ways a side stops existing as far as the
        other one is concerned. Releasing a side that is parked here is the
        other half: a stopped worker blocked on a partner would not notice it
        had been stopped until the partner arrived.
        """
        with self._cond:
            self._joined.discard(side)
            self._waiting.pop(side, None)
            self._parked.pop(side, None)
            self._at.pop(side, None)
            self._cond.notify_all()
    # def

    @contextlib.contextmanager
    def busy(self, side, at=None):
        """Move this side outside a rendezvous, with nothing settling around it.

        For the things that cannot be a rendezvous: a parameter write blocks
        for seconds, and the flight controller takes the outputs back part way
        through and parks the surface. That movement cannot be prevented - only
        kept out of the other side's window, which is what this does. It waits
        for any round in flight to finish, and holds off the next one until the
        write is done.

        `at` is the command the surface is left at, so that a side which waits
        immediately afterwards is held there rather than at the position it
        occupied before the write - which no longer means the same PWM, since
        the write is what changed the mapping.
        """
        with self._cond:
            self._settle_out()
            self._busy.add(side)
        try:
            yield
        finally:
            with self._cond:
                self._busy.discard(side)
                if at is not None:
                    self._at[side] = at
                self._cond.notify_all()
    # def

    def _settle_out(self):
        """Wait for any round in flight to finish. Called under the lock."""
        while self._leader is not None:
            self._cond.wait(self._refresh_s)
    # def

    def participants(self):
        with self._cond:
            return set(self._joined)
    # def

    # ---- Barriers --------------------------------------------------------

    def barrier(self, side, name):
        """Wait until every side has reached this phase boundary.

        True if everyone arrived at this same boundary, False otherwise. Park
        the surface before calling: a side idling against an end stop for as
        long as a phase can take is exactly the lingering the procedure avoids
        everywhere else.

        The name has to match, not just the count. Releasing on "everyone is
        parked somewhere" looks like it works and quietly does the opposite: a
        side that reached its second boundary while the other was still at its
        first would find everyone parked and go through, and from then on the
        two run exactly one phase apart for ever, each releasing the other from
        the boundary behind. Which is a pipeline, and the thing barriers exist
        to stop.

        A mismatch is therefore a fault, and it is reported rather than waited
        out - but it still releases, because two sides stuck at different
        boundaries would otherwise wait for each other until the timeout with
        nothing able to break it.
        """
        with self._cond:
            if side not in self._joined:
                return True

            gate = self._gate
            self._parked[side] = name
            gate.names[side] = name

            # The partner may be mid-measurement and waiting on a quorum that
            # this side has just dropped out of, so it can now go ahead alone.
            self._cond.notify_all()

            deadline = self._now() + self._barrier_timeout_s

            aligned = True

            while not gate.open:
                if all(other in self._parked for other in self._joined):
                    aligned = len(set(gate.names.values())) <= 1
                    gate.aligned = aligned
                    gate.open = True
                    self._parked.clear()
                    self._gate = _Gate()
                    self._cond.notify_all()
                    break

                if side not in self._joined:
                    self._parked.pop(side, None)
                    return True

                remaining = deadline - self._now()
                if remaining <= 0.0:
                    self._parked.pop(side, None)
                    self._cond.notify_all()
                    return False

                self._cond.wait(min(self._refresh_s, remaining))
                self._hold_station(side)

            return gate.aligned
    # def

    # ---- The rendezvous --------------------------------------------------

    def measure(self, side, command, dwell_s):
        """Drive to `command`, settle with everyone, and return this side's angle.

        Returns None if the side is dropped while it waits.
        """
        mine = None
        group = None
        answer = None
        contended = False
        led = False

        with self._cond:
            if side in self._joined:
                self._waiting[side] = (command, dwell_s)
                self._cond.notify_all()

                mine = self._round
                deadline = self._now() + self._timeout_s

                # Wait for the others, held where this side already is.
                while (self._leader is None and not mine.done
                       and side in self._joined and not self._ready()):
                    remaining = deadline - self._now()
                    if remaining <= 0.0:
                        contended = True
                        break
                    self._cond.wait(min(self._refresh_s, remaining))
                    if (self._leader is None and not mine.done
                            and side in self._waiting):
                        self._hold_station(side)

                # Someone else claimed this round. It is driving our surface
                # too, so there is nothing to do but collect the answer.
                while not mine.done and self._leader is not None:
                    self._cond.wait(self._refresh_s)

                if mine.done:
                    answer = mine.results.get(side)
                    contended = mine.contended
                elif side not in self._joined:
                    self._waiting.pop(side, None)
                else:
                    # Claiming and taking the group happen in the same hold of
                    # the lock as the readiness check above, with no wait in
                    # between, which is what stops two arrivals both deciding
                    # they are the leader of the same round.
                    led = True
                    self._leader = side
                    group = dict(self._waiting)
                    self._waiting.clear()
                    self._round = _Round()
                    mine.contended = contended
                    for member, (cmd, _dwell) in group.items():
                        self._at[member] = cmd
            else:
                # Nothing is coordinating this call - no one to wait for, and
                # no one waiting on us. It runs below, outside the lock: the
                # steps take the better part of a second, and a participant
                # must not be shut out of the rendezvous that long by a caller
                # that is not in it.
                led = True
                group = {side: (command, dwell_s)}

        if not led:
            self._report_contention(side, contended)
            return answer

        angles = {}
        try:
            angles = self._run(group)
        finally:
            # In the finally so a failed round wakes its partners with None
            # rather than leaving them parked on a leader that has gone. The
            # exception still reaches the leader's own worker, which is where
            # the run is recorded as failed.
            if mine is not None:
                with self._cond:
                    mine.results = angles
                    mine.done = True
                    self._leader = None
                    self._cond.notify_all()

        self._report_contention(side, contended)
        return angles.get(side)
    # def

    # ---- Internals -------------------------------------------------------

    def _ready(self):
        """Is everyone who still owes us a command here?

        Sides parked at a barrier are excluded. They are not going to ask to
        measure until the barrier releases, and the barrier will not release
        until the sides still working reach it - so counting them would be the
        deadlock.
        """
        if self._busy:
            # Someone is moving outside a rendezvous. Nothing may settle until
            # it stops, whoever else is ready.
            return False
        return all(side in self._waiting
                   for side in self._joined if side not in self._parked)
    # def

    def _hold_station(self, side):
        """Re-send where this side already is. Called under the lock.

        Under the lock deliberately: it is one short packet, and releasing here
        would let a round fire in the middle of a send. Where it *is*, never
        where it is going - moving early would start a traverse outside the
        window meant to contain it.
        """
        held = self._at.get(side)
        if held is not None:
            self._command(side, held)
    # def

    def _report_contention(self, side, contended):
        """Tell this side, and only this side, that its reading was unguarded.

        Each participant reports its own, on its own thread. The leader knows
        the whole group and could file the lot, but a side's calibration model
        is written by that side's worker and read by the Tk thread on the
        strength of that - one thread posting into another side's chart is the
        race the snapshot arrangement exists to avoid.
        """
        if contended and self._on_contended is not None:
            self._on_contended(side)
    # def

    def _run(self, group):
        """Command every side, then hold one window and read each at its own dwell.

        The commands go out first so both surfaces set off together, and
        nothing is commanded again until every reading is taken, so the whole
        span is one still window. Within it each side is read when the dwell it
        asked for has elapsed rather than when the longest has: a creep step
        held for a partner's endpoint dwell is a creep step that settles
        further than the creep intends, which moves where it stops.
        """
        commands = {side: cmd for side, (cmd, _dwell) in group.items()}

        for side in sorted(commands):
            self._command(side, commands[side])

        angles = {}
        elapsed = 0.0
        for dwell_s in sorted({dwell for _cmd, dwell in group.values()}):
            if dwell_s > elapsed:
                self._hold(commands, dwell_s - elapsed)
                elapsed = dwell_s
            for side in sorted(commands):
                if group[side][1] == dwell_s:
                    angles[side] = self._read(side)

        return angles
    # def
# class
