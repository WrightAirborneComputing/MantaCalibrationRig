"""What the calibration is doing, and why it stopped. Nothing attached to it.

The calibration reports itself by printing lines into the instrumentation log,
which is a fine record and a poor display: two sides run at once, so their lines
interleave, and a failure is one line among hundreds. This module holds the
model behind the flow charts that fix that - the nodes, their states, and the
wording of every way a run can end.

Imports nothing, like endpoint_logic.py next to it, and for the same reason:
every decision here is about a step name or a message, none of it needs a
window, and all of it can be tested without one. The drawing lives with the
caller, which is what keeps the geometry out of the wording and the wording out
of the geometry.

The procedure these describe is documented in SERVO_SETTING.md.
"""

# ---- States ---------------------------------------------------------------

PENDING = "pending"
RUNNING = "running"
DONE = "done"
WARNED = "warned"
FAILED = "failed"
SKIPPED = "skipped"

# Which PALETTE keys a state draws with: (outline, fill, text, width).
# Names, not colours - manta_theme owns the actual values.
STYLE = {
    PENDING: ("rule",      "panel",     "ink_faint", 1),
    RUNNING: ("accent",    "accent_lo", "ink",       2),
    DONE:    ("ok",        "panel",     "ink",       1),
    WARNED:  ("warn",      "panel_alt", "ink",       1),
    FAILED:  ("bad",       "panel_alt", "bad",       2),
    SKIPPED: ("rule_soft", "panel",     "ink_faint", 1),
}

# A state a node cannot move on from. Used to decide what a failure skips.
SETTLED = (DONE, WARNED, FAILED, SKIPPED)


# ---- The graph ------------------------------------------------------------

# (key, label, row, column). Column "full" spans the canvas; "left" and "right"
# are the two halves of a fork.
#
# The two ends fork because _cal_refine_endpoints alternates between them -
# both are live at once, each with its own attempt count, and a single line of
# progress cannot show that. The free travel check is split out from the find
# it belongs to because it is the step that fails, it costs up to 16 s every
# time it runs, and burying it inside "Find min" is exactly what made the
# failure this module exists for invisible.
NODES = (
    ("open_range",  "Open the range",        0, "full"),
    ("sweep",       "Initial sweep",         1, "full"),
    ("load_coarse", "Load coarse end stops", 2, "full"),
    ("find_max",    "Find max",              3, "left"),
    ("find_min",    "Find min",              3, "right"),
    ("travel_max",  "Free travel",           4, "left"),
    ("travel_min",  "Free travel",           4, "right"),
    ("verify",      "Verify the end stops",  5, "full"),
    ("trim",        "Find the trim point",   6, "full"),
    ("write_trim",  "Write the trim",        7, "full"),
    ("verify_trim", "Verify with the trim",  8, "full"),
)

EDGES = (
    ("open_range", "sweep"),
    ("sweep", "load_coarse"),
    ("load_coarse", "find_max"),
    ("load_coarse", "find_min"),
    ("find_max", "travel_max"),
    ("find_min", "travel_min"),
    ("travel_max", "verify"),
    ("travel_min", "verify"),
    ("verify", "trim"),
    ("trim", "write_trim"),
    ("write_trim", "verify_trim"),
)

NODE_KEYS = tuple(key for key, _label, _row, _col in NODES)
LABELS = {key: label for key, label, _row, _col in NODES}

# What a node is called away from its column. Inside the chart "Free travel"
# is unambiguous - it sits directly under "Find min" - but the failure box is
# below both columns, so there it has to say which end it means.
DETAIL_LABELS = dict(LABELS)
DETAIL_LABELS["travel_max"] = "Max free travel"
DETAIL_LABELS["travel_min"] = "Min free travel"

# Which node each end's work is reported against, so callers can say "MAX"
# rather than knowing the key.
FIND_NODE = {"MAX": "find_max", "MIN": "find_min"}
TRAVEL_NODE = {"MAX": "travel_max", "MIN": "travel_min"}


def node_for(which, stage):
    """The node key for one end's find or free-travel step."""
    table = FIND_NODE if stage == "find" else TRAVEL_NODE
    if which not in table:
        raise ValueError("unknown end %r" % which)
    return table[which]
# def


# ---- Failure wording ------------------------------------------------------

# key -> (headline, detail, advice). Both headline and detail are %-templates
# filled from the fields the caller passes; advice is fixed or empty.
#
# One wording per fault, in one place, so the same fault always reads the same
# way whichever of the six functions it was raised in. The prints at the
# failure sites stay exactly as they are - they are the detailed record, and
# several tests read them.
FAILURES = {
    "param_write": (
        "write refused",
        "%(param)s would not take %(value)s - the flight controller did not "
        "acknowledge the write.",
        "Check the link and that the vehicle is disarmed.",
    ),
    "command_exhausted": (
        "ran out of command",
        "Ran out of command before %(target)+.1f deg: full command %(command)+.3f "
        "reached only %(angle)+.2f deg. The servo will not travel that far.",
        "Check the horn is fitted in the right orientation. Otherwise the "
        "range asked for is wider than this servo allows, and the target "
        "needs lowering.",
    ),
    "creep_timeout": (
        "timed out",
        "Timed out before reaching %(target)+.1f deg: 60 s of stepping left the "
        "surface at %(angle)+.2f deg. It is moving too slowly, or not at all.",
        "",
    ),
    "no_angle": (
        "no angle",
        "The position reader gave no reading for this side.",
        "Check the Pico is streaming and that this side has been zeroed.",
    ),
    "actuator_refused": (
        "PX4 refused the test",
        "%(refusal)s.",
        "Check that the vehicle is disarmed, the safety switch is off, and "
        "COM_MOT_TEST_EN is 1.",
    ),
    "endpoint_load": (
        "write did not land",
        "The min/max write did not land. Every value measured after this "
        "would be against a range the flight controller is not using.",
        "",
    ),
    "pwm_convert": (
        "no PWM for those commands",
        "The sweep found commands the expected-PWM mapping cannot resolve. "
        "The min/max params or the reverse bit are inconsistent.",
        "",
    ),

    # The one this module exists for. Two strikes, not one: a single
    # unconfirmed probe could still be an end stop written far past the
    # surface's reach, and pulling in 210 us is the recovery for that. A second
    # one at the same end, after the endpoint has already moved 210 us inward,
    # is not a pin - it is friction.
    "free_travel_unconfirmed": (
        "stiction suspected",
        "Could not confirm the elevon moved off the end stop: %(ceiling).0f us "
        "of inward travel produced less than %(threshold).2f deg, twice in a "
        "row. Stiction is likely too high.",
        "Free the surface or reduce friction, then re-run this side.",
    ),
    "no_angle_at_stop": (
        "no angle here",
        "No angle at the %(which)s end stop: the position reader gave no reading "
        "while the surface was held there. A sensor or link fault, not a "
        "mechanical one.",
        "",
    ),

    "diverging": (
        "diverging",
        "%(error)+.2f deg out after %(previous)+.2f deg on the previous "
        "attempt - the correction is pushing the wrong way. The end stops "
        "have been left where they are.",
        "",
    ),
    "at_limit": (
        "against its limit",
        "%(which)s is against its limit at %(pwm)d us and still %(error)+.2f deg "
        "out. The %(target)+.1f deg target is past this servo's travel.",
        "Check the horn orientation, or reduce the target.",
    ),
    "no_settle": (
        "did not settle",
        "%(which)s did not settle in %(attempts)d attempts. The closest it came "
        "was %(error).2f deg out, against a %(tolerance).2f deg tolerance.",
        "",
    ),
    "no_gain": (
        "no usable gain",
        "The back-off probe measured no movement and the average deg/us is "
        "undefined, so there is no correction to make.",
        "",
    ),

    "trim_diverging": (
        "diverging",
        "%(error)+.2f deg out after %(previous)+.2f deg on the previous "
        "attempt.",
        "",
    ),
    "trim_pinned": (
        "pinned at its limit",
        "The trim pinned at command %(command)+.3f and the angle is still "
        "%(error)+.2f deg from the trim target.",
        "",
    ),
    "trim_no_settle": (
        "did not settle",
        "The trim did not settle in %(attempts)d attempts. The closest it came "
        "was %(error).2f deg out, against a %(tolerance).2f deg tolerance.",
        "",
    ),

    "unexpected": (
        "unexpected error",
        "%(error)s",
        "The run stopped here. The instrumentation log has the detail.",
    ),
}

STOPPED_NOTE = "Stopped by the operator"


# Field names render_failure and SideFlow.fail already use for themselves. A
# template asking for one of these would collide with the parameter and raise
# while reporting a failure, which is the worst possible time to raise.
RESERVED_FIELDS = ("reason", "key", "now", "self")


def render_failure(reason, **fields):
    """(headline, detail) for a failure key, filled from its fields.

    An unknown key or a missing field is reported rather than raised: a chart
    that says "no wording for 'foo'" is repairable, one that takes the run
    down with a KeyError while reporting a failure is not.
    """
    entry = FAILURES.get(reason)
    if entry is None:
        return "Failed", "No wording for %r." % reason

    headline, detail, advice = entry
    try:
        headline = headline % fields
        detail = detail % fields
    except (KeyError, TypeError, ValueError) as e:
        return headline.split("%")[0].strip() or "Failed", \
            "Could not describe %r: %s" % (reason, e)

    return headline, (detail + " " + advice).strip() if advice else detail
# def


def reached_note(targets, reached):
    """What each end actually reaches, against what it was asked for.

    "MAX +35.0->+28.4  MIN -33.0->-33.1" rather than a single worst-case
    number. The worst case says a run is out without saying which end or by
    how much, and after the trim is written one end is *expected* to fall
    short - by exactly the travel the trim spent. The two angles say that
    outright; "the worst error is 6.6 deg" leaves the reader to work out
    which end it was and what it now reaches.
    """
    parts = []
    for which in ("MAX", "MIN"):
        angle = (reached or {}).get(which)
        if angle is None:
            parts.append("%s %+.1f->?" % (which, targets[which]))
        else:
            parts.append("%s %+.1f->%+.1f" % (which, targets[which], angle))
    return "  ".join(parts)
# def


def worst_error(targets, reached):
    """The largest distance from target across both ends, or None."""
    errors = [abs(angle - targets[which])
              for which, angle in (reached or {}).items() if angle is not None]
    return max(errors) if errors else None
# def


def fit(text, measure, room_px):
    """`text` shortened until it fits, with an ellipsis to say it was cut.

    A node is drawn at a fixed width, so a note longer than that would run over
    its own border and into the state mark. Cutting it is safe here in a way it
    would not be elsewhere: everything a failure has to say in full is in the
    detail box below, and the node is only the pointer to it.
    """
    if not text or measure(text) <= room_px:
        return text
    if measure("...") > room_px:
        return ""
    trimmed = text
    while trimmed and measure(trimmed + "...") > room_px:
        trimmed = trimmed[:-1]
    return trimmed.rstrip() + "..."
# def


# ---- One side's run -------------------------------------------------------

IDLE = "idle"
STOPPED = "stopped"


class SideFlow:
    """The state of one side's calibration, as the chart needs to draw it.

    Mutated only by that side's worker thread, and read only through
    snapshot(), which returns tuples. That split is load-bearing: the drawing
    happens on the Tk thread, and handing it a live model would let it iterate
    a dict the worker is still growing. _drain_gui_queue swallows the
    RuntimeError that produces, which would leave the chart silently frozen -
    the exact failure a chart is supposed to prevent.

    Clock-free on purpose. `now` comes in from the caller so the module keeps
    importing nothing, and so a test's fake clock reaches it for free.
    """

    def __init__(self):
        self.reset(0.0)
    # def

    def reset(self, now):
        self.states = {key: PENDING for key in NODE_KEYS}
        self.notes = {key: "" for key in NODE_KEYS}
        self.started_at = float(now)
        self.finished_at = None
        self.verdict = IDLE
        self.failure = None
    # def

    # ---- transitions ----

    def start(self, key, note=""):
        """Mark a node running. The first one starts the clock's verdict."""
        self._require(key)
        if self.verdict == IDLE:
            self.verdict = RUNNING
        self.states[key] = RUNNING
        self.notes[key] = note
    # def

    def note(self, key, note):
        """Update what a node is saying without changing its state.

        This is what makes the free travel check bearable to watch: at the
        ceiling the probe takes 20 steps of 0.8 s, and a node that only
        reported at its start and end would sit still for 16 seconds.
        """
        self._require(key)
        self.notes[key] = note
    # def

    def finish(self, key, note=""):
        self._require(key)
        self.states[key] = DONE
        self.notes[key] = note
    # def

    def warn(self, key, note=""):
        """Done, but worth a look. The run carries on."""
        self._require(key)
        self.states[key] = WARNED
        self.notes[key] = note
    # def

    def fail(self, key, reason, now, **fields):
        """End the side here. Everything unfinished is skipped, not left pending.

        A chart still showing later steps as pending after the run has ended
        reads as "not there yet" when the truth is "never going to happen".
        """
        self._require(key)
        headline, detail = render_failure(reason, **fields)

        self.states[key] = FAILED
        self.notes[key] = headline
        self.failure = (DETAIL_LABELS[key], headline, detail, reason)
        self.verdict = FAILED
        self.finished_at = float(now)
        self._skip_unsettled()
    # def

    def stop(self, now):
        """The operator pressed stop. Not a failure, and nothing goes red."""
        for key in NODE_KEYS:
            if self.states[key] == RUNNING:
                self.notes[key] = STOPPED_NOTE
        self.verdict = STOPPED
        self.finished_at = float(now)
        self._skip_unsettled()
    # def

    def complete(self, now):
        self.verdict = DONE
        self.finished_at = float(now)
    # def

    # ---- reading ----

    def snapshot(self, now):
        """Everything the renderer needs, as tuples it can hold on to.

        Built on the worker thread and handed across; nothing here refers back
        to this object.
        """
        rows = tuple(
            (key, label, row, column, self.states[key], self.notes[key])
            for key, label, row, column in NODES
        )
        return (self.verdict, self.elapsed(now), rows, self.failure)
    # def

    def elapsed(self, now):
        if self.verdict == IDLE:
            return None
        end = self.finished_at if self.finished_at is not None else float(now)
        return max(0.0, end - self.started_at)
    # def

    # ---- internals ----

    def _require(self, key):
        if key not in self.states:
            raise ValueError("unknown node %r" % key)
    # def

    def _skip_unsettled(self):
        for key in NODE_KEYS:
            if self.states[key] not in SETTLED:
                self.states[key] = SKIPPED
    # def
# class


def format_elapsed(seconds):
    """m:ss, or "--" when nothing has started."""
    if seconds is None:
        return "--"
    total = int(seconds)
    return "%d:%02d" % (total // 60, total % 60)
# def


def header_text(verdict, seconds):
    """The one-line verdict a side's chart is titled with."""
    if verdict == IDLE:
        return IDLE
    return "%s %s %s" % (verdict, "·", format_elapsed(seconds))
# def
