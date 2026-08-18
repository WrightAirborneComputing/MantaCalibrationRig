"""The trim point's decision logic, with nothing attached to it.

The companion to endpoint_logic.py, and built the same way: every function
here is a decision about numbers - where the trim command should move to,
whether it has landed, what the written trim costs the travel. The driving
stays with the callers, which is what lets all of this be tested without a rig.

The trim point is defined by one approach direction, not by both agreeing.
These servos have more backlash than any acceptance band worth having, so the
opposite approach is measured once at the end and reported as the lash it is.

Imports endpoint_logic for the command/endpoint mapping and for usable_gain,
which guards the secant gain here for the same reason it guards the probed gain
there: a gain measured over a short move while the surface is still settling can
come out with the wrong sign, and correcting on it walks away from the target.

The procedure these implement is documented in SERVO_SETTING.md.
"""

from endpoint_logic import endpoint_command, usable_gain

# The approach the trim is set from. There is more backlash in these servos
# than any acceptance band could close, so the trim point is defined as the
# angle reached arriving from one named direction rather than as something both
# directions agree on. Change this constant and the whole procedure follows.
TRIM_APPROACH_CMD = 1.0

# How close the approach from TRIM_APPROACH_CMD must land. The other direction
# is measured once at the end and reported - it is not required to agree, and
# requiring it would mean never finishing.
TRIM_TOLERANCE_DEG = 0.25

# The most one attempt may move the trim command. About 5 deg on this airframe.
TRIM_MAX_CORRECTION_CMD = 0.15

# Refuse to carry the trim past here whatever the maths says. A trim of 0.9
# would give up 45% of the travel at one end, which is not a calibration.
TRIM_COMMAND_LIMIT = 0.9

TRIM_ATTEMPTS = 8

# Consecutive attempts whose error grew before the trim is called diverging,
# and the margin below which a change is not called growth. Same reasoning as
# the end stops: one growth can be noise, two in a row is the correction
# pushing the wrong way.
TRIM_DIVERGENCE_ATTEMPTS = 2
TRIM_DIVERGENCE_MARGIN_DEG = 0.1


def trim_estimate(target_deg, angle_at_neg_cmd, angle_at_pos_cmd):
    """The command that should sit on target, from the two accepted end stops.

    They are two known (command, angle) pairs at cmd -1 and cmd +1, so the trim
    command falls straight out of them - no creep needed to find it, and the
    first approach lands close enough to measure the local gain properly.

    Which end carries which command is the caller's business: pass the angle
    reached at cmd -1 first, and the reversal never enters into it.
    """
    span_deg = float(angle_at_pos_cmd) - float(angle_at_neg_cmd)
    if span_deg == 0.0:
        return None
    fraction = (float(target_deg) - float(angle_at_neg_cmd)) / span_deg
    return clamp_trim(2.0 * fraction - 1.0)
# def


def command_gain_deg(angle_at_neg_cmd, angle_at_pos_cmd):
    """Degrees per command unit across the whole travel.

    Coarse - the linkage gain varies about 2:1 end to end - but the trim sits
    mid-range where it is closest to this, and the secant below replaces it as
    soon as there are two attempts to draw a line through.
    """
    return (float(angle_at_pos_cmd) - float(angle_at_neg_cmd)) / 2.0
# def


def secant_gain_deg(previous_cmd, previous_angle, cmd, angle):
    """Local deg per command unit, from two attempts at the trim point.

    Both readings come from the same approach direction, so the lash is common
    to them and cancels in the difference. Measured where the trim actually
    sits rather than averaged across a travel the trim is nowhere near. Returns
    None when there is nothing to draw a line through yet.
    """
    if previous_cmd is None:
        return None
    delta_cmd = float(cmd) - float(previous_cmd)
    if abs(delta_cmd) < 1e-9:
        return None
    delta_angle = float(angle) - float(previous_angle)
    if abs(delta_angle) < 1e-9:
        return None
    return delta_angle / delta_cmd
# def


def trim_gain_deg(previous_cmd, previous_angle, cmd, angle, nominal_gain):
    """The gain to correct on: the secant when it is believable, else nominal.

    usable_gain() applies the same check the end stops use - same sign as the
    geometry, within 0.3x to 3x of it - because the same thing goes wrong here.
    A surface still settling between two attempts returns the settling rather
    than the gain, and a correction built on it walks away from the target.
    """
    secant = secant_gain_deg(previous_cmd, previous_angle, cmd, angle)
    gain, _source = usable_gain(secant, nominal_gain)
    return gain
# def


def backlash_deg(angle_from_approach, angle_from_opposite):
    """How far the opposite approach landed from the set one.

    Signed, and reported rather than corrected: it is the lash in the servo and
    the linkage, and no trim value makes it smaller. It is the number that says
    where the surface actually sits when it arrives the other way, which is
    worth having on the record for every airframe.
    """
    return float(angle_from_opposite) - float(angle_from_approach)
# def


def trim_accepted(angle_from_approach, target_deg,
                  tolerance=TRIM_TOLERANCE_DEG):
    """One direction, the one named by TRIM_APPROACH_CMD.

    Asking both to agree was tried and cannot be met: the lash on these servos
    is wider than any useful band, so a two-sided test would loop until it ran
    out of attempts and write nothing.
    """
    return abs(float(angle_from_approach) - float(target_deg)) <= tolerance
# def


def trim_correction(angle_deg, target_deg, gain_deg_per_cmd):
    """Command change that moves the set approach onto target."""
    if not gain_deg_per_cmd:
        return None
    return -(float(angle_deg) - float(target_deg)) / gain_deg_per_cmd
# def


def limit_trim_correction(delta_cmd, limit=TRIM_MAX_CORRECTION_CMD):
    if delta_cmd is None:
        return None
    delta = float(delta_cmd)
    if delta > limit:
        return limit
    if delta < -limit:
        return -limit
    return delta
# def


def clamp_trim(cmd, limit=TRIM_COMMAND_LIMIT):
    return min(limit, max(-limit, float(cmd)))
# def


def trim_error_grew(previous_error_deg, error_deg,
                    margin=TRIM_DIVERGENCE_MARGIN_DEG):
    if previous_error_deg is None:
        return False
    return abs(float(error_deg)) > abs(float(previous_error_deg)) + margin
# def


def trim_has_diverged(consecutive_growths):
    return int(consecutive_growths) >= TRIM_DIVERGENCE_ATTEMPTS
# def


def travel_lost_us(trim, span_us):
    """Microseconds of travel the written trim gives up at one end.

    PX4 computes clamp(cmd + trim, -1, +1), so with a trim applied one of the
    two full-scale commands no longer reaches its end stop. The arithmetic is
    exact: the end falls short by |trim| / 2 of the span.
    """
    return abs(float(trim)) / 2.0 * abs(float(span_us))
# def


def dead_band_cmd(trim):
    """Command range at the *other* end that produces no movement at all.

    The end the trim leans toward is reached early, so the last |trim| of
    command travel there does nothing. The cost of a trim is one end short and
    the other end deaf, and both are worth printing.
    """
    return abs(float(trim))
# def


def shortened_endpoint(trim, rev):
    """Which end stop cmd +-1 no longer reaches once the trim is written.

    Taken from the command mapping rather than from the PWM order, so it holds
    on a reversed channel: a negative trim shortens whichever end sits at cmd
    +1, and that is the MIN end when the channel is reversed.
    """
    if not trim:
        return None
    short_cmd = 1.0 if float(trim) < 0.0 else -1.0
    return "MAX" if endpoint_command("MAX", rev) == short_cmd else "MIN"
# def
