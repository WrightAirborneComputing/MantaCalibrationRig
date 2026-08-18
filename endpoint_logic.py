"""The end stop calibration's decision logic, with nothing attached to it.

Separated from endpoint_cal.py so MantaTrimmer can use the same implementation
without importing it - endpoint_cal reaches the flight controller through
servo_probe.py, which is a temporary diagnostic meant to be deleted, and the
production tool must not depend on something disposable.

Imports nothing. Every function here is a decision about numbers: where an end
stop should move to, whether it is jammed, whether it is close enough. The
driving - commanding surfaces, waiting for them, reading angles - stays with the
callers, which is what lets all of this be tested without a rig.

The procedure these implement is documented in SERVO_SETTING.md.
"""

# Acceptance, as agreed on the rig.
ENDPOINT_TOLERANCE_DEG = 0.5
MOVEMENT_THRESHOLD_DEG = 0.25      # above the 0.23-0.27 deg hold noise floor measured
BACKOFF_STEP_US = 10.0
HARD_STOP_LIMIT_US = 50.0          # breakaway beyond this counts as a hard stop
HARD_STOP_MARGIN_US = 10.0         # pull in by breakaway + this
BACKOFF_CEILING_US = 200.0         # give up looking for breakaway past here
ENDPOINT_DWELL_S = 0.8             # one 0.5 s averaging window plus margin
MAX_ATTEMPTS = 10

# The wide-open range the coarse creep crosses, and with it the band a refined
# endpoint may be written in - the same numbers deliberately. The calibration
# opens the range to these before it creeps, so an endpoint outside them is one
# the creep could not have found in the first place.
COARSE_MIN = 900
COARSE_MAX = 2100

# Refuse to write an endpoint outside that band, whatever the maths says. These
# are servo limits, not PX4 limits - PWM_MAIN_MIN accepts 800 quite happily, and
# a RIGHT run that wrote 800 drove the elevon so far past its stop that the
# flight controller browned out mid-calibration.
PWM_FLOOR = COARSE_MIN
PWM_CEILING = COARSE_MAX

# MIN and MAX must stay this far apart. PX4 silently swaps them at param load
# when MIN > MAX, so a crossed pair does not fail - it quietly calibrates
# against a range nobody asked for, which is what a runaway MIN did on the rig.
MIN_ENDPOINT_SPAN_US = 200

# How far a refined endpoint may wander from where the coarse creep found it.
# The creep is rough, but it is measured: an endpoint 300 us away from it is not
# a correction, it is a symptom.
ENDPOINT_DRIFT_LIMIT_US = 300.0

# The most one attempt may move an endpoint. A gain that passes the sanity check
# below can still be off by a factor of two, and spending another attempt costs
# one traverse - far cheaper than driving the surface into its stop.
MAX_CORRECTION_US = 200.0

# A probed gain is believed when it agrees in sign with the nominal one and
# lands within these multiples of it. Outside that it is settling drift, not
# linkage gain.
GAIN_SANITY_LOW = 0.3
GAIN_SANITY_HIGH = 3.0

# Consecutive attempts whose error grew before an end stop is called diverging.
# One growth can be noise on a surface that is nearly there; two in a row is the
# correction pushing the wrong way.
DIVERGENCE_ATTEMPTS = 2
DIVERGENCE_MARGIN_DEG = MOVEMENT_THRESHOLD_DEG


def endpoint_command(which, rev):
    """The command that drives the output to MAX or MIN at trim 0.

    Not reversed, +1 runs to MAX; reversed, the mapping flips. Derived from the
    same expected_pwm() the probe verified against the FC to 0 us.
    """
    if which not in ("MAX", "MIN"):
        raise ValueError("unknown endpoint %r" % which)
    if which == "MAX":
        return -1.0 if rev else 1.0
    return 1.0 if rev else -1.0
# def


def inward_sign(which):
    """Which way the PWM moves when backing off an endpoint into the range."""
    return -1.0 if which == "MAX" else 1.0
# def


def command_delta_for_pwm(delta_us, pwm_min, pwm_max, rev):
    """Command change that moves the output by delta_us microseconds."""
    span = abs(float(pwm_max) - float(pwm_min))
    if span <= 0.0:
        raise ValueError("degenerate PWM span %r..%r" % (pwm_min, pwm_max))
    delta_cmd = (2.0 * float(delta_us)) / span
    return -delta_cmd if rev else delta_cmd
# def


def hard_stop_verdict(breakaway_us):
    """Is the endpoint jammed, and if so how far in should it be pulled?

    breakaway_us is how far inward the output had to travel before the elevon
    moved by MOVEMENT_THRESHOLD_DEG, or None if it never moved within
    BACKOFF_CEILING_US. Within HARD_STOP_LIMIT_US the servo still has authority
    at the endpoint, which is the whole point of the test.
    """
    if breakaway_us is None:
        return True, BACKOFF_CEILING_US + HARD_STOP_MARGIN_US
    if breakaway_us <= HARD_STOP_LIMIT_US:
        return False, 0.0
    return True, breakaway_us + HARD_STOP_MARGIN_US
# def


def angle_gain_per_us(angle_at_endpoint, angle_backed_off, backed_off_us, which):
    """Signed deg/us at this endpoint, measured by the back-off probe itself.

    Taking the gain from the probe rather than assuming it removes any need to
    know the linkage geometry or the reversal - the sign falls out of the
    measurement. Returns None when the probe saw no usable movement.
    """
    if backed_off_us is None or backed_off_us <= 0.0:
        return None
    delta_angle = float(angle_backed_off) - float(angle_at_endpoint)
    delta_pwm = inward_sign(which) * float(backed_off_us)
    if abs(delta_angle) < 1e-9:
        return None
    return delta_angle / delta_pwm
# def


def correction_us(angle_deg, target_deg, gain_deg_per_us):
    """Microseconds to shift the endpoint so the rapid approach hits target."""
    if not gain_deg_per_us:
        return None
    error_deg = float(angle_deg) - float(target_deg)
    return -error_deg / gain_deg_per_us
# def


def travel_direction(target_deg, other_target_deg):
    """+1 when this end lies at higher angles than the other, -1 otherwise.

    Taken from the two targets rather than from the reverse bit, so it holds for
    either wiring and for asymmetric travel.
    """
    return 1.0 if float(target_deg) > float(other_target_deg) else -1.0
# def


def overshot_target(angle_deg, target_deg, other_target_deg,
                    tolerance=ENDPOINT_TOLERANCE_DEG):
    """True when the surface travelled *past* its target, away from the far end.

    A surface beyond its target plainly has travel to spare, so probing it for a
    hard stop answers a question nobody asked and costs a back-off cycle to do
    it. Probing earns its place when the surface landed on target - where being
    jammed is the difference between a good end stop and a meaningless one - or
    fell short, where jamming is the likely explanation.

    An end stop 17 deg past target needs pulling in, and that is true whether or
    not there is a stop somewhere beyond it.
    """
    direction = travel_direction(target_deg, other_target_deg)
    return (float(angle_deg) - float(target_deg)) * direction > tolerance
# def


def nominal_gain_deg_per_us(target_at_max, target_at_min, pwm_min, pwm_max):
    """Average deg/us across the whole travel, from the targets and the span.

    A stand-in for the local gain the back-off probe measures, for corrections
    made without probing. Coarse - the linkage gain was measured to vary about
    2:1 end to end - but a correction of many degrees does not need a precise
    gain, and the attempt that follows lands near target and measures properly.
    """
    span = float(pwm_max) - float(pwm_min)
    if span == 0:
        return None
    return (float(target_at_max) - float(target_at_min)) / span
# def


def endpoint_accepted(angle_deg, target_deg, is_hard_stop):
    return (not is_hard_stop
            and abs(float(angle_deg) - float(target_deg)) <= ENDPOINT_TOLERANCE_DEG)
# def


def alternating_order(rounds):
    """MAX, MIN, MAX, MIN ... - the traverse between ends is the rapid approach."""
    order = []
    for _ in range(int(rounds)):
        order.extend(("MAX", "MIN"))
    return order
# def


def usable_gain(probed_gain, nominal_gain):
    """The gain to correct on, and where it came from: (gain, source).

    The probe measures the local gain over a single 10 us step taken 0.8 s after
    a full-span slam into the end stop, while the surface is still creeping back
    from the overshoot. When the creep is larger than the step the subtraction
    returns the creep, and on a RIGHT run it returned it with the wrong sign -
    three corrections in a row pushed MIN away from its target, 1.8 deg of error
    becoming 44 deg. The magnitude looked right the whole way.

    So the probed gain is believed only when it agrees with what the geometry
    says it must be: the same sign, and within a factor of GAIN_SANITY_HIGH
    either way. The linkage gain was measured to vary about 2:1 end to end,
    which that band comfortably holds. Anything else is the creep, and the
    nominal gain - crude, but right about which way the surface moves - is the
    better number to correct on.
    """
    if not nominal_gain:
        return (probed_gain, "probe") if probed_gain else (None, "none")
    if not probed_gain:
        return nominal_gain, "nominal"

    if (probed_gain > 0.0) != (nominal_gain > 0.0):
        return nominal_gain, "nominal (probe disagreed on direction)"

    ratio = abs(probed_gain) / abs(nominal_gain)
    if ratio < GAIN_SANITY_LOW or ratio > GAIN_SANITY_HIGH:
        return nominal_gain, "nominal (probe was %.1fx nominal)" % ratio

    return probed_gain, "probe"
# def


def limit_correction(shift_us, limit=MAX_CORRECTION_US):
    """Cap one attempt's shift. None passes through untouched."""
    if shift_us is None:
        return None
    shift = float(shift_us)
    if shift > limit:
        return limit
    if shift < -limit:
        return -limit
    return shift
# def


def error_grew(previous_error_deg, error_deg):
    """Did this attempt land further out than the last one at the same end?

    The margin is the hold noise floor, so a surface sitting still is never
    called worse than it was.
    """
    if previous_error_deg is None:
        return False
    return abs(float(error_deg)) > abs(float(previous_error_deg)) + DIVERGENCE_MARGIN_DEG
# def


def has_diverged(consecutive_growths):
    return int(consecutive_growths) >= DIVERGENCE_ATTEMPTS
# def


def clamp_endpoint(pwm, which=None, other_pwm=None, anchor_pwm=None):
    """The value that may actually be written, given everything constraining it.

    Four bounds, narrowest wins: the servo band, the coarse creep's finding, and
    - when the other end is known - the requirement that MIN and MAX stay
    MIN_ENDPOINT_SPAN_US apart. PX4 swaps them at param load when MIN > MAX, so
    a crossed pair calibrates on silently and every reading after it is taken
    against a range nobody chose.

    which and other_pwm come as a pair; either missing and the crossing check is
    skipped, which is what the old single-argument callers get.
    """
    low = float(PWM_FLOOR)
    high = float(PWM_CEILING)

    if anchor_pwm is not None:
        low = max(low, float(anchor_pwm) - ENDPOINT_DRIFT_LIMIT_US)
        high = min(high, float(anchor_pwm) + ENDPOINT_DRIFT_LIMIT_US)

    if which is not None and other_pwm is not None:
        if which == "MIN":
            high = min(high, float(other_pwm) - MIN_ENDPOINT_SPAN_US)
        else:
            low = max(low, float(other_pwm) + MIN_ENDPOINT_SPAN_US)

    # Bounds that cannot all be met means the two ends are already too close or
    # the anchor sits outside the servo band. Keeping the ends apart matters
    # more than honouring the anchor, so the upper bound wins.
    if low > high:
        low = high

    return int(round(min(high, max(low, float(pwm)))))
# def


