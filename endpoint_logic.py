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

# The wide-open range the coarse creep crosses.
COARSE_MIN = 900
COARSE_MAX = 2100

# Refuse to write an endpoint outside this band, whatever the maths says.
PWM_FLOOR = 800
PWM_CEILING = 2200


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


def clamp_endpoint(pwm):
    return int(round(min(PWM_CEILING, max(PWM_FLOOR, float(pwm)))))
# def


