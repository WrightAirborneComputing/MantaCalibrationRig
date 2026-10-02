"""The six-face sensor calibration's decision logic, with nothing attached to it.

Separated from sensor_cal.py the same way endpoint_logic.py is separated from
endpoint_cal.py, and for the same reason: everything here is a decision about
numbers - which face is the plate on, has it stopped moving, what bias and scale
make gravity come out at 1 g, is the answer believable - and none of it needs a
rig to test. The driving (opening the port, prompting the operator, waiting)
stays in sensor_cal.py.

Imports `math`, `statistics` and `re` and nothing else. In particular it does
not import range_test, whose `_solve` is the same Gaussian elimination used
below: range_test pulls in pyserial and pico_monitor, and a module that exists
to be importable headless and tested without hardware must not acquire a
transport dependency to borrow twenty lines of arithmetic. No numpy, per the
standing policy in range_test.fit_polynomial's docstring.

The procedure these implement is documented in SENSORS.md, "The six-face
calibration".
"""

import math
import re
import statistics


# Channel order is fixed by the firmware: LEFT, CENTRE, RIGHT are Serial1/2/3.
CHANNELS = ("LEFT", "CENTRE", "RIGHT")

AXES = ("X", "Y", "Z")

# WitMotion scaling, from SENSORS.md "The wire format is WitMotion".
# Acceleration is raw / 32768 * 16 g; angular velocity is raw / 32768 * 2000 dps.
ACC_COUNTS_PER_G = 32768.0 / 16.0        # 2048
GYRO_COUNTS_PER_DPS = 32768.0 / 2000.0   # 16.384

# One raw sample line from teensy/rig, three channels, always in CHANNELS order:
#
#   {<t_us>:<ax>,<ay>,<az>,<gx>,<gy>,<gz>|<...>|<...>}
#
# Braces rather than the brackets pico/sampler.py uses, because SENSORS.md asks
# raw mode to carry a different delimiter from degrees mode: a host that missed
# a mode-change acknowledgement then physically cannot mistake raw counts for
# degrees, and the failure that prevents is writing garbage angles into a
# drone's calibration. A channel with no fresh frame emits "x" - an explicit
# sentinel, because a number cannot be allowed to masquerade as a reading.
SAMPLE_REGEX = re.compile(r"\{(\d+):([-\dx,|]+)\}")

# The corrected counterpart, in brackets rather than braces:
#
#   [<t_us>:<ax_ug>,<ay_ug>,<az_ug>,<gx>,<gy>,<gz>|<...>|<...>]
#
# Acceleration in micro-g, because the modules' own noise floor is 0.4 to 1.4 mg
# and milli-g would quantise at the noise. Still a vector, not an angle: the
# correction belongs in the native format, and the scalar is formed once, at the
# end, by whoever knows which plane they want it in.
CORRECTED_REGEX = re.compile(r"\[(\d+):([-\dx,|]+)\]")

MICRO_G = 1000000.0

# Board-to-host replies, as everywhere else on this rig.
REPLY_PREFIX = "#"


# --- Faces -----------------------------------------------------------------

# Gravity, in the sensor's own frame, for each of the six faces. "Z+" names the
# face where the measured acceleration vector points along +Z, which is the
# plate resting with its -Z side down. Naming the measurement rather than the
# physical side is deliberate: the operator's idea of which side is "the top"
# is exactly the thing this procedure must not depend on.
FACE_VECTORS = {
    "X+": (1.0, 0.0, 0.0),
    "X-": (-1.0, 0.0, 0.0),
    "Y+": (0.0, 1.0, 0.0),
    "Y-": (0.0, -1.0, 0.0),
    "Z+": (0.0, 0.0, 1.0),
    "Z-": (0.0, 0.0, -1.0),
}

# Suggested order, in which every consecutive pair is a single 90-degree flip
# rather than a re-orientation. This is a Hamiltonian path on the face
# adjacency graph - each face is adjacent to the four that are not its opposite
# - and one exists, so no step in the sequence has to be a 180-degree roll.
#
# It is a suggestion and nothing more. capture_state below accepts whichever of
# the six faces the plate is actually presenting and ticks it off, because the
# operator will not get the absolute orientation right and does not need to:
# the whole method depends on relative readings, so demanding a named face at a
# named step would add a way to fail without adding any information.
SUGGESTED_FACE_ORDER = ("Z+", "X+", "Y+", "Z-", "X-", "Y-")

# Acceptance for how flat a face is, in degrees from nominal. The justification
# is in SENSORS.md "Guiding the operator": an 8 degree tilt costs 1.0% of scale
# under a naive min/max solve, but refine_solution uses the measured vector, so
# what survives is well under 0.1 degrees - far below the 0.23-0.27 degree hold
# noise floor in endpoint_logic.py. Past 15 degrees the face classification
# itself turns ambiguous, so REJECT is a correctness gate, not a quality one.
FACE_TILT_WARN_DEG = 8.0
FACE_TILT_REJECT_DEG = 15.0

# A face is only classified when its dominant axis is genuinely dominant. At 15
# degrees off nominal the dominant component is cos(15)=0.966 and the largest
# other is sin(15)=0.259, a ratio of 3.7; at 45 degrees two axes tie exactly.
# Requiring 2.0 puts the cut at about 26.6 degrees, comfortably outside the
# reject threshold, so a plate the classifier accepts is never a plate the tilt
# gate is being asked to adjudicate on ambiguous evidence.
FACE_DOMINANCE_RATIO = 2.0


# --- Motion gating ---------------------------------------------------------

# The gyro reads exactly zero on all three axes while stationary (SENSORS.md,
# "The wire format is WitMotion"), which is what makes it usable as a
# first-order motion gate.
#
# This was 1.0 dps, which is a bench threshold, and the plate is held in two
# hands. The number to gate on is not "how still can a person be" but "how still
# does this measurement actually need them to be", and the answer is: barely at
# all. Simulated against a hand-held plate wandering 5 degrees, the recovered
# scale is wrong by 0.0036% - which at 35 degrees of elevon is 0.0013 degrees,
# two orders below the 0.23-0.27 degree hold noise floor in endpoint_logic.py.
# Wander is simply not what limits this calibration.
#
# So the gate's real job is to tell a plate being *held* apart from a plate
# being *carried to the next face*, and 10 dps splits those cleanly - a
# deliberate reorientation runs to tens or hundreds.
QUIET_GYRO_DPS = 10.0

# Gravity magnitude may not depart from 1 g by more than this for a sample to
# count as static. This is an envelope check, not a precision one, and the
# distinction is the whole trap: the accelerometer being gated here is the
# uncalibrated one, so a perfectly still plate legitimately reads well off 1 g.
# At the edge of what sanity_problems will accept - scale 0.90, bias 100 mg -
# a face reads 0.80 g, and a tight bound here would reject exactly the sensors
# this procedure exists to fix. It was set to 0.05 g first, and a synthetic
# capture with a 0.974 scale and a 45 mg bias lost two of its six faces to it.
#
# So 0.25 g, which spans that envelope with margin. The real stillness signal is
# the gyro, which is unbiased and reads exactly zero stationary; this only
# catches the case the gyro cannot, which is a plate under enough linear
# acceleration to matter.
QUIET_ACC_TOLERANCE_G = 0.25

# How long the plate must stay quiet, on one face, before a capture starts.
# Long enough not to trigger mid-flip, short enough not to be a test of nerve.
SETTLE_SECONDS = 0.75

# How far the plate must be disturbed after a capture before the wizard will
# look for the next face. Without this the tracker re-captures the face it is
# already sitting on the instant the capture ends. Well above QUIET_GYRO_DPS so
# that merely failing to hold still is never mistaken for moving on.
RELEASE_GYRO_DPS = 60.0

# A capture needs this many accepted samples, not this many seconds. At 200 Hz
# it is a fraction of a second of data, and the measurement says that is
# genuinely enough: against the bench noise floor, 50 samples already pin bias
# to 0.03 mg and scale to 0.003%, where 6000 samples get to 0.003 mg and
# 0.0003%. Both are so far inside the sanity envelope - 100 mg and 10% - that
# the difference is not a difference. Time spent holding buys nothing here, and
# under wander it actively costs; see mean_observation.
MIN_CAPTURE_SAMPLES = 200


# --- Solution acceptance ---------------------------------------------------

# Sanity gates on the answer itself, from SENSORS.md "Judging the result". A
# scale outside this or a bias past SANITY_MAX_BIAS_G means an axis is
# mislabelled or a face was badly wrong - refuse to write, and say which sensor
# and which axis.
SANITY_MIN_SCALE = 0.90
SANITY_MAX_SCALE = 1.10
SANITY_MAX_BIAS_G = 0.100

# The plate check compares, for each pair of sensors, the angle each one turned
# through between two orientations. Both are bolted to the same plate, so they
# turned through the same angle, and any disagreement is measurement.
#
# This is not the check SENSORS.md describes, and the difference matters. That
# one compares the angle between the two sensors' corrected gravity vectors and
# asserts it is "the same in every orientation". It is not. If sensor B's frame
# is a fixed rotation R from sensor A's, that angle is angle(g, R g), which
# depends on where gravity sits relative to R's axis: it runs from zero when g
# lies along the axis up to the full rotation angle when g is perpendicular to
# it. Measured against synthetic data, a 2 degree mounting misalignment gives a
# spread of exactly 2.000 degrees on a perfectly rigid plate whose sensors are
# perfectly calibrated - which the documented thresholds would reject. The
# modules are bolted on by hand and 2 degrees is unremarkable, so that is a
# guaranteed false failure rather than a hypothetical one.
#
# The inter-face form has no such term. Relative rotation is what both sensors
# genuinely share, so the comparison is exact regardless of how they are
# mounted, and it still catches everything the original was aimed at: a plate
# that flexed, a sensor that shifted mid-run, or a solve that is wrong, because
# all three break the correspondence between one sensor's geometry and the
# other's.
#
# Gated on the rigid-fit residual: with the mounting rotation fitted out, what
# is left is whether the modules stayed put. Set from the two real runs of
# 2026-08-28, nine orientations each and hand-held throughout:
#
#   clean run                 0.091 deg rms   (0.17 max)
#   run with a module loose   1.04 - 1.32 deg rms   (2.31 max)
#
# Per pair, the clean run came in at 0.091 / 0.091 / 0.155 and the loose one at
# 1.322 / 1.039 / 0.500. Reject at 0.40 rather than 0.50 because 0.50 lands
# exactly on that last figure, and a threshold a real measurement sits on is a
# coin toss rather than a decision. At 0.40 the good run has 2.6x of margin
# below and the bad run's weakest pair 1.25x above, with the two populations
# themselves an order of magnitude apart.
#
# Gating on the rms rather than the max, because one awkward orientation should
# not condemn a run on its own.
PLATE_DISAGREE_WARN_DEG = 0.25
PLATE_DISAGREE_REJECT_DEG = 0.40

PLATE_MOUNT_NOTE_DEG = 5.0


# --- Vector helpers --------------------------------------------------------

def vector_norm(v):
    """Euclidean length of a 3-vector."""
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
# def


def angle_between_deg(a, b):
    """Angle between two 3-vectors in degrees, or None if either is degenerate.

    The dot product is clamped before acos because a unit vector dotted with
    itself comes out at 1.0000000000000002 often enough to matter, and acos of
    that raises rather than returning zero.
    """
    na, nb = vector_norm(a), vector_norm(b)
    if na < 1e-9 or nb < 1e-9:
        return None
    dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))
# def


def counts_to_g(counts):
    """Raw accelerometer counts to g, per axis."""
    return tuple(c / ACC_COUNTS_PER_G for c in counts)
# def


def counts_to_dps(counts):
    """Raw gyro counts to degrees per second, per axis."""
    return tuple(c / GYRO_COUNTS_PER_DPS for c in counts)
# def


# --- Wire format -----------------------------------------------------------

def parse_sample_line(line):
    """One raw sample line to {"t_us": int, "LEFT": (ax,ay,az,gx,gy,gz), ...}.

    Returns None for anything that is not a sample line, which includes every
    "#"-prefixed reply, so replies stay inert to this parser exactly as they are
    to POSITION_REGEX on the Pico side. A channel that emitted the "x" sentinel
    maps to None rather than to zeros: no reading is a different thing from a
    reading of zero, and zeros would average into a plausible wrong answer.
    """
    match = SAMPLE_REGEX.search(line)
    if not match:
        return None

    fields = match.group(2).split("|")
    if len(fields) != len(CHANNELS):
        return None

    sample = {"t_us": int(match.group(1))}
    for name, field in zip(CHANNELS, fields):
        if field == "x":
            sample[name] = None
            continue
        parts = field.split(",")
        if len(parts) != 6:
            return None
        try:
            sample[name] = tuple(int(p) for p in parts)
        except ValueError:
            return None
    return sample
# def


def parse_corrected_line(line):
    """A corrected sample line to the same shape parse_sample_line returns.

    Values stay in the board's micro-g integers rather than being scaled here,
    so the two parsers return the same kind of thing and the caller decides what
    a count means. Mixing the two up is what the differing delimiters exist to
    prevent, and a parser that silently accepted either would give that away.
    """
    match = CORRECTED_REGEX.search(line)
    if not match:
        return None

    fields = match.group(2).split("|")
    if len(fields) != len(CHANNELS):
        return None

    sample = {"t_us": int(match.group(1))}
    for name, field in zip(CHANNELS, fields):
        if field == "x":
            sample[name] = None
            continue
        parts = field.split(",")
        if len(parts) != 6:
            return None
        try:
            sample[name] = tuple(int(p) for p in parts)
        except ValueError:
            return None
    return sample
# def


def split_sample(reading):
    """A six-tuple of counts to (acceleration in g, angular velocity in dps)."""
    return counts_to_g(reading[0:3]), counts_to_dps(reading[3:6])
# def


# --- Face classification ---------------------------------------------------

def classify_face(acc_g):
    """Which of the six faces this acceleration vector represents, or None.

    None means no axis dominates by FACE_DOMINANCE_RATIO, which is the honest
    answer for a plate held at an angle - and is what lets the wizard tell
    "still moving it" apart from "resting on a face, badly".
    """
    magnitudes = [abs(a) for a in acc_g]
    order = sorted(range(3), key=lambda i: magnitudes[i], reverse=True)
    top, second = order[0], order[1]

    if magnitudes[second] < 1e-9:
        pass
    elif magnitudes[top] / magnitudes[second] < FACE_DOMINANCE_RATIO:
        return None

    return AXES[top] + ("+" if acc_g[top] >= 0 else "-")
# def


def tilt_from_nominal_deg(acc_g, face):
    """How far off flat this face is, in degrees. The direct quality measure.

    Reported per face per sensor because "LEFT is 14 degrees off" is actionable
    while "a sensor is off" is not, and the operator is holding a plate with
    both hands while reading it.
    """
    return angle_between_deg(acc_g, FACE_VECTORS[face])
# def


def face_verdict(tilt_deg):
    """"ok", "warn" or "reject" for a face's tilt from nominal."""
    if tilt_deg is None:
        return "reject"
    if tilt_deg >= FACE_TILT_REJECT_DEG:
        return "reject"
    if tilt_deg >= FACE_TILT_WARN_DEG:
        return "warn"
    return "ok"
# def


def sample_is_quiet(sample, channels=CHANNELS):
    """Is the plate static, by both the gyro and the gravity magnitude?

    Both tests, not either: the gyro misses pure linear translation and the
    magnitude test misses rotation about the gravity vector. Together they miss
    only a constant-velocity translation, which a hand does not produce.
    """
    seen = False
    for name in channels:
        reading = sample.get(name)
        if reading is None:
            continue
        seen = True
        acc, gyro = split_sample(reading)
        if vector_norm(gyro) > QUIET_GYRO_DPS:
            return False
        if abs(vector_norm(acc) - 1.0) > QUIET_ACC_TOLERANCE_G:
            return False
    return seen
# def


def sample_face(sample, channels=CHANNELS):
    """The face all live channels agree on, or None if they do not.

    Disagreement is a real finding rather than a nuisance: the sensors are
    bolted to one plate in the same orientation, so they must classify the same
    face. If they do not, either a module is mounted rotated or a channel is
    wired to the wrong UART - which is exactly the class of fault that produces
    a plausible-looking and completely wrong calibration.
    """
    faces = set()
    for name in channels:
        reading = sample.get(name)
        if reading is None:
            continue
        acc, _ = split_sample(reading)
        face = classify_face(acc)
        if face is None:
            return None
        faces.add(face)
    if len(faces) != 1:
        return None
    return faces.pop()
# def


# --- The face-change state machine -----------------------------------------

class FaceTracker:
    """Turns a stream of samples into "the plate has settled on face F" events.

    Three states, and the third is the one that is easy to leave out:

        WAITING  - looking for a quiet, classified, not-yet-captured face
        SETTLING - found one, timing how long it stays
        RELEASED - a capture just finished; wait for the plate to be picked up

    Without RELEASED the tracker re-fires on the face it is already resting on
    the moment a capture ends, and the operator gets six captures of face one.
    The release test is on the gyro rather than on the classification changing,
    because lifting a plate and putting it back on the same face is a legitimate
    thing to do after a rejected capture.

    Time is passed in rather than read, so the tests can drive this at any speed
    without sleeping.
    """

    def __init__(self, settle_seconds=SETTLE_SECONDS, channels=CHANNELS):
        self.settle_seconds = settle_seconds
        self.channels = channels
        self.captured = {}
        self.state = "WAITING"
        self._face = None
        self._since = None
    # def

    def settling_on(self):
        """The face currently being timed, for the operator hint. May be None."""
        return self._face
    # def

    def restart(self):
        """Abandon a capture and require the plate to be lifted before retrying.

        RELEASED rather than WAITING, and the difference matters. After a
        rejected face the plate is still sitting on that face, so a tracker that
        merely went back to looking would settle on it again and re-capture the
        same bad hold - forever, if the face is genuinely too far off flat.
        Requiring a lift makes the retry something the operator does rather than
        something that happens to them.
        """
        self.state = "RELEASED"
        self._face = None
        self._since = None
    # def

    def mark_captured(self, face, record):
        """Record a completed capture and require a disturbance before the next."""
        self.captured[face] = record
        self.state = "RELEASED"
        self._face = None
        self._since = None
    # def

    def forget(self, face):
        """Drop a face so it can be presented again, after a rejected capture."""
        self.captured.pop(face, None)
    # def

    def remaining(self):
        """Faces still wanted, in the suggested single-flip order."""
        return [f for f in SUGGESTED_FACE_ORDER if f not in self.captured]
    # def

    def feed(self, sample, now):
        """Advance the machine. Returns a face name when one is ready to capture.

        Returns None every other time, which is the overwhelming majority of
        calls - this runs on every sample at 200 Hz.
        """
        quiet = sample_is_quiet(sample, self.channels)
        face = sample_face(sample, self.channels)

        if self.state == "RELEASED":
            if not quiet:
                self.state = "WAITING"
            return None

        if not quiet or face is None or face in self.captured:
            self.state = "WAITING"
            self._face = None
            self._since = None
            return None

        if face != self._face:
            self._face = face
            self._since = now
            self.state = "SETTLING"
            return None

        if now - self._since >= self.settle_seconds:
            return face
        return None
    # def

    def moving(self, sample):
        """Is the plate being handled right now? Drives the "put it down" hint."""
        for name in self.channels:
            reading = sample.get(name)
            if reading is None:
                continue
            _, gyro = split_sample(reading)
            if vector_norm(gyro) > RELEASE_GYRO_DPS:
                return True
        return False
    # def
# class


# --- Reducing a capture to one observation ---------------------------------

def mean_observation(accelerations):
    """The one vector a held orientation contributes to the solve.

    Not the plain mean, and the difference is the whole reason a hand-held plate
    works. Averaging vectors that wander over a cone shortens the resultant -
    the directions partly cancel while the magnitudes do not - so a plain mean
    reads systematically short and the solve reports a scale that is too small.
    It is a bias, not noise, so holding for longer does not average it away; it
    makes it worse, because more wander accumulates. Measured against a
    simulated hand-held plate at 2 degrees of wander, the plain mean's scale
    error grew from 0.0111% at a 2 second hold to 0.0131% at 10 seconds.

    The fix is that a sample's *magnitude* does not care which way the plate is
    pointed. So take the direction from the mean vector and the length from the
    mean of the individual lengths. At 5 degrees of wander that takes the scale
    error from 0.0679% to 0.0036%, and it stops depending on how long the hold
    was.

    With no wander at all the two agree exactly, so this costs nothing in the
    case it is not needed.
    """
    count = len(accelerations)
    mean = tuple(statistics.fmean(a[i] for a in accelerations) for i in range(3))

    length = vector_norm(mean)
    if length < 1e-9 or count < 2:
        return mean

    target = statistics.fmean(vector_norm(a) for a in accelerations)
    return tuple(c * target / length for c in mean)
# def


def wander_deg(accelerations):
    """RMS angle between the individual samples and their mean direction.

    The honest measure of how still the hold actually was, and the one to put in
    front of the operator - "you wandered 3 degrees" is something a person can
    act on, where a gyro figure in degrees per second is not.
    """
    if len(accelerations) < 2:
        return 0.0
    mean = tuple(statistics.fmean(a[i] for a in accelerations) for i in range(3))
    angles = [angle_between_deg(a, mean) for a in accelerations]
    angles = [a for a in angles if a is not None]
    if not angles:
        return 0.0
    return math.sqrt(statistics.fmean(a * a for a in angles))
# def


def summarise_capture(samples, face, channels=CHANNELS):
    """A held window of samples to one observation per channel.

    The mean acceleration vector is the observation the solve consumes. The
    scatter is carried alongside because it is one of the three real quality
    measures - the residuals cannot be, since six faces against six unknowns is
    exactly determined and its residuals are approximately zero by construction.

    Returns {channel: {...}}; a channel absent from every sample is absent here
    rather than present and empty.
    """
    out = {}
    for name in channels:
        readings = [s[name] for s in samples if s.get(name) is not None]
        if not readings:
            continue

        accs = [counts_to_g(r[0:3]) for r in readings]
        gyros = [counts_to_dps(r[3:6]) for r in readings]

        mean = mean_observation(accs)
        if len(accs) > 1:
            scatter = tuple(statistics.stdev(a[i] for a in accs) for i in range(3))
        else:
            scatter = (0.0, 0.0, 0.0)

        tilt = tilt_from_nominal_deg(mean, face) if face in FACE_VECTORS else None

        out[name] = {
            "face": face,
            "n": len(readings),
            "mean_g": mean,
            "scatter_g": scatter,
            "magnitude_g": vector_norm(mean),
            "gyro_mean_dps": statistics.fmean(vector_norm(g) for g in gyros),
            "gyro_max_dps": max(vector_norm(g) for g in gyros),
            "wander_deg": wander_deg(accs),
            "tilt_deg": tilt,
            "verdict": face_verdict(tilt),
        }
    return out
# def


# --- The solve -------------------------------------------------------------

def solve_linear(matrix, vector):
    """Gaussian elimination with partial pivoting. None if singular.

    The same routine as range_test._solve; see this module's docstring for why
    it is here rather than imported.
    """
    size = len(vector)
    rows = [list(matrix[i]) + [vector[i]] for i in range(size)]

    for column in range(size):
        pivot = max(range(column, size), key=lambda r: abs(rows[r][column]))
        if abs(rows[pivot][column]) < 1e-12:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]

        for row in range(column + 1, size):
            factor = rows[row][column] / rows[column][column]
            for col in range(column, size + 1):
                rows[row][col] -= factor * rows[column][col]

    result = [0.0] * size
    for row in range(size - 1, -1, -1):
        total = rows[row][size] - sum(rows[row][c] * result[c]
                                      for c in range(row + 1, size))
        result[row] = total / rows[row][row]
    return result
# def


def seed_solution(observations):
    """Closed-form bias and scale from the extremes of each axis.

    observations is [(face, mean_g)]. For each axis, bias is the mean of the
    largest and smallest readings and scale is half their difference - which is
    exact if the two faces were perfectly flat and is the starting point for
    refine_solution if they were not.

    Returns None if an axis never saw both a positive and a negative g, because
    a scale derived from one side is not a seed, it is a guess.
    """
    if not observations:
        return None

    bias, scale = [], []
    for axis in range(3):
        values = [m[axis] for _, m in observations]
        high, low = max(values), min(values)
        if high < 0.5 or low > -0.5:
            return None
        bias.append((high + low) / 2.0)
        scale.append((high - low) / 2.0)
    return {"bias": bias, "scale": scale}
# def


def refine_solution(observations, seed, iterations=40):
    """Gauss-Newton on the six parameters, against every captured orientation.

    Each orientation contributes one scalar residual:

        v = (m - bias) / scale        r = |v|^2 - 1

    which is zero exactly when the corrected vector has magnitude g. This is
    what removes the cosine error from a face that was not perfectly flat: the
    residual is built from the *measured* vector, so a face 8 degrees off still
    constrains the parameters correctly, where the min/max seed would have taken
    its cos(8) = 0.990 projection at face value and lost 1.0% of scale.

    It is also what lets extra tilted orientations help. Six faces against six
    unknowns is exactly determined; every orientation past the sixth is a
    genuine degree of freedom.

    Levenberg damping, small and fixed, because the normal equations go
    ill-conditioned if the captured orientations happen to be nearly coplanar
    and a solver that returns a wild answer is worse than one that returns the
    seed.
    """
    bias = list(seed["bias"])
    scale = list(seed["scale"])

    for _ in range(iterations):
        normal = [[0.0] * 6 for _ in range(6)]
        rhs = [0.0] * 6

        for _, mean in observations:
            v = [(mean[i] - bias[i]) / scale[i] for i in range(3)]
            residual = v[0] * v[0] + v[1] * v[1] + v[2] * v[2] - 1.0

            jacobian = [0.0] * 6
            for i in range(3):
                jacobian[i] = -2.0 * v[i] / scale[i]          # d/d bias
                jacobian[3 + i] = -2.0 * v[i] * v[i] / scale[i]  # d/d scale

            for r in range(6):
                rhs[r] -= jacobian[r] * residual
                for c in range(6):
                    normal[r][c] += jacobian[r] * jacobian[c]

        for k in range(6):
            normal[k][k] *= 1.0 + 1e-9
            normal[k][k] += 1e-12

        step = solve_linear(normal, rhs)
        if step is None:
            break

        for i in range(3):
            bias[i] += step[i]
            scale[i] += step[3 + i]

        if max(abs(s) for s in step) < 1e-12:
            break

    return {"bias": bias, "scale": scale}
# def


def apply_solution(measured_g, solution):
    """The corrected vector: true = (measured - bias) / scale."""
    return tuple((measured_g[i] - solution["bias"][i]) / solution["scale"][i]
                 for i in range(3))
# def


def solve_sensor(observations):
    """Seed, refine and score one sensor. None if the faces do not span.

    observations is [(face_or_label, mean_g)] and may include tilted
    orientations that are not faces at all; the refinement does not care what
    they are called, only that they were static.
    """
    seed = seed_solution(observations)
    if seed is None:
        return None

    solution = refine_solution(observations, seed)

    residuals = []
    for _, mean in observations:
        corrected = apply_solution(mean, solution)
        residuals.append(vector_norm(corrected) - 1.0)

    # Six unknowns. With exactly six orientations this is zero and the RMS below
    # is meaningless by construction, which is why dof is reported next to it
    # rather than left for the reader to work out.
    dof = len(observations) - 6
    rms = math.sqrt(statistics.fmean(r * r for r in residuals)) if residuals else 0.0

    solution["residual_rms_g"] = rms
    solution["residuals_g"] = residuals
    solution["dof"] = dof
    solution["n_orientations"] = len(observations)
    solution["seed"] = seed
    return solution
# def


def sanity_problems(name, solution):
    """Reasons to refuse to write this sensor's coefficients. [] means fine."""
    problems = []
    for i, axis in enumerate(AXES):
        scale = solution["scale"][i]
        bias = solution["bias"][i]
        if not (SANITY_MIN_SCALE <= scale <= SANITY_MAX_SCALE):
            problems.append(
                "%s %s scale %.4f outside %.2f-%.2f"
                % (name, axis, scale, SANITY_MIN_SCALE, SANITY_MAX_SCALE))
        if abs(bias) > SANITY_MAX_BIAS_G:
            problems.append(
                "%s %s bias %+.1f mg beyond %.0f mg"
                % (name, axis, bias * 1000.0, SANITY_MAX_BIAS_G * 1000.0))
    return problems
# def


def fit_rotation(pairs, iterations=200):
    """The rotation R minimising sum |R a - b|^2, over [(a, b)] of unit vectors.

    Wahba's problem, solved by Davenport's q-method: build the 4x4 symmetric K
    from the attitude profile matrix, and the optimal rotation is its
    largest-eigenvalue eigenvector read as a quaternion.

    Power iteration finds that eigenvector, which is the whole reason this
    approach is used here rather than an SVD - it needs nothing but multiply and
    normalise, so the no-numpy policy costs a dozen lines instead of a
    dependency. K is shifted by the sum of its absolute entries first, which
    cannot reorder the eigenvalues and does guarantee the one we want is the
    largest in magnitude rather than merely the largest.

    Returns None for fewer than two pairs, or two parallel ones - a single
    gravity vector constrains a rotation only up to a spin about itself, and a
    function that returned one of those arbitrarily would be worse than one that
    admits it cannot tell.
    """
    if len(pairs) < 2:
        return None

    profile = [[sum(b[i] * a[j] for a, b in pairs) for j in range(3)]
               for i in range(3)]
    trace = profile[0][0] + profile[1][1] + profile[2][2]
    axis = [profile[1][2] - profile[2][1],
            profile[2][0] - profile[0][2],
            profile[0][1] - profile[1][0]]

    k = [[0.0] * 4 for _ in range(4)]
    for i in range(3):
        for j in range(3):
            k[i][j] = profile[i][j] + profile[j][i] - (trace if i == j else 0.0)
        k[i][3] = k[3][i] = axis[i]
    k[3][3] = trace

    shift = sum(abs(k[i][j]) for i in range(4) for j in range(4))
    if shift < 1e-12:
        return None
    for i in range(4):
        k[i][i] += shift

    # Not axis-aligned, so a start vector orthogonal to the answer is not a
    # thing that quietly happens.
    q = [0.3, 0.1, 0.2, 0.9]
    for _ in range(iterations):
        nxt = [sum(k[i][j] * q[j] for j in range(4)) for i in range(4)]
        length = math.sqrt(sum(c * c for c in nxt))
        if length < 1e-12:
            return None
        q = [c / length for c in nxt]

    x, y, z, w = q
    # Transposed relative to the usual quaternion-to-matrix form, because
    # Davenport's q rotates b onto a and the caller wants a onto b. Getting this
    # backwards yields a rotation with exactly the right *angle* and a useless
    # axis, which fits the data no better than the identity and is not obvious
    # from the fitted angle alone - so it is asserted in the tests.
    matrix = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
              [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
              [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    return [[matrix[j][i] for j in range(3)] for i in range(3)]
# def


def apply_rotation(rotation, v):
    """R v."""
    return tuple(sum(rotation[i][j] * v[j] for j in range(3)) for i in range(3))
# def


def rotation_angle_deg(rotation):
    """How far the rotation turns, in degrees."""
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))
# def


def unit(v):
    """v scaled to unit length, or None if it has none."""
    length = vector_norm(v)
    if length < 1e-12:
        return None
    return tuple(c / length for c in v)
# def


def plate_consistency(orientations, solutions):
    """Are the three modules one rigid body, and how are they mounted?

    The payoff for calibrating all three at once, and something a single-sensor
    calibration cannot do at all.

    For each pair, fit the single rotation carrying one sensor's corrected
    gravity vectors onto the other's across every captured orientation. If the
    modules share a rigid plate, one rotation explains all nine and the residual
    is measurement noise. If one worked loose, no single rotation fits and the
    residual is the size of the movement.

    That splits two things an earlier version of this check ran together. The
    **mounting rotation** is how far apart the modules are bolted: a real
    quantity, worth reporting, and not a fault. The **residual** is whether they
    stayed that way, and it is the only part worth gating on.

    Measured on the two real runs of 2026-08-28, nine orientations each and
    hand-held throughout: the clean run fitted to 0.091 deg rms, and the run with
    a module loose fitted to 1.04-1.32 deg rms with mounting rotations that were
    themselves wrong. A factor of eleven between good and bad.

    The rotation is also what a native-format CENTRE correction needs - the
    datum is a rotation to undo, not a number to subtract - so this fit is not
    only a check. It is where that coefficient comes from.

    orientations is {label: {channel: summary}} and should carry the extra tilts
    as well as the six faces; every orientation constrains the fit.

    Returns {pair: {...}}.
    """
    out = {}
    names = [n for n in CHANNELS if n in solutions]
    labels = sorted(orientations)

    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]

            shared = [k for k in labels
                      if a in orientations[k] and b in orientations[k]]

            pairs, kept = [], []
            for k in shared:
                va = unit(apply_solution(orientations[k][a]["mean_g"], solutions[a]))
                vb = unit(apply_solution(orientations[k][b]["mean_g"], solutions[b]))
                if va is None or vb is None:
                    continue
                pairs.append((va, vb))
                kept.append(k)

            rotation = fit_rotation(pairs)
            if rotation is None:
                continue

            residuals = {}
            for label, (va, vb) in zip(kept, pairs):
                angle = angle_between_deg(apply_rotation(rotation, va), vb)
                if angle is not None:
                    residuals[label] = angle
            if not residuals:
                continue

            values = list(residuals.values())
            worst_label = max(residuals, key=lambda k: residuals[k])
            rms = math.sqrt(statistics.fmean(v * v for v in values))

            out["%s-%s" % (a, b)] = {
                "rotation": rotation,
                "mount_deg": rotation_angle_deg(rotation),
                "residual_deg": residuals,
                "residual_rms_deg": rms,
                "residual_max_deg": residuals[worst_label],
                "worst_orientation": worst_label,
                "n_orientations": len(pairs),
                "verdict": ("reject" if rms > PLATE_DISAGREE_REJECT_DEG
                            else "warn" if rms > PLATE_DISAGREE_WARN_DEG
                            else "ok"),
            }
    return out
# def
