"""The sensor driver seam: everything below `get_side_angle()`.

A driver owns one sensor set - its transport, its wire format and its native
units - and hands the application angles in degrees. Two exist:

- `PicoDriver`, the rig as it stands: potentiometers sampled by a Pico,
  arriving as ADC counts;
- `FakeDriver`, canned degrees with no hardware, which is how the contract gets
  tested and how the application gets run without a rig.

A third, `TeensyDriver`, is the point of the exercise and is not written yet.
It is deliberately not written yet: the seam is proven against the hardware we
already have first, so that when the Teensy driver misbehaves there is only one
new thing in the room. SENSORS.md carries the reasoning.

The rule this module exists to enforce: **nothing above a driver may know which
driver it has.** Callers ask for an angle on a channel and are given degrees or
None. They do not see counts, scalers, ports, baud rates or wire formats. Any
caller that would need to branch on the driver is a defect in this contract, not
a special case to write down - which is exactly what a `FakeDriver` that needs
no upstream special-casing demonstrates.

Sides are not channels. `LEFT` and `RIGHT` are both, which is why the pot rig
never had to tell them apart; `CENTRE` is a channel and never a side. Drivers
speak channels. See SENSORS.md.
"""

import threading
import time


# Every channel name any driver may offer. Sides are a subset: the elevon code
# above this layer only ever asks for LEFT and RIGHT, and must keep working when
# a driver offers more than it asks about.
CHANNEL_LEFT = "LEFT"
CHANNEL_RIGHT = "RIGHT"
CHANNEL_CENTRE = "CENTRE"

ALL_CHANNELS = (CHANNEL_LEFT, CHANNEL_CENTRE, CHANNEL_RIGHT)

# The two sides, restated here so this module can validate a channel without
# importing the GUI. Kept as plain strings on purpose - SENSORS.md argues
# against a Side class, and nothing here needs one.
SIDES = (CHANNEL_LEFT, CHANNEL_RIGHT)


class Capabilities(object):
    """What a driver can actually do, declared rather than guessed at.

    Measurements that depend on sample rate or angular resolution have to ask
    before they move, because the honest answer differs between sensor sets: the
    pots sustain 1000 Hz, the inclinometer modules cap near 200 Hz. A caller that
    assumes gets a transit figure computed from eight samples and no warning.
    """

    def __init__(self, name, channels, native_units, max_sample_rate_hz,
                 resolution_deg, max_requestable_rate_hz=None,
                 device_timestamps=False, raw_capture=False,
                 reference_channel=False, motion_flag=False):
        self.name = name
        self.channels = tuple(channels)
        self.native_units = native_units
        self.resolution_deg = resolution_deg

        # The rate a measurement may *rely* on. This is the number that decides
        # whether a transit figure is sound.
        self.max_sample_rate_hz = max_sample_rate_hz

        # The highest rate the device will accept, which is deliberately higher
        # on the pot rig: pico/sampler.py sets MAX_HZ above what the board can
        # sustain so that asking for too much produces an honest "could not
        # sustain" report instead of silent drift. Keeping the two apart is what
        # lets a driver grant an over-rate for diagnostics while still refusing
        # to call it sustainable. Defaults to the sustainable rate, which is the
        # right answer for a device with no such affordance.
        self.max_requestable_rate_hz = (
            max_sample_rate_hz if max_requestable_rate_hz is None
            else max_requestable_rate_hz
        )

        # Does the device stamp its own samples? Host arrival times are good to
        # a few ms at best over USB CDC, which is the same order as the transit
        # being measured.
        self.device_timestamps = device_timestamps

        # Can it record every sample, unaveraged, with timestamps?
        self.raw_capture = raw_capture

        # Is there an airframe reference channel (CENTRE) to subtract, so that
        # rocking the whole rig does not read as elevon movement?
        self.reference_channel = reference_channel

        # Can the device say "I was moving while you read that"?
        self.motion_flag = motion_flag
    # def

    def supports_rate(self, needed_hz):
        """True if this driver can *sustain* the rate a measurement asks for.

        Deliberately about sustainability, not acceptance. A driver may well
        grant a rate this returns False for; what it may never do is let a
        caller believe the resulting samples are trustworthy.
        """
        return float(self.max_sample_rate_hz) >= float(needed_hz)
    # def

    def has_channel(self, channel):
        return channel in self.channels
    # def

    def describe(self):
        return "%s: %s at up to %g Hz, %g deg resolution" % (
            self.name,
            "/".join(self.channels),
            self.max_sample_rate_hz,
            self.resolution_deg,
        )
    # def
# class


class SensorDriver(object):
    """The contract. Every method here is what the application is allowed to use.

    Subclasses override; the base raises rather than returning a plausible
    default, because a driver that silently answers 0.0 for something it cannot
    do is the failure mode this whole layer exists to prevent.

    Lifecycle mirrors what the pot rig already did, because that shape is known
    to work over a real serial port under a real GUI: construct, `set_port()` or
    `start()`, read, `stop()`. Reading before starting returns None rather than
    raising - the GUI polls on a timer and must not need to know.
    """

    def identity(self):
        """Short stable name for this driver, for logs and CSV provenance."""
        raise NotImplementedError
    # def

    def capabilities(self):
        raise NotImplementedError
    # def

    def channels(self):
        return self.capabilities().channels
    # def

    # -- lifecycle ---------------------------------------------------------

    def set_port(self, port):
        raise NotImplementedError
    # def

    def start(self):
        raise NotImplementedError
    # def

    def stop(self, join_timeout=2.0):
        raise NotImplementedError
    # def

    def is_connected(self):
        raise NotImplementedError
    # def

    def is_streaming(self, max_age=1.0):
        """True if a sample arrived recently enough to be worth believing.

        Distinct from is_connected(): a port can be open and the board silent,
        which is what a wedged firmware or an unplugged sensor looks like.
        """
        raise NotImplementedError
    # def

    def last_error(self):
        """Human-readable reason the link is down, or None."""
        raise NotImplementedError
    # def

    # -- rate --------------------------------------------------------------

    def sample_rate(self):
        raise NotImplementedError
    # def

    def set_sample_rate(self, hz):
        """Ask for a rate; return what was actually granted, or None if unknown.

        Returning the achieved rate rather than a bool is deliberate. Callers
        need the real number to decide whether their measurement is sound, and a
        driver that cannot negotiate should say None rather than claim success.
        """
        raise NotImplementedError
    # def

    # -- reading -----------------------------------------------------------

    def get_angle(self, channel, window_s=None):
        """Windowed mean for one channel, **in degrees**, or None if stale.

        Degrees is the whole point: unit conversion is the driver's business and
        nothing above this line performs any.
        """
        raise NotImplementedError
    # def

    def to_degrees(self, channel, native_value):
        """Convert one native reading to degrees, or None.

        The escape hatch for the raw-capture path, which hands back samples in
        native units and has them converted afterwards. Everything else should
        use get_angle(); this exists so that code analysing a capture converts
        *through the driver* rather than by helping itself to a scaler, which is
        the one way units still leak above the seam today.

        A driver whose native units are already degrees returns the value.
        """
        raise NotImplementedError
    # def

    def clear(self):
        """Discard buffered samples, so the next read cannot see pre-move data."""
        raise NotImplementedError
    # def

    # -- raw capture -------------------------------------------------------

    def start_capture(self, limit=None):
        raise NotImplementedError
    # def

    def stop_capture(self):
        """Return (samples, truncated)."""
        raise NotImplementedError
    # def

    def is_capturing(self):
        raise NotImplementedError
    # def

    # -- device commands ---------------------------------------------------

    def send_command(self, command, timeout=1.5, attempts=3):
        """Round-trip a device command. None if there is no answer, or no device."""
        raise NotImplementedError
    # def
# class


# The pots resolve about 0.004 degrees per ADC count, which is the number
# MOVEMENT_THRESHOLD_DEG and ENDPOINT_TOLERANCE_DEG in endpoint_logic.py were
# chosen against. Declared here so a measurement can ask rather than assume, and
# so the inclinometer rig's figure can be compared with it directly.
POT_RESOLUTION_DEG = 0.004

# The supported ceiling, not the wall. Under MicroPython 1.19.1 the board broke
# down near 1520 Hz; since the 2026-09-05 reflash to 1.29.0 it meets 2000 Hz and
# the wall has not been located. 1000 stays the fast preset either way, because a
# driver advertising the wall would let a measurement ask for a rate that only
# works on a quiet day.
POT_MAX_RATE_HZ = 1000

# What the board will actually accept: pico/sampler.py's MAX_HZ. It was set above
# the sustainable rate on purpose so an overrun is diagnosable rather than silent;
# on 1.29.0 the board sustains it, so it is currently a plain clamp. A caller that
# asks for this gets it, and gets samples that have not been characterised.
POT_MAX_REQUESTABLE_HZ = 2000


class PicoDriver(SensorDriver):
    """The rig as it stands: pots on a Pico, ADC counts in, degrees out.

    Composition over a `PositionReader` rather than a rewrite of it. The reader
    already owns a tested serial thread, reply routing, rate negotiation and
    capture, and re-implementing that to introduce a seam would be putting the
    highest-risk change first for no gain. The reader is injected rather than
    imported because it lives in MantaTrimmer.py alongside Tk, and this module
    must stay importable - and testable - without a display.

    The scaler-and-tare conversion stays whole inside `position_to_degrees()`
    for now. SENSORS.md describes a pipeline where the driver applies the
    hardware scaler and the tare is subtracted above it; splitting them is
    lossless but it is not this change, and doing it here would mean the numbers
    could no longer be compared byte-for-byte against a pre-migration run. That
    comparison is the point of this step.
    """

    def __init__(self, reader):
        self._reader = reader
    # def

    @property
    def reader(self):
        """The underlying PositionReader.

        Exposed for the migration only: existing call sites and tests still
        reach for it directly, and pretending otherwise would mean changing them
        all in the same commit that introduces the seam. New code must not use
        it. When the last caller is gone this goes with it.
        """
        return self._reader
    # def

    def identity(self):
        return "pico-pots"
    # def

    def capabilities(self):
        return Capabilities(
            name="Pico potentiometers",
            channels=(CHANNEL_LEFT, CHANNEL_RIGHT),
            native_units="counts",
            max_sample_rate_hz=POT_MAX_RATE_HZ,
            max_requestable_rate_hz=POT_MAX_REQUESTABLE_HZ,
            resolution_deg=POT_RESOLUTION_DEG,
            device_timestamps=True,     # "t_us" in the position line, on current firmware
            raw_capture=True,
            reference_channel=False,    # no CENTRE: the pots cannot see the airframe
            motion_flag=False,
        )
    # def

    def set_port(self, port):
        self._reader.set_port(port)
    # def

    def start(self):
        self._reader.start()
    # def

    def stop(self, join_timeout=2.0):
        self._reader.stop(join_timeout=join_timeout)
    # def

    def is_connected(self):
        return bool(self._reader.connected)
    # def

    def is_streaming(self, max_age=1.0):
        return self._reader.is_streaming(max_age=max_age)
    # def

    def last_error(self):
        return self._reader.last_error
    # def

    def sample_rate(self):
        return self._reader.sample_hz
    # def

    def set_sample_rate(self, hz):
        return self._reader.set_sample_rate(hz)
    # def

    def get_angle(self, channel, window_s=None):
        if channel not in (CHANNEL_LEFT, CHANNEL_RIGHT):
            return None

        raw = self._reader.get_average_position_nonblocking(channel, window_s=window_s)
        if raw is None:
            return None

        return self._reader.position_to_degrees(channel, raw)
    # def

    def to_degrees(self, channel, native_value):
        if channel not in (CHANNEL_LEFT, CHANNEL_RIGHT) or native_value is None:
            return None

        return self._reader.position_to_degrees(channel, native_value)
    # def

    def clear(self):
        self._reader.clear_queues()
    # def

    def start_capture(self, limit=None):
        if limit is None:
            self._reader.start_capture()
        else:
            self._reader.start_capture(limit)
    # def

    def stop_capture(self):
        return self._reader.stop_capture()
    # def

    def is_capturing(self):
        return self._reader.is_capturing()
    # def

    def send_command(self, command, timeout=1.5, attempts=3):
        return self._reader.send_command(command, timeout=timeout, attempts=attempts)
    # def
# class


class FakeDriver(SensorDriver):
    """Canned degrees, no hardware. The contract's proof and the GUI's stand-in.

    Two jobs, and the first is the important one. Every test that has ever
    faked a sensor did it by monkeypatching `get_side_angle()` - which tests
    everything above the seam and nothing at it. This driver plugs in at the
    seam instead, so the question it answers is the one that matters: can the
    application be driven by a sensor set that is not the Pico, without a single
    special case anywhere above the driver? If this class ever needs upstream
    help, the contract is wrong, and that is far cheaper to learn now than with
    a Teensy on the bench.

    It defaults to offering **three** channels, including CENTRE, for the same
    reason. The pot rig gives LEFT and RIGHT, so a two-channel fake would let a
    "sides are channels" assumption survive untested. The inclinometer rig will
    offer CENTRE, and code that mishandles an extra channel should fail here.

    The second job is ordinary: running the GUI on a desk with no rig attached.
    """

    def __init__(self, channels=ALL_CHANNELS, max_sample_rate_hz=200,
                 resolution_deg=0.01, device_timestamps=True, raw_capture=True,
                 motion_flag=True, name="Fake sensors"):
        self._channels = tuple(channels)
        self._name = name
        self._max_rate = max_sample_rate_hz
        self._resolution = resolution_deg
        self._device_timestamps = device_timestamps
        self._raw_capture = raw_capture
        self._motion_flag = motion_flag

        self._lock = threading.RLock()
        self._angles = dict((c, 0.0) for c in self._channels)
        self._last_sample_time = 0.0

        self._port = None
        self._running = False
        self._rate = 10
        self._error = None

        self._capture = None
        self._capture_limit = 0
        self._capture_truncated = False

        # Commands this fake answers, so a caller that round-trips one is
        # exercising the same path it would against a board.
        self.commands_seen = []
    # def

    # -- test control, not part of the contract ----------------------------

    def set_angle(self, channel, degrees):
        """Place a channel. Stamps the sample clock, so staleness behaves."""
        with self._lock:
            if channel not in self._channels:
                raise KeyError("no such channel: %r" % channel)

            self._angles[channel] = None if degrees is None else float(degrees)
            self._last_sample_time = time.monotonic()

            if self._capture is not None:
                if len(self._capture) < self._capture_limit:
                    self._capture.append(
                        (self._last_sample_time, None, dict(self._angles)))
                else:
                    self._capture_truncated = True
    # def

    def set_angles(self, **by_channel):
        for channel, degrees in by_channel.items():
            self.set_angle(channel, degrees)
    # def

    def go_stale(self):
        """Pretend the last sample arrived long ago, without sleeping."""
        with self._lock:
            self._last_sample_time = time.monotonic() - 3600.0
    # def

    def fail(self, message):
        """Simulate a dead link: connected goes false and an error is readable."""
        with self._lock:
            self._running = False
            self._error = message
    # def

    # -- the contract ------------------------------------------------------

    def identity(self):
        return "fake"
    # def

    def capabilities(self):
        return Capabilities(
            name=self._name,
            channels=self._channels,
            native_units="degrees",
            max_sample_rate_hz=self._max_rate,
            resolution_deg=self._resolution,
            device_timestamps=self._device_timestamps,
            raw_capture=self._raw_capture,
            reference_channel=CHANNEL_CENTRE in self._channels,
            motion_flag=self._motion_flag,
        )
    # def

    def set_port(self, port):
        with self._lock:
            self._port = port
            self._error = None

        if port:
            self.start()
        else:
            self.stop()
    # def

    def start(self):
        with self._lock:
            self._running = True
            self._error = None
            if self._last_sample_time == 0.0:
                self._last_sample_time = time.monotonic()
    # def

    def stop(self, join_timeout=2.0):
        with self._lock:
            self._running = False
            self._rate = 10
    # def

    def is_connected(self):
        with self._lock:
            return self._running
    # def

    def is_streaming(self, max_age=1.0):
        with self._lock:
            if not self._running:
                return False
            last = self._last_sample_time

        return last > 0.0 and (time.monotonic() - last) <= max_age
    # def

    def last_error(self):
        with self._lock:
            return self._error
    # def

    def sample_rate(self):
        with self._lock:
            return self._rate
    # def

    def set_sample_rate(self, hz):
        """Grants what it can and says so - never more than it advertised.

        Clamping rather than refusing is the honest behaviour for a device with
        a ceiling, and it is what makes the achieved-rate return value worth
        having: a caller that asks for 1000 and is told 200 can decide what to
        do about it.
        """
        with self._lock:
            if not self._running:
                return None

            self._rate = int(min(int(hz), self._max_rate))
            return self._rate
    # def

    def get_angle(self, channel, window_s=None):
        with self._lock:
            if channel not in self._channels:
                return None
            if not self._running:
                return None
            if not self.is_streaming(max_age=1.0):
                return None

            return self._angles.get(channel)
    # def

    def to_degrees(self, channel, native_value):
        # Native units are already degrees, so this is the identity - which is
        # exactly what the Teensy driver's will be too.
        if channel not in self._channels or native_value is None:
            return None

        return float(native_value)
    # def

    def clear(self):
        with self._lock:
            self._last_sample_time = 0.0
    # def

    def start_capture(self, limit=None):
        with self._lock:
            self._capture = []
            self._capture_limit = 60000 if limit is None else int(limit)
            self._capture_truncated = False
    # def

    def stop_capture(self):
        with self._lock:
            samples = self._capture if self._capture is not None else []
            truncated = self._capture_truncated
            self._capture = None
            self._capture_truncated = False

        return samples, truncated
    # def

    def is_capturing(self):
        with self._lock:
            return self._capture is not None
    # def

    def send_command(self, command, timeout=1.5, attempts=3):
        with self._lock:
            self.commands_seen.append(command)

            if not self._running:
                return None

            return "# ACK %s" % command
    # def
# class
