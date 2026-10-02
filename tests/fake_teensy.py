"""Synthesise a teensy/rig raw capture, for tests and for exercising the wizard.

The counterpart to fake_pico.py: that one stands in for the board over a serial
port, this one produces the file the board's stream would have produced. The
six-face wizard's expensive part is a person holding a plate still for thirty
seconds nine times over, and none of the arithmetic downstream of that should
need one to be tested.

Emits exactly the grammar teensy/rig emits, including the "x" sentinel and the
board's own micros() stamp, so a file from here and a file from the rig are the
same kind of thing and --replay cannot tell them apart.
"""

import math
import random

from sensor_cal_logic import (
    ACC_COUNTS_PER_G,
    CHANNELS,
    FACE_VECTORS,
    GYRO_COUNTS_PER_DPS,
    SUGGESTED_FACE_ORDER,
)


# Measured on the rig 2026-08-28, all three modules resting on the desk, 4002
# samples over 20 s through teensy/rig at 200 Hz:
#
#   per-axis sd   X 0.48   Y 0.39   Z 1.33 mg   (worst of the three channels
#                                                1.37 mg, best 0.36 mg)
#   as an angle   0.073 - 0.083 degrees
#
# Note this is six times the 0.0128 degree figure SENSORS.md records for these
# modules. That figure is the module's own *fused* attitude output, which is
# filtered; this is the unfiltered accelerometer the calibration actually
# consumes, and it is the number that belongs in a synthetic capture. Z is
# consistently about three times noisier than X and Y on all three modules.
BENCH_NOISE_G = 1.4e-3


def rotate(v, axis, degrees):
    a = math.radians(degrees)
    c, s = math.cos(a), math.sin(a)
    x, y, z = v
    if axis == "x":
        return (x, c * y - s * z, s * y + c * z)
    if axis == "y":
        return (c * x + s * z, y, -s * x + c * z)
    return (c * x - s * y, s * x + c * y, z)
# def


def format_line(t_us, readings):
    """One sample line. readings is {channel: 6-tuple} or None for a dead one."""
    fields = []
    for name in CHANNELS:
        reading = readings.get(name)
        if reading is None:
            fields.append("x")
        else:
            fields.append(",".join(str(int(v)) for v in reading))
    return "{%d:%s}" % (t_us, "|".join(fields))
# def


class FakeSensor:
    """One module: its true bias and scale, and its mounting on the plate.

    mount_deg is a small rotation applied before the sensor's own errors, which
    is what makes the plate-consistency check have something to find - three
    sensors bolted to one plate are never quite parallel, and the angle between
    any two must come out constant on every face.
    """

    def __init__(self, bias, scale, mount_deg=(0.0, 0.0)):
        self.bias = bias
        self.scale = scale
        self.mount_deg = mount_deg
    # def

    def measure(self, gravity, noise, rng):
        v = rotate(rotate(gravity, "x", self.mount_deg[0]),
                   "y", self.mount_deg[1])
        return tuple(
            self.scale[i] * v[i] + self.bias[i] + rng.gauss(0.0, noise)
            for i in range(3))
    # def
# class


DEFAULT_SENSORS = {
    "LEFT":   FakeSensor([0.031, -0.018, 0.045], [0.985, 1.012, 0.974], (0.4, -0.2)),
    "CENTRE": FakeSensor([-0.012, 0.026, -0.008], [1.004, 0.991, 1.007], (0.0, 0.0)),
    "RIGHT":  FakeSensor([0.007, 0.041, 0.019], [0.996, 1.003, 0.988], (-0.3, 0.5)),
}


def synthesise(path, sensors=None, hold_s=3.0, rate_hz=200, tilts=3,
               face_tilt_deg=6.0, noise=BENCH_NOISE_G, seed=1, dead=(),
               slip=None, motion_s=2.0):
    """Write a full nine-orientation capture. Returns the sensors used.

    Between orientations it emits a stretch of high gyro, because that is what
    the plate being picked up looks like and the wizard's release logic exists
    precisely to require it - without it the tracker would re-fire on the face
    it is already resting on.

    slip is (channel, orientation_index, (dx_deg, dy_deg)): from that
    orientation onward the channel is mounted differently, which is a sensor
    working loose partway through a calibration. It is the fault the plate
    consistency check exists to catch, and a check that has never been shown to
    catch it is a check nobody should trust.
    """
    rng = random.Random(seed)
    sensors = sensors or DEFAULT_SENSORS
    period_us = int(1e6 / rate_hz)
    t_us = 1000

    orientations = []
    for face in SUGGESTED_FACE_ORDER:
        vec = FACE_VECTORS[face]
        orientations.append(
            rotate(rotate(vec, "x", rng.uniform(-face_tilt_deg, face_tilt_deg)),
                   "y", rng.uniform(-face_tilt_deg, face_tilt_deg)))
    for _ in range(tilts):
        orientations.append(
            rotate(rotate((0.0, 0.0, 1.0), "x", rng.uniform(25.0, 55.0)),
                   "z", rng.uniform(0.0, 360.0)))

    with open(path, "w") as handle:
        handle.write("# MANTA rig ready mode=raw\n")

        for index, gravity in enumerate(orientations):
            if slip is not None and index == slip[1]:
                sensors = dict(sensors)
                moved = FakeSensor(sensors[slip[0]].bias,
                                   sensors[slip[0]].scale, slip[2])
                sensors[slip[0]] = moved

            if index:
                # Handling: two seconds of tumbling, well above the release
                # threshold, with an acceleration magnitude that is not 1 g.
                for _ in range(int(motion_s * rate_hz)):
                    readings = {}
                    for name, sensor in sensors.items():
                        if name in dead:
                            readings[name] = None
                            continue
                        acc = [rng.uniform(-1.4, 1.4) for _ in range(3)]
                        gyro = [rng.uniform(-90.0, 90.0) for _ in range(3)]
                        readings[name] = tuple(
                            [a * ACC_COUNTS_PER_G for a in acc] +
                            [g * GYRO_COUNTS_PER_DPS for g in gyro])
                    handle.write(format_line(t_us, readings) + "\n")
                    t_us += period_us

            for _ in range(int(hold_s * rate_hz)):
                readings = {}
                for name, sensor in sensors.items():
                    if name in dead:
                        readings[name] = None
                        continue
                    acc = sensor.measure(gravity, noise, rng)
                    # Stationary gyro reads exactly zero on these modules, so
                    # the only thing on it is quantisation.
                    gyro = [rng.choice((-1, 0, 0, 0, 1)) for _ in range(3)]
                    readings[name] = tuple(
                        [a * ACC_COUNTS_PER_G for a in acc] + gyro)
                handle.write(format_line(t_us, readings) + "\n")
                t_us += period_us

    return sensors
# def


class FakeTeensy(object):
    """A fake rig board on a pty: replays a capture and answers commands.

    The counterpart to fake_pico.FakeTeensy's role for the pot rig, and it
    exists for the same reason - the wizard's interactive half (waiting for a
    face, holding, abandoning a hold that moved) is the part that cannot be
    reached by --replay, and it is exactly the part that is awkward to get right.

    Lines are paced at `rate_hz` of wall-clock so the FaceTracker's settle
    timing, which reads the host clock, means the same thing here as on the rig.

        with FakeTeensy(path, rate_hz=400) as board:
            ser = serial.Serial(board.device, timeout=1.0)
    """

    def __init__(self, capture_path, rate_hz=400):
        import os
        import tty
        with open(capture_path) as handle:
            self.lines = [l.rstrip("\n") for l in handle
                          if l.startswith("{")]
        self.rate_hz = rate_hz
        self._master_fd, self._slave_fd = os.openpty()
        # Raw on both ends, exactly as fake_pico does. Without it the line
        # discipline echoes every line the fake emits straight back at it, and
        # the fake then reads its own sample lines as commands.
        tty.setraw(self._master_fd)
        tty.setraw(self._slave_fd)
        self.device = os.ttyname(self._slave_fd)
        self._stop = None
        self._thread = None
        self.commands = []
    # def

    def _serve(self):
        import os
        import select
        import time as _time

        period = 1.0 / self.rate_hz
        index = 0
        streaming = True
        next_at = _time.time()

        while not self._stop.is_set() and index < len(self.lines):
            ready, _, _ = select.select([self._master_fd], [], [], 0.0)
            if ready:
                try:
                    chunk = os.read(self._master_fd, 256).decode("ascii", "ignore")
                except OSError:
                    break
                for command in chunk.replace("\r", "\n").split("\n"):
                    command = command.strip().upper()
                    if not command:
                        continue
                    # A reply or a sample line is not a command. The real
                    # firmware ignores anything starting with "#" for exactly
                    # this reason; the fake needs the same guard so a loopback
                    # cannot manufacture an "# ERR".
                    if command[0] in "#{[$":
                        continue
                    self.commands.append(command)
                    if command == "H":
                        streaming = False
                        self._write("# ACK H")
                    elif command.startswith("F"):
                        streaming = True
                        self._write("# ACK F 200")
                    elif command == "S":
                        streaming = True
                        self._write("# ACK S 10")
                    elif command == "I":
                        self._write("# ID dev=manta-rig proto=1 fw=0.1.0 "
                                    "hw=teensy40 chans=LEFT,CENTRE,RIGHT "
                                    "units=counts mode=raw cal=none "
                                    "feat=rawstream,rateset rate=200 "
                                    "sensors=ready")
                    elif command == "?":
                        self._write("# STATUS uptime_ms=1000 hz=200 "
                                    "streaming=1 dropped=0 sensors=ready")
                    else:
                        self._write("# ERR %s" % command)

            now = _time.time()
            if now < next_at:
                _time.sleep(min(period, next_at - now))
                continue
            next_at += period

            if streaming:
                self._write(self.lines[index])
                index += 1
    # def

    def _write(self, text):
        import os
        try:
            os.write(self._master_fd, (text + "\n").encode("ascii"))
        except OSError:
            pass
    # def

    def start(self):
        import threading
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        return self
    # def

    def stop(self):
        import os
        if self._stop is not None:
            self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for fd in (self._slave_fd, self._master_fd):
            try:
                os.close(fd)
            except OSError:
                pass
    # def

    def __enter__(self):
        return self.start()
    # def

    def __exit__(self, exc_type, exc, tb):
        self.stop()
    # def
# class
