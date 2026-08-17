#!/usr/bin/env python3
"""Servo output probe - a temporary bolt-on diagnostic. Delete when done.

Answers one question: **does the PWM the trimmer thinks it is setting match the
PWM the flight controller is actually commanding the servo with?**

MantaTrimmer never observes the servo output. It measures an elevon angle,
converts the command that produced it into a PWM number with `expected_pwm()`,
and writes that number into PWM_MAIN_MIN/MAX. If PX4's own command-to-PWM
mapping differs from that model - most obviously in how CA_SV_CSx_TRIM and the
PWM_MAIN_REV bit interact - every endpoint is written wrong and nothing in the
tool notices.

This probe drives one output function at a series of commands and reads back the
PWM the FC is actually commanding, alongside the elevon angle from the Pico. It
writes no parameters: it only reads, commands actuator tests, and compares.

Getting that readback needs the NSH shell. Measured on this rig against PX4
v1.16.0: SERVO_OUTPUT_RAW streams at 20 Hz but every channel reads 0, including
while a surface is visibly moving, and ACTUATOR_OUTPUT_STATUS is never emitted
even though SET_MESSAGE_INTERVAL and REQUEST_MESSAGE both ACK with ACCEPTED. The
board publishes two `actuator_outputs` instances: instance 0 is all zeros and is
what SERVO_OUTPUT_RAW mirrors, and instance 1 holds the real PWM. Only the NSH
`listener` reaches instance 1, so the probe opens a MAVLink shell and reads it
there. `listener` is a read-only command.

    python3 servo_probe.py sweep --side LEFT
    python3 servo_probe.py sweep --side BOTH --commands -1,-0.5,0,0.5,1
    python3 servo_probe.py monitor            # read-only, commands nothing

Requires the vehicle disarmed, the safety switch off, and COM_MOT_TEST_EN=1 -
the same preconditions as the trimmer's calibration.

The `expected_pwm()` here is an intentional independent copy of the trimmer's.
Importing it would make the probe agree with the code under test by
construction; MantaTrimmer also imports tkinter at module scope, so it cannot be
imported on a headless box anyway.
"""

import argparse
import csv
import os
import re
import struct
import sys
import threading
import time
from collections import deque

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink is not installed: pip install -r requirements.txt")

import serial

from manta_common import (
    POSITION_REGEX,
    SERIAL_BAUD,
    find_pico_port,
    position_to_degrees,
    report_path,
)

APP_DIR = os.path.dirname(os.path.abspath(__file__))
SETTINGS_PATH = os.path.join(APP_DIR, "settings.json")

MAV_CMD_ACTUATOR_TEST = 310
MAV_RESULT_ACCEPTED = 0

# SERIAL_CONTROL_DEV_SHELL, and RESPOND | EXCLUSIVE | MULTI. Without RESPOND the
# board runs the command and sends nothing back.
SERIAL_CONTROL_DEV_SHELL = 10
SERIAL_CONTROL_FLAGS = 2 | 4 | 16

# MAIN5 / MAIN6 carry the elevons; the actuator-test function ids and the
# parameter names come straight from MantaTrimmer.
SIDES = {
    "LEFT": {
        "function": 1201,
        "channel": 5,
        "min_param": "PWM_MAIN_MIN5",
        "max_param": "PWM_MAIN_MAX5",
        "trim_param": "CA_SV_CS0_TRIM",
    },
    "RIGHT": {
        "function": 1202,
        "channel": 6,
        "min_param": "PWM_MAIN_MIN6",
        "max_param": "PWM_MAIN_MAX6",
        "trim_param": "CA_SV_CS1_TRIM",
    },
}

DEFAULT_COMMANDS = (-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0)

# How long a surface gets to arrive before the PWM and the angle are sampled.
SETTLE_S = 1.5
SAMPLE_S = 0.5

# An actuator test expires on the FC. The requested timeout is NOT honoured:
# param2 asks for 10 s, but measured on this board the outputs revert after
# about 2 s, after which PX4 drives the surfaces itself. Every wait longer than
# TEST_REFRESH_S must therefore be spent re-sending the same command, or the FC
# silently takes the outputs back mid-measurement - which reads as a settling
# artefact and is not one.
TEST_TIMEOUT_S = 10.0
TEST_REFRESH_S = 0.5

# A readback this far from the commanded value means the override has lapsed.
AUTHORITY_TOLERANCE_US = 2

# ---- MantaTrimmer's calibration sweep dynamics, replayed by `settle` ----
#
# move_elevon_to_angle() steps the command by 0.01 every 0.25 s and stops the
# first time get_average_position_nonblocking() - a 0.5 s trailing mean over a
# 10 Hz stream - crosses the target. The increment is expressed here in *PWM
# microseconds* rather than command units on purpose: 0.01 command is 6 us only
# on the 900..2100 span the endpoint sweep runs at, and 4.05 us on the narrowed
# span the trim sweep runs at. Holding the physical step size fixed is what lets
# the probe reproduce the endpoint sweep's dynamics without writing any
# parameters to widen the range back out.
SWEEP_STEP_US = 6.0
SWEEP_TRIM_STEP_US = 4.05
SWEEP_PERIOD_S = 0.25
SWEEP_WINDOW_S = 0.5
SWEEP_COARSE_MULTIPLIER = 3.0
SWEEP_COARSE_BEYOND_DEG = 20.0
SWEEP_ACK_EVERY = 8


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)
# def


def expected_pwm(cmd, pwm_min, pwm_max, trim, rev):
    """MantaTrimmer's model of PX4, copied verbatim. This is the hypothesis."""
    effective_cmd = clamp(float(cmd) + float(trim), -1.0, 1.0)
    if rev:
        pwm = float(pwm_max) - ((effective_cmd + 1.0) * 0.5 * (float(pwm_max) - float(pwm_min)))
    else:
        pwm = float(pwm_min) + ((effective_cmd + 1.0) * 0.5 * (float(pwm_max) - float(pwm_min)))
    return int(round(pwm))
# def


def expected_pwm_no_trim(cmd, pwm_min, pwm_max, rev):
    """The same model with the trim ignored, i.e. trim applied downstream."""
    return expected_pwm(cmd, pwm_min, pwm_max, 0.0, rev)
# def


LISTENER_INSTANCE_RE = re.compile(
    r"Instance (?P<index>\d+):(?P<body>.*?)(?=Instance \d+:|\Z)", re.S)
LISTENER_OUTPUT_RE = re.compile(r"output: \[(?P<values>[^\]]*)\]")

# `listener <topic> -i N` prints a single-instance listing with no "Instance N:"
# block at all - the index only appears in the TOPIC header.
LISTENER_TOPIC_RE = re.compile(r"TOPIC: \S+ instance (?P<index>\d+)")


def parse_listener_outputs(text):
    """Pull {instance: [pwm, ...]} out of `listener actuator_outputs` output.

    Two shapes come back. Without -i, each instance gets an "Instance N:" block.
    With -i N, there is one listing and the index appears only in the TOPIC
    header - keying that as 0 would silently mislabel instance 1's PWM.
    """
    instances = {}

    for match in LISTENER_INSTANCE_RE.finditer(text):
        values = LISTENER_OUTPUT_RE.search(match.group("body"))
        if values is None:
            continue
        instances[int(match.group("index"))] = _parse_float_list(values.group("values"))

    if not instances:
        values = LISTENER_OUTPUT_RE.search(text)
        if values is not None:
            header = LISTENER_TOPIC_RE.search(text)
            index = int(header.group("index")) if header else 0
            instances[index] = _parse_float_list(values.group("values"))

    return instances
# def


def _parse_float_list(text):
    values = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            values.append(float(part))
        except ValueError:
            continue
    return values
# def


def command_increment_for_step_us(step_us, pwm_min, pwm_max):
    """Command units that move the output by step_us on the current span.

    The command-to-PWM map covers the span over a command range of 2.0, so one
    command unit is span/2 microseconds.
    """
    span = abs(float(pwm_max) - float(pwm_min))
    if span <= 0.0:
        raise ValueError("degenerate PWM span %r..%r" % (pwm_min, pwm_max))
    return (2.0 * float(step_us)) / span
# def


def reached_target(angle_deg, target_deg):
    """MantaTrimmer's stop test, copied from move_elevon_to_angle()."""
    if target_deg < 0.0 and angle_deg <= target_deg:
        return True
    if target_deg > 0.0 and angle_deg >= target_deg:
        return True
    if abs(target_deg) < 1e-6 and abs(angle_deg) <= 0.5:
        return True
    return False
# def


def parse_commands(text):
    values = []
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        values.append(clamp(float(part), -1.0, 1.0))
    if not values:
        raise ValueError("no commands parsed from %r" % text)
    return values
# def


class Fcu:
    """Just enough MAVLink for the probe. Single-threaded, no locking needed."""

    def __init__(self, device, baud=SERIAL_BAUD):
        self.device = device
        self.baud = baud
        self.master = None
        self._servo_raw = {}
        self.output_instance = None

    def connect(self, timeout=10.0):
        print("Connecting to %s..." % self.device)
        self.master = mavutil.mavlink_connection(self.device, self.baud)
        heartbeat = self.master.wait_heartbeat(timeout=timeout)
        if heartbeat is None:
            print("No heartbeat on %s after %.0fs" % (self.device, timeout))
            self.master = None
            return False
        print("Heartbeat from system %s component %s" %
              (self.master.target_system, self.master.target_component))
        return True
    # def

    def close(self):
        if self.master is not None:
            try:
                self.master.close()
            except Exception as e:
                print("MAVLink close failed: %s" % str(e))
            self.master = None
    # def

    def request_output_streams(self, rate_hz=20.0):
        """Ask for SERVO_OUTPUT_RAW anyway - it is free, and a cross-check.

        It reads zeros on this airframe (see the module docstring), so nothing
        depends on it; if a future PX4 build starts populating it, the sweep
        prints it beside the shell reading and the disagreement is visible.
        """
        interval_us = 1e6 / float(rate_hz)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, float(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW),
            float(interval_us), 0, 0, 0, 0, 0)
    # def

    def pump(self, duration_s):
        """Drain the link for a while, keeping the latest servo message."""
        deadline = time.time() + duration_s
        while time.time() < deadline:
            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            if msg.get_type() == "SERVO_OUTPUT_RAW":
                for channel in range(1, 17):
                    value = getattr(msg, "servo%d_raw" % channel, None)
                    if value is not None:
                        self._servo_raw[channel] = int(value)
    # def

    def servo_pwm(self, channel):
        """SERVO_OUTPUT_RAW's view. Zero on this airframe - cross-check only."""
        return self._servo_raw.get(int(channel))
    # def

    # ---- NSH shell, for the actuator_outputs readback ----

    def open_shell(self, timeout=3.0):
        self._shell_send("")
        banner = self._shell_drain(timeout)
        if "nsh>" not in banner:
            print("No NSH prompt over MAVLink; PWM readback unavailable")
            return False
        return True
    # def

    def _shell_send(self, command):
        data = (command + "\n").encode("ascii")
        self.master.mav.serial_control_send(
            SERIAL_CONTROL_DEV_SHELL, SERIAL_CONTROL_FLAGS, 0, 0,
            len(data), data.ljust(70, b"\x00"))
    # def

    def _shell_drain(self, seconds, until=None):
        out = ""
        deadline = time.time() + seconds
        while time.time() < deadline:
            msg = self.master.recv_match(type="SERIAL_CONTROL",
                                         blocking=True, timeout=0.3)
            if msg is None or not msg.count:
                continue
            out += bytes(msg.data[:msg.count]).decode("ascii", errors="replace")
            if until is not None and until in out:
                break
        return out
    # def

    def shell_command(self, command, timeout=5.0):
        self._shell_send(command)
        return self._shell_drain(timeout, until="nsh>")
    # def

    def read_actuator_outputs(self, instance=None):
        """The FC's own commanded PWM, straight off the actuator_outputs topic."""
        suffix = "" if instance is None else " -i %d" % instance
        text = self.shell_command("listener actuator_outputs -n 1" + suffix)
        return parse_listener_outputs(text)
    # def

    def find_live_instance(self):
        """Pick the actuator_outputs instance that is not all zeros."""
        instances = self.read_actuator_outputs()
        for index in sorted(instances):
            if any(value for value in instances[index]):
                return index
        return 0 if instances else None
    # def

    def _decode_param(self, msg):
        raw = msg.param_value
        if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
            return float(raw)
        if msg.param_type in (mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                              mavutil.mavlink.MAV_PARAM_TYPE_UINT32):
            return struct.unpack("<i", struct.pack("<f", raw))[0]
        return raw
    # def

    def get_param(self, name, timeout=5.0):
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component,
            name.encode("ascii"), -1)

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            got = msg.param_id
            if isinstance(got, bytes):
                got = got.decode("ascii", errors="ignore")
            if got.rstrip("\x00") == name:
                return self._decode_param(msg)

        print("Timed out reading %s" % name)
        return None
    # def

    def actuator_test(self, function, value, wait_ack=True, ack_timeout=1.0):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            MAV_CMD_ACTUATOR_TEST, 0,
            float(value), float(TEST_TIMEOUT_S), 0, 0, float(function), 0, 0)

        if not wait_ack:
            return None

        deadline = time.time() + ack_timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.2)
            if msg is not None and msg.command == MAV_CMD_ACTUATOR_TEST:
                return int(msg.result)
        return None
    # def
# class


class PicoAngles:
    """Background reader for the Pico's pot stream. Optional."""

    def __init__(self, device, scalers):
        self.device = device
        self.scalers = scalers
        self._samples = {"LEFT": deque(maxlen=64), "RIGHT": deque(maxlen=64)}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
    # def

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
    # def

    def _run(self):
        try:
            with serial.Serial(self.device, baudrate=SERIAL_BAUD, timeout=1.0) as port:
                while not self._stop.is_set():
                    line = port.readline()
                    if not line:
                        continue
                    match = POSITION_REGEX.search(line.decode("ascii", errors="ignore"))
                    if match is None:
                        continue
                    now = time.monotonic()
                    with self._lock:
                        self._samples["LEFT"].append(
                            (now, int(match.group("position1"))))
                        self._samples["RIGHT"].append(
                            (now, int(match.group("position2"))))
        except (serial.SerialException, OSError) as e:
            print("Pico reader stopped: %s" % str(e))
    # def

    def angle(self, side, window_s=0.5):
        """Mean angle over the last window_s, or None if nothing that recent."""
        cutoff = time.monotonic() - window_s
        with self._lock:
            values = [raw for (stamp, raw) in self._samples[side] if stamp >= cutoff]
        if not values:
            return None
        config = self.scalers.get(side, {})
        return position_to_degrees(
            side,
            sum(values) / float(len(values)),
            config.get("scaler", 0.0),
            config.get("offset", 0.0),
        )
    # def
# class


def load_scalers():
    import json
    try:
        with open(SETTINGS_PATH, "r") as handle:
            document = json.load(handle)
    except Exception as e:
        print("Could not read %s (%s); angles will be omitted" % (SETTINGS_PATH, e))
        return {}
    return {side: document.get(side, {}) for side in ("LEFT", "RIGHT")}
# def


def read_side_params(fcu, side):
    config = SIDES[side]
    params = {
        "pwm_min": fcu.get_param(config["min_param"]),
        "pwm_max": fcu.get_param(config["max_param"]),
        "trim": fcu.get_param(config["trim_param"]),
        "rev_bits": fcu.get_param("PWM_MAIN_REV"),
    }
    if None in (params["pwm_min"], params["pwm_max"], params["trim"]):
        return None
    rev_bits = int(params["rev_bits"] or 0)
    params["rev"] = ((rev_bits >> (config["channel"] - 1)) & 0x1) != 0
    return params
# def


def probe_command(fcu, pico, side, params, cmd):
    """Command one value, wait, and sample what the FC and the rig report."""
    config = SIDES[side]
    result = fcu.actuator_test(fcu_function(side), cmd)

    if result is not None and result != MAV_RESULT_ACCEPTED:
        print("  PX4 refused the actuator test (result=%d). Disarmed? Safety off? "
              "COM_MOT_TEST_EN=1?" % result)
        return None

    # The test expires on the FC after ~2 s, so the wait is spent refreshing it.
    sleep_with_keepalive(fcu, fcu_function(side), cmd, SETTLE_S + SAMPLE_S)
    fcu.pump(0.2)

    ok, actual, model_expected = check_authority(fcu, side, params, cmd)
    if not ok:
        print("  lost command authority at cmd %+.3f (read %s, expected %d)"
              % (cmd, actual, model_expected))
        return None

    servo_raw = fcu.servo_pwm(config["channel"])
    angle = None if pico is None else pico.angle(side)

    model = expected_pwm(cmd, params["pwm_min"], params["pwm_max"],
                         params["trim"], params["rev"])
    model_no_trim = expected_pwm_no_trim(cmd, params["pwm_min"], params["pwm_max"],
                                         params["rev"])

    return {
        "side": side,
        "cmd": cmd,
        "expected_pwm": model,
        "expected_pwm_no_trim": model_no_trim,
        "actual_pwm": actual,
        "servo_output_raw": servo_raw,
        "error_pwm": None if actual is None else int(actual) - model,
        "error_pwm_no_trim": None if actual is None else int(actual) - model_no_trim,
        "angle_deg": angle,
    }
# def


def fcu_function(side):
    return SIDES[side]["function"]
# def


def format_row(row):
    def number(value, fmt="%d"):
        return "--" if value is None else fmt % value

    return "  %-5s %+6.3f  %6s %6s %7s   %6s %7s   %8s" % (
        row["side"],
        row["cmd"],
        number(row["expected_pwm"]),
        number(row["actual_pwm"]),
        number(row["error_pwm"], "%+d"),
        number(row["expected_pwm_no_trim"]),
        number(row["error_pwm_no_trim"], "%+d"),
        number(row["angle_deg"], "%.2f"),
    )
# def


HEADER = ("  side    cmd   exp_pwm    act    err   exp(T=0)   err(T=0)  angle_deg")

CSV_COLUMNS = ["side", "cmd", "expected_pwm", "actual_pwm", "error_pwm",
               "expected_pwm_no_trim", "error_pwm_no_trim", "servo_output_raw",
               "angle_deg"]


SETTLE_COLUMNS = ["side", "target_deg", "period_s", "window_s", "step_us",
                  "repeat", "stop_cmd", "stop_pwm", "reported_deg", "settled_deg",
                  "overshoot_deg", "target_error_deg", "ticks", "sweep_s"]

TRACE_COLUMNS = ["side", "period_s", "repeat", "t_s", "angle_deg"]


def write_rows(rows, columns, filename):
    path = report_path(APP_DIR, filename)
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path
# def


def write_csv(rows, filename):
    return write_rows(rows, CSV_COLUMNS, filename)
# def


def summarise(rows, params_by_side):
    """The verdict. Two models are on trial; the errors say which PX4 uses."""
    print("")
    for side, params in params_by_side.items():
        side_rows = [r for r in rows if r["side"] == side and r["actual_pwm"] is not None]
        if not side_rows:
            print("%s: no PWM readback from actuator_outputs" % side)
            continue

        with_trim = max(abs(r["error_pwm"]) for r in side_rows)
        without_trim = max(abs(r["error_pwm_no_trim"]) for r in side_rows)

        print("%s: MIN=%d MAX=%d TRIM=%+.4f REV=%s" %
              (side, params["pwm_min"], params["pwm_max"], params["trim"],
               params["rev"]))
        print("      worst error vs the trimmer's model (trim added to cmd): %d us" % with_trim)
        print("      worst error vs the same model with trim ignored:       %d us" % without_trim)

        if with_trim <= 3:
            print("      -> the trimmer's model matches the FC. PWM is not the problem.")
        elif without_trim <= 3:
            print("      -> the FC does NOT fold the trim into the command the way the "
                  "trimmer assumes. expected_pwm() is wrong wherever trim != 0.")
        else:
            print("      -> neither model fits. Check REV, and whether MIN/MAX were "
                  "actually written (the calibration workers ignore the write result).")

        span = abs(params["pwm_max"] - params["pwm_min"])
        ends = {r["cmd"]: r["actual_pwm"] for r in side_rows}
        if -1.0 in ends and 1.0 in ends and span:
            reachable = abs(ends[1.0] - ends[-1.0])
            lost = span - reachable
            print("      commanded span %d us, reachable span %d us, lost %d us (%.1f%%)"
                  % (span, reachable, lost, 100.0 * lost / span))
    print("")
# def


def sleep_with_keepalive(fcu, function, cmd, duration_s):
    """time.sleep(), but the actuator test stays alive across it.

    The step cadence is what the settle test is measuring, so the command value
    must only change on a step boundary - this re-sends the *same* value, which
    refreshes the FC's timeout without disturbing the timing being replayed.
    """
    deadline = time.monotonic() + duration_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        time.sleep(min(TEST_REFRESH_S, remaining))
        if time.monotonic() < deadline:
            fcu.actuator_test(function, cmd, wait_ack=False)
# def


def read_side_pwm(fcu, side):
    outputs = fcu.read_actuator_outputs(fcu.output_instance)
    values = outputs.get(fcu.output_instance, [])
    index = SIDES[side]["channel"] - 1
    return int(round(values[index])) if index < len(values) else None
# def


def check_authority(fcu, side, params, cmd):
    """Is the actuator test still driving the output, or has PX4 taken it back?

    Returns (ok, actual, expected). Proving authority at each measurement is
    better than measuring when it lapses: it needs no deliberate expiry, and it
    catches the failure at the exact sample it would have corrupted.
    """
    expected = expected_pwm(cmd, params["pwm_min"], params["pwm_max"],
                            params["trim"], params["rev"])
    actual = read_side_pwm(fcu, side)
    if actual is None:
        return False, None, expected
    return abs(actual - expected) <= AUTHORITY_TOLERANCE_US, actual, expected
# def


def sweep_to_threshold(fcu, pico, side, params, target_deg, step_us,
                       period_s, window_s, timeout_s):
    """Replay MantaTrimmer's move_elevon_to_angle() and record where it stops.

    Faithful to the original in the ways that affect the result: the same
    centring command and 1 s pause, the same trailing-average stop test, the
    same coarse step beyond 20 deg, the same ACK cadence (its 0.3 s timeout is
    part of the loop's wall clock), and the same per-step sleep. `period_s` is
    the one knob - at 0.25 it *is* the rig today.
    """
    config = SIDES[side]
    function = config["function"]
    index = config["channel"] - 1

    increment = command_increment_for_step_us(step_us, params["pwm_min"],
                                              params["pwm_max"])
    if target_deg < 0.0:
        increment = -increment

    fcu.actuator_test(function, 0.0)
    sleep_with_keepalive(fcu, function, 0.0, 1.0)

    ok, actual, expected = check_authority(fcu, side, params, 0.0)
    if not ok:
        print("      the actuator test is not driving the output (read %s, "
              "expected %d) - aborting" % (actual, expected))
        return None

    cmd = 0.0
    ticks = 0
    started = time.monotonic()
    deadline = started + timeout_s

    while time.monotonic() < deadline:
        angle_deg = pico.angle(side, window_s=window_s)

        if angle_deg is None:
            time.sleep(0.25)
            continue

        if reached_target(angle_deg, target_deg):
            ok, actual, expected = check_authority(fcu, side, params, cmd)
            if not ok:
                print("      lost command authority at the stop point (read %s, "
                      "expected %d) - discarding this run" % (actual, expected))
                return None
            return {
                "stop_cmd": cmd,
                "stop_pwm": actual,
                "reported_deg": angle_deg,
                "ticks": ticks,
                "sweep_s": time.monotonic() - started,
            }

        if abs(target_deg - angle_deg) > SWEEP_COARSE_BEYOND_DEG:
            cmd += increment * SWEEP_COARSE_MULTIPLIER
        else:
            cmd += increment

        cmd = clamp(cmd, -1.0, 1.0)

        ticks += 1
        check_ack = (ticks % SWEEP_ACK_EVERY == 0)
        fcu.actuator_test(function, cmd, wait_ack=check_ack, ack_timeout=0.3)

        if (increment < 0.0 and cmd <= -1.0) or (increment > 0.0 and cmd >= 1.0):
            print("      hit the command limit before reaching %.1f deg" % target_deg)
            return None

        sleep_with_keepalive(fcu, function, cmd, period_s)

    print("      timed out before reaching %.1f deg" % target_deg)
    return None
# def


def hold_and_settle(fcu, pico, side, params, cmd, hold_s, window_s):
    """Hold the stop command still and watch the angle stop moving.

    Command authority is re-proved every second throughout. The first version of
    this held for 6 s while refreshing every 5 s and never sending at t=0, so the
    FC drove the surface for roughly 3 s of every cycle - the angle wandered as
    far as +16 deg mid-hold and the "settled" reading was taken while it was on
    its way back. Any lapse now discards the run instead of reporting it.
    """
    function = SIDES[side]["function"]
    trace = []

    fcu.actuator_test(function, cmd, wait_ack=False)
    started = time.monotonic()
    last_sent = started
    last_checked = started

    while True:
        now = time.monotonic()
        elapsed = now - started
        if elapsed >= hold_s:
            break

        if now - last_sent >= TEST_REFRESH_S:
            fcu.actuator_test(function, cmd, wait_ack=False)
            last_sent = now

        if now - last_checked >= 1.0:
            ok, actual, expected = check_authority(fcu, side, params, cmd)
            last_checked = time.monotonic()
            last_sent = last_checked
            if not ok:
                print("      lost command authority %.1f s into the hold (read %s, "
                      "expected %d) - discarding this run" % (elapsed, actual, expected))
                return None, trace

        angle_deg = pico.angle(side, window_s=window_s)
        if angle_deg is not None:
            trace.append((elapsed, angle_deg))
        time.sleep(0.1)

    return pico.angle(side, window_s=window_s), trace
# def


def run_settle(args):
    """Quantify how much of the endpoint error is settling time.

    Runs the rig's own sweep at its own cadence, then again at longer step
    periods. The difference between the angle the sweep *reported* when it
    stopped and the angle the surface *settles* at is the part a longer settle
    would recover; whatever is left against the target is not settling.
    """
    scalers = load_scalers()
    periods = [float(p) for p in args.periods.split(",") if p.strip()]

    fcu = Fcu(args.port or default_fcu_port())
    if fcu.device is None or not fcu.connect():
        return 2

    pico_port = args.pico_port or find_pico_port()
    if pico_port is None:
        print("The settle test needs the Pico for angles; none found")
        fcu.close()
        return 2
    pico = PicoAngles(pico_port, scalers)
    pico.start()

    rows = []
    traces = []
    try:
        if not fcu.open_shell():
            return 3
        fcu.output_instance = fcu.find_live_instance()
        if fcu.output_instance is None:
            print("actuator_outputs is not published; nothing to read back")
            return 3

        params = read_side_params(fcu, args.side)
        if params is None:
            print("%s: could not read the parameters" % args.side)
            return 3

        target = args.target if args.target is not None else load_target_angle()
        increment = command_increment_for_step_us(args.step_us, params["pwm_min"],
                                                  params["pwm_max"])

        print("\n%s  MIN=%d MAX=%d TRIM=%+.4f REV=%s" %
              (args.side, params["pwm_min"], params["pwm_max"], params["trim"],
               params["rev"]))
        print("target %.1f deg, step %.2f us (%.4f command units on this span), "
              "window %.2f s" % (target, args.step_us, increment, args.window))
        print("periods: %s s x %d repeats, %.0f s hold each\n" %
              (", ".join("%g" % p for p in periods), args.repeats, args.hold))

        for period in periods:
            for repeat in range(1, args.repeats + 1):
                label = "period %.2fs run %d" % (period, repeat)
                print("  %s ..." % label)

                result = sweep_to_threshold(fcu, pico, args.side, params, target,
                                            args.step_us, period, args.window,
                                            args.timeout)
                if result is None:
                    continue

                settled, trace = hold_and_settle(fcu, pico, args.side, params,
                                                 result["stop_cmd"], args.hold,
                                                 args.window)
                if settled is None:
                    fcu.actuator_test(SIDES[args.side]["function"], 0.0, wait_ack=False)
                    continue
                fcu.actuator_test(SIDES[args.side]["function"], 0.0, wait_ack=False)

                row = {
                    "side": args.side,
                    "target_deg": target,
                    "period_s": period,
                    "window_s": args.window,
                    "step_us": args.step_us,
                    "repeat": repeat,
                    "stop_cmd": round(result["stop_cmd"], 4),
                    "stop_pwm": result["stop_pwm"],
                    "reported_deg": None if result["reported_deg"] is None else round(result["reported_deg"], 2),
                    "settled_deg": None if settled is None else round(settled, 2),
                    "overshoot_deg": None if settled is None else round(settled - result["reported_deg"], 2),
                    "target_error_deg": None if settled is None else round(settled - target, 2),
                    "ticks": result["ticks"],
                    "sweep_s": round(result["sweep_s"], 1),
                }
                rows.append(row)
                traces.extend({"side": args.side, "period_s": period,
                               "repeat": repeat, "t_s": round(t, 2),
                               "angle_deg": round(a, 2)} for (t, a) in trace)

                print("    stop cmd %+.3f  pwm %s  reported %.2f  settled %.2f  "
                      "overshoot %+.2f  vs target %+.2f  (%d ticks, %.0f s)" %
                      (row["stop_cmd"], row["stop_pwm"], row["reported_deg"],
                       row["settled_deg"], row["overshoot_deg"],
                       row["target_error_deg"], row["ticks"], row["sweep_s"]))

                time.sleep(2.0)
    finally:
        try:
            fcu.actuator_test(SIDES[args.side]["function"], 0.0, wait_ack=False)
        except Exception:
            pass
        pico.stop()
        fcu.close()

    if rows:
        summarise_settle(rows)
        print("Wrote %s" % write_rows(rows, SETTLE_COLUMNS, "servo_probe_settle.csv"))
        print("Wrote %s" % write_rows(traces, TRACE_COLUMNS,
                                      "servo_probe_settle_trace.csv"))
    return 0
# def


def summarise_settle(rows):
    """Per-period means, and how much of the error the settle time explains."""
    periods = sorted({row["period_s"] for row in rows})

    print("")
    print("  period   n   reported   settled   overshoot   vs target")
    for period in periods:
        group = [r for r in rows if r["period_s"] == period
                 and r["settled_deg"] is not None]
        if not group:
            continue
        mean = lambda key: sum(r[key] for r in group) / float(len(group))
        print("  %5.2fs  %2d   %8.2f  %8.2f   %+9.2f   %+9.2f" %
              (period, len(group), mean("reported_deg"), mean("settled_deg"),
               mean("overshoot_deg"), mean("target_error_deg")))

    baseline = [r for r in rows if r["period_s"] == min(periods)
                and r["settled_deg"] is not None]
    slowest = [r for r in rows if r["period_s"] == max(periods)
               and r["settled_deg"] is not None]

    if baseline and slowest and len(periods) > 1:
        base_err = sum(abs(r["target_error_deg"]) for r in baseline) / len(baseline)
        slow_err = sum(abs(r["target_error_deg"]) for r in slowest) / len(slowest)
        print("")
        print("  at %.2fs (the rig today) the settled angle misses target by %.2f deg"
              % (min(periods), base_err))
        print("  at %.2fs it misses by %.2f deg" % (max(periods), slow_err))
        if base_err > 0.01:
            print("  -> a longer settle accounts for %.0f%% of the error; %.2f deg "
                  "is something else" % (100.0 * (base_err - slow_err) / base_err,
                                         slow_err))
    print("")
# def


def load_target_angle():
    import json
    try:
        with open(SETTINGS_PATH, "r") as handle:
            return float(json.load(handle)["ANGLES"]["angle_neg_degs"])
    except Exception:
        return -33.0
# def


def run_sweep(args):
    scalers = load_scalers()
    sides = ["LEFT", "RIGHT"] if args.side == "BOTH" else [args.side]
    commands = parse_commands(args.commands)

    if not args.yes:
        print("This moves the elevons through %d commands per side. Vehicle must be "
              "disarmed with the safety switch off." % len(commands))
        if input("Continue? [y/N] ").strip().lower() not in ("y", "yes"):
            return 1

    fcu = Fcu(args.port or default_fcu_port())
    if fcu.device is None or not fcu.connect():
        return 2

    pico = None
    if not args.no_pico:
        pico_port = args.pico_port or find_pico_port()
        if pico_port is None:
            print("No Pico found; continuing without angles")
        else:
            print("Pico on %s" % pico_port)
            pico = PicoAngles(pico_port, scalers)
            pico.start()

    rows = []
    params_by_side = {}
    try:
        fcu.request_output_streams()
        fcu.pump(1.0)

        if not fcu.open_shell():
            return 3

        fcu.output_instance = fcu.find_live_instance()
        if fcu.output_instance is None:
            print("actuator_outputs is not published; nothing to read back")
            return 3
        print("Reading actuator_outputs instance %d" % fcu.output_instance)

        for side in sides:
            params = read_side_params(fcu, side)
            if params is None:
                print("%s: could not read the parameters, skipping" % side)
                continue
            params_by_side[side] = params

            print("\n%s  MIN=%d MAX=%d TRIM=%+.4f REV=%s" %
                  (side, params["pwm_min"], params["pwm_max"], params["trim"],
                   params["rev"]))
            print(HEADER)

            for cmd in commands:
                row = probe_command(fcu, pico, side, params, cmd)
                if row is None:
                    break
                rows.append(row)
                print(format_row(row))

            fcu.actuator_test(fcu_function(side), 0.0, wait_ack=False)
    finally:
        for side in sides:
            try:
                fcu.actuator_test(fcu_function(side), 0.0, wait_ack=False)
            except Exception:
                pass
        if pico is not None:
            pico.stop()
        fcu.close()

    if rows:
        summarise(rows, params_by_side)
        print("Wrote %s" % write_csv(rows, "servo_probe_sweep.csv"))
    return 0
# def


def run_monitor(args):
    """Read-only: stream what the FC is commanding. Commands nothing itself."""
    scalers = load_scalers()
    fcu = Fcu(args.port or default_fcu_port())
    if fcu.device is None or not fcu.connect():
        return 2

    pico = None
    if not args.no_pico:
        pico_port = args.pico_port or find_pico_port()
        if pico_port is not None:
            pico = PicoAngles(pico_port, scalers)
            pico.start()

    try:
        fcu.request_output_streams()
        if fcu.open_shell():
            fcu.output_instance = fcu.find_live_instance()
        params_by_side = {side: read_side_params(fcu, side) for side in SIDES}
        deadline = time.time() + args.seconds

        while time.time() < deadline:
            fcu.pump(0.5)
            outputs = fcu.read_actuator_outputs(fcu.output_instance)
            values = outputs.get(fcu.output_instance, [])
            parts = []
            for side, config in SIDES.items():
                index = config["channel"] - 1
                actual = int(round(values[index])) if index < len(values) else None
                angle = None if pico is None else pico.angle(side)
                parts.append("%s pwm=%s angle=%s" % (
                    side,
                    "--" if actual is None else str(actual),
                    "--" if angle is None else "%.2f" % angle))
            print("  ".join(parts))
    except KeyboardInterrupt:
        pass
    finally:
        if pico is not None:
            pico.stop()
        fcu.close()
    return 0
# def


def default_fcu_port():
    """The trimmer's remembered drone port, else nothing - keep it simple."""
    import json
    try:
        with open(SETTINGS_PATH, "r") as handle:
            port = json.load(handle).get("PORTS", {}).get("drone")
    except Exception:
        port = None
    if port is None:
        print("No drone port in settings.json; pass --port")
    return port
# def


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("mode", choices=("sweep", "monitor", "settle"),
                        nargs="?", default="sweep")
    parser.add_argument("--port", help="MAVLink device or connection string")
    parser.add_argument("--pico-port", help="Pico serial device")
    parser.add_argument("--no-pico", action="store_true", help="skip the angle readout")
    parser.add_argument("--side", choices=("LEFT", "RIGHT", "BOTH"), default="BOTH")
    parser.add_argument("--commands", default=",".join("%g" % c for c in DEFAULT_COMMANDS))
    parser.add_argument("--seconds", type=float, default=30.0, help="monitor duration")
    parser.add_argument("--yes", action="store_true", help="skip the movement prompt")

    settle = parser.add_argument_group("settle mode")
    settle.add_argument("--target", type=float,
                        help="target angle (default: angle_neg_degs from settings.json)")
    settle.add_argument("--periods", default="0.25,1.0,2.0",
                        help="per-step sleeps to compare; 0.25 is the rig today")
    settle.add_argument("--repeats", type=int, default=2)
    settle.add_argument("--hold", type=float, default=6.0,
                        help="seconds to hold the stop command and watch it settle")
    settle.add_argument("--step-us", type=float, default=SWEEP_STEP_US, dest="step_us",
                        help="PWM step per tick; %g is the endpoint sweep, %g the trim sweep"
                             % (SWEEP_STEP_US, SWEEP_TRIM_STEP_US))
    settle.add_argument("--window", type=float, default=SWEEP_WINDOW_S,
                        help="angle averaging window (POSITION_WINDOW_S)")
    settle.add_argument("--timeout", type=float, default=240.0,
                        help="per-sweep ceiling; the rig uses 60 s at 0.25 s steps")
    args = parser.parse_args(argv)

    if args.mode == "monitor":
        return run_monitor(args)
    if args.mode == "settle":
        if not args.yes:
            print("This sweeps %s to the target angle %d times and holds it there. "
                  "Vehicle disarmed, safety off, servos powered."
                  % (args.side, len(args.periods.split(",")) * args.repeats))
            if input("Continue? [y/N] ").strip().lower() not in ("y", "yes"):
                return 1
        if args.side == "BOTH":
            print("settle runs one side at a time; pass --side LEFT or --side RIGHT")
            return 1
        return run_settle(args)
    return run_sweep(args)
# def


if __name__ == "__main__":
    sys.exit(main())
