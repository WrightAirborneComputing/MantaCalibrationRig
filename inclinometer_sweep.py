#!/usr/bin/env python3
"""Measure elevon angles with the inclinometers, by hand or under command.

    python3 inclinometer_sweep.py pose reports/sweep.json neutral
    python3 inclinometer_sweep.py slam --cycles 10

`pose` reads one static pose and appends it to a sweep record: put the elevons
somewhere by hand, read, move, read again. It is how the hard stops in
ELEVON_HARD_STOPS.md were measured.

`slam` drives both elevons hard over between -1 and +1 through MAV_CMD_ACTUATOR_TEST
and reads where they settle. A command of +-1 lands on PX4's configured output
limits, the soft stops, not the mechanical ones.

Method, and why it is built this way:

  * Angles come from the board's corrected vectors (`D` mode), so the stored
    bias and scale are applied and nothing here re-derives them.

  * The elevon angle is the rotation about the hinge line, atan2(z, x), with X
    up and Y along the hinge. A module mounted upside down is a half-turn about
    Z, so its X and Y are negated first - see --flip.

  * Under command, each elevon is measured against CENTRE, which is fixed to
    the airframe body. The subtraction is sample by sample: every board line
    carries all three channels from one instant, so airframe movement during a
    hold cancels rather than being averaged in. CENTRE's own spread is reported
    so a hold the airframe moved through is visible.

  * Only the tail of each hold is used. An accelerometer on a surface that is
    still swinging reads its acceleration as well as gravity, so the transit and
    the overshoot after it are not angles. The window spread is reported so a
    surface that had not settled is visible too.
"""

import argparse
import json
import math
import os
import statistics
import sys
import threading
import time
from datetime import datetime

import serial

import sensor_cal as sc
from sensor_cal_logic import parse_corrected_line
from manta_common import APP_DIR, report_path

# Actuator output functions, matching range_test.py and MantaTrimmer.py.
OUTPUT_FUNCTION = {"LEFT": 1201, "RIGHT": 1202}
ELEVONS = ("LEFT", "RIGHT")
DATUM = "CENTRE"

# The modules' rated ceiling, and what the board is configured to.
RATE_HZ = 200

# The FC drops an actuator test override after about 2 s (MantaTrimmer.py,
# MEASURE_REFRESH_S), so a hold re-sends well inside that.
REFRESH_S = 0.5

# A hold's spread past this means the surface or the airframe was still moving.
UNSETTLED_DEG = 0.5

# Below this between -1 and +1 a surface was not driven, as in range_test.py.
MIN_TRAVEL_DEG = 3.0

# PJRC's USB vendor ID, which every Teensy enumerates with.
TEENSY_VID = 0x16C0

# LEFT is mounted X-down since 2026-10-02 (ELEVON_HARD_STOPS.md).
DEFAULT_FLIP = ("LEFT",)


def half_turn_z(vector):
    """A module mounted upside down by a half-turn about Z: negate X and Y."""
    return (-vector[0], -vector[1], vector[2])
# def


def hinge_angle(vector):
    """Rotation about Y, the hinge line, in degrees. Positive is PX4's positive."""
    return math.degrees(math.atan2(vector[2], vector[0]))
# def


def out_of_plane(vector):
    """How far the up vector leaves the X-Z plane, in degrees.

    asin(y / |a|) rather than atan2(y, x): the latter divides by x, which
    shrinks as the elevon swings, and overstates the tilt by 1/cos of the
    elevon angle.
    """
    magnitude = math.sqrt(sum(c * c for c in vector))
    return math.degrees(math.asin(max(-1.0, min(1.0, vector[1] / magnitude))))
# def


def to_frame(sample, flip):
    """A parsed corrected line to {channel: (x, y, z) in g}, flips applied."""
    out = {}
    for name in sc.CHANNELS:
        raw = sample.get(name)
        if raw is None:
            continue
        vector = tuple(c / 1e6 for c in raw[0:3])
        out[name] = half_turn_z(vector) if name in flip else vector
    return out
# def


def spread(values):
    """(mean, sd, min, max) of a list, sd 0 for a single value."""
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return statistics.fmean(values), sd, min(values), max(values)
# def


def summarise_static(frames):
    """Per channel: mean vector, its angles, magnitude and per-axis sd."""
    out = {}
    for name in sc.CHANNELS:
        rows = [f[name] for f in frames if name in f]
        if not rows:
            continue
        mean = [statistics.fmean(r[i] for r in rows) for i in range(3)]
        angles = [hinge_angle(r) for r in rows]
        out[name] = {
            "n": len(rows),
            "mean_g": mean,
            "sd_mg": [statistics.pstdev([r[i] for r in rows]) * 1000.0
                      for i in range(3)],
            "magnitude_g": math.sqrt(sum(c * c for c in mean)),
            "about_y_deg": hinge_angle(mean),
            "out_of_plane_deg": out_of_plane(mean),
            "about_y_min": min(angles),
            "about_y_max": max(angles),
        }
    return out
# def


def summarise_hold(frames):
    """One hold's tail: each elevon against CENTRE, sample by sample.

    Only frames carrying all three channels are used, since a relative angle
    needs both ends from one instant.
    """
    usable = [f for f in frames if DATUM in f and all(e in f for e in ELEVONS)]
    if not usable:
        return None

    datum = [hinge_angle(f[DATUM]) for f in usable]
    out = {"n": len(usable), DATUM: spread(datum)}
    for name in ELEVONS:
        absolute = [hinge_angle(f[name]) for f in usable]
        out[name] = {
            "relative": spread([a - d for a, d in zip(absolute, datum)]),
            "absolute": spread(absolute),
            "out_of_plane": statistics.fmean(out_of_plane(f[name])
                                             for f in usable),
        }
    return out
# def


class Stream:
    """Reads the board's corrected stream on a thread, timestamped on arrival.

    The thread owns the port for reading; commands are only sent before it
    starts and after it stops.
    """

    def __init__(self, ser, flip):
        self.ser = ser
        self.flip = flip
        self.frames = []        # (host_monotonic, board_t_us, {name: vector})
        self.lines = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
    # def

    def start(self):
        self._thread.start()
    # def

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
    # def

    def _run(self):
        while not self._stop.is_set():
            line = self.ser.readline().decode("ascii", "ignore").strip()
            sample = parse_corrected_line(line)
            if sample is None:
                continue
            frame = to_frame(sample, self.flip)
            with self._lock:
                self.frames.append((time.monotonic(), sample["t_us"], frame))
                self.lines.append(line)
    # def

    def between(self, t0, t1):
        with self._lock:
            return [f for (t, _, f) in self.frames if t0 <= t < t1]
    # def
# class


def open_board(port):
    """The board in corrected mode at full rate, or raise with why not."""
    ser = serial.Serial(port, baudrate=sc.SERIAL_BAUD, timeout=0.2)
    time.sleep(sc.SERIAL_SETTLE_S)
    ser.reset_input_buffer()

    identity = sc.send_command(ser, "I") or ""
    if sc.EXPECTED_DEVICE not in identity:
        ser.close()
        raise RuntimeError("%s is not teensy/rig: %r" % (port, identity))
    if "cal=none" in identity:
        ser.close()
        raise RuntimeError("the board has no stored calibration")

    sc.send_command(ser, "F%d" % RATE_HZ)
    reply = sc.send_command(ser, "D") or ""
    if not reply.startswith("# ACK"):
        ser.close()
        raise RuntimeError("the board would not switch to corrected mode: %r"
                           % reply)
    ser.reset_input_buffer()
    return ser, identity
# def


def close_board(ser):
    """Back to raw mode and stopped, as every other tool leaves it."""
    try:
        sc.send_command(ser, "R")
        sc.send_command(ser, "S")
    finally:
        ser.close()
# def


def find_board_port(explicit):
    """The Teensy's port by USB vendor ID, since there is exactly one on the rig.

    Not sensor_cal.find_board(): that calls PortCandidate methods which no
    longer exist since the port-label rework, and raises.
    """
    if explicit:
        return explicit
    from manta_common import list_serial_ports
    matches = [c.device for c in list_serial_ports() if c.vid == TEENSY_VID]
    if len(matches) != 1:
        raise RuntimeError("expected one Teensy, found %d; pass --board-port"
                           % len(matches))
    return matches[0]
# def


# ---------------------------------------------------------------- pose


def run_pose(args):
    flip = set(args.flip)
    ser, identity = open_board(find_board_port(args.board_port))
    stream = Stream(ser, flip)
    try:
        stream.start()
        t0 = time.monotonic()
        time.sleep(args.seconds)
        frames = stream.between(t0, time.monotonic())
    finally:
        stream.stop()
        close_board(ser)

    if os.path.exists(args.record):
        with open(args.record) as handle:
            record = json.load(handle)
    else:
        record = {
            "note": "Board-corrected vectors, no datum rotation. Modules in "
                    "'flipped' are mounted X-down, a half-turn about Z, and "
                    "have X and Y negated before any angle.",
            "flipped": sorted(flip),
            "identity": identity,
            "poses": [],
        }

    pose = {"label": args.label,
            "time": datetime.now().strftime("%Y%m%d_%H%M%S"),
            "sensors": summarise_static(frames)}
    neutral = next((p for p in record["poses"] if p["label"] == "neutral"),
                   None)

    for name in sc.CHANNELS:
        s = pose["sensors"].get(name)
        if s is None:
            print("%-7s NO DATA" % name)
            continue
        travel = ""
        if neutral and args.label != "neutral" and name in neutral["sensors"]:
            travel = "  travel %+8.3f" % (s["about_y_deg"]
                                          - neutral["sensors"][name]["about_y_deg"])
        print("%-7s n=%4d |a|=%.5f  about-Y %+8.3f (%+.2f/%+.2f)%s  "
              "out-of-plane %+6.2f  sd %s"
              % (name, s["n"], s["magnitude_g"], s["about_y_deg"],
                 s["about_y_min"], s["about_y_max"], travel,
                 s["out_of_plane_deg"],
                 " ".join("%.2f" % v for v in s["sd_mg"])))

    record["poses"].append(pose)
    with open(args.record, "w") as handle:
        json.dump(record, handle, indent=1)
    print("saved %s pose '%s' (%d poses)"
          % (args.record, args.label, len(record["poses"])))
    return 0
# def


# ---------------------------------------------------------------- slam


def hold(drone, stream, value, seconds, window):
    """Command both elevons to value, keep re-sending, summarise the tail."""
    t_cmd = time.monotonic()
    next_send = t_cmd
    while True:
        now = time.monotonic()
        if now >= t_cmd + seconds:
            break
        if now >= next_send:
            for name in ELEVONS:
                drone.command_elevon(OUTPUT_FUNCTION[name], value)
            next_send = now + REFRESH_S
        time.sleep(0.01)
    t_end = time.monotonic()
    return summarise_hold(stream.between(t_end - window, t_end))
# def


def fmt_hold(label, result):
    if result is None:
        return "  %-9s no complete frames" % label
    parts = ["  %-9s" % label]
    for name in ELEVONS:
        mean, sd, lo, hi = result[name]["relative"]
        flag = "*" if hi - lo > UNSETTLED_DEG else " "
        parts.append("%s %+8.3f (sd %.3f)%s" % (name, mean, sd, flag))
    d_mean, d_sd, d_lo, d_hi = result[DATUM]
    parts.append("CENTRE %+6.3f (span %.2f)" % (d_mean, d_hi - d_lo))
    return "  ".join(parts)
# def


def run_slam(args):
    flip = set(args.flip)

    # Imported here: MantaTrimmer pulls in tkinter and pymavlink, and --help
    # should work on a box with neither.
    from MantaTrimmer import DroneInterface

    board_port = find_board_port(args.board_port)
    drone = DroneInterface()
    if not drone.connect(args.drone_port):
        print("No MAVLink link on %s." % args.drone_port)
        return 1

    # Refused while armed, with the safety on, or with COM_MOT_TEST_EN unset -
    # all of which look like a stuck surface unless the ACK is read. A missing
    # ACK is only warned about, as the GUI does: a lossy link should not stop a
    # run that is otherwise working, and the summary flags a surface that did
    # not move.
    for name in ELEVONS:
        result = drone.command_elevon(OUTPUT_FUNCTION[name], 0.0,
                                      wait_ack=True)
        if result == drone.ACK_NOT_RECEIVED:
            print("Actuator test for %s: no ACK, continuing" % name)
        elif result != drone.MAV_RESULT_ACCEPTED:
            print("Actuator test for %s refused: %s"
                  % (name, drone.MAV_RESULT_NAMES.get(result, result)))
            drone.close()
            return 1

    ser, identity = open_board(board_port)
    stream = Stream(ser, flip)
    legs = []
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    print("\n*** LIVE - the control surfaces will move ***")
    print("%d cycles, hold %.1f s, tail %.2f s, %d Hz; angles relative to "
          "CENTRE, * = spread past %.1f deg\n"
          % (args.cycles, args.hold, args.window, RATE_HZ, UNSETTLED_DEG))

    try:
        stream.start()
        result = hold(drone, stream, 0.0, args.hold, args.window)
        legs.append({"cycle": 0, "command": 0.0, "result": result})
        print(fmt_hold("start 0", result))

        for cycle in range(1, args.cycles + 1):
            for value in (+1.0, -1.0):
                result = hold(drone, stream, value, args.hold, args.window)
                legs.append({"cycle": cycle, "command": value,
                             "result": result})
                print(fmt_hold("%2d  %+.0f" % (cycle, value), result))

        result = hold(drone, stream, 0.0, args.hold, args.window)
        legs.append({"cycle": args.cycles + 1, "command": 0.0,
                     "result": result})
        print(fmt_hold("end 0", result))

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        # Always park the surfaces, including on Ctrl-C.
        for name in ELEVONS:
            drone.command_elevon(OUTPUT_FUNCTION[name], 0.0)
        stream.stop()
        close_board(ser)
        drone.close()

    print("\nSUMMARY  (relative to CENTRE, mean +/- sd over cycles)")
    summary = {}
    for value in (+1.0, -1.0, 0.0):
        results = [l["result"] for l in legs
                   if l["command"] == value and l["result"] is not None]
        if not results:
            continue
        summary[value] = {}
        line = ["  %+.0f" % value]
        for name in ELEVONS:
            means = [r[name]["relative"][0] for r in results]
            mean, sd, lo, hi = spread(means)
            summary[value][name] = {"mean": mean, "sd": sd, "min": lo,
                                    "max": hi, "n": len(means)}
            line.append("%s %+8.3f +/- %.3f  (%+.2f to %+.2f)"
                        % (name, mean, sd, lo, hi))
        print("   ".join(line))

    if +1.0 in summary and -1.0 in summary:
        print("  range " + "   ".join(
            "%s %.3f" % (name, summary[+1.0][name]["mean"]
                         - summary[-1.0][name]["mean"])
            for name in ELEVONS))
        for name in ELEVONS:
            span = abs(summary[+1.0][name]["mean"] - summary[-1.0][name]["mean"])
            if span < MIN_TRAVEL_DEG:
                print("  %s moved %.2f deg: not actuated. Check the vehicle "
                      "is disarmed and COM_MOT_TEST_EN is 1." % (name, span))

    label = (args.name.strip() or "elevon").replace(" ", "_")
    json_path = report_path(APP_DIR, "%s_%s_slam.json" % (label, stamp))
    with open(json_path, "w") as handle:
        json.dump({
            "note": "Elevon angles about the hinge line relative to CENTRE, "
                    "sample by sample, over the tail of each hold. 'relative' "
                    "and 'absolute' are (mean, sd, min, max) in degrees.",
            "identity": identity, "flipped": sorted(flip),
            "cycles": args.cycles, "hold_s": args.hold,
            "window_s": args.window, "rate_hz": RATE_HZ,
            "legs": legs,
            "summary": {"%+.0f" % k: v for k, v in summary.items()},
        }, handle, indent=1)
    raw_path = report_path(APP_DIR, "%s_%s_slam_raw.txt" % (label, stamp))
    with open(raw_path, "w") as handle:
        handle.write("\n".join(stream.lines) + "\n")
    print("\nWritten:\n  %s\n  %s" % (json_path, raw_path))
    return 0
# def


def main():
    ap = argparse.ArgumentParser(
        description="Measure elevon angles with the inclinometers.")
    ap.add_argument("--board-port", help="Teensy device (default: auto-detect)")
    ap.add_argument("--flip", nargs="*", default=list(DEFAULT_FLIP),
                    choices=sc.CHANNELS,
                    help="modules mounted X-down, a half-turn about Z "
                         "(default: %s)" % " ".join(DEFAULT_FLIP))
    sub = ap.add_subparsers(dest="mode", required=True)

    pose = sub.add_parser("pose", help="read one static pose into a record")
    pose.add_argument("record", help="sweep JSON to append to")
    pose.add_argument("label", help="pose name; 'neutral' is the travel datum")
    pose.add_argument("--seconds", type=float, default=5.0)

    slam = sub.add_parser("slam", help="drive both elevons between -1 and +1")
    slam.add_argument("--drone-port", required=True,
                      help="flight controller device")
    slam.add_argument("--name", default="", help="label for the output files")
    slam.add_argument("--cycles", type=int, default=10)
    slam.add_argument("--hold", type=float, default=1.5,
                      help="seconds at each end (default: 1.5)")
    slam.add_argument("--window", type=float, default=0.75,
                      help="seconds at the end of a hold to average "
                           "(default: 0.75)")

    args = ap.parse_args()
    try:
        if args.mode == "pose":
            return run_pose(args)
        return run_slam(args)
    except (RuntimeError, serial.SerialException, OSError) as e:
        print(e)
        return 1
# def


if __name__ == "__main__":
    sys.exit(main())
