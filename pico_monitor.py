#!/usr/bin/env python3
"""Stream the Pico's raw pot data to the terminal, and measure its sample rate.

A diagnostic companion to MantaTrimmer.py: it opens the same serial port, uses
the same line format and the same scaler/offset maths from settings.json, and
prints every stage of the conversion so a suspect number in the GUI can be
traced back to the ADC counts that produced it.

Against the dual-rate firmware (pico/sampler.py) it also drives the mode switch
and reports the achieved rate two independent ways - see --stats.

    python3 pico_monitor.py                       # decoded table, auto-detected port
    python3 pico_monitor.py --raw                 # verbatim lines, no interpretation
    python3 pico_monitor.py --csv out.csv         # also append samples to a CSV
    python3 pico_monitor.py --mode fast --stats   # switch to 500 Hz and measure it
"""

import argparse
import json
import os
import re
import statistics
import sys
import time
from collections import deque

import serial

from manta_common import (
    APP_DIR,
    PICO_TICKS_MODULO,
    POSITION_REGEX,
    REPLY_PREFIX,
    SERIAL_BAUD,
    describe_serial_error,
    find_pico_port,
    position_to_degrees as raw_to_degrees,
)

SETTINGS_PATH = os.path.join(APP_DIR, "settings.json")

DEFAULTS = {
    "LEFT": {"scaler": 0.0042, "offset": -75.68},
    "RIGHT": {"scaler": 0.0045, "offset": +117.75},
}


def load_calibration(path):
    cal = {
        "LEFT": dict(DEFAULTS["LEFT"]),
        "RIGHT": dict(DEFAULTS["RIGHT"]),
    }

    try:
        with open(path, "r") as f:
            data = json.load(f)
    except (IOError, ValueError) as e:
        print("Could not read %s (%s) - using built-in defaults" % (path, e))
        return cal

    for side in ("LEFT", "RIGHT"):
        block = data.get(side, {})
        if "scaler" in block:
            cal[side]["scaler"] = float(block["scaler"])
        if "offset" in block:
            cal[side]["offset"] = float(block["offset"])

    return cal
# def


def position_to_degrees(cal, side, raw_position):
    return raw_to_degrees(side, raw_position, cal[side]["scaler"], cal[side]["offset"])
# def


class Window(object):
    """Rolling stats over the last N samples of one channel."""

    def __init__(self, size):
        self.samples = deque(maxlen=size)

    def add(self, value):
        self.samples.append(value)

    def mean(self):
        if not self.samples:
            return 0.0
        return sum(self.samples) / float(len(self.samples))

    def spread(self):
        if not self.samples:
            return 0
        return max(self.samples) - min(self.samples)
# def


class TickTracker(object):
    """Unwraps the Pico's ticks_us() and collects the inter-sample deltas.

    This is the board's own account of when it sampled, and it is the only
    trustworthy timing source: USB CDC delivers in bursts on the 1 ms frame
    boundary, so host arrival times describe the bus, not the ADC.
    """

    def __init__(self):
        self.deltas = []
        self._last_raw = None
    # def

    def feed(self, raw_us):
        if self._last_raw is not None:
            # Modulo subtraction, so a counter wrap costs nothing.
            self.deltas.append((raw_us - self._last_raw) % PICO_TICKS_MODULO)

        self._last_raw = raw_us
    # def

    def report(self, nominal_us=None):
        """Timing stats, or None if the stream carried no timestamps.

        The cadence estimate is the *median* delta, not the mean. A lost line
        merges two intervals into one double-length delta, which drags the mean
        up until the derived rate happens to match the host's line count - so
        mean-derived rate can never reveal a dropped line, while the median is
        untroubled by them. Drops show up instead as deltas that are a clean
        multiple of the cadence, which is what "dropped" counts.
        """
        if not self.deltas:
            return None

        median_us = statistics.median(self.deltas)

        # Guard: a degenerate stream (every sample same timestamp) would divide
        # by zero below.
        if median_us <= 0:
            return None

        gaps = 0
        dropped = 0
        residuals = []

        for delta in self.deltas:
            ratio = delta / median_us
            steps = int(round(ratio))

            if steps > 1:
                gaps += 1
                dropped += steps - 1
                # How close the gap is to a whole number of sample periods.
                # A line lost in transport leaves a hole of exactly N periods,
                # because the board went on sampling regardless. A board that
                # stalled leaves an arbitrary hole. See gap_shape below.
                residuals.append(abs(ratio - steps))

        return {
            "count": len(self.deltas),
            "mean_us": statistics.mean(self.deltas),
            "median_us": median_us,
            "min_us": min(self.deltas),
            "max_us": max(self.deltas),
            "stdev_us": statistics.stdev(self.deltas) if len(self.deltas) > 1 else 0.0,
            "sustained_hz": 1000000.0 / median_us,
            "gaps": gaps,
            "dropped": dropped,
            "nominal_us": nominal_us,
            "gap_residual": (statistics.mean(residuals) if residuals else None),
        }
    # def

    @staticmethod
    def gap_shape(residual):
        """Classify what a gap's shape says about its cause.

        A hole that is an exact multiple of the sample period means the board
        kept sampling and the host missed lines. A hole of some arbitrary length
        means the board itself stopped sampling for a while.

        Measured on an RP2040 at 500 Hz, collector stalls land around 3.8
        periods - clearly off a whole number - while lines dropped in transport
        come out within a percent of exactly 2.0.
        """
        if residual is None:
            return None
        if residual < 0.08:
            return "transport"
        return "stall"
    # def
# class


def build_command(mode, rate):
    """Translate --mode/--rate into a firmware command, or None to leave it be."""
    if rate is not None:
        return "F%d" % int(rate)
    if mode == "fast":
        return "F"
    if mode == "slow":
        return "S"
    return None
# def


# A freshly opened USB CDC port swallows writes sent too soon after open: the
# device's CDC stack is still settling and, on Linux, ModemManager may still be
# poking at it. Measured on the rig, a write issued immediately after open is
# reliably lost while one issued after ~0.3 s is not.
SERIAL_SETTLE_S = 0.3


def send_command(ser, command, timeout=1.5, attempts=3):
    """Send a command and drain the stream until the board acknowledges it.

    Discarding up to the ack matters for measurement: it drops the samples still
    in flight from the previous mode, so the rate report that follows is not
    polluted by lines produced at the old cadence.

    Retried because a lost write is indistinguishable from a board that does not
    answer, and every command here is idempotent - sending "F" twice leaves the
    board in the same place as sending it once.

    Returns the reply, or None if the board never answered after every attempt,
    which is what the legacy firmware does since it never reads stdin.
    """
    for attempt in range(attempts):
        ser.reset_input_buffer()
        ser.write((command + "\n").encode("ascii"))
        ser.flush()

        deadline = time.time() + timeout

        while time.time() < deadline:
            line = ser.readline().decode("ascii", errors="ignore").strip()

            if line.startswith(REPLY_PREFIX):
                return line

    return None
# def


def print_rate_report(elapsed, good, bad, replies, tracker, nominal_us):
    print("\nRate report")
    print("  duration          %.2f s" % elapsed)

    host_hz = (good / elapsed) if elapsed > 0 else 0.0
    print("  lines parsed      %d  (%.1f /s host-side)" % (good, host_hz))
    print("  unparsed          %d" % bad)
    print("  board replies     %d" % replies)

    timing = tracker.report(nominal_us)

    if timing is None:
        print("  board timing      unavailable - this mode emits no timestamp")
        print("                    (slow mode emits [L/R]; use --mode fast)")
        return

    nominal_us = timing["nominal_us"]

    print("  board timing (from the Pico's own ticks_us):")
    print("    intervals       %d" % timing["count"])
    print("    cadence         %.1f us  (%.2f Hz sustained)"
          % (timing["median_us"], timing["sustained_hz"]))
    print("    mean            %.1f us" % timing["mean_us"])
    print("    min / max       %d / %d us" % (timing["min_us"], timing["max_us"]))
    print("    stdev           %.1f us" % timing["stdev_us"])
    print("    gaps            %d  (%d samples missing from the stream)"
          % (timing["gaps"], timing["dropped"]))

    if nominal_us:
        print("    requested       %.1f us  (%.0f Hz)" % (nominal_us, 1000000.0 / nominal_us))

    print("")

    # Two distinct failure modes, and the board-side timestamp is what tells
    # them apart. Without it, both look identical from the host: fewer lines
    # per second than expected.
    problems = False

    if nominal_us and timing["median_us"] > nominal_us * 1.10:
        problems = True
        print("  WARNING: the board did not sustain the requested rate")
        print("           (asked %.0f Hz, held %.1f Hz). Every interval is long, not"
              % (1000000.0 / nominal_us, timing["sustained_hz"]))
        print("           just a few - the per-sample print cost has caught up with")
        print("           the sample period. Back the rate off.")

    if timing["dropped"]:
        problems = True
        loss = 100.0 * timing["dropped"] / float(timing["count"] + timing["dropped"])
        shape = TickTracker.gap_shape(timing["gap_residual"])

        print("  NOTE: %d samples missing (%.2f%%) across %d gaps; the cadence"
              % (timing["dropped"], loss, timing["gaps"]))
        print("        either side of them held at %.1f Hz." % timing["sustained_hz"])

        if shape == "transport":
            print("        The gaps are whole multiples of the sample period, so the")
            print("        board kept sampling and these lines were lost in transport")
            print("        or dropped by this reader.")
        else:
            print("        The gaps are not whole multiples of the sample period, so")
            print("        the board itself stopped sampling briefly. On MicroPython")
            print("        that is the garbage collector (~7 ms, unavoidable). Send G")
            print("        before a capture to take the stall up front - that buys a")
            print("        clean window of a few seconds.")

    if not problems:
        print("  OK: %d intervals, no gaps, cadence held to +/-%.0f us."
              % (timing["count"], max(abs(timing["max_us"] - timing["median_us"]),
                                      abs(timing["median_us"] - timing["min_us"]))))
# def


def main():
    ap = argparse.ArgumentParser(description="Stream raw Pico pot data to the terminal.")
    ap.add_argument("--port", help="serial device (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=SERIAL_BAUD)
    ap.add_argument("--raw", action="store_true",
                    help="print lines exactly as received, no decoding")
    ap.add_argument("--window", type=int, default=10,
                    help="samples in the rolling mean/spread (default: 10)")
    ap.add_argument("--csv", help="append decoded samples to this CSV file")
    ap.add_argument("--settings", default=SETTINGS_PATH,
                    help="calibration file to read (default: %s)" % SETTINGS_PATH)
    ap.add_argument("--mode", choices=("slow", "fast"),
                    help="switch the board's sample mode before reading "
                         "(slow: 10 Hz readable, fast: 500 Hz timestamped)")
    ap.add_argument("--rate", type=int,
                    help="switch the board to this rate in Hz; implies --mode fast")
    ap.add_argument("--seconds", type=float,
                    help="stop after this many seconds instead of waiting for Ctrl-C")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress per-sample output; use when measuring, so terminal "
                         "rendering is not mistaken for the board's ceiling")
    ap.add_argument("--stats", action="store_true",
                    help="print a sample-rate report on exit")
    args = ap.parse_args()

    port = args.port or find_pico_port()
    if not port:
        print("No Pico found. Plug it in, or pass --port /dev/ttyACM0")
        return 1

    cal = load_calibration(args.settings)

    print("Port     : %s @ %d baud (USB CDC - the baud rate is not applied)"
          % (port, args.baud))
    print("LEFT     : scaler=%.6f offset=%.4f   deg = (scaler * raw) + offset"
          % (cal["LEFT"]["scaler"], cal["LEFT"]["offset"]))
    print("RIGHT    : scaler=%.6f offset=%.4f   deg = -(scaler * raw) + offset"
          % (cal["RIGHT"]["scaler"], cal["RIGHT"]["offset"]))

    csv_file = None
    if args.csv:
        new = not os.path.exists(args.csv)
        csv_file = open(args.csv, "a")
        if new:
            csv_file.write("time,raw_left,raw_right,deg_left,deg_right,t_us\n")

    left_win = Window(args.window)
    right_win = Window(args.window)
    tracker = TickTracker()

    header = ("%-12s %8s %8s %9s %9s %9s %9s %6s %6s"
              % ("time", "rawL", "rawR", "degL", "degR",
                 "avgDegL", "avgDegR", "sprL", "sprR"))

    good = 0
    bad = 0
    replies = 0

    # Expected interval when a specific rate was requested. Sizes the "gap"
    # threshold; without it TickTracker falls back to the median delta.
    nominal_us = None

    t0 = time.time()
    elapsed = 0.0

    try:
        with serial.Serial(port, baudrate=args.baud, timeout=1.0) as ser:
            command = build_command(args.mode, args.rate)

            if command is not None:
                # Let the CDC link settle before the first write, or it is lost.
                time.sleep(SERIAL_SETTLE_S)

                print("Command  : %s" % command)
                reply = send_command(ser, command)

                if reply is None:
                    print("           no reply - this board is running the legacy "
                          "10 Hz firmware, which never reads commands")
                else:
                    print("Reply    : %s" % reply)
                    replies += 1

                    ack = re.search(r"ACK [SF] (\d+)", reply)
                    if ack:
                        nominal_us = 1000000.0 / float(ack.group(1))

                    # Take MicroPython's ~7 ms collector stall now, before the
                    # clock starts, rather than have it land somewhere in the
                    # middle of the measurement. Buys a few gap-free seconds.
                    send_command(ser, "G")

            if args.seconds:
                print("Reading for %.1f s...\n" % args.seconds)
            else:
                print("Ctrl-C to stop.\n")

            # Started after the mode switch, so the reported duration covers
            # only samples produced at the requested rate.
            t0 = time.time()
            deadline = (t0 + args.seconds) if args.seconds else None

            while True:
                if deadline is not None and time.time() >= deadline:
                    break

                line = ser.readline()
                if not line:
                    continue

                text = line.decode("ascii", errors="ignore").strip()
                if not text:
                    continue

                elapsed = time.time() - t0

                if text.startswith(REPLY_PREFIX):
                    replies += 1
                    if not args.quiet:
                        print("%8.3f  %s" % (elapsed, text))
                    continue

                if args.raw:
                    good += 1
                    if not args.quiet:
                        print("%8.3f  %r" % (elapsed, text))
                    continue

                m = POSITION_REGEX.search(text)
                if not m:
                    bad += 1
                    if not args.quiet:
                        print("%8.3f  UNPARSED: %r" % (elapsed, text))
                    continue

                raw_l = int(m.group("position1"))
                raw_r = int(m.group("position2"))

                t_us = m.group("t_us")
                if t_us is not None:
                    tracker.feed(int(t_us))

                deg_l = position_to_degrees(cal, "LEFT", raw_l)
                deg_r = position_to_degrees(cal, "RIGHT", raw_r)

                left_win.add(raw_l)
                right_win.add(raw_r)

                if not args.quiet:
                    if good % 20 == 0:
                        print(header)

                    print("%-12.3f %8d %8d %9.2f %9.2f %9.2f %9.2f %6d %6d"
                          % (elapsed, raw_l, raw_r, deg_l, deg_r,
                             position_to_degrees(cal, "LEFT", left_win.mean()),
                             position_to_degrees(cal, "RIGHT", right_win.mean()),
                             left_win.spread(), right_win.spread()))

                if csv_file:
                    # No flush per sample: at 500 Hz that is 500 fsyncs a second
                    # and it becomes the bottleneck. Closed in finally.
                    csv_file.write("%.6f,%d,%d,%.4f,%.4f,%s\n"
                                   % (elapsed, raw_l, raw_r, deg_l, deg_r,
                                      t_us if t_us is not None else ""))

                good += 1

            elapsed = time.time() - t0

    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print("\nStopped.")

    except (serial.SerialException, OSError) as e:
        print(describe_serial_error(port, e))
        return 1

    finally:
        if csv_file:
            csv_file.close()

    if args.stats:
        print_rate_report(elapsed, good, bad, replies, tracker, nominal_us)
    else:
        print("\n%d parsed, %d unparsed lines in %.2f s (%.1f /s)."
              % (good, bad, elapsed, (good / elapsed) if elapsed > 0 else 0.0))

    return 0
# def


if __name__ == "__main__":
    sys.exit(main())
