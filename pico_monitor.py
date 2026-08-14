#!/usr/bin/env python3
"""Stream the Pico's raw pot data to the terminal.

A diagnostic companion to MantaTrimmer.py: it opens the same serial port, uses
the same line format and the same scaler/offset maths from settings.json, and
prints every stage of the conversion so a suspect number in the GUI can be
traced back to the ADC counts that produced it.

    python3 pico_monitor.py                 # decoded table, auto-detected port
    python3 pico_monitor.py --raw           # verbatim lines, no interpretation
    python3 pico_monitor.py --csv out.csv   # also append samples to a CSV
"""

import argparse
import json
import os
import sys
import time
from collections import deque

import serial

from manta_common import (
    APP_DIR,
    POSITION_REGEX,
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
    args = ap.parse_args()

    port = args.port or find_pico_port()
    if not port:
        print("No Pico found. Plug it in, or pass --port /dev/ttyACM0")
        return 1

    cal = load_calibration(args.settings)

    print("Port     : %s @ %d baud" % (port, args.baud))
    print("LEFT     : scaler=%.6f offset=%.4f   deg = (scaler * raw) + offset"
          % (cal["LEFT"]["scaler"], cal["LEFT"]["offset"]))
    print("RIGHT    : scaler=%.6f offset=%.4f   deg = -(scaler * raw) + offset"
          % (cal["RIGHT"]["scaler"], cal["RIGHT"]["offset"]))
    print("Ctrl-C to stop.\n")

    csv_file = None
    if args.csv:
        new = not os.path.exists(args.csv)
        csv_file = open(args.csv, "a")
        if new:
            csv_file.write("time,raw_left,raw_right,deg_left,deg_right\n")

    left_win = Window(args.window)
    right_win = Window(args.window)

    header = ("%-12s %8s %8s %9s %9s %9s %9s %6s %6s"
              % ("time", "rawL", "rawR", "degL", "degR",
                 "avgDegL", "avgDegR", "sprL", "sprR"))

    good = 0
    bad = 0
    t0 = time.time()

    try:
        with serial.Serial(port, baudrate=args.baud, timeout=1.0) as ser:
            while True:
                line = ser.readline()
                if not line:
                    continue

                text = line.decode("ascii", errors="ignore").strip()
                if not text:
                    continue

                elapsed = time.time() - t0

                if args.raw:
                    print("%8.3f  %r" % (elapsed, text))
                    continue

                m = POSITION_REGEX.search(text)
                if not m:
                    bad += 1
                    print("%8.3f  UNPARSED: %r" % (elapsed, text))
                    continue

                raw_l = int(m.group("position1"))
                raw_r = int(m.group("position2"))

                deg_l = position_to_degrees(cal, "LEFT", raw_l)
                deg_r = position_to_degrees(cal, "RIGHT", raw_r)

                left_win.add(raw_l)
                right_win.add(raw_r)

                if good % 20 == 0:
                    print(header)

                print("%-12.3f %8d %8d %9.2f %9.2f %9.2f %9.2f %6d %6d"
                      % (elapsed, raw_l, raw_r, deg_l, deg_r,
                         position_to_degrees(cal, "LEFT", left_win.mean()),
                         position_to_degrees(cal, "RIGHT", right_win.mean()),
                         left_win.spread(), right_win.spread()))

                if csv_file:
                    csv_file.write("%.3f,%d,%d,%.4f,%.4f\n"
                                   % (elapsed, raw_l, raw_r, deg_l, deg_r))
                    csv_file.flush()

                good += 1

    except KeyboardInterrupt:
        print("\nStopped. %d parsed, %d unparsed lines." % (good, bad))

    except (serial.SerialException, OSError) as e:
        print(describe_serial_error(port, e))
        return 1

    finally:
        if csv_file:
            csv_file.close()

    return 0
# def


if __name__ == "__main__":
    sys.exit(main())
