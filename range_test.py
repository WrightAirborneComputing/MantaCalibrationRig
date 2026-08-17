#!/usr/bin/env python3
"""Measure elevon travel range and slew rate from the console.

Drives the elevons hard over from -1 to +1 and back while capturing position at
1000 Hz, then reports how far they moved and how fast, averaged over 30 cycles.

By default both servos are driven together. Running them one at a time was how
this tool started - the point was to expose cross-coupling through shared power
or shared linkage - and it measured the same travel and rate either way. LEFT and
RIGHT therefore remain available through --phases for isolating a single servo,
but the default no longer spends three times the wall clock proving a null.

    python3 range_test.py --dry-run     # full pipeline, no actuator commands
    python3 range_test.py               # the real thing - surfaces will move

Method, and why it is built this way:

  * Position comes from the Pico's raw 1000 Hz stream, not from the GUI's
    get_average_position_nonblocking(). That getter averages over a trailing
    0.5 s window, which carries ~250 ms of group delay - the same order as the
    transit being measured. It would swamp the answer.

  * Timing comes from the Pico's own microsecond stamps, not from host arrival
    times. USB CDC delivers in 1 ms bursts, so host timestamps describe the bus.

  * Each leg is preceded by a "G" (collect garbage) so MicroPython's ~7 ms
    collector stall is paid before the capture rather than landing mid-transit.
    At 1000 Hz that buys a clean window of ~2.5 s against a ~2.4 s leg, so the
    margin is thin: an occasional GC-shaped gap late in a leg is expected. It
    costs ~7 consecutive samples, which is 7 ms of a ~100 ms transit, and the
    10-90% fit is taken across hundreds of samples either side of it.

  * The headline number is the 10-90% transit time. Endpoints are where a servo
    is slowest and least repeatable (approach, backlash, current limit), so
    10-90% is both the standard measure and the stable one. Rate is derived from
    it: 0.8 * travel / (t90 - t10).

Command latency is reported but is the weakest number here: it includes MAVLink
transport, the FC's own scheduling and the servo's dead time, and its zero is a
host timestamp with ~1-2 ms of USB uncertainty. Transit and rate are immune to
all of that, being differences between two Pico-stamped samples.
"""

import argparse
import csv
import math
import os
import statistics
import sys
import time
from datetime import datetime

import serial

from manta_common import (
    APP_DIR,
    DEFAULT_CYCLES,
    DEFAULT_PHASES,
    FAST_RATE_HZ,
    PICO_TICKS_MODULO,
    POSITION_REGEX,
    REPLY_PREFIX,
    SERIAL_BAUD,
    describe_serial_error,
    find_pico_port,
    report_path,
)
from pico_monitor import (
    SERIAL_SETTLE_S,
    SETTINGS_PATH,
    load_calibration,
    position_to_degrees,
    send_command,
)

# Actuator output functions, matching FourSliderGUI in MantaTrimmer.py.
LEFT_OUTPUT_FUNCTION = 1201
RIGHT_OUTPUT_FUNCTION = 1202

SIDES = ("LEFT", "RIGHT")

OUTPUT_FUNCTION = {
    "LEFT": LEFT_OUTPUT_FUNCTION,
    "RIGHT": RIGHT_OUTPUT_FUNCTION,
}

# One at a time, then all - so a difference under load would be attributable.
# Measured: there is none. DEFAULT_PHASES is BOTH alone; these other two stay
# selectable for isolating a single servo.
PHASES = ("LEFT", "RIGHT", "BOTH")

PHASE_SIDES = {
    "LEFT": ("LEFT",),
    "RIGHT": ("RIGHT",),
    "BOTH": ("LEFT", "RIGHT"),
}

# Below this the surface did not meaningfully move: servo unpowered, linkage
# disconnected, or a dry run. Reported as such rather than dividing by noise.
MIN_TRAVEL_DEG = 3.0

# Baseline captured before the command is issued, so t=0 has a defined start.
PRE_ROLL_S = 0.4


def signed_tick_delta(later, earlier):
    """Microseconds from `earlier` to `later`, correct across a counter wrap.

    ticks_us() wraps at 2^30. Pre-roll samples are *before* the anchor, so this
    has to yield negative values too - hence the half-modulo fold rather than a
    plain modulo.
    """
    delta = (later - earlier) % PICO_TICKS_MODULO

    if delta > PICO_TICKS_MODULO // 2:
        delta -= PICO_TICKS_MODULO

    return delta
# def


def read_until(ser, deadline, samples):
    """Collect parsed samples until the host clock passes `deadline`."""
    while time.monotonic() < deadline:
        line = ser.readline()
        if not line:
            continue

        text = line.decode("ascii", errors="ignore").strip()
        if not text or text.startswith(REPLY_PREFIX):
            continue

        match = POSITION_REGEX.search(text)
        if not match:
            continue

        t_us = match.group("t_us")

        samples.append((
            time.monotonic(),
            int(t_us) if t_us is not None else None,
            int(match.group("position1")),
            int(match.group("position2")),
        ))
# def


def to_series(samples, t_cmd_host, cal, side):
    """Convert raw samples to [(t_rel_s, angle_deg)], zeroed at the command.

    The zero is anchored on the first sample that arrived at or after the command
    was issued, and every other sample is placed relative to it using the Pico's
    clock. So the *shape* of the trace is exact and only the position of zero
    carries host-side uncertainty.
    """
    if not samples:
        return []

    anchor_index = None
    for index, (host_t, _, _, _) in enumerate(samples):
        if host_t >= t_cmd_host:
            anchor_index = index
            break

    if anchor_index is None:
        return []

    anchor_pico = samples[anchor_index][1]
    anchor_host = samples[anchor_index][0]

    series = []

    for host_t, pico_us, raw_l, raw_r in samples:
        if pico_us is not None and anchor_pico is not None:
            t_rel = signed_tick_delta(pico_us, anchor_pico) / 1000000.0
        else:
            # Legacy firmware, no stamps. Far coarser, but not wrong.
            t_rel = host_t - anchor_host

        raw = raw_l if side == "LEFT" else raw_r
        series.append((t_rel, position_to_degrees(cal, side, raw)))

    return series
# def


def crossing_time(series, baseline, travel, fraction):
    """First time the trace passes `fraction` of the way from baseline to final."""
    threshold = baseline + (fraction * travel)
    rising = travel > 0

    for t_rel, angle in series:
        if t_rel < 0:
            continue
        if (rising and angle >= threshold) or (not rising and angle <= threshold):
            return t_rel

    return None
# def


def settled_angle(values):
    """Where a trace ends up: the median of its last fifth, at least 5 samples.

    Shared by the swing analysis and the creep measurement so the two are
    estimated identically - their difference is the stiction number, and it
    would be meaningless if each end of the subtraction were computed a
    different way. Median, not mean, so one ADC outlier cannot define an
    endpoint.
    """
    values = list(values)
    if not values:
        return None
    tail = max(5, len(values) // 5)
    return statistics.median(values[-tail:])
# def


def analyse_leg(series):
    """Travel and transit metrics for one hard-over, or a 'did not move' result."""
    pre = [a for t, a in series if t < 0.0]
    post = [(t, a) for t, a in series if t >= 0.0]

    if len(post) < 20:
        return {"ok": False, "reason": "too few samples (%d)" % len(post)}

    # Medians throughout: a single ADC outlier must not define an endpoint.
    baseline = statistics.median(pre) if len(pre) >= 3 else \
        statistics.median([a for _, a in post[:5]])

    final = settled_angle([a for _, a in post])

    travel = final - baseline

    if abs(travel) < MIN_TRAVEL_DEG:
        return {
            "ok": False,
            "reason": "no movement (%.2f deg)" % travel,
            "baseline_deg": baseline,
            "final_deg": final,
            "travel_deg": travel,
        }

    t10 = crossing_time(series, baseline, travel, 0.10)
    t90 = crossing_time(series, baseline, travel, 0.90)
    t95 = crossing_time(series, baseline, travel, 0.95)

    if t10 is None or t90 is None or t90 <= t10:
        return {"ok": False, "reason": "could not resolve 10-90% crossings"}

    transit = t90 - t10

    return {
        "ok": True,
        "baseline_deg": baseline,
        "final_deg": final,
        "travel_deg": travel,
        "latency_s": t10,
        "t90_s": t90,
        "t95_s": t95,
        "transit_s": transit,
        "rate_deg_s": abs(0.8 * travel) / transit,
        "samples": len(post),
    }
# def


def endpoint_stats(legs):
    """((max mean, sd), (min mean, sd)) of the settled angles a side reaches.

    A hard-over ends at one of two places, so the settled angles form two
    clusters and which one is the maximum depends on the side's sign convention
    rather than on the direction name. They are therefore split by direction and
    then ordered by value, not by label.

    Reported alongside range because range is their difference and conceals
    them: a surface whose whole travel has shifted a few degrees reports an
    unchanged range, while its endpoints have both moved. The sd is per
    endpoint, so an endpoint that wanders shows up here even when travel does
    not - which is exactly the failure that a mean-of-travel cannot express.
    """
    by_direction = {}

    for leg in legs:
        by_direction.setdefault(leg["direction"], []).append(leg["final_deg"])

    clusters = [mean_sd(v) for v in by_direction.values() if v]

    if not clusters:
        return ((None, None), (None, None))

    if len(clusters) == 1:
        # Only one direction ran, so the surface has one known endpoint.
        return (clusters[0], (None, None))

    clusters.sort(key=lambda pair: pair[0])
    return (clusters[-1], clusters[0])
# def


def creep_commands(start, target, step):
    """The command sequence a creep walks through, start exclusive, target last.

    Mirrors the step logic the trim calibration uses - a fixed increment at a
    fixed period - but walks to a *command* rather than to an angle, because the
    end stop is where the command runs out, not where a target angle is crossed.
    The final entry is always exactly the target: a creep that stopped one step
    short would be measuring a different PWM than the swing it is compared with,
    and the whole point is that the two arrive at the same place.
    """
    step = abs(float(step))
    if step <= 0.0:
        raise ValueError("creep step must be positive")

    start = float(start)
    target = float(target)
    direction = 1.0 if target > start else -1.0

    commands = []
    value = start
    while abs(target - value) > step:
        value += direction * step
        commands.append(round(value, 6))

    commands.append(target)
    return commands
# def


def stiction_stats(creep_values, swing_values):
    """Compare crept-to and swung-to settled angles at the same end stop.

    The difference is the stiction estimate: how much further the surface
    travels when it arrives with momentum than when it is walked in.

    The two sets are measured in separate phases, so there is no meaningful
    pairing between an individual creep and an individual swing - a per-rep
    difference would be an artefact of the order they happened to run in. The
    estimate is therefore the difference of the means, and its spread is the
    two sample deviations added in quadrature, which is the scatter a single
    paired comparison would show. `stiction_se` is the standard error of the
    difference of the means, and is the one to read when asking whether the
    difference is real rather than how much it varies.
    """
    creep_mean, creep_sd = mean_sd(list(creep_values))
    swing_mean, swing_sd = mean_sd(list(swing_values))

    result = {
        "creep_mean": creep_mean, "creep_sd": creep_sd, "creep_n": len(creep_values),
        "swing_mean": swing_mean, "swing_sd": swing_sd, "swing_n": len(swing_values),
        "stiction": None, "stiction_sd": None, "stiction_se": None,
    }

    if creep_mean is None or swing_mean is None:
        return result

    result["stiction"] = swing_mean - creep_mean

    if creep_sd is None or swing_sd is None:
        return result

    result["stiction_sd"] = math.sqrt((creep_sd ** 2) + (swing_sd ** 2))
    result["stiction_se"] = math.sqrt((creep_sd ** 2) / len(creep_values)
                                      + (swing_sd ** 2) / len(swing_values))
    return result
# def


def num(value, fmt_spec):
    """A number for the CSV, or an empty cell when it is undefined."""
    return "" if value is None else fmt_spec % value
# def


def mean_sd(values):
    """(mean, sample stdev) - stdev is None when a single value makes it undefined."""
    if not values:
        return (None, None)
    if len(values) == 1:
        return (values[0], None)
    return (statistics.mean(values), statistics.stdev(values))
# def


def fmt(mean, sd, unit, width=0):
    if mean is None:
        return "%*s" % (width, "-")
    if sd is None:
        return "%*s" % (width, "%.2f %s" % (mean, unit))
    return "%*s" % (width, "%.2f +/- %.2f %s" % (mean, sd, unit))
# def


class RangeRateTest:
    def __init__(self, drone, ser, cal, args):
        self.drone = drone
        self.ser = ser
        self.cal = cal
        self.args = args

        self.results = []     # one dict per (phase, cycle, direction, side)
        self.raw_rows = []    # every sample, for plotting later
    # def

    def command(self, sides, value):
        for side in sides:
            self.drone.command_elevon(OUTPUT_FUNCTION[side], value)
    # def

    def run_leg(self, phase, cycle, direction, sides, target):
        """One hard-over: settle, collect garbage, capture, command, capture."""
        # Take the collector stall now, so the capture window is clean.
        send_command(self.ser, "G")
        self.ser.reset_input_buffer()

        samples = []

        # Baseline before the command, so t=0 has something to be measured from.
        read_until(self.ser, time.monotonic() + PRE_ROLL_S, samples)

        t_cmd = time.monotonic()
        self.command(sides, target)

        read_until(self.ser, t_cmd + self.args.settle, samples)

        for side in PHASE_SIDES[phase]:
            series = to_series(samples, t_cmd, self.cal, side)
            metrics = analyse_leg(series)

            metrics.update({
                "phase": phase,
                "cycle": cycle,
                "direction": direction,
                "side": side,
                "target": target,
            })
            self.results.append(metrics)

            if self.args.samples_csv:
                for t_rel, angle in series:
                    self.raw_rows.append([
                        phase, cycle, direction, side,
                        "%.6f" % t_rel, "%.4f" % angle,
                    ])

            status = "%.2f deg in %.0f ms (%.0f deg/s)" % (
                metrics["travel_deg"], metrics["transit_s"] * 1000.0,
                metrics["rate_deg_s"]) if metrics["ok"] else metrics["reason"]

            print("    %-5s %-9s %s" % (side, direction, status))
    # def

    def run_phase(self, phase):
        sides = PHASE_SIDES[phase]

        print("\n=== %s ===" % phase)
        print("  parking at -1 ...")

        self.command(sides, -1.0)
        time.sleep(self.args.settle)

        for cycle in range(1, self.args.cycles + 1):
            print("  cycle %d/%d" % (cycle, self.args.cycles))

            self.run_leg(phase, cycle, "neg_to_pos", sides, +1.0)
            self.run_leg(phase, cycle, "pos_to_neg", sides, -1.0)

        self.command(sides, 0.0)
    # def

    def run(self):
        for phase in self.args.phases:
            self.run_phase(phase)
    # def

    def summarise(self):
        print("\n" + "=" * 78)
        print("SUMMARY  (mean +/- sample stdev over %d cycles)" % self.args.cycles)
        print("=" * 78)

        rows = []

        for phase in self.args.phases:
            for side in PHASE_SIDES[phase]:
                ok = [r for r in self.results
                      if r["phase"] == phase and r["side"] == side and r["ok"]]

                if not ok:
                    print("\n%-5s %-5s   no valid measurements" % (phase, side))
                    continue

                # Range is the span between the two settled endpoints, which is
                # the same physical quantity whichever way you crossed it.
                ends = [r["final_deg"] for r in ok]
                span = max(ends) - min(ends)

                angle_max, angle_min = endpoint_stats(ok)

                print("\n%s phase, %s elevon" % (phase, side))
                print("  range           %.2f deg   (%.2f to %.2f)"
                      % (span, min(ends), max(ends)))
                print("  endpoints   max %s   min %s"
                      % (fmt(*angle_max, unit="deg"), fmt(*angle_min, unit="deg")))

                for direction in ("neg_to_pos", "pos_to_neg"):
                    legs = [r for r in ok if r["direction"] == direction]
                    if not legs:
                        continue

                    transit = mean_sd([r["transit_s"] * 1000.0 for r in legs])
                    rate = mean_sd([r["rate_deg_s"] for r in legs])
                    travel = mean_sd([abs(r["travel_deg"]) for r in legs])
                    latency = mean_sd([r["latency_s"] * 1000.0 for r in legs])

                    print("  %-11s travel %s   transit %s   rate %s   latency %s"
                          % (direction,
                             fmt(*travel, unit="deg"),
                             fmt(*transit, unit="ms"),
                             fmt(*rate, unit="deg/s"),
                             fmt(*latency, unit="ms")))

                    rows.append([
                        phase, side, direction, len(legs),
                        "%.3f" % span,
                        num(angle_max[0], "%.3f"), num(angle_max[1], "%.3f"),
                        num(angle_min[0], "%.3f"), num(angle_min[1], "%.3f"),
                        "%.3f" % travel[0],
                        "" if travel[1] is None else "%.3f" % travel[1],
                        "%.2f" % transit[0],
                        "" if transit[1] is None else "%.2f" % transit[1],
                        "%.1f" % rate[0],
                        "" if rate[1] is None else "%.1f" % rate[1],
                        "%.2f" % latency[0],
                        "" if latency[1] is None else "%.2f" % latency[1],
                    ])

        return rows
    # def
# class


def write_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
    print("  %s  (%d rows)" % (path, len(rows)))
# def


def main():
    ap = argparse.ArgumentParser(
        description="Measure elevon travel range and slew rate.")
    ap.add_argument("--pico-port", help="Pico serial device (default: auto-detect)")
    ap.add_argument("--drone-port", help="flight controller device (default: auto-detect)")
    ap.add_argument("--settings", default=SETTINGS_PATH)
    ap.add_argument("--name", default="", help="label for the output files")
    ap.add_argument("--cycles", type=int, default=DEFAULT_CYCLES,
                    help="hard-over cycles per phase (default: %d)" % DEFAULT_CYCLES)
    ap.add_argument("--settle", type=float, default=2.0,
                    help="seconds allowed for a hard-over to complete (default: 2.0)")
    ap.add_argument("--rate", type=int, default=FAST_RATE_HZ,
                    help="Pico sample rate in Hz (default: %d)" % FAST_RATE_HZ)
    ap.add_argument("--phases", nargs="+", default=list(DEFAULT_PHASES), choices=PHASES,
                    help="which phases to run (default: %s). LEFT and RIGHT measure "
                         "the same travel and rate as BOTH, so they are for isolating "
                         "one servo rather than for routine runs."
                         % " ".join(DEFAULT_PHASES))
    ap.add_argument("--samples-csv", action="store_true",
                    help="also write every captured sample, for plotting. At the "
                         "default cycles and rate this is ~290k rows, ~20 MB.")
    ap.add_argument("--dry-run", action="store_true",
                    help="exercise everything but send no actuator commands")
    args = ap.parse_args()

    if args.dry_run:
        # Honoured inside DroneInterface.command_elevon().
        os.environ["MANTA_NO_ACTUATE"] = "1"

    # Imported here: MantaTrimmer pulls in tkinter and pymavlink, and --help
    # should work on a box with neither.
    from MantaTrimmer import DroneInterface

    pico_port = args.pico_port or find_pico_port()
    if not pico_port:
        print("No Pico found. Plug it in, or pass --pico-port /dev/ttyACM0")
        return 1

    cal = load_calibration(args.settings)

    print("Pico     : %s" % pico_port)
    print("LEFT     : scaler=%.6f offset=%.4f"
          % (cal["LEFT"]["scaler"], cal["LEFT"]["offset"]))
    print("RIGHT    : scaler=%.6f offset=%.4f"
          % (cal["RIGHT"]["scaler"], cal["RIGHT"]["offset"]))

    drone_port = args.drone_port or _autodetect_drone(pico_port)

    drone = DroneInterface()

    if not drone.connect(drone_port):
        print("\nNo MAVLink link on %s - cannot command the elevons." % drone_port)
        return 1

    print("Phases   : %s" % " ".join(args.phases))
    print("Cycles   : %d per phase, settle %.1f s" % (args.cycles, args.settle))

    if args.dry_run:
        print("\n*** DRY RUN - no actuator commands will be sent ***")
    else:
        print("\n*** LIVE - the control surfaces will move ***")

    test = None

    try:
        with serial.Serial(pico_port, baudrate=SERIAL_BAUD, timeout=0.1) as ser:
            time.sleep(SERIAL_SETTLE_S)

            reply = send_command(ser, "F%d" % args.rate)
            if reply is None:
                print("\nThe Pico did not accept a rate command. It is probably running")
                print("the legacy 10 Hz firmware, which is far too slow to resolve a")
                print("transit. Upload pico/sampler.py as main.py first.")
                return 1

            print("Pico mode: %s\n" % reply.lstrip("# "))

            test = RangeRateTest(drone, ser, cal, args)

            try:
                test.run()
            finally:
                # Always park the surfaces, including on Ctrl-C. The actuator
                # test command holds its value for 60 s otherwise, so skipping
                # this would leave the elevons hard over.
                print("\nCentring elevons...")
                for side in SIDES:
                    drone.command_elevon(OUTPUT_FUNCTION[side], 0.0)

                send_command(ser, "S")

    except (serial.SerialException, OSError) as e:
        print(describe_serial_error(pico_port, e))
        return 1

    except KeyboardInterrupt:
        # Partial results are still worth reporting.
        print("\nInterrupted.")

    finally:
        drone.close()

    if test is None or not test.results:
        print("\nNo measurements captured.")
        return 1

    rows = test.summarise()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    label = (args.name.strip() or "elevon").replace(" ", "_")

    print("\nWritten:")
    write_csv(
        report_path(APP_DIR, "%s_%s_rangerate.csv" % (label, stamp)),
        ["phase", "side", "direction", "cycles", "range_deg",
         "angle_max_deg", "angle_max_sd", "angle_min_deg", "angle_min_sd",
         "travel_deg", "travel_sd", "transit_ms", "transit_sd",
         "rate_deg_s", "rate_sd", "latency_ms", "latency_sd"],
        rows,
    )

    if args.samples_csv:
        write_csv(
            report_path(APP_DIR, "%s_%s_rangerate_samples.csv" % (label, stamp)),
            ["phase", "cycle", "direction", "side", "t_rel_s", "angle_deg"],
            test.raw_rows,
        )

    return 0
# def


def _autodetect_drone(pico_port):
    """First non-Pico USB serial port. The heartbeat is what actually decides."""
    from manta_common import list_serial_ports

    for candidate in list_serial_ports():
        if candidate.device != pico_port:
            return candidate.device

    return None
# def


if __name__ == "__main__":
    sys.exit(main())
