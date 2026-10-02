"""Six-face calibration wizard for the rig's three inclinometer modules.

Guides the operator through resting the sensor plate on each of its six faces,
detects each face change on its own, holds a long capture, then solves bias and
scale per sensor per axis and reports whether the answer is believable.

    python3 sensor_cal.py                       # find the board, run the wizard
    python3 sensor_cal.py --seconds 60          # longer holds
    python3 sensor_cal.py --tilts 0             # six faces only, no extra
    python3 sensor_cal.py --replay reports/...  # re-solve a saved capture

Requires teensy/rig, which streams all three channels continuously. The
bring-up firmware cannot drive this: its passthrough carries one channel at a
time and stops after 20 seconds, and a plate-consistency check between sensors
means nothing unless the three readings are simultaneous.

All the arithmetic and every acceptance decision lives in sensor_cal_logic.py,
which imports nothing and is tested without hardware. This file is the driving:
the port, the prompts, the waiting, the files. That split is the same one
endpoint_cal.py and endpoint_logic.py already use, and the reason is the same -
the method is the part that has to be reviewable and re-runnable, and it should
not be reachable only by holding a plate.

Every raw sample is written to a capture file as it arrives, and --replay
re-runs the whole solve from one. Refining the method should not require the
rig, and a capture that produced a surprising answer is the thing you want to
still have next week.

The procedure and its justifications are documented in SENSORS.md, "The
six-face calibration".
"""

import argparse
import csv
import json
import os
import sys
import time
import zlib
from datetime import datetime

import serial

from manta_common import (
    APP_DIR,
    REPLY_PREFIX,
    SERIAL_BAUD,
    describe_serial_error,
    list_serial_ports,
    report_path,
)
from sensor_cal_logic import (
    CHANNELS,
    MIN_CAPTURE_SAMPLES,
    angle_between_deg,
    apply_rotation,
    counts_to_g,
    mean_observation,
    parse_corrected_line,
    vector_norm,
    FACE_TILT_REJECT_DEG,
    FaceTracker,
    SUGGESTED_FACE_ORDER,
    parse_sample_line,
    plate_consistency,
    sample_face,
    sample_is_quiet,
    sanity_problems,
    solve_sensor,
    summarise_capture,
)


# The board's own reply to "I" must start with this or the wizard is talking to
# the wrong firmware, which is worth catching immediately rather than after six
# faces of captures that decode to nothing.
EXPECTED_DEVICE = "dev=manta-rig"

# A write issued immediately after opening a CDC port is reliably lost while one
# issued after ~0.3 s is not - the device's stack is still settling and, on
# Linux, ModemManager may still be poking at it. Measured on this rig; the same
# constant is in pico_monitor.py for the same reason.
SERIAL_SETTLE_S = 0.3

# Three seconds, not thirty. Measured on the rig: the modules' raw accelerometer
# scatter is 0.36-1.37 mg per axis, so 600 samples pin the mean to well under a
# tenth of a milligravity - against sanity gates that care about 100 mg. Holding
# for ten times as long improves the mean by a factor of three and improves
# nothing that matters, while asking the operator to stay still for thirty
# seconds per face, six to nine times over. Under hand wander a longer hold is
# actively worse; see sensor_cal_logic.mean_observation.
DEFAULT_HOLD_S = 3.0
DEFAULT_TILTS = 3
DEFAULT_RATE_HZ = 200

# The tracked, top-level record of rig sensor calibrations. Deliberately not
# calibration_log.csv, which is a fixed-schema append-only record of per-drone
# elevon calibrations keyed on drone name and UID - a rig sensor calibration has
# neither key and is a different kind of event.
SENSOR_CAL_LOG = "sensor_cal_log.csv"

LOG_COLUMNS = [
    "timestamp", "sensor", "checksum",
    "bias_x_g", "bias_y_g", "bias_z_g",
    "scale_x", "scale_y", "scale_z",
    "orientations", "dof", "residual_rms_g",
    "worst_tilt_deg", "worst_scatter_mg", "plate_residual_deg", "verdict",
]


def coefficient_checksum(name, solution):
    """A stable checksum over one sensor's coefficients.

    SENSORS.md's argument for a checksum is that it should be stamped into every
    run artefact the rig produces afterwards, so "which calibration were those
    forty drones measured against" is answerable six months later. That
    ultimately wants the checksum the board's own write reported, which will
    exist when the firmware stores a record. Until then this is computed
    host-side over exactly the numbers that would be written, so the field is
    real now and its meaning does not change when the firmware catches up.

    Rounded before hashing, because a checksum that changes with the last bit of
    a float is not a checksum of the calibration.
    """
    canonical = "%s|%s|%s" % (
        name,
        ",".join("%.6f" % b for b in solution["bias"]),
        ",".join("%.6f" % s for s in solution["scale"]),
    )
    return "%08X" % (zlib.crc32(canonical.encode("ascii")) & 0xFFFFFFFF)
# def


# --- Talking to the board ---------------------------------------------------

def send_command(ser, command, timeout=1.5, attempts=3):
    """Send a command and read until the board acknowledges.

    Unlike pico_monitor's version this does not reset_input_buffer() first: the
    board is streaming continuously and always will be, so there is no "drain to
    the ack" that leaves the stream clean - there is only the next sample. It
    skips sample lines instead and returns the first reply.

    Retried because a lost write is indistinguishable from a board that does not
    answer, and every command here is idempotent.
    """
    for _ in range(attempts):
        ser.write((command + "\n").encode("ascii"))
        ser.flush()

        deadline = time.time() + timeout
        while time.time() < deadline:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if line.startswith(REPLY_PREFIX):
                return line
    return None
# def


def send_block(ser, command, terminators, timeout=3.0):
    """Send a command and collect every reply line up to `terminator`.

    The multi-line counterpart to send_command, which returns exactly one line -
    and so, used on "K?", silently eats the first line of the dump and leaves
    the rest to be mistaken for the next command's reply. Multi-line replies
    need their own path rather than a loop around the single-line one.
    """
    ser.reset_input_buffer()
    ser.write((command + "\n").encode("ascii"))
    ser.flush()
    return read_block(ser, terminators, timeout)
# def


# Every way the "K?" dump can end. A block reader given a terminator that also
# matches an *interior* line stops early and leaves the remainder in the buffer,
# where the next command mistakes it for its own reply - which is what "CAL"
# alone did here, matching the header and orphaning four lines behind it.
CAL_TERMINATORS = ("CAL end", "CAL none")


def read_block(ser, terminators, timeout=3.0):
    """Collect reply lines until one contains any of `terminators`.

    Sample lines are skipped rather than collected: the board streams
    continuously, so a reply block arrives interleaved with data rather than
    instead of it.
    """
    if isinstance(terminators, str):
        terminators = (terminators,)

    lines = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line.startswith(REPLY_PREFIX):
            continue
        lines.append(line)
        if any(t in line for t in terminators):
            return lines
    return lines
# def


def _frame_for(name, consistency):
    """The rotation carrying CENTRE's frame onto this sensor's, row-major.

    plate_consistency keys pairs in CHANNELS order and fits the rotation from
    the first named sensor to the second, so LEFT-CENTRE carries LEFT onto
    CENTRE and must be inverted here, while CENTRE-RIGHT already points the way
    the record wants. A rotation's inverse is its transpose, and getting this
    backwards produces a matrix with the right rotation *angle* and a useless
    axis - which is not visible in any single number, so it is spelled out
    rather than left to be re-derived.
    """
    if name == "CENTRE":
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    forward = consistency.get("CENTRE-%s" % name)
    if forward is not None:
        rotation = forward["rotation"]
    else:
        backward = consistency.get("%s-CENTRE" % name)
        if backward is None:
            return None
        r = backward["rotation"]
        rotation = [[r[j][i] for j in range(3)] for i in range(3)]

    return [rotation[i][j] for i in range(3) for j in range(3)]
# def


def write_calibration(port, record_path, baud=SERIAL_BAUD):
    """Stage every sensor, commit once, then read back and verify.

    Staged then committed rather than one command per sensor, because a
    per-sensor write leaves the store holding one sensor's new coefficients and
    two sensors' old ones if the cable comes out mid-sequence.

    Both the values and the CRC are checked on read-back. The CRC catches a
    transcription error the value comparison would miss, and the value
    comparison catches a CRC computed over the wrong thing.
    """
    with open(record_path) as handle:
        record = json.load(handle)

    sensors = record.get("sensors", {})
    consistency = record.get("plate_consistency", {})

    missing = [n for n in CHANNELS if n not in sensors]
    if missing:
        print("%s has no coefficients for %s." % (record_path, ", ".join(missing)))
        return 1

    for name in CHANNELS:
        problems = sanity_problems(name, {"bias": sensors[name]["bias_g"],
                                          "scale": sensors[name]["scale"]})
        for problem in problems:
            print("REFUSE: %s" % problem)
        if problems:
            return 1

    print("Writing %s" % os.path.basename(record_path))
    for name in CHANNELS:
        s = sensors[name]
        print("  %-7s bias %s mg   scale %s   %s"
              % (name,
                 " ".join("%+7.3f" % (b * 1000.0) for b in s["bias_g"]),
                 " ".join("%.5f" % v for v in s["scale"]),
                 s["checksum"]))

    with serial.Serial(port, baudrate=baud, timeout=1.0) as ser:
        time.sleep(SERIAL_SETTLE_S)
        ser.reset_input_buffer()

        identity = send_command(ser, "I") or ""
        if EXPECTED_DEVICE not in identity:
            print("\nThat is not teensy/rig.")
            return 1
        if "calstore" not in identity:
            print("\nThis firmware has no calibration store - it does not")
            print("advertise calstore. Flash the current teensy/rig first.")
            return 1
        print("\n  %s" % identity)

        before = send_block(ser, "K?", CAL_TERMINATORS, timeout=2.0)
        print("\n  before:")
        for line in before or ["    # CAL none"]:
            print("    %s" % line)

        # Stage every sensor before committing any of it.
        written_frames = {}
        for index, name in enumerate(CHANNELS):
            s = sensors[name]
            payload = ",".join(["%.6f" % b for b in s["bias_g"]] +
                               ["%.6f" % v for v in s["scale"]])
            reply = send_command(ser, "K%d:%s" % (index, payload))
            print("  stage %-7s -> %s" % (name, reply))
            if reply is None or not reply.startswith("# ACK"):
                print("\nStaging failed; nothing has been committed.")
                return 1

            frame = _frame_for(name, consistency)
            if frame is None:
                print("  frame  %-7s -> no rotation in this record; the stored"
                      % name)
                print("          one stays identity and the datum correction")
                print("          for this sensor will do nothing.")
                continue

            reply = send_command(ser, "KF%d:%s"
                                 % (index, ",".join("%.6f" % v for v in frame)))
            if reply is None or not reply.startswith("# ACK"):
                print("  frame  %-7s -> %s" % (name, reply))
                print("\nStaging failed; nothing has been committed.")
                return 1
            written_frames[name] = frame

        commit = send_command(ser, "KC", timeout=5.0)
        print("\n  commit -> %s" % commit)
        if commit is None or not commit.startswith("# ACK"):
            print("\nCommit failed. The store is unchanged.")
            return 1

        crc = commit.rsplit("crc=", 1)[-1].strip().upper()

        block = send_block(ser, "K?", CAL_TERMINATORS, timeout=3.0)
        print("\n  after:")
        for line in block:
            print("    %s" % line)

        return 0 if verify_readback(block, sensors, crc,
                                    written_frames) else 2
# def


def verify_readback(block, sensors, crc, expected_frames=None):
    """Check the board's dump against what we meant to write, and the CRC."""
    header = [l for l in block if l.startswith("# CAL ver=")]
    if not header:
        print("\nThe board did not report a stored calibration after the commit.")
        return False

    # Pull the field out and compare the values, rather than substring-matching
    # against a case-folded line: upper()ing the whole header turns "crc=" into
    # "CRC=" and the match then fails on a calibration that is perfectly good,
    # which is exactly what it did the first time this ran.
    stored = ""
    for field in header[0].split():
        if field.startswith("crc="):
            stored = field.split("=", 1)[1].strip().upper()
    if crc and stored != crc.upper():
        print("\nThe CRC the commit reported (%s) is not the one the store now"
              % crc)
        print("reports (%s). Do not trust this calibration." % (stored or "?"))
        return False

    seen, frames = {}, {}
    for line in block:
        parts = line.split()
        if len(parts) < 4 or parts[1] != "CAL":
            continue
        name = parts[2]
        if "frame=" in line:
            frames[name] = [float(v)
                            for v in parts[3].split("=", 1)[1].split(",")]
            continue
        if "bias=" not in line:
            continue
        bias = [float(v) for v in parts[3].split("=", 1)[1].split(",")]
        scale = [float(v) for v in parts[4].split("=", 1)[1].split(",")]
        seen[name] = (bias, scale)

    ok = True
    for name in CHANNELS:
        if name not in seen:
            print("\n%s is missing from the read-back." % name)
            ok = False
            continue
        bias, scale = seen[name]
        want = sensors[name]
        # float32 on the board against float64 here, so exact equality is the
        # wrong test; 1e-6 is far tighter than the 0.26 mg run-to-run spread and
        # far looser than the rounding.
        for i in range(3):
            if abs(bias[i] - want["bias_g"][i]) > 1e-6:
                print("\n%s bias axis %d read back as %.6f, wrote %.6f"
                      % (name, i, bias[i], want["bias_g"][i]))
                ok = False
            if abs(scale[i] - want["scale"][i]) > 1e-6:
                print("\n%s scale axis %d read back as %.6f, wrote %.6f"
                      % (name, i, scale[i], want["scale"][i]))
                ok = False

    # The datum rotations too. Staging a coefficient and never reading it back
    # leaves one number in the record that nothing checks, which is the same
    # class of gap the CRC exists to close for the others.
    for name in CHANNELS:
        want = expected_frames.get(name) if expected_frames else None
        if want is None:
            continue
        got = frames.get(name)
        if got is None:
            print("\n%s has no datum rotation in the read-back." % name)
            ok = False
            continue
        if any(abs(a - b) > 1e-5 for a, b in zip(got, want)):
            print("\n%s datum rotation read back as %s, wrote %s"
                  % (name, ["%.5f" % v for v in got],
                     ["%.5f" % v for v in want]))
            ok = False

    print("\n%s" % ("Read-back matches, CRC %s." % crc if ok else
                    "READ-BACK MISMATCH - do not use this calibration."))
    return ok
# def


def check_alignment(port, baud=SERIAL_BAUD, seconds=3.0):
    """Show how well the three sensors agree, raw against corrected.

    The demonstration that a written calibration did anything. Re-running the
    six-face wizard does not show it: the wizard corrects host-side already, so
    its numbers are the same either way, and the mounting rotation it reports is
    a property of the bolts.

    What changes is the stream. Uncalibrated, the three modules disagree on the
    magnitude of gravity by about a percent and carry milligravity biases;
    corrected, they should all read 1.000 g and differ only by how they are
    mounted.
    """
    def gather(ser, corrected):
        ser.reset_input_buffer()
        rows = {n: [] for n in CHANNELS}
        deadline = time.time() + seconds
        while time.time() < deadline:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            sample = (parse_corrected_line(line) if corrected
                      else parse_sample_line(line))
            if sample is None:
                continue
            for name in CHANNELS:
                if sample.get(name) is not None:
                    rows[name].append(sample[name][0:3])
        return rows

    def show(label, rows, to_g):
        print("\n  %s" % label)
        vectors = {}
        for name in CHANNELS:
            if not rows[name]:
                print("    %-7s no data" % name)
                continue
            accs = [to_g(r) for r in rows[name]]
            mean = mean_observation(accs)
            vectors[name] = mean
            print("    %-7s |a| %.5f g   n=%d" % (name, vector_norm(mean),
                                                  len(accs)))
        names = [n for n in CHANNELS if n in vectors]
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                angle = angle_between_deg(vectors[names[i]], vectors[names[j]])
                print("    %-7s to %-7s %6.3f deg apart"
                      % (names[i], names[j], angle))
        return vectors

    with serial.Serial(port, baudrate=baud, timeout=1.0) as ser:
        time.sleep(SERIAL_SETTLE_S)
        ser.reset_input_buffer()

        identity = send_command(ser, "I") or ""
        print("  %s" % identity)
        if "cal=none" in identity:
            print("\nThe board has no stored calibration, so there is nothing")
            print("to compare against. Write one with --write first.")
            return 1

        send_command(ser, "F200")

        send_command(ser, "R")
        show("raw (uncorrected):", gather(ser, False), counts_to_g)

        reply = send_command(ser, "D")
        if reply is None or not reply.startswith("# ACK"):
            print("\nThe board would not switch to corrected mode: %s" % reply)
            return 1
        vectors = show("corrected (board applying the stored calibration):",
                       gather(ser, True), lambda r: tuple(c / 1e6 for c in r))

        send_command(ser, "R")
        frames = read_frames(send_block(ser, "K?", CAL_TERMINATORS, timeout=3.0))
        show_datum(vectors, frames)

        send_command(ser, "R")
        send_command(ser, "S")

    print("\nThe angle left after the datum rotation is the measurement floor.")
    print("What the datum removes is how the modules are bolted to the plate,")
    print("which is not a calibration error and cannot be calibrated away.")
    return 0
# def


def read_frames(block):
    """The stored datum rotations, {channel: 3x3}, from a "K?" dump."""
    out = {}
    for line in block:
        parts = line.split()
        if len(parts) < 4 or parts[1] != "CAL" or "frame=" not in line:
            continue
        flat = [float(v) for v in parts[3].split("=", 1)[1].split(",")]
        if len(flat) == 9:
            out[parts[2]] = [flat[0:3], flat[3:6], flat[6:9]]
    return out
# def


def show_datum(vectors, frames):
    """How close the sensors come once CENTRE is used as the datum.

    The part the raw-against-corrected comparison cannot show, and the reason
    the record carries a rotation at all. Bias and scale make each sensor read
    the right *magnitude*; the datum rotation is what makes them agree on a
    *direction*, and it is applied to the vector rather than to an angle
    because subtracting two inclinations is only valid when both rotations
    share an axis.
    """
    if "CENTRE" not in vectors:
        print("\n  datum: CENTRE has no reading to be the datum.")
        return

    print("\n  with CENTRE as the datum (rotation from the stored record):")
    for name in CHANNELS:
        if name == "CENTRE" or name not in vectors:
            continue
        rotation = frames.get(name)
        if rotation is None:
            print("    %-7s no stored rotation" % name)
            continue
        # The record stores CENTRE -> sensor, so the inverse brings the
        # sensor's vector back into CENTRE's frame. A rotation's inverse is
        # its transpose.
        inverse = [[rotation[j][i] for j in range(3)] for i in range(3)]
        before = angle_between_deg(vectors[name], vectors["CENTRE"])
        after = angle_between_deg(apply_rotation(inverse, vectors[name]),
                                  vectors["CENTRE"])
        print("    %-7s to CENTRE  %6.3f deg  ->  %6.3f deg"
              % (name, before, after))
# def


def find_board(explicit):
    """The Teensy's port, or None with something printed about why not."""
    if explicit:
        return explicit

    ports = list_serial_ports()

    for candidate in ports:
        if candidate.is_teensy_rawhid():
            print("Found a Teensy in Raw HID mode at %s." % candidate.hwid)
            print("It has no serial port. Reflash teensy/rig, which builds with")
            print("-D USB_SERIAL; see teensy/rig/platformio.ini.")
            return None

    matches = [c for c in ports if c.is_teensy_by_id()]
    if len(matches) == 1:
        print("Board: %s" % matches[0].label())
        return matches[0].device
    if len(matches) > 1:
        print("More than one Teensy is attached; name one with --port:")
        for c in matches:
            print("  %s" % c.label())
        return None

    print("No Teensy found. Ports seen:")
    for c in ports:
        print("  %s" % c.label())
    if not ports:
        print("  (none)")
    return None
# def


def handshake(ser, rate_hz):
    """Identify the board, set the rate, and report per-channel health.

    Returns False when this is not the rig firmware. Being strict here is
    deliberate: the bring-up build answers "I" too, and its reply is close
    enough in shape to be mistaken for this one at a glance.
    """
    reply = send_command(ser, "I")
    if reply is None:
        print("The board did not answer the identity command.")
        print("The legacy Pico firmware never reads stdin and answers nothing;")
        print("if this is the Pico, it is the wrong board for this tool.")
        return False

    print("  %s" % reply)
    if EXPECTED_DEVICE not in reply:
        print("\nThat is not teensy/rig. This wizard needs all three channels")
        print("streaming at once, which only the rig firmware does.")
        return False

    # Assert raw mode rather than assume it, because the consequence of getting
    # this wrong is silent and compounding. Once the board carries a stored
    # calibration and can emit corrected values, a wizard that calibrated
    # against those would be solving for a correction on top of a correction -
    # and the result would look entirely reasonable and be wrong by the square.
    #
    # Firmware without modes answers "# ERR R", which is the correct outcome and
    # not a failure: a board that has no degrees mode cannot be in it.
    mode_reply = reply
    raw = send_command(ser, "R")
    if raw and not raw.startswith("# ERR"):
        print("  %s" % raw)
        mode_reply = send_command(ser, "I") or ""

    if "mode=" in mode_reply and "mode=raw" not in mode_reply:
        print("\nThe board is not in raw mode and would not switch.")
        print("This wizard must see uncorrected counts: calibrating against")
        print("already-corrected values solves for a correction on top of a")
        print("correction, which looks plausible and is wrong by the square.")
        return False

    ack = send_command(ser, "F%d" % rate_hz)
    print("  %s" % (ack or "# (no ack for the rate command)"))

    status = send_command(ser, "?")
    if status:
        print("  %s" % status)
        if "sensors=degraded" in status:
            print("\nNote: the board's hz: and sensors= fields are measured once")
            print("at boot and are not refreshed, so they still describe whatever")
            print("was true then - a channel that has since been reconnected")
            print("still reads hz:0. age_ms is live and is the field to trust:")
            print("a few ms means the channel is delivering right now. The")
            print("pre-flight below reads the stream itself and settles it.")

    return preflight(ser)
# def


def preflight(ser, seconds=2.0):
    """Refuse to start unless all three channels are actually delivering.

    Checked here, before the operator has picked anything up, because the
    alternative is finding out six faces in - and because a channel that dies
    mid-run is the failure this rig has actually had, twice, both times a serial
    lead working loose rather than anything to do with the sensor.

    Reading the stream rather than trusting the board's own status: the status
    reply says what the board thinks, and this says what arrived.
    """
    seen = {name: 0 for name in CHANNELS}
    total = 0
    deadline = time.time() + seconds

    while time.time() < deadline:
        line = ser.readline().decode("ascii", errors="ignore")
        sample = parse_sample_line(line)
        if sample is None:
            continue
        total += 1
        for name in CHANNELS:
            if sample.get(name) is not None:
                seen[name] += 1

    if total == 0:
        print("\nNo sample lines arrived at all in %.0f s." % seconds)
        return False

    dead = [n for n in CHANNELS if seen[n] == 0]
    partial = [n for n in CHANNELS if 0 < seen[n] < total * 0.9]

    print("  channels: " + ", ".join(
        "%s %d/%d" % (n, seen[n], total) for n in CHANNELS))

    if dead:
        print("\n%s silent over %.0f s - %d sample lines and not one carried it."
              % (", ".join(dead), seconds, total))
        print("Check the module's serial line before anything else. On this rig")
        print("a LEFT lead has worked loose twice, and it presents exactly like")
        print("this. A byte-counting probe finds it in seconds: teensy/bringup's")
        print("\"U\" reports per-channel byte counts, and \"X\" dumps the last")
        print("bytes so a header repeating every 11 is visible by eye.")
        return False

    if partial:
        print("\n%s is intermittent - present on fewer than 9 in 10 lines."
              % ", ".join(partial))
        print("That is a marginal connection, and it will corrupt a capture")
        print("silently by dropping samples from one channel and not the others.")
        return False

    return True
# def


# --- Capturing --------------------------------------------------------------

class Stream:
    """Line reader that parses samples and mirrors every raw line to disk.

    The mirror is not a debugging afterthought. A face capture is thirty seconds
    of the operator holding still, and it cannot be repeated later or from a
    different angle; the raw file is what makes --replay possible and what turns
    a surprising result into something examinable instead of something to be
    taken on trust.
    """

    def __init__(self, ser, sink=None):
        self.ser = ser
        self.sink = sink
        self.parsed = 0
        self.unparsed = 0
        self.replies = []
    # def

    def read(self):
        """The next sample, or None if the line was not one."""
        raw = self.ser.readline().decode("ascii", errors="ignore").strip()
        if not raw:
            return None
        if self.sink is not None:
            self.sink.write(raw + "\n")

        if raw.startswith(REPLY_PREFIX):
            self.replies.append(raw)
            return None

        sample = parse_sample_line(raw)
        if sample is None:
            self.unparsed += 1
            return None
        self.parsed += 1
        return sample
    # def
# class


def wait_for_face(stream, tracker):
    """Block until the plate settles on an uncaptured face. Returns its name.

    The hint printed while waiting is rewritten in place rather than scrolled,
    because the operator is looking at this while holding a plate and a
    scrolling log is unreadable from arm's length.
    """
    hint = None

    while True:
        sample = stream.read()
        if sample is None:
            continue

        face = tracker.feed(sample, time.time())
        if face is not None:
            sys.stdout.write("\r%-70s\n" % ("  settled on %s" % face))
            return face

        if tracker.state == "RELEASED":
            new_hint = "  lift the plate and turn it to the next face"
        elif tracker.moving(sample):
            new_hint = "  moving..."
        elif tracker.state == "SETTLING":
            new_hint = "  holding %s, keep still" % tracker.settling_on()
        else:
            new_hint = "  rest the plate flat on a face and let go"

        if new_hint != hint:
            hint = new_hint
            sys.stdout.write("\r%-70s" % hint)
            sys.stdout.flush()
# def


def capture_hold(stream, seconds, face, expect_face=None,
                 min_samples=MIN_CAPTURE_SAMPLES):
    """Collect quiet samples for `seconds`, discarding the disturbed ones.

    Returns (samples, reason). reason is None on success, or a string naming
    what went wrong.

    An earlier version abandoned the whole hold the moment a single sample came
    in above the motion threshold, which made a capture a test of nerve: one
    twitch at 29 seconds threw away 29 seconds. It was also unnecessary. The
    solve needs a good *mean* vector and enough samples to make it, not an
    unbroken run of them - so a disturbed sample is dropped and the hold
    continues, and the only thing that ends a capture early is the plate
    genuinely arriving somewhere else.

    That is what expect_face is for, and it is a different question from
    stillness. A plate rotated gently enough stays under the motion threshold
    the whole way and would otherwise average two orientations into one
    observation - the worst possible input to the solve, because it looks
    entirely healthy.

    The yield is returned rather than hidden: a hold that kept 40% of its
    samples is a usable observation and an honest thing to say out loud.
    """
    samples = []
    seen = 0
    started = time.time()
    last_shown = -1.0

    while True:
        elapsed = time.time() - started
        if elapsed >= seconds:
            if len(samples) < min_samples:
                return samples, ("only %d of %d samples were still enough"
                                 % (len(samples), min_samples))
            return samples, None

        sample = stream.read()
        if sample is None:
            continue
        seen += 1

        if expect_face is not None:
            where = sample_face(sample)
            if where is not None and where != expect_face:
                return samples, ("it moved from %s to %s at %.1f s"
                                 % (expect_face, where, elapsed))

        if not sample_is_quiet(sample):
            continue

        samples.append(sample)

        if elapsed - last_shown >= 0.5:
            last_shown = elapsed
            bar = int(28 * elapsed / seconds)
            kept = (100.0 * len(samples) / seen) if seen else 0.0
            sys.stdout.write("\r  holding %-6s [%s%s] %4.1f/%.0f s  "
                             "%d samples, %.0f%% steady"
                             % (face, "=" * bar, " " * (28 - bar),
                                elapsed, seconds, len(samples), kept))
            sys.stdout.flush()
# def


def report_capture(summary):
    """Print one face's per-sensor result. Returns a list of problems, [] if fine.

    Problems are returned as sentences rather than a bool because the two ways a
    face fails want completely different things from the operator, and an
    earlier version ran them together into "past 15 deg from flat, or a channel
    is silent". On the rig that message cost a run: a sensor's serial line had
    worked loose, and the wizard's advice was to hold the plate flatter. It had
    the evidence and reported the wrong half of it.
    """
    print()
    problems = []
    for name in CHANNELS:
        if name not in summary:
            print("  %-7s NO DATA - the channel is silent" % name)
            problems.append("%s sent nothing" % name)
            continue
        s = summary[name]
        worst_scatter = max(s["scatter_g"]) * 1000.0
        print("  %-7s tilt %5.1f deg   |a| %.4f g   scatter %.2f mg   "
              "wander %.2f deg   n=%d   %s"
              % (name, s["tilt_deg"], s["magnitude_g"], worst_scatter,
                 s["wander_deg"], s["n"], s["verdict"].upper()))
        if s["verdict"] == "reject":
            problems.append("%s was %.0f deg from flat, past the %.0f deg limit"
                            % (name, s["tilt_deg"], FACE_TILT_REJECT_DEG))
    return problems
# def


# A face gets this many attempts before the wizard gives up on it and says so.
# Without a cap a face the operator physically cannot present - a fixture that
# will not sit that way up, a threshold set too tight - retries silently and for
# ever, which reads as a hang rather than as the answerable problem it is.
MAX_FACE_ATTEMPTS = 5


def run_faces(stream, tracker, hold_s, min_samples=MIN_CAPTURE_SAMPLES):
    """The six-face pass. Returns {face: {channel: summary}}.

    Faces are accepted in whatever order they arrive. The suggested order is
    printed because consecutive entries in it are a single 90-degree flip rather
    than a re-orientation, which is faster and much harder to get wrong - but
    the operator will not get the absolute orientation right and does not need
    to, since every quantity this procedure recovers is a relative one.
    """
    captures = {}
    attempts = {}

    while tracker.remaining():
        remaining = tracker.remaining()
        print("\n%d of 6 captured. Still wanted: %s"
              % (len(captures), " ".join(remaining)))
        print("Suggested next: %s" % remaining[0])

        face = wait_for_face(stream, tracker)
        attempts[face] = attempts.get(face, 0) + 1

        samples, reason = capture_hold(stream, hold_s, face, expect_face=face,
                                       min_samples=min_samples)

        if reason is None:
            summary = summarise_capture(samples, face)
            problems = report_capture(summary)
            if not problems:
                captures[face] = summary
                tracker.mark_captured(face, summary)
                continue
            reason = "; ".join(problems)

        print("\n  not usable: %s" % reason)

        if "sent nothing" in reason:
            print()
            print("  That is a dead channel, not a bad hold - retrying it will")
            print("  not help. Check the module's serial line: on this rig a")
            print("  LEFT lead has worked loose twice. \"?\" on the board reports")
            print("  per-channel frame counts and the age of the last one, and a")
            print("  rising bad-checksum count means a marginal connection")
            print("  rather than an unpowered module.")
            return captures

        if attempts[face] >= MAX_FACE_ATTEMPTS:
            print("  Giving up on %s after %d attempts, and skipping it. The"
                  % (face, attempts[face]))
            print("  solve needs all six, so this run will not produce")
            print("  coefficients - but the report will say what went wrong.")
            tracker.mark_captured(face, None)
            captures.pop(face, None)
            continue

        print("  Lift the plate and present %s again (attempt %d of %d)."
              % (face, attempts[face] + 1, MAX_FACE_ATTEMPTS))
        tracker.restart()

    return captures
# def


def run_tilts(stream, count, hold_s):
    """Extra arbitrary static orientations. Returns [(label, {channel: summary})].

    The largest quality win available for the least operator effort, and the
    reason is a counting argument. Six faces against six unknowns is exactly
    determined, so its residuals are approximately zero by construction and
    reporting them as a quality score would be a fit statistic with no degrees
    of freedom. Three extra orientations make nine equations against six
    unknowns, and only then does a residual mean anything.

    Measured against synthetic data at this rig's noise level: with six faces
    the residual RMS comes out at 8e-17 g, which is the arithmetic telling you
    it has nothing to say. With three extra tilts it comes out at 5.2e-5 g,
    which tracks the injected noise - a number that would move if something were
    wrong.

    Deliberately not classified as faces: any static orientation constrains the
    solve, and awkward ones constrain it better than near-flat ones.
    """
    out = []
    while len(out) < count:
        print("\nExtra orientation %d of %d." % (len(out) + 1, count))
        print("Rest the plate at any awkward angle - a corner, an edge, propped")
        print("on something. It does not need to be a face and should not be.")

        settled = False
        quiet_since = None
        while not settled:
            sample = stream.read()
            if sample is None:
                continue
            if sample_is_quiet(sample):
                if quiet_since is None:
                    quiet_since = time.time()
                elif time.time() - quiet_since >= 1.5:
                    settled = True
            else:
                quiet_since = None

        label = "tilt%d" % (len(out) + 1)
        samples, reason = capture_hold(stream, hold_s, label)
        if reason is not None:
            print("\n  abandoned: %s - trying again" % reason)
            continue

        summary = summarise_capture(samples, None)
        print()
        for name in CHANNELS:
            if name in summary:
                s = summary[name]
                print("  %-7s |a| %.4f g   scatter %.1f mg   n=%d"
                      % (name, s["magnitude_g"], max(s["scatter_g"]) * 1000.0,
                         s["n"]))
        out.append((label, summary))
    return out
# def


# --- Solving and reporting --------------------------------------------------

def solve_all(captures, tilts):
    """Solve every channel that has a full set of orientations."""
    solutions = {}

    for name in CHANNELS:
        observations = []
        for face in SUGGESTED_FACE_ORDER:
            summary = captures.get(face)
            if summary and name in summary:
                observations.append((face, summary[name]["mean_g"]))
        for label, summary in tilts:
            if name in summary:
                observations.append((label, summary[name]["mean_g"]))

        if len(observations) < 6:
            continue
        solution = solve_sensor(observations)
        if solution is not None:
            solutions[name] = solution

    return solutions
# def


def all_orientations(captures, tilts):
    """Faces and extra tilts as one {label: {channel: summary}}.

    The rigid fit wants every orientation there is - nine constrain it better
    than six, and an awkward tilt constrains it better than a face.
    """
    merged = dict(captures)
    for label, summary in tilts:
        merged[label] = summary
    return merged
# def


def print_report(captures, solutions, tilts=()):
    """The whole verdict. Returns True if every sensor passed every gate."""
    print("\n" + "=" * 72)
    print("Calibration")
    print("=" * 72)

    ok = True

    for name in CHANNELS:
        solution = solutions.get(name)
        print("\n%s" % name)
        if solution is None:
            print("  no solution - the captured faces do not span all three axes")
            ok = False
            continue

        for i, axis in enumerate(("X", "Y", "Z")):
            print("  %s   bias %+8.2f mg   scale %.5f"
                  % (axis, solution["bias"][i] * 1000.0, solution["scale"][i]))

        print("  orientations %d, dof %d, residual rms %.2e g"
              % (solution["n_orientations"], solution["dof"],
                 solution["residual_rms_g"]))
        if solution["dof"] <= 0:
            print("  (dof 0: the residual is zero by construction and means"
                  " nothing. Re-run with --tilts to get a real one.)")

        problems = sanity_problems(name, solution)
        for problem in problems:
            print("  REFUSE: %s" % problem)
        if problems:
            print("  An axis is mislabelled or a face was badly wrong.")
            ok = False

        print("  checksum %s" % coefficient_checksum(name, solution))

    # Worst tilt and scatter across faces, which are the quality measures that
    # the residual cannot be.
    print("\nHold quality, worst across the six faces")
    print("  Resting on a desk this rig measures 0.07-0.08 deg of wander;")
    print("  hand-held, a few degrees costs well under a thousandth of a deg.")
    for name in CHANNELS:
        tilts_seen = [s[name]["tilt_deg"] for s in captures.values()
                      if name in s and s[name]["tilt_deg"] is not None]
        scatter = [max(s[name]["scatter_g"]) for s in captures.values()
                   if name in s]
        wander = [s[name]["wander_deg"] for s in captures.values()
                  if name in s]
        if tilts_seen:
            print("  %-7s tilt %5.1f deg   scatter %5.2f mg   wander %5.2f deg"
                  % (name, max(tilts_seen), max(scatter) * 1000.0, max(wander)))

    print("\nPlate consistency")
    print("  One rotation should carry each sensor's vectors onto the next's in")
    print("  every orientation. What is left over is whether they stayed put.")
    consistency = plate_consistency(all_orientations(captures, tilts), solutions)
    if not consistency:
        print("  not computable - fewer than two sensors solved")
        ok = False
    for pair, result in sorted(consistency.items()):
        print("  %-15s mount %5.2f deg   residual rms %5.3f  max %5.3f (%s)  "
              "n=%d  %s"
              % (pair, result["mount_deg"], result["residual_rms_deg"],
                 result["residual_max_deg"], result["worst_orientation"],
                 result["n_orientations"], result["verdict"].upper()))
        if result["verdict"] == "reject":
            ok = False
    if any(r["verdict"] != "ok" for r in consistency.values()):
        print("  The plate flexed, a module shifted, or one solve is wrong.")
        print("  Which pairs fail names the culprit: a sensor that appears in")
        print("  both bad pairs is the sensor.")
    if consistency:
        print("  (mount is how far apart the modules are bolted, not a fault -")
        print("   and it is the rotation a CENTRE datum correction needs.)")

    print("\n%s" % ("PASS - the coefficients above are usable." if ok else
                    "FAIL - do not write these. See the reasons above."))
    return ok
# def


def _migrate_log(log_path):
    """Bring an older log up to the current header, keeping a .bak.

    The plate column changed meaning when the check became a rigid-body fit:
    it held the worst inter-face disagreement and now holds the fit residual.
    Those are different quantities, so the old values are carried across as
    blank rather than relabelled - a number under a heading that no longer
    describes it is worse than an absent one, and this log exists to be read
    months later by someone deciding whether a calibration was any good.

    Backing up rather than rewriting in place follows the path the elevon log
    migration already set, and `*.csv.bak` is already gitignored for it.
    """
    if not os.path.exists(log_path):
        return

    with open(log_path, newline="") as handle:
        rows = list(csv.reader(handle))
    if not rows or rows[0] == LOG_COLUMNS:
        return

    old_columns = rows[0]
    if len(old_columns) != len(LOG_COLUMNS):
        print("Cannot migrate %s: unexpected header. Move it aside and re-run."
              % log_path)
        raise SystemExit(1)

    changed = [i for i, (a, b) in enumerate(zip(old_columns, LOG_COLUMNS))
               if a != b]

    backup = log_path + ".bak"
    os.replace(log_path, backup)
    with open(log_path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(LOG_COLUMNS)
        for row in rows[1:]:
            row = list(row)
            for i in changed:
                row[i] = ""
            writer.writerow(row)

    print("Migrated %s (%s -> %s); previous file kept as %s"
          % (os.path.basename(log_path),
             ", ".join(old_columns[i] for i in changed),
             ", ".join(LOG_COLUMNS[i] for i in changed),
             os.path.basename(backup)))
# def


def _row_verdict(name, solution, consistency):
    """The log's verdict for one sensor: every gate, not just the sanity ones.

    It read "pass" on nothing but sanity_problems at first, which meant a run
    whose plate check rejected outright - a module loose enough to disagree by
    4 degrees - still logged three rows saying "pass". The log is the record
    someone reaches for months later to decide whether a calibration was good,
    and a verdict that ignores the check most likely to catch a bad one is worse
    than no verdict at all.
    """
    if sanity_problems(name, solution):
        return "refuse"

    verdicts = [r["verdict"] for pair, r in consistency.items() if name in pair]
    if "reject" in verdicts:
        return "plate-reject"
    if "warn" in verdicts:
        return "plate-warn"
    if not verdicts:
        return "unchecked"
    return "pass"
# def


def write_artefacts(stamp, captures, tilts, solutions, raw_path):
    """The JSON record, and one row per sensor in the tracked top-level log."""
    consistency = plate_consistency(all_orientations(captures, tilts),
                                    solutions)

    record = {
        "timestamp": stamp,
        "raw_capture": os.path.basename(raw_path) if raw_path else None,
        "faces": {
            face: {name: {k: v for k, v in s.items()}
                   for name, s in summary.items()}
            for face, summary in captures.items()
        },
        "tilts": {
            label: {name: {k: v for k, v in s.items()}
                    for name, s in summary.items()}
            for label, summary in tilts
        },
        "sensors": {
            name: {
                "bias_g": solution["bias"],
                "scale": solution["scale"],
                "orientations": solution["n_orientations"],
                "dof": solution["dof"],
                "residual_rms_g": solution["residual_rms_g"],
                "checksum": coefficient_checksum(name, solution),
                "sanity": sanity_problems(name, solution),
            }
            for name, solution in solutions.items()
        },
        "plate_consistency": consistency,
    }

    json_path = report_path(APP_DIR, "sensor_cal_%s.json" % stamp)
    with open(json_path, "w") as handle:
        json.dump(record, handle, indent=2, sort_keys=True, default=list)
    print("\nWrote %s" % json_path)

    log_path = os.path.join(APP_DIR, SENSOR_CAL_LOG)
    _migrate_log(log_path)
    fresh = not os.path.exists(log_path)
    with open(log_path, "a", newline="") as handle:
        writer = csv.writer(handle)
        if fresh:
            writer.writerow(LOG_COLUMNS)
        for name, solution in sorted(solutions.items()):
            face_tilts = [s[name]["tilt_deg"] for s in captures.values()
                          if name in s and s[name]["tilt_deg"] is not None]
            scatter = [max(s[name]["scatter_g"]) for s in captures.values()
                       if name in s]
            spreads = [r["residual_rms_deg"] for pair, r in consistency.items()
                       if name in pair]
            writer.writerow([
                stamp, name, coefficient_checksum(name, solution),
                "%.6f" % solution["bias"][0],
                "%.6f" % solution["bias"][1],
                "%.6f" % solution["bias"][2],
                "%.6f" % solution["scale"][0],
                "%.6f" % solution["scale"][1],
                "%.6f" % solution["scale"][2],
                solution["n_orientations"], solution["dof"],
                "%.3e" % solution["residual_rms_g"],
                "%.2f" % max(face_tilts) if face_tilts else "",
                "%.2f" % (max(scatter) * 1000.0) if scatter else "",
                "%.2f" % max(spreads) if spreads else "",
                _row_verdict(name, solution, consistency),
            ])
    print("Wrote %s" % log_path)
# def


# --- Replay -----------------------------------------------------------------

def static_segments(path, hold_s):
    """Every static stretch of at least hold_s in a saved raw capture.

    Yields (samples, seconds). Segmenting on stillness alone, rather than
    replaying the FaceTracker, is what lets a replay recover the extra tilted
    orientations as well as the six faces: the tracker only ever fires on a
    classifiable face, so driving the replay through it would silently discard
    exactly the orientations that give the residual its meaning.

    Time comes from the board's own t_us rather than from arrival times, which
    is the whole reason the firmware puts it on the wire. It wraps at 2^32 us,
    about 71.6 minutes, so the difference is taken modulo that.
    """
    current = []
    base = None
    started = None
    last = None

    def close():
        if current and last is not None and started is not None:
            span = last - started
            if span >= hold_s * 0.9:
                return list(current), span
        return None

    with open(path) as handle:
        for line in handle:
            sample = parse_sample_line(line)
            if sample is None:
                continue
            if base is None:
                base = sample["t_us"]
            now = ((sample["t_us"] - base) % (1 << 32)) / 1e6

            if sample_is_quiet(sample):
                if not current:
                    started = now
                current.append(sample)
                last = now
                continue

            done = close()
            if done:
                yield done
            current = []
            started = None

    done = close()
    if done:
        yield done
# def


def replay(path, hold_s):
    """Re-solve from a saved raw capture, with no board attached.

    Refining the method should not require the rig, and a capture that produced
    a surprising answer is the thing you want to still have next week.

    A segment is a face if all live channels agree on one and it has not been
    seen already; anything else static for long enough is an extra orientation,
    which is what it would have been on the day.
    """
    captures = {}
    tilts = []

    for samples, span in static_segments(path, hold_s):
        face = sample_face(samples[len(samples) // 2])
        if face is not None and face not in captures:
            captures[face] = summarise_capture(samples, face)
        else:
            label = "tilt%d" % (len(tilts) + 1)
            tilts.append((label, summarise_capture(samples, None)))
        print("  %-7s %5.1f s  %d samples"
              % (face or "tilt", span, len(samples)))

    print("\nReplayed %s: %d faces (%s), %d extra orientations"
          % (path, len(captures), " ".join(sorted(captures)) or "-", len(tilts)))

    if len(captures) < 6:
        print("Fewer than six faces in this capture; the solve needs all six.")

    solutions = solve_all(captures, tilts)
    print_report(captures, solutions, tilts)
    return 0 if len(captures) == 6 else 2
# def


# --- Entry point ------------------------------------------------------------

def explain():
    print("""
Six-face sensor calibration
---------------------------
You will rest the plate on each of its six faces in turn. The wizard notices
each face on its own - put the plate down, let go, and it starts the capture
when the plate has been still for a moment. It stops if you touch it mid-hold
and asks for that face again.

Both faces of each axis are needed, and it is worth knowing why, because it is
not obvious: bias error dominates near level - 5 mg is about 0.29 degrees at
zero - while scale error dominates at the extremes - 1 percent is about 0.35
degrees at 35. Only both faces of an axis separate the two.

You do not need to get the orientation right. Faces are accepted in whatever
order they arrive, and a face a few degrees off flat costs nothing: the solve
uses the vector you actually measured, not the one you meant to. It only
refuses past %.0f degrees, where which face you are on becomes ambiguous.

Ctrl-C stops at any point.
""" % FACE_TILT_REJECT_DEG)
# def


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Six-face calibration for the rig's inclinometer modules.")
    parser.add_argument("--port", help="serial port; found automatically if omitted")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD,
                        help="ignored by USB CDC; kept for habit")
    parser.add_argument("--seconds", type=float, default=DEFAULT_HOLD_S,
                        help="hold duration per orientation (default %.0f; "
                             "longer buys almost nothing and is harder to hold)"
                             % DEFAULT_HOLD_S)
    parser.add_argument("--tilts", type=int, default=DEFAULT_TILTS,
                        help="extra arbitrary orientations after the six faces "
                             "(default %d; these are what give the residual any "
                             "meaning)" % DEFAULT_TILTS)
    parser.add_argument("--rate", type=int, default=DEFAULT_RATE_HZ,
                        help="board output rate in Hz (default %d)"
                             % DEFAULT_RATE_HZ)
    parser.add_argument("--replay", metavar="FILE",
                        help="re-solve a saved raw capture instead of running")
    parser.add_argument("--write", metavar="JSON",
                        help="write a completed run's coefficients to the board "
                             "and verify the read-back, instead of running")
    parser.add_argument("--check", action="store_true",
                        help="show the live corrected stream and how well the "
                             "three sensors agree, instead of running")
    args = parser.parse_args(argv)

    if args.replay:
        return replay(args.replay, args.seconds)

    if args.write:
        port = find_board(args.port)
        if port is None:
            return 1
        return write_calibration(port, args.write, baud=args.baud)

    if args.check:
        port = find_board(args.port)
        if port is None:
            return 1
        return check_alignment(port, baud=args.baud)

    port = find_board(args.port)
    if port is None:
        return 1

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = report_path(APP_DIR, "sensor_cal_%s_raw.txt" % stamp)

    try:
        with serial.Serial(port, baudrate=args.baud, timeout=1.0) as ser:
            time.sleep(SERIAL_SETTLE_S)
            ser.reset_input_buffer()

            if not handshake(ser, args.rate):
                return 1

            # Halt across the explanation. The board streams continuously by
            # design, so anything printed to the operator is time the OS buffer
            # spends filling with samples of a plate sitting on a bench - and
            # the first thing the tracker would then see is several seconds of
            # stale readings that predate the run. Halting is cheaper and more
            # honest than draining a buffer of unknown depth afterwards.
            send_command(ser, "H")

            explain()
            input("Press Return when the plate is in your hands and ready. ")

            send_command(ser, "F%d" % args.rate)
            ser.reset_input_buffer()

            with open(raw_path, "w") as sink:
                stream = Stream(ser, sink)
                tracker = FaceTracker()

                captures = run_faces(stream, tracker, args.seconds)
                tilts = run_tilts(stream, args.tilts, args.seconds) \
                    if args.tilts > 0 else []

            # Leave the board somewhere harmless. It streams continuously by
            # design, and a firehose left running is unkind to whatever opens
            # the port next.
            send_command(ser, "S")

            print("\nRaw capture: %s" % raw_path)
            print("Parsed %d samples, %d unparsed lines"
                  % (stream.parsed, stream.unparsed))

            solutions = solve_all(captures, tilts)
            ok = print_report(captures, solutions, tilts)
            write_artefacts(stamp, captures, tilts, solutions, raw_path)
            return 0 if ok else 2

    except serial.SerialException as exc:
        print(describe_serial_error(port, exc))
        return 1
    except KeyboardInterrupt:
        print("\nStopped. The raw capture so far is at %s" % raw_path)
        return 130
# def


if __name__ == "__main__":
    sys.exit(main())
