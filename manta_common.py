"""Shared pieces of the Manta calibration rig tooling.

Imported by both MantaTrimmer.py (the GUI) and pico_monitor.py (the CLI
diagnostic), so the wire format, the USB IDs and the angle maths exist in one
place. Changing the Pico's output format should mean editing POSITION_REGEX
once, not hunting for copies.

Deliberately depends on nothing beyond pyserial and the standard library - no
tkinter, no pymavlink - so a headless box can import it. MAVLink probing lives
in MantaTrimmer.py for that reason.
"""

import errno
import os
import re
import sys
import time

import serial
import serial.tools.list_ports


APP_DIR = os.path.dirname(os.path.abspath(__file__))

REPORTS_DIRNAME = "reports"


def report_path(base_dir, filename):
    """Path for a per-run artefact, creating base_dir/reports/ if needed.

    Every CSV a run produces goes here so the repo root stays source-only, and
    so .gitignore needs one entry rather than a glob per output kind.
    calibration_log.csv is deliberately not one of these - it is the tracked,
    append-only record and stays at the top level.

    base_dir is a parameter rather than APP_DIR closed over here so callers
    resolve their own module-level APP_DIR at call time, which is what lets the
    tests redirect the output.
    """
    directory = os.path.join(base_dir, REPORTS_DIRNAME)
    os.makedirs(directory, exist_ok=True)
    return os.path.join(directory, filename)


IS_LINUX = sys.platform.startswith("linux")
IS_WINDOWS = sys.platform.startswith("win")

SERIAL_BAUD = 115200

# The rig Pico enumerates as a Raspberry Pi USB CDC device. This is the
# authoritative way to spot it; probe_pico() is only a fallback.
PICO_VIDS = {0x2E8A}
PICO_VID_PIDS = {
    (0x2E8A, 0x0005),  # RP2040 CDC, MicroPython
    (0x2E8A, 0x000A),  # RP2040 CDC, CircuitPython
}

# Only a hint used to order the MAVLink probe. The heartbeat is what decides.
FCU_VID_HINTS = {
    0x3185,  # ARK Electronics
    0x26AC,  # 3DR / PX4
    0x1209,  # pid.codes (generic PX4 boards)
    0x0483,  # STMicroelectronics
}

# One line per sample from the Pico, in one of two formats:
#
#   slow mode   "[<left_u16>/<right_u16>]"
#   fast mode   "[<ticks_us>:<left_u16>/<right_u16>]"
#
# The timestamp group is optional so both parse through one path: the legacy
# firmware (pico/main.py) and the dual-rate firmware's slow preset emit the
# first, fast mode emits the second. Consumers that do not care about timing can
# keep reading position1/position2 and ignore t_us entirely.
POSITION_REGEX = re.compile(
    r"\[(?:(?P<t_us>\d+):)?(?P<position1>-?\d+)\s*/\s*(?P<position2>-?\d+)\]"
)

# The dual-rate firmware (pico/sampler.py) prefixes every reply with this. It
# cannot match POSITION_REGEX, so replies are inert to every sample parser.
REPLY_PREFIX = "#"

# MicroPython's ticks_us() counter wraps at 2^30 us (~17.9 minutes). Deltas must
# be taken modulo this or a wrap reads as a colossal backwards jump.
PICO_TICKS_MODULO = 1 << 30

# Sample-rate presets understood by pico/sampler.py, mirrored here because the
# GUI, the console test and the monitor all need them and none of them should be
# the authority. SLOW_RATE_HZ is the board's boot default and the legacy format;
# it is fixed by compatibility, not by choice.
SLOW_RATE_HZ = 10

# The fast preset. Measured on this rig 2026-08-14 under MicroPython 1.19.1, the
# board free-ran at about 1520 Hz and saturated above it. Re-measured 2026-09-05
# after the reflash to 1.29.0, it meets every rate up to 2000 Hz (1996.01 Hz
# sustained) with no saturation, so the ceiling is now above the requestable
# range and unmeasured. 1000 Hz is kept regardless: it holds exactly (1000.00 Hz
# over 20 s) and keeps captures comparable with the baselines already recorded.
# See pico/sampler.py for the full measurement.
FAST_RATE_HZ = 1000

# Hard-over cycles per phase in the range/rate test. 30 is enough for the sample
# stdev of travel, transit and rate to mean something; 3 was not.
DEFAULT_CYCLES = 30

# Which phases the range/rate test runs unless told otherwise. Driving the servos
# one at a time was measured to give the same travel and rate as driving them in
# unison, so LEFT and RIGHT are diagnostic options now rather than the default -
# which is what makes 30 cycles affordable in wall-clock terms.
DEFAULT_PHASES = ("BOTH",)


def position_to_degrees(side, raw_position, scaler, offset):
    """Convert raw ADC counts to degrees.

    The two sides carry opposite sign conventions because the pots are mounted
    mirrored: LEFT is (scaler * raw) + offset, RIGHT is -(scaler * raw) + offset.
    """
    if side == "LEFT":
        return (scaler * raw_position) + offset
    if side == "RIGHT":
        return -(scaler * raw_position) + offset
    return None
# def


class PortCandidate:
    def __init__(self, info):
        self.device = info.device
        self.description = info.description or ""
        self.hwid = info.hwid or ""
        self.vid = info.vid
        self.pid = info.pid
        self.manufacturer = info.manufacturer or ""
        self.product = info.product or ""
        self.serial_number = info.serial_number or ""
    # def

    def label(self):
        detail = ("%s %s" % (self.manufacturer, self.product)).strip()
        if not detail:
            detail = self.description.strip()
        if not detail or detail == "n/a":
            return self.device
        return "%s  -  %s" % (self.device, detail)
    # def

    def is_pico_by_id(self):
        if self.vid is None:
            return False
        if (self.vid, self.pid) in PICO_VID_PIDS:
            return True
        return self.vid in PICO_VIDS
    # def

    def is_fcu_by_id(self):
        return self.vid is not None and self.vid in FCU_VID_HINTS
    # def

    def is_legacy(self):
        # Motherboard UARTs: no USB VID and nothing useful in hwid.
        if self.vid is not None:
            return False
        return self.device.startswith("/dev/ttyS") or self.hwid in ("", "n/a")
    # def
# class


def list_serial_ports(include_legacy=False):
    candidates = []

    try:
        infos = serial.tools.list_ports.comports()
    except Exception as e:
        print("Failed to enumerate serial ports: %s" % str(e))
        return candidates

    for info in infos:
        candidate = PortCandidate(info)
        if candidate.is_legacy() and not include_legacy:
            continue
        candidates.append(candidate)

    # USB devices first, then stable ordering by device name.
    candidates.sort(key=lambda c: (c.vid is None, c.device))
    return candidates
# def


def find_candidate(ports, device):
    if not device:
        return None
    for candidate in ports:
        if candidate.device == device:
            return candidate
    return None
# def


def describe_serial_error(device, exc):
    text = str(exc)
    lowered = text.lower()

    errno_value = getattr(exc, "errno", None)

    is_permission = (
        isinstance(exc, PermissionError)
        or errno_value == errno.EACCES
        or "permission denied" in lowered
        or "access is denied" in lowered
    )
    is_busy = (
        errno_value == errno.EBUSY
        or "resource busy" in lowered
        or "device or resource busy" in lowered
        or "in use" in lowered
        # pyserial's wording when another process already holds the port.
        or "multiple access on port" in lowered
    )

    if is_permission:
        if IS_LINUX:
            return (
                "Permission denied opening %s. Add yourself to the dialout group:\n"
                "    sudo usermod -aG dialout $USER\n"
                "then log out and back in." % device
            )
        return "Permission denied opening %s. Close any program already using it." % device

    if is_busy:
        if IS_LINUX:
            return (
                "%s is busy. Another program (QGroundControl, a second copy of this tool, "
                "or a serial monitor) may have it open. Note ModemManager also grabs "
                "/dev/ttyACM* for a few seconds after plug-in, so a retry often works." % device
            )
        return "%s is in use by another application." % device

    return "Error opening %s: %s" % (device, text)
# def


def probe_pico(device, timeout=2.0):
    """Fallback identification: open the port and look for the Pico's line format."""
    try:
        with serial.Serial(device, baudrate=SERIAL_BAUD, timeout=0.3) as ser:
            deadline = time.time() + timeout
            while time.time() < deadline:
                line = ser.readline()
                if not line:
                    continue
                text = line.decode("ascii", errors="ignore").strip()
                if text and POSITION_REGEX.search(text):
                    return True
    except (serial.SerialException, OSError) as e:
        print(describe_serial_error(device, e))

    return False
# def


def find_pico_port(ports=None):
    """Pico by USB ID, else by sniffing each port for the [n/n] format."""
    ports = list_serial_ports() if ports is None else ports

    for candidate in ports:
        if candidate.is_pico_by_id():
            return candidate.device

    for candidate in ports:
        if probe_pico(candidate.device):
            return candidate.device

    return None
# def
