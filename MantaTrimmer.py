try:
    import tkinter as tk
    from tkinter import messagebox, ttk
except ImportError:
    raise SystemExit(
        "tkinter is not available.\n"
        "  Debian/Ubuntu: sudo apt install python3-tk\n"
        "  Fedora:        sudo dnf install python3-tkinter\n"
        "  Windows/macOS: reinstall Python with the Tcl/Tk option enabled"
    )

import threading
import time
import re
import struct
import builtins
import queue
import sys
import errno
from collections import deque
from pymavlink import mavutil
import json
import os
import serial
import serial.tools.list_ports
import csv
from datetime import datetime

from manta_theme import PALETTE, apply_theme

from manta_common import (
    APP_DIR,
    DEFAULT_CYCLES,
    DEFAULT_PHASES,
    FAST_RATE_HZ,
    IS_LINUX,
    IS_WINDOWS,
    SERIAL_BAUD,
    SLOW_RATE_HZ,
    PICO_VIDS,
    PICO_VID_PIDS,
    FCU_VID_HINTS,
    POSITION_REGEX,
    REPLY_PREFIX,
    PortCandidate,
    describe_serial_error,
    find_candidate,
    list_serial_ports,
    position_to_degrees,
    probe_pico,
    report_path,
)

# The analysis is shared with the console tool rather than reimplemented here:
# it is already tested against synthetic servo traces with known answers, and a
# second copy would be a second thing to get wrong. range_test imports
# DroneInterface lazily inside main(), so this is not a cycle.
from pico_monitor import TickTracker
import curve_plot
from endpoint_logic import (
    BACKOFF_CEILING_US,
    BACKOFF_STEP_US,
    COARSE_MAX,
    COARSE_MIN,
    ENDPOINT_DWELL_S,
    ENDPOINT_TOLERANCE_DEG,
    MAX_ATTEMPTS,
    MOVEMENT_THRESHOLD_DEG,
    alternating_order,
    angle_gain_per_us,
    nominal_gain_deg_per_us,
    overshot_target,
    clamp_endpoint,
    command_delta_for_pwm,
    correction_us,
    endpoint_accepted,
    endpoint_command,
    hard_stop_verdict,
    inward_sign,
)
from range_test import (
    MIN_TRAVEL_DEG,
    curve_series,
    PHASES,
    PHASE_SIDES,
    PRE_ROLL_S,
    analyse_leg,
    creep_commands,
    creep_grid,
    endpoint_stats,
    mean_sd,
    settled_angle,
    stiction_stats,
    to_series,
)


# Bounds on the instrumentation pane. Overflow only drops lines from the GUI
# widget - _ORIGINAL_PRINT still puts everything on stdout.
# Position samples are averaged over a trailing time window rather than being
# consumed. The window must be at least 2x the Pico's 10 Hz sample period so a
# dropped serial line still leaves samples to average, and shorter than
# PositionReader.is_streaming()'s max_age so "link dead" and "window empty"
# cannot disagree. 0.5 s gives ~5 samples (about 2.2x noise reduction) for 250 ms
# of group delay, which is well inside the calibration mover's 250 ms step cadence.
POSITION_WINDOW_S = 0.5

# The stiction test's creep, matching the trim calibration's own step logic so
# the two arrive at an end stop the same way. Repetitions default low because a
# single creep from centre is 100 steps - a couple of minutes of wall clock per
# end before the swings are even started.
CREEP_STEP_CMD = 0.01
CREEP_PERIOD_S = 0.25
DEFAULT_STICTION_REPS = 3

# Curve samples per creep, including the arrival at the end stop.
DEFAULT_CREEP_POINTS = 21

# How long a curve sample dwells before it is read. It MUST exceed
# POSITION_WINDOW_S, or the trailing mean still contains samples from before the
# step and every reading lags the surface. That lag is not harmless noise: it
# biases an upward sweep low and a downward sweep high, so the two errors add
# into the apparent hysteresis band. At 0.01 cmd per 0.25 s the sweep covers
# 0.7-1.5 deg/s, and a 0.25 s effective lag would inflate the band by 0.35-0.75
# deg - the same order as the effect being measured, and systematic, so it would
# look like a clean result rather than an error.
CREEP_SAMPLE_DWELL_S = POSITION_WINDOW_S + 0.3

# MAV_CMD_ACTUATOR_TEST asks for a 60 s timeout and does not get it: measured on
# this board the FC drops the override after about 2 s and drives the surfaces
# itself again. Anything that waits longer than this between commands has to
# re-send, or the surface wanders off mid-measurement and is yanked back by the
# next command.
MEASURE_REFRESH_S = 0.5

# Named once so the writer and any reader cannot drift apart - the mismatch that
# ISSUES.md #4 records against calibration_log.csv.
CREEP_CURVE_COLUMNS = ("phase", "side", "direction", "rep", "cmd", "pwm_us",
                       "angle_deg")

# Backlog is sized in *seconds*, not samples, because the Pico's rate is now
# negotiable. A fixed 200 entries meant 20 s at 10 Hz but only 0.2 s at 1000 Hz -
# shorter than the averaging window it has to feed, which would have made
# POSITION_WINDOW_S quietly stop meaning anything. The floor keeps the 10 Hz
# case byte-for-byte as it was.
POSITION_HISTORY_S = 4.0
MIN_POSITION_HISTORY = 200

# SLOW_RATE_HZ / FAST_RATE_HZ are the pico/sampler.py presets and now live in
# manta_common, so the GUI, the console test and the monitor cannot disagree
# about them. They are imported above and re-exported here by that import.

# A capture left running by a crashed worker must not grow without bound.
# 60000 samples is a minute at 1000 Hz - far longer than any single leg.
MAX_CAPTURE_SAMPLES = 60000

# Samples kept across a whole run so "Save samples" can be pressed *after* the
# fact - the tab cannot know in advance whether it will be wanted, so it always
# retains. The default run is 30 cycles x 2 legs x 2 sides x ~2400 samples, i.e.
# ~290k rows, so this ceiling is ~4x a normal run and exists only to stop a very
# long or very fast one from eating the machine. Passing it drops later samples
# and says so rather than silently keeping a partial record.
MAX_EXPORT_SAMPLES = 1200000

# "# ACK F 1000" / "# ACK S 10"
ACK_RATE_REGEX = re.compile(r"ACK\s+[SF]\s+(\d+)")


def position_history_for(hz):
    """Backlog depth for a given sample rate. Never shorter than the old default."""
    return max(MIN_POSITION_HISTORY, int(POSITION_HISTORY_S * float(hz)))
# def

MAX_LOG_QUEUE = 2000
MAX_LOG_LINES = 2000

# Column order of calibration_log.csv. The existing file on disk ends in
# "Folding?", which was hand-added; the writer must match it exactly.
CAL_LOG_COLUMNS = [
    "date",
    "time",
    "drone_name",
    "uid",
    "angle_neg_degs",
    "angle_pos_degs",
    "angle_trim_degs",
    "left_min",
    "left_max",
    "left_trim",
    "right_min",
    "right_max",
    "right_trim",
    "Folding?",
]


class InstrumentationLog:
    def __init__(self):
        self._queue = queue.Queue(maxsize=MAX_LOG_QUEUE)
    # def

    def write(self, text):
        try:
            self._queue.put_nowait(str(text))
        except Exception:
            pass
    # def

    def drain(self):
        items = []
        while True:
            try:
                items.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return items
    # def
# class


INSTRUMENTATION_LOG = InstrumentationLog()
_ORIGINAL_PRINT = builtins.print


def print(*args, **kwargs):
    sep = kwargs.get("sep", " ")
    end = kwargs.get("end", "\n")
    text = sep.join(str(arg) for arg in args) + end

    _ORIGINAL_PRINT(*args, **kwargs)
    INSTRUMENTATION_LOG.write(text)
# def


def probe_mavlink(device, baud=SERIAL_BAUD, timeout=3.0):
    """Return (system, component) if a MAVLink heartbeat arrives, else None."""
    master = None
    try:
        master = mavutil.mavlink_connection(device, baud)
        heartbeat = master.wait_heartbeat(timeout=timeout)
        if heartbeat is None:
            return None
        return (master.target_system, master.target_component)

    except (serial.SerialException, OSError) as e:
        print(describe_serial_error(device, e))
        return None

    except Exception as e:
        print("MAVLink probe of %s failed: %s" % (device, str(e)))
        return None

    finally:
        # Must always close, or the real connect afterwards hits "device busy".
        if master is not None:
            try:
                master.close()
            except Exception:
                pass
# def


def discover_ports(prefer_pico=None, prefer_drone=None):
    """Identify the Pico and the flight controller. Slow (probes) - never call on the Tk thread."""
    ports = list_serial_ports()
    messages = []

    pico_device = None

    preferred = find_candidate(ports, prefer_pico)
    if preferred is not None:
        pico_device = preferred.device
        messages.append("Pico: reusing remembered port %s" % pico_device)

    if pico_device is None:
        for candidate in ports:
            if candidate.is_pico_by_id():
                pico_device = candidate.device
                messages.append("Pico: identified %s by USB ID (%04X:%04X)" %
                                (candidate.device, candidate.vid, candidate.pid or 0))
                break

    if pico_device is None:
        for candidate in ports:
            if probe_pico(candidate.device):
                pico_device = candidate.device
                messages.append("Pico: identified %s by data format" % candidate.device)
                break

    if pico_device is None:
        messages.append("Pico: not found")

    # Drone: heartbeat is authoritative. Never probe the port already claimed
    # by the Pico - it wastes the timeout and disturbs the MicroPython REPL.
    drone_device = None
    drone_ids = None

    ordered = [c for c in ports if c.device != pico_device]
    ordered.sort(key=lambda c: (c.device != prefer_drone, not c.is_fcu_by_id(), c.device))

    for candidate in ordered:
        ids = probe_mavlink(candidate.device)
        if ids is not None:
            drone_device = candidate.device
            drone_ids = ids
            messages.append("Drone: MAVLink heartbeat on %s (system %d component %d)" %
                            (candidate.device, ids[0], ids[1]))
            break

    if drone_device is None:
        messages.append("Drone: no MAVLink heartbeat found")

    return {
        "ports": ports,
        "pico": pico_device,
        "drone": drone_device,
        "drone_ids": drone_ids,
        "messages": messages,
    }
# def


class DroneInterface:

    MAV_CMD_ACTUATOR_TEST = 310

    MAV_RESULT_ACCEPTED = 0
    ACK_NOT_RECEIVED = -1

    MAV_RESULT_NAMES = {
        0: "ACCEPTED",
        1: "TEMPORARILY_REJECTED",
        2: "DENIED",
        3: "UNSUPPORTED",
        4: "FAILED",
        5: "IN_PROGRESS",
        6: "CANCELLED",
    }

    def __init__(self, port=None):
        self.port = port
        self.baud = SERIAL_BAUD
        self.timeout = 5.0
        self.connect_timeout = 5.0
        self.master = None
        self.last_error = None
        self.last_error_kind = None   # "open" | "heartbeat" | "no_port" | None

        self._mav_lock = threading.Lock()
        self._param_cache = {}
    # def

    def is_connected(self):
        return self.master is not None
    # def

    def set_port(self, port):
        self.port = port
    # def

    def connect(self, port=None, timeout=None):
        """Open the link and wait for a heartbeat. Returns True on success.

        Never blocks indefinitely - wait_heartbeat() is bounded so a missing
        flight controller cannot hang the caller (or the GUI thread).
        """
        if port is not None:
            self.port = port

        if not self.port:
            self.last_error = "No MAVLink port selected"
            self.last_error_kind = "no_port"
            print(self.last_error)
            return False

        timeout = self.connect_timeout if timeout is None else timeout
        self.last_error = None
        self.last_error_kind = None

        print("Connecting to %s at %d baud..." % (self.port, self.baud))
        try:
            self.master = mavutil.mavlink_connection(self.port, self.baud)
        except (serial.SerialException, OSError) as e:
            self.last_error = describe_serial_error(self.port, e)
            self.last_error_kind = "open"
            print(self.last_error)
            self.master = None
            return False
        except Exception as e:
            self.last_error = "Failed to open MAVLink on %s: %s" % (self.port, str(e))
            self.last_error_kind = "open"
            print(self.last_error)
            self.master = None
            return False

        print("Waiting for heartbeat from PX4 (up to %.0fs)..." % timeout)
        heartbeat = self.master.wait_heartbeat(timeout=timeout)

        if heartbeat is None:
            self.last_error = "No heartbeat on %s after %.0fs" % (self.port, timeout)
            self.last_error_kind = "heartbeat"
            print(self.last_error)
            self.close()
            return False

        print("Heartbeat from system %s component %s" % (
            self.master.target_system,
            self.master.target_component
        ))
        return True
    # def

    def close(self):
        try:
            if self.master is not None:
                close_fn = getattr(self.master, "close", None)
                if callable(close_fn):
                    close_fn()
        except Exception as e:
            print("MAVLink close failed: %s" % str(e))

        self.master = None
        with self._mav_lock:
            self._param_cache.clear()
    # def

    def reconnect(self, port=None, timeout=None):
        self.close()
        return self.connect(port, timeout)
    # def

    def get_id(self, timeout=5.0):
        if not self.is_connected():
            return None

        print("Requesting AUTOPILOT_VERSION...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0, 0, 0, 0, 0, 0
        )

        start = time.time()
        msg = None

        while time.time() - start < timeout:
            msg = self.master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=1)
            if msg is not None:
                break

        if msg is None:
            print("Did not receive AUTOPILOT_VERSION")
            return None

        print("\nAUTOPILOT_VERSION received UID[%d]" % (msg.uid))
        return msg.uid
    # def

    def clean_param_id(self, raw):
        if isinstance(raw, bytes):
            return raw.decode("ascii", errors="ignore").rstrip("\x00")
        else:
            return str(raw).rstrip("\x00")
    # def

    def _decode_param_msg(self, msg):
        raw_value = msg.param_value

        if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
            decoded = float(raw_value)

        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            decoded = struct.unpack("<i", struct.pack("<f", raw_value))[0]

        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            decoded = struct.unpack("<I", struct.pack("<f", raw_value))[0]

        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            decoded = struct.unpack("<i", struct.pack("<f", raw_value))[0] & 0xFFFF
            if decoded & 0x8000:
                decoded -= 0x10000

        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            decoded = struct.unpack("<I", struct.pack("<f", raw_value))[0] & 0xFFFF

        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            decoded = struct.unpack("<i", struct.pack("<f", raw_value))[0] & 0xFF
            if decoded & 0x80:
                decoded -= 0x100

        elif msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            decoded = struct.unpack("<I", struct.pack("<f", raw_value))[0] & 0xFF

        else:
            decoded = raw_value

        return decoded
    # def

    def _handle_incoming_param_msg(self, msg):
        if msg is None:
            return

        if msg.get_type() == "PARAM_VALUE":
            param_name = self.clean_param_id(msg.param_id)
            decoded = self._decode_param_msg(msg)
            self._param_cache[param_name] = decoded
            print("%s decoded = %s" % (param_name, decoded))
    # def

    def get_param(self, param_name, py_type=None, timeout=5.0):
        if not self.is_connected():
            return None

        with self._mav_lock:
            if param_name in self._param_cache:
                del self._param_cache[param_name]

            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                param_name.encode("ascii"),
                -1,
            )

        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._mav_lock:
                msg = self.master.recv_match(blocking=True, timeout=0.2)
                self._handle_incoming_param_msg(msg)

                if param_name in self._param_cache:
                    decoded = self._param_cache[param_name]

                    if py_type is bool:
                        return bool(decoded)
                    if py_type is int:
                        return int(decoded)
                    if py_type is float:
                        return float(decoded)

                    return decoded

        print("Timed out waiting for current value of %s." % param_name)
        return None
    # def

    def set_param_value(self, param_name, py_type, value, verify_timeout=5.0, retry_interval=0.2):
        if not self.is_connected():
            print("Cannot set %s: not connected" % param_name)
            return False

        if py_type in (int, bool):
            if isinstance(value, bool):
                target_value = 1 if value else 0
            elif isinstance(value, (int, float)):
                target_value = int(round(value))
            else:
                raise ValueError("Cannot set INT param %s from value %r" %
                                 (param_name, value))

            packed = struct.pack("<i", int(target_value))
            wire_value = struct.unpack("<f", packed)[0]
            send_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32

        elif py_type is float:
            target_value = float(value)
            wire_value = target_value
            send_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

        else:
            raise ValueError("Unsupported py_type %r for param %s" %
                             (py_type, param_name))

        deadline = time.time() + verify_timeout
        last_readback = None

        while time.time() < deadline:
            with self._mav_lock:
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    param_name.encode("ascii"),
                    wire_value,
                    send_type,
                )

            readback = self.get_param(param_name, py_type, timeout=1.0)
            last_readback = readback

            if readback is None:
                time.sleep(retry_interval)
                continue

            if py_type is float:
                if abs(float(readback) - float(target_value)) < 1e-6:
                    print("Verified %s = %s" % (param_name, readback))
                    return True
            else:
                if int(readback) == int(target_value):
                    print("Verified %s = %s" % (param_name, readback))
                    return True

            print("Waiting for %s to update: wrote %s, read back %s" %
                  (param_name, target_value, readback))
            time.sleep(retry_interval)

        print("Failed to verify %s. Wanted %s, last read back %s" %
              (param_name, target_value, last_readback))
        return False
    # def

    def describe_mav_result(self, result):
        if result is None:
            return "not sent"
        if result == self.ACK_NOT_RECEIVED:
            return "no ACK"
        return self.MAV_RESULT_NAMES.get(int(result), "RESULT_%d" % int(result))
    # def

    def _wait_command_ack(self, command_id, timeout):
        """Drain the link for a COMMAND_ACK. The caller must hold _mav_lock.

        Every message is fed to the param handler on the way past, because this
        consumes whatever else is in the buffer while it waits.
        """
        deadline = time.time() + timeout

        while time.time() < deadline:
            msg = self.master.recv_match(blocking=True, timeout=0.1)

            if msg is None:
                continue

            self._handle_incoming_param_msg(msg)

            if msg.get_type() == "COMMAND_ACK" and msg.command == command_id:
                return int(msg.result)

        return self.ACK_NOT_RECEIVED
    # def

    def command_elevon(self, output_function, value, wait_ack=False, ack_timeout=1.0):
        """Send one actuator test command.

        Returns None when nothing was sent, otherwise a MAV_RESULT (or
        ACK_NOT_RECEIVED) when wait_ack is set. PX4 refuses the command when the
        vehicle is armed, when the safety switch is on, or when COM_MOT_TEST_EN
        is not 1 - all of which look exactly like a stuck surface unless the
        caller reads the ACK.
        """
        if not self.is_connected():
            return None

        # Test escape hatch: exercise the whole app without moving surfaces.
        if os.environ.get("MANTA_NO_ACTUATE"):
            return self.MAV_RESULT_ACCEPTED if wait_ack else None

        timeout_s = 60.0

        with self._mav_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                self.MAV_CMD_ACTUATOR_TEST,
                0,
                float(value),
                float(timeout_s),
                0,
                0,
                float(output_function),
                0,
                0,
            )

            if not wait_ack:
                return None

            return self._wait_command_ack(self.MAV_CMD_ACTUATOR_TEST, ack_timeout)
    # def

# class


class PositionReader:
    def __init__(self, port=None):
        self.port = port
        self.baud = SERIAL_BAUD
        self.timeout = 1.0

        # (monotonic_timestamp, raw_value) pairs. Read non-destructively by any
        # number of consumers; maxlen bounds the backlog.
        self.sample_hz = SLOW_RATE_HZ
        self._queue_left = deque(maxlen=position_history_for(self.sample_hz))
        self._queue_right = deque(maxlen=position_history_for(self.sample_hz))
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()

        # The reader thread owns the port. Commands are written from other
        # threads through send_command(), and the reply comes back via the
        # reader - it stays the only thing that ever reads the port.
        self._serial = None
        self._write_lock = threading.Lock()
        self._replies = queue.Queue(maxsize=32)

        # Raw timestamped capture, for measurements that cannot use the
        # averaged getter. None means "not capturing".
        self._capture = None
        self._capture_limit = MAX_CAPTURE_SAMPLES
        self._capture_truncated = False

        self.connected = False
        self.last_error = None
        self._last_sample_time = 0.0

        # Serialises every read-modify-write of the settings file. Reentrant
        # because set_center() mutates and then saves through one lock scope.
        self._settings_lock = threading.RLock()

        self.calibration_file = os.path.join(APP_DIR, "settings.json")
        self.backup_file = self.calibration_file + ".bak"

        self.drone_name = ""

        # Ports remembered from the previous run; used to pre-select in the UI.
        self.pico_port = None
        self.drone_port = None

        self.left_offset = -75.68
        self.left_scaler = 0.0042

        self.right_offset = +117.75
        self.right_scaler = 0.0045

        self.angle_neg_degs = -33.0
        self.angle_pos_degs = 35.0
        self.angle_trim_degs = -5.0

        self.load_calibration()
    # def

    def _read_settings_file(self):
        """Return the settings document as a dict. Never raises; {} on any problem."""
        try:
            with open(self.calibration_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except Exception:
            return {}
    # def

    def _write_settings_atomic(self, data, path=None):
        """Write the whole document via a temp file and an atomic rename.

        os.replace is atomic on Linux and Windows, so a crash or a concurrent
        reader can never observe a half-written file. The fsync matters because
        the rig gets powered off abruptly.
        """
        path = self.calibration_file if path is None else path
        tmp = path + ".tmp"

        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4)
            f.flush()
            os.fsync(f.fileno())

        os.replace(tmp, path)
    # def

    def load_calibration(self):
        if not os.path.exists(self.calibration_file):
            print("Calibration file %s not found, using defaults" % self.calibration_file)
            return

        try:
            with open(self.calibration_file, "r", encoding="utf-8") as f:
                data = json.load(f)

            left = data.get("LEFT", {})
            right = data.get("RIGHT", {})
            angles = data.get("ANGLES", {})
            ports = data.get("PORTS", {})

            self.pico_port = ports.get("pico") or None
            self.drone_port = ports.get("drone") or None

            if "scaler" in left:
                self.left_scaler = float(left["scaler"])
            if "offset" in left:
                self.left_offset = float(left["offset"])

            if "scaler" in right:
                self.right_scaler = float(right["scaler"])
            if "offset" in right:
                self.right_offset = float(right["offset"])

            if "angle_neg_degs" in angles:
                self.angle_neg_degs = float(angles["angle_neg_degs"])
            if "angle_pos_degs" in angles:
                self.angle_pos_degs = float(angles["angle_pos_degs"])
            if "angle_trim_degs" in angles:
                self.angle_trim_degs = float(angles["angle_trim_degs"])

            print(
                "Loaded calibration from %s: LEFT scaler=%.6f offset=%.6f, RIGHT scaler=%.6f offset=%.6f, angles=(%.2f, %.2f, %.2f)"
                % (
                    self.calibration_file,
                    self.left_scaler,
                    self.left_offset,
                    self.right_scaler,
                    self.right_offset,
                    self.angle_neg_degs,
                    self.angle_pos_degs,
                    self.angle_trim_degs,
                )
            )

            # Keep a known-good copy. If the live file is ever damaged, this is
            # the difference between a restore and re-calibrating the rig.
            try:
                self._write_settings_atomic(data, self.backup_file)
            except Exception as e:
                print("Could not write calibration backup %s: %s" % (self.backup_file, str(e)))

        except Exception as e:
            print("FAILED TO LOAD CALIBRATION from %s: %s" % (self.calibration_file, str(e)))
            print("Running on built-in defaults - your rig calibration is NOT loaded.")
            if os.path.exists(self.backup_file):
                print("A previous good copy is at %s - copy it over %s and restart."
                      % (self.backup_file, self.calibration_file))
    # def

    def _section_payload(self, section):
        if section == "LEFT":
            return {"scaler": self.left_scaler, "offset": self.left_offset}
        if section == "RIGHT":
            return {"scaler": self.right_scaler, "offset": self.right_offset}
        if section == "ANGLES":
            return {
                "angle_neg_degs": self.angle_neg_degs,
                "angle_pos_degs": self.angle_pos_degs,
                "angle_trim_degs": self.angle_trim_degs,
            }
        if section == "PORTS":
            return {"pico": self.pico_port, "drone": self.drone_port}
        return {}
    # def

    def save_calibration(self, sections=None):
        """Merge the named sections into what is on disk, then replace atomically.

        Only the sections the caller actually changed are written. This object is
        constructed once and never reloads, so writing every section on every save
        would let an unrelated save - recording a port, say - stamp this instance's
        stale calibration over newer values written by another instance or by hand.

        sections=None means "all of them", for a deliberate full save.
        """
        if sections is None:
            sections = ("LEFT", "RIGHT", "ANGLES", "PORTS")

        with self._settings_lock:
            data = self._read_settings_file()

            for section in sections:
                data.setdefault(section, {}).update(self._section_payload(section))

            try:
                self._write_settings_atomic(data)

                print(
                    "Saved calibration to %s: LEFT scaler=%.6f offset=%.6f, RIGHT scaler=%.6f offset=%.6f, angles=(%.2f, %.2f, %.2f)"
                    % (
                        self.calibration_file,
                        self.left_scaler,
                        self.left_offset,
                        self.right_scaler,
                        self.right_offset,
                        self.angle_neg_degs,
                        self.angle_pos_degs,
                        self.angle_trim_degs,
                    )
                )

            except Exception as e:
                print("Failed to save calibration to %s: %s" % (self.calibration_file, str(e)))
    # def

    def set_remembered_ports(self, pico_port=None, drone_port=None):
        with self._settings_lock:
            changed = False

            if pico_port is not None and pico_port != self.pico_port:
                self.pico_port = pico_port
                changed = True

            if drone_port is not None and drone_port != self.drone_port:
                self.drone_port = drone_port
                changed = True

            if changed:
                self.save_calibration(("PORTS",))
    # def

    def set_angle_settings(self, angle_neg_degs=None, angle_pos_degs=None, angle_trim_degs=None):
        with self._settings_lock:
            if angle_neg_degs is not None:
                self.angle_neg_degs = float(angle_neg_degs)
            if angle_pos_degs is not None:
                self.angle_pos_degs = float(angle_pos_degs)
            if angle_trim_degs is not None:
                self.angle_trim_degs = float(angle_trim_degs)

            self.save_calibration(("ANGLES",))
    # def

    def position_to_degrees(self, side, raw_position):
        if side == "LEFT":
            return position_to_degrees(side, raw_position, self.left_scaler, self.left_offset)
        if side == "RIGHT":
            return position_to_degrees(side, raw_position, self.right_scaler, self.right_offset)
        return None
    # def

    def set_center(self, side):
        raw = self.get_average_position_nonblocking(side)
        if raw is None:
            print("No data for centering %s" % side)
            return

        with self._settings_lock:
            if side == "LEFT":
                self.left_offset = -(self.left_scaler * raw)
                print("Left centred. Scaler=%.4f Offset = %.4f" % (self.left_scaler, self.left_offset))
                self.save_calibration(("LEFT",))

            elif side == "RIGHT":
                self.right_offset = (self.right_scaler * raw)
                print("Right centred. Scaler=%.4f Offset = %.4f" % (self.right_scaler, self.right_offset))
                self.save_calibration(("RIGHT",))
    # def

    def set_scaler_and_offset(self, side, scaler=None, offset=None):
        with self._settings_lock:
            if side == "LEFT":
                if scaler is not None:
                    self.left_scaler = float(scaler)
                if offset is not None:
                    self.left_offset = float(offset)

                print("LEFT calibration set. Scaler=%.6f Offset=%.6f" % (self.left_scaler, self.left_offset))
                self.save_calibration(("LEFT",))

            elif side == "RIGHT":
                if scaler is not None:
                    self.right_scaler = float(scaler)
                if offset is not None:
                    self.right_offset = float(offset)

                print("RIGHT calibration set. Scaler=%.6f Offset=%.6f" % (self.right_scaler, self.right_offset))
                self.save_calibration(("RIGHT",))
    # def

    def _position_reader_loop(self):
        port = self.port
        print("Opening position stream on %s at %d baud..." % (port, self.baud))

        try:
            with serial.Serial(port, baudrate=self.baud, timeout=self.timeout) as ser:
                with self._lock:
                    self._serial = ser

                self.connected = True
                self.last_error = None

                while not self._stop.is_set():
                    line = ser.readline()
                    if not line:
                        continue

                    text = line.decode("ascii", errors="ignore").strip()
                    if not text:
                        continue

                    # Firmware replies ("# ACK F 500"). Cannot match
                    # POSITION_REGEX, but routing them explicitly is what lets
                    # send_command() wait for an answer without a second reader.
                    if text.startswith(REPLY_PREFIX):
                        try:
                            self._replies.put_nowait(text)
                        except queue.Full:
                            pass
                        continue

                    m = POSITION_REGEX.search(text)
                    if m:
                        try:
                            p1 = int(m.group("position1"))
                            p2 = int(m.group("position2"))
                        except ValueError:
                            continue

                        raw_t_us = m.group("t_us")
                        pico_us = int(raw_t_us) if raw_t_us is not None else None

                        now = time.monotonic()
                        with self._lock:
                            self._queue_left.append((now, p1))
                            self._queue_right.append((now, p2))
                            self._last_sample_time = now

                            if self._capture is not None:
                                if len(self._capture) < self._capture_limit:
                                    self._capture.append((now, pico_us, p1, p2))
                                else:
                                    self._capture_truncated = True

        except (serial.SerialException, OSError) as e:
            self.last_error = describe_serial_error(port, e)
            print(self.last_error)

        finally:
            with self._lock:
                self._serial = None

            self.connected = False
            print("Position stream on %s closed" % port)
    # def

    def send_command(self, command, timeout=1.5, attempts=3):
        """Send a firmware command and return its reply, or None.

        Retried because a freshly opened USB CDC port swallows writes issued
        before it settles, and a lost write is indistinguishable from a board
        that never answers. Every command pico/sampler.py accepts is idempotent,
        so a duplicate is harmless.

        Returns None against the legacy firmware, which never reads stdin.
        """
        for _ in range(max(1, attempts)):
            with self._lock:
                ser = self._serial

            if ser is None:
                return None

            with self._write_lock:
                # Drop replies to earlier commands so we cannot mistake one for
                # the answer to this one.
                while True:
                    try:
                        self._replies.get_nowait()
                    except queue.Empty:
                        break

                try:
                    ser.write((command + "\n").encode("ascii"))
                    ser.flush()
                except (serial.SerialException, OSError) as e:
                    print("Failed to send %r to the Pico: %s" % (command, str(e)))
                    return None

            try:
                return self._replies.get(timeout=timeout)
            except queue.Empty:
                continue

        return None
    # def

    def set_sample_rate(self, hz):
        """Negotiate a sample rate with the Pico. Returns the rate it accepted.

        Resizing the backlog is not optional: POSITION_HISTORY_S has to follow
        the rate or the averaging window silently outruns the buffer feeding it.
        """
        hz = int(hz)
        command = "S" if hz <= SLOW_RATE_HZ else "F%d" % hz

        reply = self.send_command(command)
        if reply is None:
            print("Pico did not accept a rate command - assuming legacy 10 Hz firmware")
            return None

        match = ACK_RATE_REGEX.search(reply)
        achieved = int(match.group(1)) if match else hz

        self._resize_history(achieved)
        self.sample_hz = achieved

        print("Pico sample rate: %d Hz" % achieved)
        return achieved
    # def

    def _resize_history(self, hz):
        size = position_history_for(hz)

        with self._lock:
            if self._queue_left.maxlen == size:
                return

            # deque(iterable, maxlen=n) keeps the newest n, which is what we want.
            self._queue_left = deque(self._queue_left, maxlen=size)
            self._queue_right = deque(self._queue_right, maxlen=size)
    # def

    def start_capture(self, limit=MAX_CAPTURE_SAMPLES):
        """Begin recording raw timestamped samples.

        Separate from the averaged getter on purpose. get_average_position_
        nonblocking() smooths over POSITION_WINDOW_S, which carries ~250 ms of
        group delay - the same order as an elevon transit, so it would swamp any
        measurement of one. Capture keeps every sample and the board's own
        microsecond stamp.
        """
        with self._lock:
            self._capture = []
            self._capture_limit = int(limit)
            self._capture_truncated = False
    # def

    def stop_capture(self):
        """Return (samples, truncated).

        samples is a list of (host_monotonic, pico_us, raw_left, raw_right);
        pico_us is None on legacy firmware, which emits no timestamp.
        """
        with self._lock:
            samples = self._capture if self._capture is not None else []
            truncated = self._capture_truncated
            self._capture = None
            self._capture_truncated = False

        if truncated:
            print("Capture hit the %d sample cap - data is incomplete" % self._capture_limit)

        return samples, truncated
    # def

    def is_capturing(self):
        with self._lock:
            return self._capture is not None
    # def

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return

        if not self.port:
            print("No Pico port selected")
            return

        self._stop.clear()
        t = threading.Thread(target=self._position_reader_loop, daemon=True)
        self._thread = t
        t.start()
    # def

    def stop(self, join_timeout=2.0, restore_slow=True):
        # Hand the board back in its readable 10 Hz mode, so whatever opens the
        # port next - a terminal, pico_monitor, the next run - is not met with a
        # 500 Hz firehose. Best effort and deliberately impatient: this is on the
        # shutdown path and must not hang it. Must happen before _stop, since the
        # reader thread is what collects the reply.
        if restore_slow and self.connected and self.sample_hz != SLOW_RATE_HZ:
            self.send_command("S", timeout=0.5, attempts=1)

        self._stop.set()

        t = self._thread
        if t is not None and t.is_alive():
            # readline() has a 1s timeout, so this returns promptly.
            t.join(join_timeout)

        self._thread = None
        self.connected = False
        self.sample_hz = SLOW_RATE_HZ
    # def

    def set_port(self, port):
        if port == self.port and self._thread is not None and self._thread.is_alive():
            return

        self.stop()
        self.port = port
        self.last_error = None
        self._last_sample_time = 0.0
        self.clear_queues()

        with self._lock:
            self._capture = None
            self._capture_truncated = False

        if port:
            self.start()
    # def

    def is_streaming(self, max_age=1.0):
        if not self.connected:
            return False

        with self._lock:
            last = self._last_sample_time

        return last > 0.0 and (time.monotonic() - last) <= max_age
    # def

    def clear_queues(self):
        with self._lock:
            self._queue_left.clear()
            self._queue_right.clear()
    # def

    def get_average_position_nonblocking(self, side, window_s=None):
        """Mean of every sample from the last window_s seconds.

        Non-destructive: the UI, the calibration workers and the centring buttons
        can all read concurrently and all see the same data. Previously this
        popped samples, so with a 10 Hz producer and a 10 Hz UI consumer each
        caller usually got one sample - and whoever asked second got None.

        Returns None only when nothing has arrived within the window, which now
        genuinely means the stream is stale.
        """
        window = POSITION_WINDOW_S if window_s is None else float(window_s)
        cutoff = time.monotonic() - window

        total = 0.0
        count = 0

        with self._lock:
            if side == "LEFT":
                queue_ref = self._queue_left
            elif side == "RIGHT":
                queue_ref = self._queue_right
            else:
                return None

            # Newest first; entries are time-ordered, so the first stale one ends it.
            for timestamp, value in reversed(queue_ref):
                if timestamp < cutoff:
                    break
                total += value
                count += 1

        if count == 0:
            return None

        return total / float(count)
    # def
# class


class FourSliderGUI:
    def __init__(self, root, position_reader, drone_interface):
        self.root = root
        self.root.title("Manta Trimmer")

        # Before any widget is built: the option database is read at widget
        # creation time, so anything constructed earlier keeps the stock look.
        apply_theme(self.root)

        self.position_reader = position_reader
        self.drone_interface = drone_interface

        self.calibration_log_file = os.path.join(APP_DIR, "calibration_log.csv")

        self.DEFAULT_PWM_MIN = 1000
        self.DEFAULT_PWM_MAX = 2000
        self.DEFAULT_TRIM = 0.0

        self._closing = False

        # Worker threads never touch widgets directly. They push callables here
        # and the Tk thread runs them in _drain_gui_queue().
        self._gui_queue = queue.Queue()

        # Parameter-entry widgets by key, and a generation counter per key so a
        # superseded write cannot stamp a stale readback over a newer edit.
        self._param_widgets = {}
        self._param_write_seq = {}

        self.LEFT_OUTPUT_FUNCTION = 1201
        self.LEFT_MIN_PARAM = "PWM_MAIN_MIN5"
        self.LEFT_MAX_PARAM = "PWM_MAIN_MAX5"
        self.LEFT_TRIM_PARAM = "CA_SV_CS0_TRIM"

        self.RIGHT_OUTPUT_FUNCTION = 1202
        self.RIGHT_MIN_PARAM = "PWM_MAIN_MIN6"
        self.RIGHT_MAX_PARAM = "PWM_MAIN_MAX6"
        self.RIGHT_TRIM_PARAM = "CA_SV_CS1_TRIM"

        self.left_cal_thread = None
        self.right_cal_thread = None
        self.left_cal_active = False
        self.right_cal_active = False

        self.sweep_thread = None
        self.sweep_active = False

        self.measure_thread = None
        self.measure_active = False

        self.stiction_results = []
        self.creep_points = []
        self.curve_window = None
        self.stiction_summary_rows = []
        self.rr_results = []
        self.rr_summary_rows = []
        self.rr_samples = []
        self.rr_samples_truncated = False
        self.rr_run_stamp = ""
        self.rr_last_trace = None

        self.drone_name_var = tk.StringVar(value="")
        self.folding_var = tk.StringVar(value="")

        self.angle_neg_degs = self.position_reader.angle_neg_degs
        self.angle_pos_degs = self.position_reader.angle_pos_degs
        self.angle_trim_degs = self.position_reader.angle_trim_degs

        self.angle_neg_var = tk.StringVar(value="%.1f" % self.angle_neg_degs)
        self.angle_pos_var = tk.StringVar(value="%.1f" % self.angle_pos_degs)
        self.angle_trim_var = tk.StringVar(value="%.1f" % self.angle_trim_degs)

        # Shell layout. Two things sit outside the notebook on purpose:
        #
        #   connection  - global state every tab depends on, and the thing you
        #                 most need to see before starting anything
        #   instrumentation - how you diagnose any tab. Per-tab copies would
        #                 guarantee the interesting line is on the other one.
        #
        # Everything else is tab content.
        main_frame = tk.Frame(root)
        main_frame.pack(padx=20, pady=20, fill=tk.BOTH, expand=True)

        status_strip = tk.Frame(main_frame)
        status_strip.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))

        self.build_connection_panel(status_strip)

        body = tk.Frame(main_frame)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.notebook = ttk.Notebook(body)
        self.notebook.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        trim_tab = tk.Frame(self.notebook, padx=6, pady=6)
        self.notebook.add(trim_tab, text="  Trim  ")

        # Widgets are built from defaults; the real values are fetched by
        # refresh_params_from_drone() once a link is up. Nothing here may block
        # on MAVLink - the window must appear even with no hardware attached.
        self.uid_var = tk.StringVar(value="--")

        left_min_param = self.DEFAULT_PWM_MIN
        left_max_param = self.DEFAULT_PWM_MAX
        left_trim_param = self.DEFAULT_TRIM
        right_min_param = self.DEFAULT_PWM_MIN
        right_max_param = self.DEFAULT_PWM_MAX
        right_trim_param = self.DEFAULT_TRIM

        self.main_rev = 0

        # Angle configuration panel
        angle_group = tk.LabelFrame(trim_tab, text="Settings and Control", padx=10, pady=10)
        angle_group.pack(side=tk.LEFT, padx=10, anchor="n", fill=tk.Y)

        name_row = tk.Frame(angle_group, bd=1, relief="groove", padx=4, pady=4)
        name_row.pack(anchor="w", pady=3, fill=tk.X)

        name_lbl = tk.Label(name_row, text="Drone name", width=10, anchor="w")
        name_lbl.pack(side=tk.LEFT, padx=(0, 5))

        name_entry = tk.Entry(name_row, textvariable=self.drone_name_var, width=22, justify="left")
        name_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        name_entry.bind("<Return>", lambda event: self.apply_drone_name())

        name_btn = tk.Button(name_row, text="Set", width=5, command=self.apply_drone_name)
        name_btn.pack(side=tk.LEFT, padx=(5, 0))

        uid_row = tk.Frame(angle_group, bd=1, relief="groove", padx=4, pady=4)
        uid_row.pack(anchor="w", pady=3, fill=tk.X)

        uid_lbl = tk.Label(uid_row, text="PX4 UID", width=10, anchor="w")
        uid_lbl.pack(side=tk.LEFT, padx=(0, 5))

        uid_box = tk.Entry(uid_row, textvariable=self.uid_var, width=22, justify="left")
        uid_box.pack(side=tk.LEFT, fill=tk.X, expand=True)
        uid_box.config(state="readonly")

        self.create_angle_entry(angle_group, "Neg deg", self.angle_neg_var, self.apply_angle_neg)
        self.create_angle_entry(angle_group, "Pos deg", self.angle_pos_var, self.apply_angle_pos)
        self.create_angle_entry(angle_group, "Trim deg", self.angle_trim_var, self.apply_angle_trim)

        zero_angles_btn = tk.Button(
            angle_group,
            text="Zero angles",
            width=18,
            command=self.zero_both_angles
        )
        zero_angles_btn.pack(pady=2, anchor="w")

        auto_both_btn = tk.Button(
            angle_group,
            text="Auto calibrate",
            width=18,
            command=self.start_both_calibration
        )
        auto_both_btn.pack(pady=2, anchor="w")

        stop_both_btn = tk.Button(
            angle_group,
            text="Stop auto",
            width=18,
            command=self.stop_both_calibration
        )
        stop_both_btn.pack(pady=2, anchor="w")

        sweep_btn = tk.Button(
            angle_group,
            text="Sweep to CSV",
            width=18,
            command=self.start_sweep_to_csv
        )
        sweep_btn.pack(pady=(8, 2), anchor="w")

        folding_row = tk.Frame(angle_group, bd=1, relief="groove", padx=4, pady=4)
        folding_row.pack(anchor="w", pady=(8, 2), fill=tk.X)

        tk.Label(folding_row, text="Folding", width=10, anchor="w").pack(side=tk.LEFT, padx=(0, 5))
        folding_check = tk.Checkbutton(folding_row, variable=self.folding_var,
                                       onvalue="y", offvalue="")
        folding_check.pack(side=tk.LEFT)

        log_cal_btn = tk.Button(
            angle_group,
            text="Log calibration",
            width=18,
            command=self.log_calibration
        )
        log_cal_btn.pack(pady=(2, 0), anchor="w")

        # LEFT group
        left_group = tk.LabelFrame(trim_tab, text="Left", padx=10, pady=10)
        left_group.pack(side=tk.LEFT, padx=10, fill=tk.Y)

        left_clear_btn = tk.Button(
            left_group,
            text="Reset Min/Max/Trim",
            width=18,
            command=self.clear_left
        )
        left_clear_btn.pack(pady=(0, 10))

        left_cal_btn = tk.Button(
            left_group,
            text="Auto calibrate",
            width=18,
            command=self.start_left_calibration
        )
        left_cal_btn.pack(pady=(0, 4))

        left_stop_btn = tk.Button(
            left_group,
            text="Auto calibrate STOP",
            width=18,
            command=self.stop_left_calibration
        )
        left_stop_btn.pack(pady=(0, 10))

        left_params = tk.Frame(left_group, bd=1, relief="groove", padx=6, pady=6)
        left_params.pack(pady=(0, 10), fill=tk.X)

        self.left_min_var = tk.StringVar(value=str(left_min_param))
        self.left_max_var = tk.StringVar(value=str(left_max_param))
        self.left_trim_var = tk.StringVar(value="%.3f" % left_trim_param)

        self.create_param_entry(left_params, "left_min", "Left-min", self.left_min_var, self.apply_left_min)
        self.create_param_entry(left_params, "left_max", "Left-max", self.left_max_var, self.apply_left_max)
        self.create_param_entry(left_params, "left_trim", "Left-trim", self.left_trim_var, self.apply_left_trim)

        left_pos_frame = tk.Frame(left_group, bd=1, relief="groove", padx=6, pady=6)
        left_pos_frame.pack()

        self.left_pos = self.create_slider(
            left_pos_frame, "Left-pos", self.on_left_pos, -1.0, 1.0, 0.0, 0.01
        )

        self.left_pwm_label = tk.Label(
            left_group,
            text="PWM exp: --",
            relief="flat",
            bd=0,
            bg=PALETTE["panel_alt"],
            width=16,
            anchor="center",
            pady=4
        )
        self.left_pwm_label.pack(pady=(10, 4))

        self.left_label = tk.Label(
            left_group,
            text="Left: --",
            relief="flat",
            bd=0,
            bg=PALETTE["panel_alt"],
            width=16,
            anchor="center",
            pady=4
        )
        self.left_label.pack(pady=(0, 10))

        left_center_btn = tk.Button(
            left_group,
            text="Zero angle",
            width=10,
            command=self.centre_left
        )
        left_center_btn.pack(pady=(0, 5))

        # RIGHT group
        right_group = tk.LabelFrame(trim_tab, text="Right", padx=10, pady=10)
        right_group.pack(side=tk.LEFT, padx=10, fill=tk.Y)

        right_clear_btn = tk.Button(
            right_group,
            text="Reset Min/Max/Trim",
            width=18,
            command=self.clear_right
        )
        right_clear_btn.pack(pady=(0, 10))

        right_cal_btn = tk.Button(
            right_group,
            text="Auto calibrate",
            width=18,
            command=self.start_right_calibration
        )
        right_cal_btn.pack(pady=(0, 4))

        right_stop_btn = tk.Button(
            right_group,
            text="Auto calibrate STOP",
            width=18,
            command=self.stop_right_calibration
        )
        right_stop_btn.pack(pady=(0, 10))

        right_params = tk.Frame(right_group, bd=1, relief="groove", padx=6, pady=6)
        right_params.pack(pady=(0, 10), fill=tk.X)

        self.right_min_var = tk.StringVar(value=str(right_min_param))
        self.right_max_var = tk.StringVar(value=str(right_max_param))
        self.right_trim_var = tk.StringVar(value="%.3f" % right_trim_param)

        self.create_param_entry(right_params, "right_min", "Right-min", self.right_min_var, self.apply_right_min)
        self.create_param_entry(right_params, "right_max", "Right-max", self.right_max_var, self.apply_right_max)
        self.create_param_entry(right_params, "right_trim", "Right-trim", self.right_trim_var, self.apply_right_trim)

        right_pos_frame = tk.Frame(right_group, bd=1, relief="groove", padx=6, pady=6)
        right_pos_frame.pack()

        self.right_pos = self.create_slider(
            right_pos_frame, "Right-pos", self.on_right_pos, -1.0, 1.0, 0.0, 0.01
        )

        self.right_pwm_label = tk.Label(
            right_group,
            text="PWM exp: --",
            relief="flat",
            bd=0,
            bg=PALETTE["panel_alt"],
            width=16,
            anchor="center",
            pady=4
        )
        self.right_pwm_label.pack(pady=(10, 4))

        self.right_label = tk.Label(
            right_group,
            text="Right: --",
            relief="flat",
            bd=0,
            bg=PALETTE["panel_alt"],
            width=16,
            anchor="center",
            pady=4
        )
        self.right_label.pack(pady=(0, 10))

        right_center_btn = tk.Button(
            right_group,
            text="Zero angle",
            width=10,
            command=self.centre_right
        )
        right_center_btn.pack(pady=(0, 5))

        rr_tab = tk.Frame(self.notebook, padx=6, pady=6)
        self.notebook.add(rr_tab, text="  Measure  ")
        self.build_measure_tab(rr_tab)

        # Instrumentation panel
        log_group = tk.LabelFrame(body, text="Instrumentation", padx=10, pady=10)
        log_group.pack(side=tk.LEFT, padx=10, fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(
            log_group,
            width=80,
            height=32,
            wrap="word"
        )
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        log_scroll = tk.Scrollbar(log_group, command=self.log_text.yview)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=log_scroll.set)

        self._drain_gui_queue()
        self.update_labels()
        self.update_actuators()
        self.update_expected_pwm()
        self.update_log_window()
    # def

    def make_safe_filename(self, text):
        safe = str(text).strip()
        safe = re.sub(r"[^A-Za-z0-9._-]+", "_", safe)
        safe = safe.strip("._")
        if safe == "":
            safe = "drone"
        return safe
    # def

    def wait_for_valid_both_angles(self, timeout=5.0, poll_s=0.05):
        deadline = time.time() + timeout

        while time.time() < deadline:
            left_angle = self.get_left_value()
            right_angle = self.get_right_value()

            if left_angle is not None and right_angle is not None:
                return left_angle, right_angle

            time.sleep(poll_s)

        return None, None
    # def

    def start_sweep_to_csv(self):
        if self.sweep_thread is not None and self.sweep_thread.is_alive():
            print("Sweep already running")
            return

        self.stop_both_calibration()

        # Read the Tk variable here, on the GUI thread, and hand the value to the
        # worker. Tk is not thread-safe, so the worker must never touch it.
        drone_name = self.drone_name_var.get().strip()
        if drone_name == "":
            drone_name = "drone"

        self.sweep_active = True
        self.sweep_thread = threading.Thread(
            target=self._sweep_to_csv_worker,
            args=(drone_name,),
            daemon=True
        )
        self.sweep_thread.start()
    # def

    def _sweep_to_csv_worker(self, drone_name):
        try:
            safe_name = self.make_safe_filename(drone_name)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = report_path(APP_DIR, "%s_%s.csv" % (safe_name, timestamp))

            print("Starting sweep to CSV: %s" % csv_filename)

            sweep_values = []
            i = -10
            while i <= 10:
                sweep_values.append(round(i / 10.0, 1))
                i += 1

            with open(csv_filename, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "date",
                    "time",
                    "drone_name",
                    "input_cmd",
                    "left_angle_deg",
                    "right_angle_deg",
                ])

                for cmd in sweep_values:
                    if not self.sweep_active:
                        print("Sweep stopped")
                        break

                    print("Sweep step: cmd=%.1f" % cmd)

                    self.drone_interface.command_elevon(self.LEFT_OUTPUT_FUNCTION, cmd)
                    self.drone_interface.command_elevon(self.RIGHT_OUTPUT_FUNCTION, cmd)

                    # Settle. This must stay longer than POSITION_WINDOW_S: it is
                    # what guarantees every sample in the averaging window was
                    # taken after the elevon finished moving.
                    time.sleep(1.0)

                    left_angle, right_angle = self.wait_for_valid_both_angles(timeout=5.0, poll_s=0.05)

                    if left_angle is None or right_angle is None:
                        print("Skipping sweep point %.1f because valid angles were not received on both sides" % cmd)
                        continue

                    now = datetime.now()
                    date_str = now.strftime("%Y-%m-%d")
                    time_str = now.strftime("%H:%M:%S")

                    writer.writerow([
                        date_str,
                        time_str,
                        drone_name,
                        "%.1f" % cmd,
                        "%.3f" % left_angle,
                        "%.3f" % right_angle,
                    ])
                    f.flush()

                    print(
                        "Logged sweep point: cmd=%.1f left_angle=%.3f right_angle=%.3f" %
                        (cmd, left_angle, right_angle)
                    )

            self.drone_interface.command_elevon(self.LEFT_OUTPUT_FUNCTION, 0.0)
            self.drone_interface.command_elevon(self.RIGHT_OUTPUT_FUNCTION, 0.0)
            self.post_to_gui(self.zero_both_sliders)

            print("Sweep finished: %s" % csv_filename)

        except Exception as e:
            print("Sweep to CSV failed: %s" % str(e))
        finally:
            self.sweep_active = False
    # def

    # ---- Range and rate test -------------------------------------------------

    def build_measure_tab(self, parent):
        left_col = tk.Frame(parent)
        left_col.pack(side=tk.LEFT, anchor="n", padx=(0, 10))

        setup = tk.LabelFrame(left_col, text="What to measure", padx=8, pady=6)
        setup.pack(anchor="w", fill=tk.X)

        # The two measurements share their hard-overs: a range/rate cycle already
        # drives the surface full span to each end stop, which is exactly the
        # swing half of the stiction comparison. Running them together therefore
        # costs only the creeps, and the swing numbers both tests quote are then
        # literally the same measurements rather than two runs that have to be
        # taken on trust as comparable.
        self.m_range_rate_var = tk.IntVar(value=1)
        self.m_stiction_var = tk.IntVar(value=0)

        for text, var in (("Range and rate", self.m_range_rate_var),
                          ("Stiction (creep vs swing)", self.m_stiction_var)):
            tk.Checkbutton(setup, text=text, variable=var, anchor="w",
                           command=self.update_measure_estimate).pack(anchor="w")

        tk.Frame(setup, height=1, bg=PALETTE["rule"]).pack(fill=tk.X, pady=6)

        # Only BOTH is on by default: driving the servos one at a time was
        # measured to give the same travel and rate as driving them together, so
        # LEFT and RIGHT are here to isolate one servo when something looks
        # wrong, not to be paid for on every run.
        self.rr_phase_vars = {}
        for phase in PHASES:
            var = tk.IntVar(value=1 if phase in DEFAULT_PHASES else 0)
            self.rr_phase_vars[phase] = var
            tk.Checkbutton(
                setup,
                text={"LEFT": "Left alone", "RIGHT": "Right alone",
                      "BOTH": "Both together"}[phase],
                variable=var,
                anchor="w",
                command=self.update_measure_estimate,
            ).pack(anchor="w")

        tk.Frame(setup, height=1, bg=PALETTE["rule"]).pack(fill=tk.X, pady=6)

        self.rr_cycles_var = tk.StringVar(value=str(DEFAULT_CYCLES))
        self.st_reps_var = tk.StringVar(value=str(DEFAULT_STICTION_REPS))
        self.rr_settle_var = tk.StringVar(value="2.0")
        self.rr_rate_var = tk.StringVar(value=str(FAST_RATE_HZ))
        self.st_step_var = tk.StringVar(value="%.3f" % CREEP_STEP_CMD)
        self.st_period_var = tk.StringVar(value="%.2f" % CREEP_PERIOD_S)
        self.st_points_var = tk.StringVar(value=str(DEFAULT_CREEP_POINTS))

        # Swing cycles and creep reps are separate counts on purpose: a swing is
        # a couple of seconds and a creep from centre is a hundred steps, so
        # tying them together would price the cheap measurement at the expensive
        # one's rate.
        for label, var in (("Swing cycles", self.rr_cycles_var),
                           ("Creep reps", self.st_reps_var),
                           ("Settle s", self.rr_settle_var),
                           ("Rate Hz", self.rr_rate_var),
                           ("Creep step", self.st_step_var),
                           ("Creep s", self.st_period_var),
                           ("Sample pts", self.st_points_var)):
            row = tk.Frame(setup)
            row.pack(anchor="w", pady=(2, 0), fill=tk.X)
            tk.Label(row, text=label, width=11, anchor="w").pack(side=tk.LEFT)
            entry = tk.Entry(row, textvariable=var, width=6)
            entry.pack(side=tk.LEFT)
            entry.bind("<KeyRelease>", lambda event: self.update_measure_estimate())

        self.rr_estimate_label = tk.Label(
            setup, text="", anchor="w", justify="left", fg=PALETTE["ink_muted"])
        self.rr_estimate_label.pack(anchor="w", pady=(6, 0))

        run_group = tk.LabelFrame(left_col, text="Run", padx=8, pady=6)
        run_group.pack(anchor="w", fill=tk.X, pady=(8, 0))

        tk.Label(
            run_group,
            text="Surfaces move to full deflection.\nThey centre when the test stops.",
            justify="left",
            anchor="w",
            fg=PALETTE["ink_muted"],
        ).pack(anchor="w", pady=(0, 6))

        button_row = tk.Frame(run_group)
        button_row.pack(anchor="w", fill=tk.X)

        self.rr_run_btn = tk.Button(
            button_row, text="Run", width=11, command=self.start_measure)
        self.rr_run_btn.pack(side=tk.LEFT)

        self.rr_stop_btn = tk.Button(
            button_row, text="Stop", width=8, state="disabled",
            command=self.stop_measure)
        self.rr_stop_btn.pack(side=tk.LEFT, padx=(6, 0))

        self.rr_progress = ttk.Progressbar(run_group, mode="determinate", maximum=100.0)
        self.rr_progress.pack(fill=tk.X, pady=(8, 4))

        self.rr_status_label = tk.Label(run_group, text="Idle", anchor="w")
        self.rr_status_label.pack(anchor="w")

        leg_group = tk.LabelFrame(left_col, text="Last leg", padx=8, pady=6)
        leg_group.pack(anchor="w", fill=tk.X, pady=(8, 0))

        self.rr_leg_label = tk.Label(
            leg_group, text="--", anchor="w", justify="left", width=26)
        self.rr_leg_label.pack(anchor="w")

        right_col = tk.Frame(parent)
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # The trace is the point of capturing at 500 Hz: a servo at its
        # mechanical limit shows a rounded start and a settling approach, while a
        # slew-limited command tracks a straight line and stops crisply. A table
        # of numbers cannot show that difference.
        trace_group = tk.LabelFrame(right_col, text="Last transit", padx=6, pady=6)
        trace_group.pack(fill=tk.X)

        self.rr_canvas = tk.Canvas(trace_group, height=210, bg=PALETTE["panel"],
                                   highlightthickness=1,
                                   highlightbackground=PALETTE["rule"])
        self.rr_canvas.pack(fill=tk.BOTH, expand=True)
        self.rr_canvas.bind("<Configure>", lambda event: self.redraw_range_rate_trace())

        # Titled for its contents, not "Results": there are two tables now, and
        # each one's Save button lives with it. A shared footer of three buttons
        # made the reader work out which of them wrote which table.
        results_group = tk.LabelFrame(right_col, text="Range and rate",
                                      padx=6, pady=6)
        results_group.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        results_body = tk.Frame(results_group)
        results_body.pack(fill=tk.BOTH, expand=True)

        # Max/min are the settled endpoints themselves, with their own spread.
        # Range is their difference and hides it: two runs can share a range
        # while one sits several degrees off the other, and only the endpoints
        # show that.
        columns = ("phase", "side", "direction", "angle_max", "angle_min",
                   "range", "travel", "transit", "rate", "n")
        headings = ("Phase", "Side", "Direction", "Max deg", "Min deg",
                    "Range deg", "Travel deg", "Transit ms", "Rate deg/s", "n")

        self.rr_tree = ttk.Treeview(
            results_body, columns=columns, show="headings", height=8)

        for column, heading in zip(columns, headings):
            self.rr_tree.heading(column, text=heading)
            self.rr_tree.column(column, width=92 if column not in ("n", "side") else 52,
                                anchor="center")

        self.rr_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        tree_scroll = tk.Scrollbar(results_body, command=self.rr_tree.yview)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.rr_tree.config(yscrollcommand=tree_scroll.set)

        self.rr_export_btn = tk.Button(
            results_group, text="Save this table", width=14, state="disabled",
            command=self.export_range_rate_csv)
        self.rr_export_btn.pack(anchor="e", pady=(6, 0))

        self.build_stiction_results(right_col)

        footer = tk.Frame(right_col)
        footer.pack(fill=tk.X, pady=(6, 0))

        self.rr_note_label = tk.Label(footer, text="", anchor="w", fg=PALETTE["warn"])
        self.rr_note_label.pack(side=tk.LEFT)

        # Neither table's raw material, but both tables' - every captured sample
        # from the run. The one to reach for when a summary raises a question the
        # aggregate cannot settle, such as whether scatter is a drift across the
        # run or a few bad cycles. Hence the footer rather than either group.
        self.rr_samples_btn = tk.Button(
            footer, text="Save every sample", width=16, state="disabled",
            command=self.export_range_rate_samples_csv)
        self.rr_samples_btn.pack(side=tk.RIGHT)

        # The creep curve belongs to neither table: it is the raw PWM/angle
        # material the tables are silent about, and the only way to look at the
        # shape until there is a plot.
        self.st_curve_btn = tk.Button(
            footer, text="Save creep curve", width=16, state="disabled",
            command=self.export_creep_curve_csv)
        self.st_curve_btn.pack(side=tk.RIGHT, padx=(0, 6))

        # A window rather than another panel: the plot wants room, it is not
        # needed continuously, and the main window is already as tall as the
        # Trim tab makes it.
        self.st_plot_btn = tk.Button(
            footer, text="Plot curve", width=11, state="disabled",
            command=self.show_creep_curve)
        self.st_plot_btn.pack(side=tk.RIGHT, padx=(0, 6))

        self.update_measure_estimate()
    # def

    def build_stiction_results(self, parent):
        group = tk.LabelFrame(parent, text="Stiction", padx=6, pady=6)
        group.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        body = tk.Frame(group)
        body.pack(fill=tk.BOTH, expand=True)

        columns = ("phase", "side", "target", "creep", "swing", "stiction",
                   "transit", "rate", "n")
        headings = ("Phase", "Side", "Cmd", "Creep deg", "Swing deg",
                    "Stiction deg", "Transit ms", "Rate deg/s", "n")

        self.st_tree = ttk.Treeview(body, columns=columns, show="headings", height=6)

        for column, heading in zip(columns, headings):
            self.st_tree.heading(column, text=heading)
            self.st_tree.column(
                column, width=92 if column not in ("n", "side", "target") else 54,
                anchor="center")

        self.st_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scroll = tk.Scrollbar(body, command=self.st_tree.yview)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.st_tree.config(yscrollcommand=scroll.set)

        self.st_export_btn = tk.Button(
            group, text="Save this table", width=14, state="disabled",
            command=self.export_stiction_csv)
        self.st_export_btn.pack(anchor="e", pady=(6, 0))
    # def

    def read_measure_settings(self):
        """Everything the run needs, parsed from the setup group."""
        return {
            "phases": [p for p in PHASES if self.rr_phase_vars[p].get()],
            "range_rate": bool(self.m_range_rate_var.get()),
            "stiction": bool(self.m_stiction_var.get()),
            "cycles": max(1, self.get_int_var(self.rr_cycles_var, DEFAULT_CYCLES)),
            "creep_reps": max(1, self.get_int_var(self.st_reps_var,
                                                  DEFAULT_STICTION_REPS)),
            "settle": max(0.3, self.get_float_var(self.rr_settle_var, 2.0)),
            "rate": max(SLOW_RATE_HZ, self.get_int_var(self.rr_rate_var,
                                                       FAST_RATE_HZ)),
            "creep_step": max(0.001, self.get_float_var(self.st_step_var,
                                                        CREEP_STEP_CMD)),
            "creep_period": max(0.05, self.get_float_var(self.st_period_var,
                                                         CREEP_PERIOD_S)),
            "creep_points": max(3, self.get_int_var(self.st_points_var,
                                                    DEFAULT_CREEP_POINTS)),
            # Snapshotted on the Tk thread: the worker needs min/max/trim to put
            # a curve sample on a PWM axis, and a command only means anything
            # relative to the values in force when it was issued.
            "pwm_map": self.read_pwm_map(),
        }
    # def

    def read_pwm_map(self):
        """Per side, what it takes to turn a command into a PWM. Tk thread only."""
        mapping = {}
        for side, channel in (("LEFT", 5), ("RIGHT", 6)):
            pwm_min, pwm_max, trim = self.read_side_param_snapshot(side)
            mapping[side] = (pwm_min, pwm_max, trim,
                             self.is_main_channel_reversed(channel))
        return mapping
    # def

    def measure_plan(self, settings):
        """(swing legs, creep legs) the settings imply.

        Swings run whenever either measurement is selected - the stiction
        comparison needs them for its swing half, and they are the whole of the
        range/rate test. Selecting both therefore adds creeps, not swings.
        """
        phases = len(settings["phases"])
        if not phases or not (settings["range_rate"] or settings["stiction"]):
            return 0, 0

        swings = phases * settings["cycles"] * 2
        creeps = phases * settings["creep_reps"] * 2 if settings["stiction"] else 0
        return swings, creeps
    # def

    def update_measure_estimate(self):
        settings = self.read_measure_settings()
        swings, creeps = self.measure_plan(settings)

        if not swings and not creeps:
            self.rr_estimate_label.config(text="Nothing selected")
            return

        settle = settings["settle"]
        seconds = len(settings["phases"]) * settle
        seconds += swings * (PRE_ROLL_S + settle)

        if creeps:
            # Full span now, and the sampled steps dwell instead of waiting a
            # period, so they are counted at the dwell rather than the period.
            walk = len(creep_commands(-1.0, 1.0, settings["creep_step"]))
            samples = len(creep_grid(-1.0, 1.0, settings["creep_points"]))
            seconds += creeps * (settle
                                 + (walk - samples) * settings["creep_period"]
                                 + samples * CREEP_SAMPLE_DWELL_S
                                 + settle)

        text = "%d hard-overs" % swings
        if creeps:
            text += ", %d creeps" % creeps
        text += "\nabout %s" % self.format_duration(seconds)

        self.rr_estimate_label.config(text=text)
    # def

    def format_duration(self, seconds):
        seconds = int(round(seconds))
        if seconds < 90:
            return "%d s" % seconds
        return "%d min" % int(round(seconds / 60.0))
    # def

    def start_measure(self):
        if self.measure_active:
            print("A measurement is already running")
            return

        if self.left_cal_active or self.right_cal_active or self.sweep_active:
            messagebox.showwarning(
                "Busy",
                "Stop the calibration or sweep first - they drive the same actuators.")
            return

        settings = self.read_measure_settings()

        if not (settings["range_rate"] or settings["stiction"]):
            messagebox.showwarning(
                "Nothing to run", "Select range and rate, stiction, or both.")
            return

        if not settings["phases"]:
            messagebox.showwarning("Nothing to run", "Select at least one phase.")
            return

        if not self.drone_interface.is_connected():
            messagebox.showwarning(
                "No link", "Connect to the flight controller before measuring.")
            return

        if not self.position_reader.is_streaming():
            messagebox.showwarning(
                "No position data",
                "The Pico is not streaming - nothing would be measured.")
            return

        swings, creeps = self.measure_plan(settings)

        detail = "%d hard-overs to full deflection" % swings
        if creeps:
            detail += ", and %d creeps to the end stops" % creeps

        if not messagebox.askyesno(
                "The surfaces will move",
                "%s across %d phase(s).\n\n"
                "The first move slams to -1 from rest. Keep hands clear.\n\n"
                "Run the measurement?" % (detail, len(settings["phases"]))):
            return

        self.rr_tree.delete(*self.rr_tree.get_children())
        self.st_tree.delete(*self.st_tree.get_children())
        self.rr_results = []
        self.stiction_results = []
        self.creep_points = []
        self.rr_summary_rows = []
        self.stiction_summary_rows = []
        self.rr_samples = []
        self.rr_samples_truncated = False
        # Stamped once here rather than at each button press, so every export
        # from one run carries the same name and they pair up on disk.
        self.rr_run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.rr_last_trace = None
        self.rr_canvas.delete("all")
        self.rr_note_label.config(text="")
        self.rr_export_btn.config(state="disabled")
        self.rr_samples_btn.config(state="disabled")
        self.st_export_btn.config(state="disabled")
        self.st_curve_btn.config(state="disabled")
        self.st_plot_btn.config(state="disabled")

        self.measure_active = True
        self.rr_run_btn.config(state="disabled")
        self.rr_stop_btn.config(state="normal")

        self.measure_thread = threading.Thread(
            target=self._measure_worker, args=(settings,), daemon=True)
        self.measure_thread.start()
    # def

    def stop_measure(self):
        if self.measure_active:
            print("Stopping measurement...")
        self.measure_active = False
    # def

    def _hold(self, sides, command, duration_s):
        """Sleep, re-sending the actuator test so the FC cannot take over.

        The surfaces are held by an override that lapses after about 2 s, so any
        wait longer than that is not a wait at all - it is the FC quietly
        resuming control part way through a measurement.
        """
        deadline = time.monotonic() + duration_s

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            time.sleep(min(MEASURE_REFRESH_S, remaining))
            for side in sides:
                self.drone_interface.command_elevon(
                    self.rr_output_function(side), command)
    # def

    def _measure_settled(self, phase, sides, command, dwell_s, cal):
        """Hold at `command` for dwell_s and return {side: settled angle}.

        One estimator for every static reading this tab takes - curve samples
        and end stop arrivals alike. Differencing two angles measured different
        ways would mean nothing, and the curve has to join up with the endpoint
        value it ends at.

        No collect-garbage first: that exists to keep MicroPython's stall out of
        a 1000 Hz transit capture, and a static reading does not care. It is a
        blocking serial round trip that retries for up to 4.5 s, which is long
        enough on its own to lose the actuator override.
        """
        reader = self.position_reader
        reader.start_capture()

        # Anchor before the wait, not after: to_series() zeroes on the first
        # sample at or after the timestamp it is given, so a post-capture time
        # would match nothing and return an empty series.
        t_start = time.monotonic()
        self._hold(sides, command, dwell_s)
        samples, _truncated = reader.stop_capture()

        angles = {}
        for side in PHASE_SIDES[phase]:
            series = to_series(samples, t_start, cal, side)
            angles[side] = settled_angle([a for _t, a in series])
        return angles
    # def

    def _stiction_capture_settled(self, phase, sides, kind, rep, target, settle, cal):
        """The end stop arrival: what the stiction comparison is built from."""
        for side, angle in self._measure_settled(
                phase, sides, target, settle, cal).items():
            record = {
                "phase": phase, "side": side, "kind": kind, "rep": rep,
                "target": target, "final_deg": angle,
                "ok": angle is not None,
                "transit_s": None, "rate_deg_s": None, "travel_deg": None,
            }
            self.stiction_results.append(record)
            self.post_to_gui(lambda r=record: self._st_show_leg(r))
    # def

    def _sample_creep_point(self, phase, sides, rep, target, command, cal, pwm_map):
        """One dwelled sample part way along a creep.

        Stored raw and separately from the stiction results: these are curve
        points, and letting them into the same list would put mid-travel
        positions into an end stop comparison.
        """
        direction = 1.0 if target > 0.0 else -1.0

        for side, angle in self._measure_settled(
                phase, sides, command, CREEP_SAMPLE_DWELL_S, cal).items():
            if angle is None:
                continue

            pwm_min, pwm_max, trim, rev = pwm_map[side]
            self.creep_points.append({
                "phase": phase,
                "side": side,
                "direction": direction,
                "rep": rep,
                "cmd": round(command, 4),
                "pwm_us": self.expected_pwm(command, pwm_min, pwm_max, trim, rev),
                "angle_deg": round(angle, 3),
            })
    # def

    def _stiction_creep_leg(self, phase, sides, rep, target, step, period,
                            points, settle, cal, pwm_map):
        """Creep the full span to the end stop, sampling the curve on the way.

        The same increment-and-wait the trim calibration uses, but walking to a
        command rather than to an angle: the end stop is where the command runs
        out.

        Full span, from the opposite end rather than from centre, so that the
        two directions cover the same ground and can be differenced. Creeping
        outward from centre gives two halves of one curve travelled in opposite
        directions, which overlap nowhere and so cannot show hysteresis at all.

        The walk pauses at each grid position for longer than the averaging
        window and records a sample - see CREEP_SAMPLE_DWELL_S for why sampling
        on the move would manufacture a band that is not there. The arrival at
        the end stop is measured separately and is still the number the stiction
        comparison uses, unchanged.
        """
        origin = -target

        for side in sides:
            self.drone_interface.command_elevon(self.rr_output_function(side), origin)
        self._hold(sides, origin, settle)

        grid = creep_grid(origin, target, points)
        next_sample = 0

        for command in creep_commands(origin, target, step):
            if not self.measure_active:
                return

            for side in sides:
                self.drone_interface.command_elevon(
                    self.rr_output_function(side), command)

            # The dwell replaces this step's wait rather than adding to it: it
            # is already longer than the period.
            if next_sample < len(grid) and \
                    abs(command - grid[next_sample]) <= step / 2.0:
                self._sample_creep_point(phase, sides, rep, target, command,
                                         cal, pwm_map)
                next_sample += 1
            else:
                time.sleep(period)

        self._stiction_capture_settled(phase, sides, "creep", rep, target, settle, cal)
    # def

    def _st_show_leg(self, record):
        if self._closing:
            return

        if record["final_deg"] is None:
            text = "%s %s %s: no reading" % (
                record["side"], record["kind"], record["rep"])
        elif record["rate_deg_s"]:
            text = "%s %s %d: %.2f deg, %.0f deg/s" % (
                record["side"], record["kind"], record["rep"],
                record["final_deg"], record["rate_deg_s"])
        else:
            text = "%s %s %d: %.2f deg" % (
                record["side"], record["kind"], record["rep"], record["final_deg"])

        self.rr_leg_label.config(text=text)
    # def

    def summarise_stiction(self):
        """One row per phase/side/end: creep, swing, their difference, and rate.

        The swing half is read straight out of the range/rate legs. A leg that
        ran neg_to_pos ended at the +1 end stop and one that ran pos_to_neg
        ended at -1, so the arrival each creep is compared against is the very
        same hard-over the range/rate table reports - not a second run of them.
        """
        rows = []
        arrival = {"neg_to_pos": 1.0, "pos_to_neg": -1.0}

        for phase in PHASES:
            for side in ("LEFT", "RIGHT"):
                for target in (1.0, -1.0):
                    creeps = [r["final_deg"] for r in self.stiction_results
                              if r["phase"] == phase and r["side"] == side
                              and r["target"] == target and r["ok"]
                              and r["kind"] == "creep"]

                    legs = [r for r in self.rr_results
                            if r["phase"] == phase and r["side"] == side and r["ok"]
                            and arrival.get(r["direction"]) == target]

                    swings = [r["final_deg"] for r in legs]
                    if not creeps or not swings:
                        continue

                    stats = stiction_stats(creeps, swings)
                    rates = mean_sd([r["rate_deg_s"] for r in legs
                                     if r.get("rate_deg_s")])
                    transits = mean_sd([r["transit_s"] * 1000.0 for r in legs
                                        if r.get("transit_s")])

                    rows.append((
                        phase, side, "%+.0f" % target,
                        self.format_mean_sd((stats["creep_mean"], stats["creep_sd"]), "%.2f"),
                        self.format_mean_sd((stats["swing_mean"], stats["swing_sd"]), "%.2f"),
                        self.format_mean_sd((stats["stiction"], stats["stiction_sd"]), "%.2f"),
                        self.format_mean_sd(transits, "%.1f"),
                        self.format_mean_sd(rates, "%.0f"),
                        "%d/%d" % (len(creeps), len(swings)),
                    ))

        return rows
    # def

    # ---- Creep curve plot ----------------------------------------------------

    CURVE_ROLE_COLOURS = {
        curve_plot.ROLE_UP: "accent",
        curve_plot.ROLE_FIT_UP: "accent",
        curve_plot.ROLE_DOWN: "bad",
        curve_plot.ROLE_FIT_DOWN: "bad",
        curve_plot.ROLE_TRAM: "ink_faint",
        curve_plot.ROLE_AXIS: "ink_muted",
    }

    def show_creep_curve(self):
        """Open (or raise) the PWM/angle plot for the captured creeps."""
        if not self.creep_points:
            messagebox.showinfo("No curve", "Run a stiction measurement first.")
            return

        if self.curve_window is not None and self.curve_window.winfo_exists():
            self.curve_window.deiconify()
            self.curve_window.lift()
            self.redraw_creep_curve()
            return

        window = tk.Toplevel(self.root)
        window.title("Creep curve - PWM against angle")
        self.curve_window = window

        controls = tk.Frame(window, padx=8, pady=6)
        controls.pack(fill=tk.X)

        sides = sorted({point["side"] for point in self.creep_points})
        self.curve_side_var = tk.StringVar(value=sides[0])
        for side in sides:
            tk.Radiobutton(controls, text=side, value=side,
                           variable=self.curve_side_var,
                           command=self.redraw_creep_curve).pack(side=tk.LEFT)

        # Deviation by default: the raw curve cannot show a 1 deg band on a
        # 62 deg axis, which is the entire quantity of interest.
        self.curve_mode_var = tk.StringVar(value=curve_plot.MODE_DEVIATION)
        tk.Checkbutton(controls, text="Deviation from fit",
                       variable=self.curve_mode_var,
                       onvalue=curve_plot.MODE_DEVIATION,
                       offvalue=curve_plot.MODE_CURVE,
                       command=self.redraw_creep_curve).pack(side=tk.LEFT,
                                                             padx=(12, 0))

        for label, var, default in (("Fit order", "curve_order_var", "2"),
                                    ("Tramline deg", "curve_tol_var", "0.5")):
            tk.Label(controls, text=label).pack(side=tk.LEFT, padx=(12, 4))
            variable = tk.StringVar(value=default)
            setattr(self, var, variable)
            entry = tk.Entry(controls, textvariable=variable, width=5)
            entry.pack(side=tk.LEFT)
            entry.bind("<KeyRelease>", lambda event: self.redraw_creep_curve())

        tk.Button(controls, text="Save SVG",
                  command=self.export_creep_curve_svg).pack(side=tk.RIGHT)

        self.curve_canvas = tk.Canvas(window, width=780, height=470,
                                      bg=PALETTE["paper"], highlightthickness=1,
                                      highlightbackground=PALETTE["rule"])
        self.curve_canvas.pack(fill=tk.BOTH, expand=True, padx=8)
        self.curve_canvas.bind("<Configure>",
                               lambda event: self.redraw_creep_curve())

        self.curve_caption = tk.Label(window, text="", anchor="w", justify="left",
                                      padx=8, pady=6, fg=PALETTE["ink_muted"])
        self.curve_caption.pack(fill=tk.X)

        self.redraw_creep_curve()
    # def

    def build_creep_plot(self, width, height):
        """The geometry for the current selection, or None when unplottable."""
        if not self.creep_points:
            return None

        series = curve_series(self.creep_points, self.curve_side_var.get())
        return curve_plot.build_plot(
            series,
            order=max(1, self.get_int_var(self.curve_order_var, 2)),
            tolerance_deg=max(0.0, self.get_float_var(self.curve_tol_var, 0.5)),
            width=width, height=height, mode=self.curve_mode_var.get())
    # def

    def redraw_creep_curve(self):
        if self.curve_window is None or not self.curve_window.winfo_exists():
            return

        canvas = self.curve_canvas
        canvas.delete("all")

        width = max(320, canvas.winfo_width())
        height = max(240, canvas.winfo_height())

        plot = self.build_creep_plot(width, height)
        if plot is None:
            canvas.create_text(width / 2, height / 2, text="Nothing to plot",
                               fill=PALETTE["ink_muted"])
            return

        for line in plot["axes"]:
            self._draw_plot_line(canvas, line, 1)

        for line in plot["polylines"]:
            self._draw_plot_line(canvas, line, line.get("width", 2))

        for marker in plot["markers"]:
            x, y = marker["point"]
            colour = PALETTE[self.CURVE_ROLE_COLOURS[marker["role"]]]
            canvas.create_oval(x - 2.5, y - 2.5, x + 2.5, y + 2.5,
                               fill=colour, outline=colour)

        for text in plot["texts"]:
            anchor = {"middle": "n", "end": "e", "start": "w"}[text["anchor"]]
            canvas.create_text(text["x"], text["y"], text=text["text"],
                               anchor=anchor, fill=PALETTE["ink_muted"])

        self.curve_caption.config(text=self.describe_creep_plot(plot))
    # def

    def _draw_plot_line(self, canvas, line, width):
        colour = PALETTE[self.CURVE_ROLE_COLOURS[line["role"]]]
        flat = []
        for x, y in line["points"]:
            flat.extend((x, y))
        if len(flat) < 4:
            return
        canvas.create_line(*flat, fill=colour, width=width,
                           dash=(4, 3) if line.get("dash") else None)
    # def

    def describe_creep_plot(self, plot):
        """The numbers the picture cannot carry: fit quality and band size."""
        stats = plot["stats"]
        parts = []

        for direction, label in ((1.0, "up"), (-1.0, "down")):
            fit = stats["fits"].get(direction)
            if fit and fit["rms_deg"] is not None:
                parts.append("%s fit rms %.2f deg, worst %+.2f at %d us"
                             % (label, fit["rms_deg"], fit["max_deg"],
                                fit["max_pwm"]))

        band = stats.get("band")
        if band:
            parts.append("band mean %.2f deg, max %.2f deg at %d us"
                         % (band["mean_deg"], band["max_deg"], band["max_pwm"]))

        return "\n".join(parts) if parts else "Not enough points to fit"
    # def

    def export_creep_curve_svg(self):
        plot = self.build_creep_plot(880, 520)
        if plot is None:
            print("Nothing to plot")
            return

        try:
            side = self.curve_side_var.get()
            path = self.rr_report_path("creepcurve_%s" % side.lower())
            path = path[:-4] + ".svg"

            with open(path, "w", encoding="utf-8") as f:
                f.write(curve_plot.to_svg(plot, "Creep curve - %s" % side))

            print("Creep curve plot written to %s" % path)

        except Exception as e:
            print("Failed to export the creep curve plot: %s" % str(e))
    # def

    def export_creep_curve_csv(self):
        """Every dwelled sample from every creep, as PWM against angle.

        One row per sample rather than a summary: the shape has not been
        characterised yet, so anything that reduced it here would be guessing at
        what matters. Direction is carried per row because the up and down
        sweeps are the two halves of the hysteresis comparison and must never be
        pooled.
        """
        if not self.creep_points:
            print("No creep curve to export")
            return

        try:
            path = self.rr_report_path("creepcurve")

            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(CREEP_CURVE_COLUMNS)
                for point in self.creep_points:
                    row = [point[column] for column in CREEP_CURVE_COLUMNS]
                    writer.writerow(row)

            print("Creep curve written to %s (%d points)"
                  % (path, len(self.creep_points)))

        except Exception as e:
            print("Failed to export the creep curve: %s" % str(e))
    # def

    def export_stiction_csv(self):
        if not self.stiction_summary_rows:
            print("Nothing to export")
            return

        try:
            path = self.rr_report_path("stiction")

            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["phase", "side", "target", "creep_deg", "swing_deg",
                                 "stiction_deg", "transit_ms", "rate_deg_s", "n"])
                writer.writerows(self.stiction_summary_rows)

            print("Stiction summary written to %s" % path)

        except Exception as e:
            print("Failed to export stiction CSV: %s" % str(e))
    # def

    def _rr_status(self, text, fraction=None):
        def apply():
            if self._closing:
                return
            self.rr_status_label.config(text=text)
            if fraction is not None:
                self.rr_progress.config(value=max(0.0, min(100.0, fraction * 100.0)))
        self.post_to_gui(apply)
    # def

    def _rr_calibration(self):
        """The reader's scalers in the shape to_series() expects."""
        reader = self.position_reader
        return {
            "LEFT": {"scaler": reader.left_scaler, "offset": reader.left_offset},
            "RIGHT": {"scaler": reader.right_scaler, "offset": reader.right_offset},
        }
    # def

    def _measure_worker(self, settings):
        """Creeps first, then the hard-overs, per phase.

        One pass drives both measurements. The hard-overs are the range/rate
        test outright, and they are also the swing half of the stiction
        comparison - a cycle parks at one end and drives full span to the other,
        which is exactly the arrival the creep is being compared against. There
        is no separate swing pass, so the two summaries quote the same legs.
        """
        restore_rate = self.position_reader.sample_hz
        phases = settings["phases"]
        settle = settings["settle"]

        try:
            if self.position_reader.set_sample_rate(settings["rate"]) is None:
                self._rr_status("Pico refused the rate - is it running sampler.py?")
                self.post_to_gui(lambda: messagebox.showerror(
                    "Old firmware",
                    "The Pico did not accept a rate command, so it is probably "
                    "running the legacy 10 Hz firmware. That is far too slow to "
                    "resolve a transit.\n\nUpload pico/sampler.py as main.py and "
                    "try again."))
                return

            cal = self._rr_calibration()
            swings, creeps = self.measure_plan(settings)
            total = max(1, swings + creeps)
            done = 0

            for phase in phases:
                if not self.measure_active:
                    break

                sides = PHASE_SIDES[phase]

                if settings["stiction"]:
                    for target in (1.0, -1.0):
                        for rep in range(1, settings["creep_reps"] + 1):
                            if not self.measure_active:
                                break

                            self._rr_status(
                                "%s: creep to %+.0f, %d/%d"
                                % (phase, target, rep, settings["creep_reps"]),
                                done / float(total))

                            self._stiction_creep_leg(
                                phase, sides, rep, target, settings["creep_step"],
                                settings["creep_period"],
                                settings["creep_points"], settle, cal,
                                settings["pwm_map"])
                            done += 1

                self._rr_status("%s: parking at -1" % phase, done / float(total))
                for side in sides:
                    self.drone_interface.command_elevon(
                        self.rr_output_function(side), -1.0)
                self._hold(sides, -1.0, settle)

                for cycle in range(1, settings["cycles"] + 1):
                    for direction, target in (("neg_to_pos", 1.0),
                                              ("pos_to_neg", -1.0)):
                        if not self.measure_active:
                            break

                        self._rr_status(
                            "%s: cycle %d/%d, %s"
                            % (phase, cycle, settings["cycles"], direction),
                            done / float(total))

                        self._rr_run_leg(phase, sides, cycle, direction, target,
                                         settle, cal)

                        done += 1
                        self._rr_status(
                            "%s: cycle %d/%d, %s"
                            % (phase, cycle, settings["cycles"], direction),
                            done / float(total))

                for side in sides:
                    self.drone_interface.command_elevon(
                        self.rr_output_function(side), 0.0)

        except Exception as e:
            print("Measurement failed: %s" % str(e))
            self._rr_status("Failed: %s" % str(e))

        finally:
            # Always park the surfaces. MAV_CMD_ACTUATOR_TEST holds its value for
            # 60 s, so an abandoned run would otherwise leave them hard over.
            try:
                self.drone_interface.command_elevon(self.LEFT_OUTPUT_FUNCTION, 0.0)
                self.drone_interface.command_elevon(self.RIGHT_OUTPUT_FUNCTION, 0.0)
            except Exception as e:
                print("Failed to centre elevons after the run: %s" % str(e))

            if restore_rate <= SLOW_RATE_HZ:
                self.position_reader.set_sample_rate(SLOW_RATE_HZ)

            self.measure_active = False
            self.post_to_gui(lambda s=settings: self._measure_finish(s))
    # def

    def rr_output_function(self, side):
        return self.LEFT_OUTPUT_FUNCTION if side == "LEFT" else self.RIGHT_OUTPUT_FUNCTION
    # def

    def _rr_run_leg(self, phase, sides, cycle, direction, target, settle, cal):
        reader = self.position_reader

        # Pay MicroPython's collector stall now rather than have it land in the
        # middle of the transit. Before the capture starts, so it is not in it.
        reader.send_command("G")

        reader.start_capture()

        # Refresh the parked hold before the baseline. send_command() above is a
        # blocking serial round trip that retries for up to 4.5 s, which is long
        # enough for the actuator override to lapse and the surface to drift off
        # the very position the pre-roll is about to measure as the baseline.
        # The surface is already here - a leg is always entered parked at the far
        # end - so this moves nothing.
        for side in sides:
            self.drone_interface.command_elevon(self.rr_output_function(side), -target)

        # Baseline before the command, so t=0 has something to be measured from.
        time.sleep(PRE_ROLL_S)

        t_cmd = time.monotonic()
        for side in sides:
            self.drone_interface.command_elevon(self.rr_output_function(side), target)

        # Held, not slept: the settle is longer than the override lasts, so the
        # FC would take the surface back before the trace had finished settling
        # and the last fifth - which is what final_deg is measured from - would
        # be of a surface on its way somewhere else.
        self._hold(sides, target, settle)

        samples, truncated = reader.stop_capture()

        tracker = TickTracker()
        for _host_t, pico_us, _l, _r in samples:
            if pico_us is not None:
                tracker.feed(pico_us)

        timing = tracker.report()
        dropped = timing["dropped"] if timing else 0

        for side in PHASE_SIDES[phase]:
            series = to_series(samples, t_cmd, cal, side)
            metrics = analyse_leg(series)

            metrics.update({
                "phase": phase, "cycle": cycle, "direction": direction,
                "side": side, "dropped": dropped, "truncated": truncated,
            })
            self.rr_results.append(metrics)
            self._rr_retain_samples(phase, cycle, direction, side, series)

            self.post_to_gui(
                lambda m=metrics, s=series: self._rr_show_leg(m, s))
    # def

    def _rr_retain_samples(self, phase, cycle, direction, side, series):
        """Keep one leg's trace for a later "Save samples", up to the ceiling.

        Worker thread only. Formatting to strings here rather than at export
        keeps the retained rows flat and makes the memory cost predictable, and
        matches what range_test.py's --samples-csv writes so one set of plotting
        scripts reads both.
        """
        room = MAX_EXPORT_SAMPLES - len(self.rr_samples)

        if room <= 0:
            self.rr_samples_truncated = True
            return

        if len(series) > room:
            series = series[:room]
            self.rr_samples_truncated = True

        for t_rel, angle in series:
            self.rr_samples.append([
                phase, cycle, direction, side,
                "%.6f" % t_rel, "%.4f" % angle,
            ])
    # def

    def _rr_show_leg(self, metrics, series):
        if self._closing:
            return

        if metrics["ok"]:
            text = ("%s %s\ntravel %.2f deg\ntransit %.0f ms\nrate %.0f deg/s\ngaps %d"
                    % (metrics["side"], metrics["direction"], metrics["travel_deg"],
                       metrics["transit_s"] * 1000.0, metrics["rate_deg_s"],
                       metrics["dropped"]))
        else:
            text = "%s %s\n%s" % (metrics["side"], metrics["direction"], metrics["reason"])

        self.rr_leg_label.config(text=text)

        self.rr_last_trace = (series, metrics)
        self.redraw_range_rate_trace()
    # def

    def redraw_range_rate_trace(self):
        canvas = self.rr_canvas
        canvas.delete("all")

        trace = getattr(self, "rr_last_trace", None)
        if not trace:
            return

        series, metrics = trace
        if not series:
            return

        # Tk reports 1, not 0, for a widget it has not laid out yet, so "or" is
        # not enough - an unrealised canvas would otherwise be drawn into a
        # 1-pixel box and appear blank.
        width = canvas.winfo_width()
        height = canvas.winfo_height()

        if width <= 1:
            width = 520
        if height <= 1:
            height = 210

        left, right, top, bottom = 46, 12, 12, 26
        plot_w = max(10, width - left - right)
        plot_h = max(10, height - top - bottom)

        times = [t for t, _ in series]
        angles = [a for _, a in series]

        t0, t1 = min(times), max(times)
        a0, a1 = min(angles), max(angles)

        if t1 - t0 < 1e-6 or a1 - a0 < 1e-6:
            return

        pad = (a1 - a0) * 0.08
        a0 -= pad
        a1 += pad

        def sx(t):
            return left + ((t - t0) / (t1 - t0)) * plot_w

        def sy(a):
            return top + (1.0 - ((a - a0) / (a1 - a0))) * plot_h

        canvas.create_line(left, top, left, top + plot_h, fill="gray60")
        canvas.create_line(left, top + plot_h, left + plot_w, top + plot_h, fill="gray60")

        # Command instant. Everything left of it is pre-roll baseline.
        if t0 < 0.0 < t1:
            canvas.create_line(sx(0.0), top, sx(0.0), top + plot_h,
                               fill="gray50", dash=(2, 3))
            canvas.create_text(sx(0.0) + 3, top + 6, text="cmd",
                               anchor="w", fill="gray40")

        if metrics.get("ok"):
            baseline = metrics["baseline_deg"]
            travel = metrics["travel_deg"]

            for fraction, label in ((0.10, "10%"), (0.90, "90%")):
                y = sy(baseline + fraction * travel)
                canvas.create_line(left, y, left + plot_w, y, fill="gray75", dash=(3, 3))
                canvas.create_text(left - 4, y, text=label, anchor="e", fill="gray40")

            for key in ("latency_s", "t90_s"):
                x = sx(metrics[key])
                canvas.create_line(x, top, x, top + plot_h, fill="gray65", dash=(2, 3))

            canvas.create_text(
                left + plot_w, top + 6, anchor="e", fill="gray30",
                text="%.0f ms  %.0f deg/s" % (metrics["transit_s"] * 1000.0,
                                              metrics["rate_deg_s"]))

        points = []
        for t, a in series:
            points.extend((sx(t), sy(a)))

        if len(points) >= 4:
            canvas.create_line(*points, fill="#1d6fa5", width=2)

        canvas.create_text(left - 4, sy(a1), text="%.0f" % a1, anchor="e", fill="gray40")
        canvas.create_text(left - 4, sy(a0), text="%.0f" % a0, anchor="e", fill="gray40")
        canvas.create_text(left, top + plot_h + 12, text="%.2f s" % t0,
                           anchor="w", fill="gray40")
        canvas.create_text(left + plot_w, top + plot_h + 12, text="%.2f s" % t1,
                           anchor="e", fill="gray40")
    # def

    def _measure_finish(self, settings):
        if self._closing:
            return

        self.rr_run_btn.config(state="normal")
        self.rr_stop_btn.config(state="disabled")

        rows = self.summarise_range_rate() if settings["range_rate"] else []
        self.rr_summary_rows = rows

        for row in rows:
            self.rr_tree.insert("", tk.END, values=row)

        stiction_rows = self.summarise_stiction() if settings["stiction"] else []
        self.stiction_summary_rows = stiction_rows

        for row in stiction_rows:
            self.st_tree.insert("", tk.END, values=row)

        notes = []

        dropped = sum(r.get("dropped", 0) for r in self.rr_results)
        if dropped:
            notes.append("%d samples missing across the run" % dropped)

        failed = [r for r in self.rr_results if not r["ok"]]
        if failed:
            notes.append("%d legs did not measure (%s)"
                         % (len(failed), failed[0]["reason"]))

        if self.rr_samples_truncated:
            notes.append("sample record truncated at %d rows" % MAX_EXPORT_SAMPLES)

        self.rr_note_label.config(text="; ".join(notes))
        self.rr_export_btn.config(state="normal" if rows else "disabled")
        self.rr_samples_btn.config(state="normal" if self.rr_samples else "disabled")
        self.st_export_btn.config(state="normal" if stiction_rows else "disabled")
        enabled = "normal" if self.creep_points else "disabled"
        self.st_curve_btn.config(state=enabled)
        self.st_plot_btn.config(state=enabled)

        any_rows = bool(rows or stiction_rows)
        self.rr_progress.config(value=100.0 if any_rows else 0.0)
        self.rr_status_label.config(text="Done" if any_rows else "No measurements")
    # def

    def summarise_range_rate(self):
        """Aggregate the per-leg results into one row per phase/side/direction."""
        rows = []

        for phase in PHASES:
            for side in ("LEFT", "RIGHT"):
                ok = [r for r in self.rr_results
                      if r["phase"] == phase and r["side"] == side and r["ok"]]
                if not ok:
                    continue

                # Range is the span between the settled endpoints, the same
                # quantity whichever way it was crossed.
                ends = [r["final_deg"] for r in ok]
                span = max(ends) - min(ends)

                angle_max, angle_min = endpoint_stats(ok)

                for direction in ("neg_to_pos", "pos_to_neg"):
                    legs = [r for r in ok if r["direction"] == direction]
                    if not legs:
                        continue

                    travel = mean_sd([abs(r["travel_deg"]) for r in legs])
                    transit = mean_sd([r["transit_s"] * 1000.0 for r in legs])
                    rate = mean_sd([r["rate_deg_s"] for r in legs])

                    rows.append((
                        phase, side, direction,
                        self.format_mean_sd(angle_max, "%.2f"),
                        self.format_mean_sd(angle_min, "%.2f"),
                        "%.2f" % span,
                        self.format_mean_sd(travel, "%.2f"),
                        self.format_mean_sd(transit, "%.1f"),
                        self.format_mean_sd(rate, "%.0f"),
                        len(legs),
                    ))

        return rows
    # def

    def format_mean_sd(self, pair, fmt):
        mean, sd = pair
        if mean is None:
            return "-"
        if sd is None:
            return fmt % mean
        return (fmt + " +/- " + fmt) % (mean, sd)
    # def

    def export_range_rate_csv(self):
        if not self.rr_summary_rows:
            print("Nothing to export")
            return

        try:
            path = self.rr_report_path("rangerate")

            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["phase", "side", "direction",
                                 "angle_max_deg", "angle_min_deg", "range_deg",
                                 "travel_deg", "transit_ms", "rate_deg_s", "cycles"])
                writer.writerows(self.rr_summary_rows)

            print("Range/rate summary written to %s" % path)

        except Exception as e:
            print("Failed to export range/rate CSV: %s" % str(e))
    # def

    def rr_report_path(self, suffix):
        """reports/<name>_<run stamp>_<suffix>.csv for the run just finished.

        The stamp is the run's, not the moment the button was pressed, so the
        summary and the sample dump share a filename stem however long after the
        run either is exported.
        """
        name = self.make_safe_filename(self.drone_name_var.get().strip() or "elevon")
        stamp = self.rr_run_stamp or datetime.now().strftime("%Y%m%d_%H%M%S")
        return report_path(APP_DIR, "%s_%s_%s.csv" % (name, stamp, suffix))
    # def

    def export_range_rate_samples_csv(self):
        """Every captured sample, one row each - the aggregate's raw material.

        Same columns and same order as range_test.py --samples-csv, so anything
        that plots the console's output plots this unchanged. t_rel_s is zeroed
        on the actuator command, so pre-roll samples are negative.
        """
        if not self.rr_samples:
            print("No samples to export")
            return

        try:
            path = self.rr_report_path("rangerate_samples")

            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["phase", "cycle", "direction", "side",
                                 "t_rel_s", "angle_deg"])
                writer.writerows(self.rr_samples)

            print("Range/rate samples written to %s (%d rows)"
                  % (path, len(self.rr_samples)))

            if self.rr_samples_truncated:
                print("  NOTE: the run exceeded %d samples - this file is the "
                      "first %d and the tail is missing."
                      % (MAX_EXPORT_SAMPLES, len(self.rr_samples)))

        except Exception as e:
            print("Failed to export range/rate samples CSV: %s" % str(e))
    # def

    def create_angle_entry(self, parent, label, text_var, apply_callback):
        row = tk.Frame(parent, bd=1, relief="groove", padx=4, pady=4)
        row.pack(anchor="w", pady=3, fill=tk.X)

        lbl = tk.Label(row, text=label, width=10, anchor="w")
        lbl.pack(side=tk.LEFT, padx=(0, 5))

        entry = tk.Entry(row, textvariable=text_var, width=6)
        entry.pack(side=tk.LEFT)
        entry.bind("<Return>", lambda event: apply_callback())

        btn = tk.Button(row, text="Set", width=5, command=apply_callback)
        btn.pack(side=tk.LEFT, padx=(5, 0))

        return entry
    # def

    def build_connection_panel(self, parent):
        """Link state, laid out horizontally so it can live above the notebook.

        Deliberately outside the tabs. Which port is connected, and what rate the
        Pico is running, are global facts that every tab depends on - and a test
        tab that lets you press Run without noticing the link dropped is a trap.
        """
        group = tk.LabelFrame(parent, text="Connection", padx=6, pady=4)
        group.pack(fill=tk.X)

        self.drone_port_var = tk.StringVar(value="")
        self.pico_port_var = tk.StringVar(value="")
        self._port_by_label = {}

        # One row. Status sits beside its combobox rather than under it, so
        # moving this out of the left column costs a strip of height rather than
        # a band of it.
        tk.Label(group, text="Drone", anchor="w").pack(side=tk.LEFT)
        self.drone_combo = ttk.Combobox(group, textvariable=self.drone_port_var, width=22)
        self.drone_combo.pack(side=tk.LEFT, padx=(4, 6))

        self.drone_status_label = tk.Label(
            group, text="not connected", anchor="w", width=30, fg=PALETTE["ink"])
        self.drone_status_label.pack(side=tk.LEFT, padx=(0, 14))

        tk.Label(group, text="Pico", anchor="w").pack(side=tk.LEFT)
        self.pico_combo = ttk.Combobox(group, textvariable=self.pico_port_var, width=22)
        self.pico_combo.pack(side=tk.LEFT, padx=(4, 6))

        self.pico_status_label = tk.Label(
            group, text="not connected", anchor="w", width=28, fg=PALETTE["ink"])
        self.pico_status_label.pack(side=tk.LEFT, padx=(0, 14))

        tk.Label(group, text="Rate", anchor="w").pack(side=tk.LEFT)

        # The rate is now mutable global state - a test can leave the board in
        # fast mode, and the Trim tab reads the same stream. Showing it means a
        # surprising angle readout has somewhere to be explained.
        self.pico_rate_label = tk.Label(
            group, text="--", anchor="center", relief="flat", bd=0,
            bg=PALETTE["panel_alt"], width=8, pady=2)
        self.pico_rate_label.pack(side=tk.LEFT, padx=(4, 0))

        button_row = tk.Frame(group)
        button_row.pack(side=tk.RIGHT)

        self.refresh_btn = tk.Button(
            button_row,
            text="Refresh ports",
            width=13,
            command=self.refresh_port_list
        )
        self.refresh_btn.pack(side=tk.LEFT)

        self.connect_btn = tk.Button(
            button_row,
            text="Connect",
            width=13,
            command=self.connect_mavlink
        )
        self.connect_btn.pack(side=tk.LEFT, padx=(6, 0))
    # def

    def update_pico_rate_label(self):
        if self._closing:
            return

        if not self.position_reader.is_streaming():
            text, colour = "--", PALETTE["ink"]
        else:
            hz = self.position_reader.sample_hz
            text = "%d Hz" % hz
            # Fast mode is a temporary state owned by a running test. Flagging it
            # stops it being mistaken for normal when a test leaves it behind.
            colour = PALETTE["ok"] if hz <= SLOW_RATE_HZ else PALETTE["warn"]

        self.pico_rate_label.config(text=text, fg=colour)
    # def

    def device_from_label(self, label):
        """Map a combobox entry back to a device path, allowing hand-typed values."""
        label = (label or "").strip()
        if not label:
            return None
        if label in self._port_by_label:
            return self._port_by_label[label]
        # Typed in by hand, e.g. "/dev/ttyUSB0" or "COM7".
        return label.split(" ")[0]
    # def

    def set_conn_status(self, which, text, ok=None):
        label = self.drone_status_label if which == "drone" else self.pico_status_label
        colour = PALETTE["ink"] if ok is None else (
            PALETTE["ok"] if ok else PALETTE["bad"])
        label.config(text=text, fg=colour)
    # def

    def set_connection_controls_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        self.refresh_btn.config(state=state)
        self.connect_btn.config(state=state, text="Connect" if enabled else "Working...")
    # def

    def populate_port_combos(self, ports, drone_device=None, pico_device=None):
        """Tk thread only."""
        labels = [c.label() for c in ports]
        self._port_by_label = dict(zip(labels, [c.device for c in ports]))

        self.drone_combo.config(values=labels)
        self.pico_combo.config(values=labels)

        for device, var in ((drone_device, self.drone_port_var), (pico_device, self.pico_port_var)):
            if device is None:
                continue
            for label, dev in self._port_by_label.items():
                if dev == device:
                    var.set(label)
                    break
            else:
                var.set(device)
    # def

    def refresh_port_list(self):
        self.set_connection_controls_enabled(False)
        threading.Thread(target=self._discover_worker, args=(False,), daemon=True).start()
    # def

    def autodetect_and_connect(self):
        self.set_connection_controls_enabled(False)
        threading.Thread(target=self._discover_worker, args=(True,), daemon=True).start()
    # def

    def _discover_worker(self, auto_connect):
        """Worker thread: enumerate and probe, then hand the result back to Tk."""
        try:
            print("Scanning serial ports...")
            result = discover_ports(
                prefer_pico=self.position_reader.pico_port,
                prefer_drone=self.position_reader.drone_port,
            )
            for message in result["messages"]:
                print(message)

        except Exception as e:
            print("Port scan failed: %s" % str(e))
            result = {"ports": [], "pico": None, "drone": None, "drone_ids": None}

        self.post_to_gui(lambda: self._apply_discovery_result(result, auto_connect))
    # def

    def _apply_discovery_result(self, result, auto_connect):
        """Tk thread."""
        if self._closing:
            return

        self.populate_port_combos(result["ports"], result["drone"], result["pico"])

        if result["pico"] is None:
            self.set_conn_status("pico", "not found", False)
        if result["drone"] is None:
            self.set_conn_status("drone", "no heartbeat found", False)

        if auto_connect and (result["pico"] or result["drone"]):
            self.connect_mavlink()
        else:
            self.set_connection_controls_enabled(True)
    # def

    def connect_mavlink(self):
        drone_device = self.device_from_label(self.drone_port_var.get())
        pico_device = self.device_from_label(self.pico_port_var.get())

        self.set_connection_controls_enabled(False)

        if drone_device:
            self.set_conn_status("drone", "connecting...", None)
        if pico_device:
            self.set_conn_status("pico", "opening...", None)

        threading.Thread(
            target=self._connect_worker,
            args=(drone_device, pico_device),
            daemon=True
        ).start()
    # def

    def _connect_worker(self, drone_device, pico_device):
        """Worker thread: open both links, then report status back to Tk."""
        drone_text = "no port selected"
        drone_ok = False

        try:
            if drone_device:
                print("Connecting MAVLink on %s..." % drone_device)
                if self.drone_interface.reconnect(drone_device):
                    drone_ok = True
                    drone_text = "connected (system %s)" % self.drone_interface.master.target_system
                    # Remember the port before the long param refresh, so a
                    # mid-refresh failure doesn't lose a known-good port.
                    self.position_reader.set_remembered_ports(drone_port=drone_device)
                    self.refresh_params_from_drone(clear_name=True)
                elif self.drone_interface.last_error_kind == "open":
                    drone_text = "cannot open %s" % drone_device
                else:
                    drone_text = "no heartbeat on %s" % drone_device

        except Exception as e:
            drone_text = "connect failed"
            print("MAVLink connect failed: %s" % str(e))

        pico_text = "no port selected"
        pico_ok = False

        try:
            if pico_device:
                print("Opening Pico on %s..." % pico_device)
                self.position_reader.set_port(pico_device)

                # Give the reader a moment to produce its first sample.
                deadline = time.time() + 2.0
                while time.time() < deadline and not self.position_reader.is_streaming():
                    time.sleep(0.1)

                if self.position_reader.is_streaming():
                    pico_ok = True
                    pico_text = "streaming on %s" % pico_device
                    self.position_reader.set_remembered_ports(pico_port=pico_device)
                elif self.position_reader.last_error is not None:
                    pico_text = "cannot open %s" % pico_device
                else:
                    pico_text = "no data on %s" % pico_device

        except Exception as e:
            pico_text = "open failed"
            print("Pico open failed: %s" % str(e))

        def finish():
            if self._closing:
                return
            self.set_conn_status("drone", drone_text, drone_ok)
            self.set_conn_status("pico", pico_text, pico_ok)
            self.set_connection_controls_enabled(True)

        self.post_to_gui(finish)
    # def

    def post_to_gui(self, fn):
        """Queue a callable to run on the Tk thread. Safe from any thread.

        Deliberately not root.after() - that registers a Tcl command and is not
        safe to call from a worker thread.
        """
        self._gui_queue.put(fn)
    # def

    def _drain_gui_queue(self):
        if self._closing:
            return

        while True:
            try:
                fn = self._gui_queue.get_nowait()
            except queue.Empty:
                break

            try:
                fn()
            except Exception as e:
                print("GUI update failed: %s" % str(e))

        self.root.after(50, self._drain_gui_queue)
    # def

    def set_var_on_gui_thread(self, text_var, value):
        self.post_to_gui(lambda: text_var.set(value))
    # def

    def call_on_gui_thread(self, fn, timeout=2.0):
        """Run fn on the Tk thread and return its result. NEVER call from the Tk thread.

        For the few places a worker must *read* a Tk variable. The timeout is
        load-bearing: once the window closes, _drain_gui_queue stops rescheduling
        and an unbounded wait would hang the calling worker through shutdown.
        """
        if self._closing:
            return None

        box = queue.Queue(maxsize=1)

        def runner():
            try:
                box.put(("ok", fn()))
            except Exception as e:
                box.put(("err", e))
        # def

        self.post_to_gui(runner)

        try:
            kind, value = box.get(timeout=timeout)
        except queue.Empty:
            print("GUI round-trip timed out")
            return None

        if kind == "err":
            print("GUI round-trip failed: %s" % str(value))
            return None

        return value
    # def

    def refresh_params_from_drone(self, clear_name=False):
        """Read UID and the six elevon params from the FCU and populate the UI.

        Safe to call from a worker thread: every widget write is marshalled
        onto the Tk thread. Returns False if there is no link.
        """
        if not self.drone_interface.is_connected():
            print("Not connected - keeping current parameter values")
            return False

        try:
            ident = self.drone_interface.get_id()
            self.set_var_on_gui_thread(self.uid_var, "--" if ident is None else str(ident))
            print("PX4 UID = %s" % str(ident))

            if clear_name:
                self.set_var_on_gui_thread(self.drone_name_var, "")
                self.position_reader.drone_name = ""
                print("Drone name cleared after connect")

            left_min_param = self.drone_interface.get_param(self.LEFT_MIN_PARAM, int)
            left_max_param = self.drone_interface.get_param(self.LEFT_MAX_PARAM, int)
            left_trim_param = self.drone_interface.get_param(self.LEFT_TRIM_PARAM, float)
            right_min_param = self.drone_interface.get_param(self.RIGHT_MIN_PARAM, int)
            right_max_param = self.drone_interface.get_param(self.RIGHT_MAX_PARAM, int)
            right_trim_param = self.drone_interface.get_param(self.RIGHT_TRIM_PARAM, float)
            main_rev_param = self.drone_interface.get_param("PWM_MAIN_REV", int)

            if left_min_param is not None:
                self.set_var_on_gui_thread(self.left_min_var, str(int(left_min_param)))
            if left_max_param is not None:
                self.set_var_on_gui_thread(self.left_max_var, str(int(left_max_param)))
            if left_trim_param is not None:
                self.set_var_on_gui_thread(self.left_trim_var, "%.3f" % float(left_trim_param))

            if right_min_param is not None:
                self.set_var_on_gui_thread(self.right_min_var, str(int(right_min_param)))
            if right_max_param is not None:
                self.set_var_on_gui_thread(self.right_max_var, str(int(right_max_param)))
            if right_trim_param is not None:
                self.set_var_on_gui_thread(self.right_trim_var, "%.3f" % float(right_trim_param))

            if main_rev_param is not None:
                self.main_rev = int(main_rev_param)

            print("PWM_MAIN_REV = %d (0x%X)" % (self.main_rev, self.main_rev))
            print("MAIN5 reversed = %s" % str(((self.main_rev >> 4) & 1) != 0))
            print("MAIN6 reversed = %s" % str(((self.main_rev >> 5) & 1) != 0))
            return True

        except Exception as e:
            print("Parameter refresh failed: %s" % str(e))
            return False
    # def

    def zero_both_angles(self):
        print("Zeroing both angles...")
        self.centre_left()
        self.centre_right()
    # def

    def start_both_calibration(self):
        print("Starting automatic calibration on both sides...")
        self.zero_both_sliders()
        self._start_left_calibration_worker()
        self._start_right_calibration_worker()
    # def

    def stop_both_calibration(self):
        print("Stopping automatic calibration on both sides...")
        self.stop_left_calibration()
        self.stop_right_calibration()
    # def

    def apply_drone_name(self):
        try:
            name = self.drone_name_var.get().strip()
            self.position_reader.drone_name = str(name)
            print("drone_name = %s" % name)
        except Exception as e:
            print("Invalid drone name: %s" % str(e))
    # def

    def apply_angle_neg(self):
        try:
            self.angle_neg_degs = float(self.angle_neg_var.get())
            self.position_reader.set_angle_settings(angle_neg_degs=self.angle_neg_degs)
            print("angle_neg_degs = %.2f" % self.angle_neg_degs)
        except Exception as e:
            print("Invalid angle_neg: %s" % str(e))
    # def

    def apply_angle_pos(self):
        try:
            self.angle_pos_degs = float(self.angle_pos_var.get())
            self.position_reader.set_angle_settings(angle_pos_degs=self.angle_pos_degs)
            print("angle_pos_degs = %.2f" % self.angle_pos_degs)
        except Exception as e:
            print("Invalid angle_pos: %s" % str(e))
    # def

    def apply_angle_trim(self):
        try:
            self.angle_trim_degs = float(self.angle_trim_var.get())
            self.position_reader.set_angle_settings(angle_trim_degs=self.angle_trim_degs)
            print("angle_trim_degs = %.2f" % self.angle_trim_degs)
        except Exception as e:
            print("Invalid angle_trim: %s" % str(e))
    # def

    def log_calibration(self):
        try:
            drone_name = self.drone_name_var.get().strip()
            if drone_name == "":
                print("Cannot log calibration: Drone name has not been set")
                return

            now = datetime.now()

            date_str = now.strftime("%Y-%m-%d")
            time_str = now.strftime("%H:%M:%S")

            uid_text = self.uid_var.get().strip()
            uid = ""

            if uid_text != "" and uid_text != "--":
                uid = str(int(uid_text))

            angle_neg = float(self.angle_neg_var.get().strip())
            angle_pos = float(self.angle_pos_var.get().strip())
            angle_trim = float(self.angle_trim_var.get().strip())

            left_min = int(self.left_min_var.get().strip())
            left_max = int(self.left_max_var.get().strip())
            left_trim = float(self.left_trim_var.get().strip())

            right_min = int(self.right_min_var.get().strip())
            right_max = int(self.right_max_var.get().strip())
            right_trim = float(self.right_trim_var.get().strip())

            folding = self.folding_var.get().strip()

            row = [
                date_str,
                time_str,
                drone_name,
                uid,
                angle_neg,
                angle_pos,
                angle_trim,
                left_min,
                left_max,
                left_trim,
                right_min,
                right_max,
                right_trim,
                folding,
            ]

            # The header and the row drifted apart once already; refuse rather
            # than silently append a misaligned line. Not an assert - asserts
            # vanish under -O and would be swallowed by the except below.
            if len(row) != len(CAL_LOG_COLUMNS):
                print("Refusing to log: %d values for %d columns"
                      % (len(row), len(CAL_LOG_COLUMNS)))
                return

            file_exists = os.path.exists(self.calibration_log_file)

            with open(self.calibration_log_file, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)

                if not file_exists:
                    writer.writerow(CAL_LOG_COLUMNS)

                writer.writerow(row)

            print("Calibration logged to %s" % self.calibration_log_file)

        except Exception as e:
            print("Failed to log calibration: %s" % str(e))
    # def

    def _set_side_param_vars_on_gui_thread(self, side, min_val=None, max_val=None, trim_val=None):
        def do_update():
            if side == "LEFT":
                if min_val is not None:
                    self.left_min_var.set(str(int(min_val)))
                if max_val is not None:
                    self.left_max_var.set(str(int(max_val)))
                if trim_val is not None:
                    self.left_trim_var.set("%.3f" % float(trim_val))
            elif side == "RIGHT":
                if min_val is not None:
                    self.right_min_var.set(str(int(min_val)))
                if max_val is not None:
                    self.right_max_var.set(str(int(max_val)))
                if trim_val is not None:
                    self.right_trim_var.set("%.3f" % float(trim_val))
        # def

        # post_to_gui, not root.after: after() registers a Tcl command and is
        # not safe to call from a worker thread. All 12 callers of this are
        # calibration workers.
        self.post_to_gui(do_update)
    # def

    def refresh_side_param_vars_from_drone(self, side):
        if side == "LEFT":
            min_param = self.LEFT_MIN_PARAM
            max_param = self.LEFT_MAX_PARAM
            trim_param = self.LEFT_TRIM_PARAM
        elif side == "RIGHT":
            min_param = self.RIGHT_MIN_PARAM
            max_param = self.RIGHT_MAX_PARAM
            trim_param = self.RIGHT_TRIM_PARAM
        else:
            return

        min_val = self.drone_interface.get_param(min_param, int)
        max_val = self.drone_interface.get_param(max_param, int)
        trim_val = self.drone_interface.get_param(trim_param, float)

        self._set_side_param_vars_on_gui_thread(
            side,
            min_val=min_val,
            max_val=max_val,
            trim_val=trim_val
        )
    # def

    def set_side_param_and_refresh(self, side, param_name, py_type, value):
        """Write a parameter and show what the FC read back. False if it failed.

        Callers must check the result. A failed MIN or MAX write leaves the
        calibration computing endpoints against a span the FC is not using, and
        every number after it is wrong with nothing to show for it.
        """
        ok = self.drone_interface.set_param_value(param_name, py_type, value)
        if not ok:
            print("%s calibration: failed to write %s" % (side, param_name))
        self.refresh_side_param_vars_from_drone(side)
        return ok
    # def

    def create_param_entry(self, parent, key, label, text_var, apply_callback):
        row = tk.Frame(parent)
        row.pack(anchor="w", pady=2)

        lbl = tk.Label(row, text=label, width=8, anchor="w")
        lbl.pack(side=tk.LEFT, padx=(0, 5))

        entry = tk.Entry(row, textvariable=text_var, width=8)
        entry.pack(side=tk.LEFT)
        entry.bind("<Return>", lambda event: apply_callback())

        btn = tk.Button(row, text="Set", width=5, command=apply_callback)
        btn.pack(side=tk.LEFT, padx=(5, 0))

        # Kept so a write in flight can lock its own field - see start_param_write.
        self._param_widgets[key] = (entry, btn)

        return entry
    # def

    def nudge_slider(self, slider, delta, vmin, vmax):
        value = float(slider.get()) + float(delta)
        if value < vmin:
            value = vmin
        if value > vmax:
            value = vmax
        slider.set(round(value, 2))
    # def

    def center_slider(self, slider):
        slider.set(0.0)
    # def

    def create_slider(self, parent, label, callback, vmin, vmax, default, resolution):
        container = tk.Frame(parent)
        container.pack(side=tk.LEFT, padx=10)

        lbl = tk.Label(container, text=label)
        lbl.pack()

        slider = tk.Scale(
            container,
            from_=vmax,
            to=vmin,
            orient=tk.VERTICAL,
            length=300,
            resolution=resolution,
            command=callback
        )
        slider.pack()
        slider.set(default)

        btn_frame = tk.Frame(container)
        btn_frame.pack(pady=(5, 0))

        btn_plus = tk.Button(
            btn_frame,
            text="+0.01",
            width=5,
            command=lambda: self.nudge_slider(slider, 0.01, vmin, vmax)
        )
        btn_plus.pack(pady=1)

        btn_center = tk.Button(
            btn_frame,
            text="0.0",
            width=4,
            command=lambda: self.center_slider(slider)
        )
        btn_center.pack(pady=1)

        btn_minus = tk.Button(
            btn_frame,
            text="-0.01",
            width=5,
            command=lambda: self.nudge_slider(slider, -0.01, vmin, vmax)
        )
        btn_minus.pack(pady=1)

        return slider
    # def

    def parse_int_param_entry(self, text_var):
        text = text_var.get().strip()
        value = int(text)
        if value < 500:
            value = 500
        if value > 2500:
            value = 2500
        text_var.set(str(value))
        return value
    # def

    def parse_trim_param_entry(self, text_var):
        text = text_var.get().strip()
        value = float(text)
        if value < -1.0:
            value = -1.0
        if value > 1.0:
            value = 1.0
        text_var.set("%.3f" % value)
        return value
    # def

    def get_int_var(self, text_var, default):
        try:
            return int(text_var.get().strip())
        except Exception:
            return default
    # def

    def get_float_var(self, text_var, default):
        try:
            return float(text_var.get().strip())
        except Exception:
            return default
    # def

    def clamp(self, value, vmin, vmax):
        if value < vmin:
            return vmin
        if value > vmax:
            return vmax
        return value
    # def

    def is_main_channel_reversed(self, channel_index_1_based):
        bit_index = int(channel_index_1_based) - 1
        return ((self.main_rev >> bit_index) & 0x1) != 0
    # def

    def expected_pwm(self, cmd, pwm_min, pwm_max, trim, rev):
        effective_cmd = self.clamp(float(cmd) + float(trim), -1.0, 1.0)
        if rev:
            pwm = float(pwm_max) - ((effective_cmd + 1.0) * 0.5 * (float(pwm_max) - float(pwm_min)))
        else:
            pwm = float(pwm_min) + ((effective_cmd + 1.0) * 0.5 * (float(pwm_max) - float(pwm_min)))
        return int(round(pwm))
    # def

    def zero_side_slider(self, side):
        if side == "LEFT":
            self.left_pos.set(0.0)
            self.left_pos.update_idletasks()
            print("Left slider zeroed")
        elif side == "RIGHT":
            self.right_pos.set(0.0)
            self.right_pos.update_idletasks()
            print("Right slider zeroed")
    # def

    def zero_both_sliders(self):
        self.zero_side_slider("LEFT")
        self.zero_side_slider("RIGHT")
    # def

    def centre_left(self):
        print("Centering LEFT...")
        self.position_reader.set_center("LEFT")
    # def

    def centre_right(self):
        print("Centering RIGHT...")
        self.position_reader.set_center("RIGHT")
    # def

    def get_side_angle(self, side):
        if side == "LEFT":
            return self.get_left_value()
        elif side == "RIGHT":
            return self.get_right_value()
        return None
    # def

    def read_side_param_snapshot(self, side):
        """Read a side's min/max/trim entries. Tk thread only."""
        if side == "LEFT":
            return (
                self.get_int_var(self.left_min_var, self.DEFAULT_PWM_MIN),
                self.get_int_var(self.left_max_var, self.DEFAULT_PWM_MAX),
                self.get_float_var(self.left_trim_var, self.DEFAULT_TRIM),
            )
        return (
            self.get_int_var(self.right_min_var, self.DEFAULT_PWM_MIN),
            self.get_int_var(self.right_max_var, self.DEFAULT_PWM_MAX),
            self.get_float_var(self.right_trim_var, self.DEFAULT_TRIM),
        )
    # def

    def load_endpoint_params(self, side, min_param, max_param, pwm_neg, pwm_pos):
        """Write the measured endpoints as the side's PWM min/max.

        PX4 has no inverted-range concept: mixer_module swaps MIN and MAX at
        param load if MIN > MAX, and travel direction comes solely from
        PWM_MAIN_REV. So the endpoints go in numerically, and the reverse bit is
        only used to sanity check which end the positive command landed on.
        """
        if pwm_neg is None or pwm_pos is None:
            print("%s automatic calibration could not convert the endpoints to PWM" % side)
            return False

        rev = self.is_main_channel_reversed(5 if side == "LEFT" else 6)

        # Not reversed: a positive command drives towards the higher PWM.
        if (pwm_pos > pwm_neg) != (not rev):
            print("%s automatic calibration WARNING: reversed=%s, but the positive "
                  "endpoint measured %d against %d for the negative. Check the channel "
                  "wiring and PWM_MAIN_REV." % (side, str(rev), pwm_pos, pwm_neg))

        pwm_min = min(pwm_neg, pwm_pos)
        pwm_max = max(pwm_neg, pwm_pos)

        print("%s automatic calibration loading Min[%d] Max[%d]" % (side, pwm_min, pwm_max))

        # Checked, not fired and forgotten: everything downstream assumes the FC
        # is using this span.
        if not self.set_side_param_and_refresh(side, min_param, int, pwm_min):
            return False
        return self.set_side_param_and_refresh(side, max_param, int, pwm_max)
    # def

    def get_side_expected_pwm(self, side, cmd):
        """Called from the calibration workers, so the Tk reads are marshalled.

        A snapshot taken once at worker start would be wrong here: the worker
        rewrites min/max/trim part-way through its own run.
        """
        if cmd is None:
            return None

        snapshot = self.call_on_gui_thread(lambda: self.read_side_param_snapshot(side))
        if snapshot is None:
            return None

        pwm_min, pwm_max, trim = snapshot
        rev = self.is_main_channel_reversed(5 if side == "LEFT" else 6)

        return self.expected_pwm(cmd, pwm_min, pwm_max, trim, rev)
    # def

    def is_calibration_active(self, side):
        if side == "LEFT":
            return self.left_cal_active
        elif side == "RIGHT":
            return self.right_cal_active
        return False
    # def

    def actuator_test_rejected(self, side, result):
        """True if PX4 explicitly refused the actuator test.

        A missing ACK is only warned about: a congested or lossy link should not
        abort a calibration that is otherwise working.
        """
        if result is None or result == DroneInterface.MAV_RESULT_ACCEPTED:
            return False

        if result == DroneInterface.ACK_NOT_RECEIVED:
            print("%s calibration move: no ACK for the actuator test, continuing" % side)
            return False

        description = self.drone_interface.describe_mav_result(result)

        print("%s calibration move: PX4 refused the actuator test (%s). Check that the "
              "vehicle is disarmed, the safety switch is off, and COM_MOT_TEST_EN is 1." %
              (side, description))
        return True
    # def

    def move_elevon_to_angle(self, side, output_function, target_angle_deg, inc_angle_deg):
        print("%s calibration move: centering elevon to 0.0 command" % side)
        result = self.drone_interface.command_elevon(output_function, 0.0, wait_ack=True)

        if self.actuator_test_rejected(side, result):
            return None

        time.sleep(1.0)

        cmd = 0.0
        ticks = 0
        deadline = time.time() + 60.0

        while time.time() < deadline:
            if not self.is_calibration_active(side):
                print("%s calibration move: stopped by user" % side)
                self.drone_interface.command_elevon(output_function, 0.0)
                return None

            angle_deg = self.get_side_angle(side)

            if angle_deg is None:
                time.sleep(0.25)
                continue

            print("%s calibration move: cmd=%.3f angle_deg=%.2f deg target=%.2f deg" %
                  (side, cmd, angle_deg, target_angle_deg))

            reached = False
            if target_angle_deg < 0.0 and angle_deg <= target_angle_deg:
                reached = True
            elif target_angle_deg > 0.0 and angle_deg >= target_angle_deg:
                reached = True
            elif abs(target_angle_deg) < 1e-6 and abs(angle_deg) <= 0.5:
                reached = True

            if reached:
                print("%s calibration move: target reached, cmd=%.3f" % (side, cmd))
                return cmd

            if abs(target_angle_deg - angle_deg) > 20.0:
                cmd += (inc_angle_deg * 3.0)
            else:
                cmd += inc_angle_deg

            if cmd < -1.0:
                cmd = -1.0
            if cmd > 1.0:
                cmd = 1.0

            ticks += 1
            check_ack = (ticks % 8 == 0)
            result = self.drone_interface.command_elevon(
                output_function, cmd, wait_ack=check_ack, ack_timeout=0.3)

            if check_ack and self.actuator_test_rejected(side, result):
                return None

            if (inc_angle_deg < 0.0 and cmd <= -1.0) or (inc_angle_deg > 0.0 and cmd >= 1.0):
                print("%s calibration move: hit command limit before reaching target" % side)
                return None

            time.sleep(0.25)

        print("%s calibration move: timed out before reaching target" % side)
        return None
    # def

    # ---- End stop calibration (the procedure in SERVO_SETTING.md) ------------

    def side_calibration_params(self, side):
        if side == "LEFT":
            return (self.LEFT_OUTPUT_FUNCTION, self.LEFT_MIN_PARAM,
                    self.LEFT_MAX_PARAM, self.LEFT_TRIM_PARAM)
        return (self.RIGHT_OUTPUT_FUNCTION, self.RIGHT_MIN_PARAM,
                self.RIGHT_MAX_PARAM, self.RIGHT_TRIM_PARAM)
    # def

    def set_calibration_active(self, side, active):
        if side == "LEFT":
            self.left_cal_active = active
        else:
            self.right_cal_active = active
    # def

    def _cal_measure(self, side, output_function, command,
                     dwell_s=ENDPOINT_DWELL_S):
        """Drive to a command in one motion, hold, and read the settled angle.

        The dwell is longer than POSITION_WINDOW_S so the trailing mean contains
        only post-move samples, and the hold re-sends the command so the FC
        cannot take the surface back part way through - the override lapses
        after about 2 s whatever timeout is requested.

        Short by design. Lingering on a surface that may be against a mechanical
        stop is exactly what the back-off probe exists to avoid.
        """
        self.drone_interface.command_elevon(output_function, command)
        self._hold([side], command, dwell_s)
        return self.get_side_angle(side)
    # def

    def _cal_probe_breakaway(self, side, output_function, which, span,
                             rev, cmd_endpoint, angle_at_endpoint):
        """Back off inward until the elevon moves. (microseconds, angle there).

        Answers two questions at once. Whether the surface still has authority
        at this end stop - if a range of PWM values all produce the same angle
        it is jammed and the value written there means nothing - and what the
        local deg/us gain is, which is what the correction needs. Taking the
        gain from the probe rather than assuming it means the sign falls out of
        the measurement, so nothing has to know the linkage direction.
        """
        pwm_min, pwm_max = span
        backed = 0.0

        while backed < BACKOFF_CEILING_US:
            if not self.is_calibration_active(side):
                return None, None

            backed += BACKOFF_STEP_US
            delta = command_delta_for_pwm(inward_sign(which) * backed,
                                          pwm_min, pwm_max, rev)
            command = self.clamp(cmd_endpoint + delta, -1.0, 1.0)

            angle = self._cal_measure(side, output_function, command)
            if angle is None:
                print("%s calibration: no angle while probing %s" % (side, which))
                return None, None

            if abs(angle - angle_at_endpoint) >= MOVEMENT_THRESHOLD_DEG:
                return backed, angle

        return None, None
    # def

    def _cal_evaluate_endpoint(self, side, output_function, which, pwm_now,
                               targets, span, rev):
        """One rapid approach: measure, probe if it is worth it, and decide."""
        command = endpoint_command(which, rev)
        target_deg = targets[which]
        other_deg = targets["MIN" if which == "MAX" else "MAX"]

        angle = self._cal_measure(side, output_function, command)
        if angle is None:
            print("%s calibration: no angle at the %s end stop" % (side, which))
            return None

        # Past target: it has travel to spare, so there is nothing a stop check
        # could tell us and the correction is already determined. Pull it in on
        # the average gain and measure properly on the next pass, which lands
        # near target.
        if overshot_target(angle, target_deg, other_deg):
            gain = nominal_gain_deg_per_us(targets["MAX"], targets["MIN"],
                                           span[0], span[1])
            shift = correction_us(angle, target_deg, gain)
            result = {"which": which, "angle_deg": angle,
                      "target_deg": target_deg, "hard_stop": False,
                      "breakaway_us": None, "probed": False, "gain": gain,
                      "new_pwm": None, "accepted": False}

            if shift is None:
                print("%s calibration: %s has no usable span" % (side, which))
                return result

            result["new_pwm"] = clamp_endpoint(pwm_now + shift)
            print("%s calibration: %s at %+.2f deg, %+.2f past target - "
                  "shifting %+.0f us without a stop check"
                  % (side, which, angle, angle - target_deg, shift))
            return result

        breakaway, angle_backed = self._cal_probe_breakaway(
            side, output_function, which, span, rev, command, angle)

        if not self.is_calibration_active(side):
            return None

        is_hard_stop, pull_in = hard_stop_verdict(breakaway)
        gain = (None if breakaway is None
                else angle_gain_per_us(angle, angle_backed, breakaway, which))

        result = {"which": which, "angle_deg": angle, "target_deg": target_deg,
                  "hard_stop": is_hard_stop, "breakaway_us": breakaway,
                  "probed": True, "gain": gain, "new_pwm": None,
                  "accepted": endpoint_accepted(angle, target_deg, is_hard_stop)}

        if result["accepted"]:
            print("%s calibration: %s at %+.2f deg (target %+.2f) - set"
                  % (side, which, angle, target_deg))
            return result

        if is_hard_stop:
            result["new_pwm"] = clamp_endpoint(pwm_now + inward_sign(which) * pull_in)
            print("%s calibration: %s is against a stop - %s of inward travel did "
                  "not move it. Pulling in %.0f us."
                  % (side, which,
                     "no amount" if breakaway is None else "%.0f us" % breakaway,
                     pull_in))
            return result

        shift = correction_us(angle, target_deg, gain)
        if shift is None:
            print("%s calibration: %s gave no usable gain" % (side, which))
            return result

        result["new_pwm"] = clamp_endpoint(pwm_now + shift)
        print("%s calibration: %s at %+.2f deg, %+.2f out - shifting %+.0f us"
              % (side, which, angle, angle - target_deg, shift))
        return result
    # def

    def _cal_refine_endpoints(self, side, output_function, min_param, max_param,
                              endpoints, rev):
        """Alternate MAX, MIN until both are set. Endpoints, or None on failure.

        Alternating costs nothing: the traverse across to the other end *is* the
        next rapid approach. It is also primed with one unmeasured traverse, so
        the opening measurement has the same full-span run-up as every later
        one - approach distance was measured to be worth 1.7 deg.
        """
        pwm = {which: value for which, (value, _target) in endpoints.items()}
        targets = {which: target for which, (_value, target) in endpoints.items()}

        attempts = {"MAX": 0, "MIN": 0}
        accepted = {"MAX": False, "MIN": False}

        self._cal_measure(side, output_function, endpoint_command("MIN", rev))

        for which in alternating_order(MAX_ATTEMPTS):
            if accepted["MAX"] and accepted["MIN"]:
                break
            if accepted[which]:
                continue
            if not self.is_calibration_active(side):
                return None

            attempts[which] += 1
            if attempts[which] > MAX_ATTEMPTS:
                print("%s calibration: %s did not settle in %d attempts"
                      % (side, which, MAX_ATTEMPTS))
                return None

            span = (pwm["MIN"], pwm["MAX"])
            result = self._cal_evaluate_endpoint(
                side, output_function, which, pwm[which], targets, span, rev)

            if result is None:
                return None

            if result["accepted"]:
                accepted[which] = True
                continue

            if result["new_pwm"] is None:
                return None

            pwm[which] = result["new_pwm"]
            param = max_param if which == "MAX" else min_param
            if not self.set_side_param_and_refresh(side, param, int, pwm[which]):
                return None

        if not (accepted["MAX"] and accepted["MIN"]):
            print("%s calibration: ran out of attempts" % side)
            return None

        return pwm
    # def

    def _cal_verify_endpoints(self, side, output_function, pwm, targets, rev):
        """Re-measure both end stops after the fact and report the error.

        A calibration marking its own work proves nothing. This drives each end
        once more, from the far side, and says what it actually reached - which
        is the number that says whether the end stops are repeatable.
        """
        print("%s calibration: verifying" % side)
        worst = 0.0

        # Primed like the refinement, and for the same reason: the refinement
        # leaves the surface next to whichever end it finished on, so an
        # unprimed verify would measure that one after a few tens of
        # microseconds of travel instead of a full-span run-up, and read it
        # short by the stiction it is supposed to be checking for.
        self._cal_measure(side, output_function, endpoint_command("MIN", rev))

        for which in ("MAX", "MIN"):
            if not self.is_calibration_active(side):
                return None
            angle = self._cal_measure(side, output_function,
                                      endpoint_command(which, rev))
            if angle is None:
                print("%s verify: no angle at %s" % (side, which))
                return None

            error = angle - targets[which]
            worst = max(worst, abs(error))
            print("%s verify: %s pwm %d reached %+.2f deg, target %+.2f, "
                  "error %+.2f%s"
                  % (side, which, pwm[which], angle, targets[which], error,
                     "" if abs(error) <= ENDPOINT_TOLERANCE_DEG else "  OUT"))

        return worst
    # def

    def _calibration_worker(self, side):
        """Coarse creep to get close, then set each end stop by rapid approach.

        The creep is only ever used to get roughly into range; nothing it
        measures is committed. An angle held while creeping up to it is not the
        angle that PWM produces when the surface is driven there normally - on
        this airframe the two differ by about 4 deg - so every value written
        here is measured the way the surface is actually used.

        The trim stage below is unchanged and still runs on the narrowed range.
        It is known to eat travel and is deliberately left alone until the end
        stops are repeatable; see SERVO_SETTING.md.
        """
        self.set_calibration_active(side, True)
        output_function, min_param, max_param, trim_param = \
            self.side_calibration_params(side)
        rev = self.is_main_channel_reversed(5 if side == "LEFT" else 6)

        try:
            print("%s automatic calibration started" % side)

            if not self.set_side_param_and_refresh(side, min_param, int, COARSE_MIN):
                return
            if not self.set_side_param_and_refresh(side, max_param, int, COARSE_MAX):
                return
            if not self.set_side_param_and_refresh(side, trim_param, float, 0.0):
                return

            if not self.is_calibration_active(side):
                print("%s automatic calibration stopped" % side)
                return

            cmd_neg = self.move_elevon_to_angle(side, output_function,
                                                self.angle_neg_degs, -0.01)
            if not self.is_calibration_active(side) or cmd_neg is None:
                print("%s automatic calibration stopped before negative endpoint completed" % side)
                return
            pwm_neg = self.get_side_expected_pwm(side, cmd_neg)

            cmd_pos = self.move_elevon_to_angle(side, output_function,
                                                self.angle_pos_degs, 0.01)
            if not self.is_calibration_active(side) or cmd_pos is None:
                print("%s automatic calibration stopped before positive endpoint completed" % side)
                return
            pwm_pos = self.get_side_expected_pwm(side, cmd_pos)

            if not self.load_endpoint_params(side, min_param, max_param,
                                             pwm_neg, pwm_pos):
                return

            # Which end carries which angle target comes from the measurement,
            # not from the reverse bit: on a reversed channel the negative angle
            # sits at the high PWM end, and assuming otherwise mirrors the
            # calibration silently.
            if pwm_neg >= pwm_pos:
                endpoints = {"MAX": (max(pwm_neg, pwm_pos), self.angle_neg_degs),
                             "MIN": (min(pwm_neg, pwm_pos), self.angle_pos_degs)}
            else:
                endpoints = {"MAX": (max(pwm_neg, pwm_pos), self.angle_pos_degs),
                             "MIN": (min(pwm_neg, pwm_pos), self.angle_neg_degs)}

            targets = {which: target for which, (_v, target) in endpoints.items()}

            pwm = self._cal_refine_endpoints(side, output_function, min_param,
                                             max_param, endpoints, rev)
            if pwm is None:
                print("%s automatic calibration did not set the end stops" % side)
                return

            print("%s end stops set: MIN=%d MAX=%d" % (side, pwm["MIN"], pwm["MAX"]))
            self._cal_verify_endpoints(side, output_function, pwm, targets, rev)

            if not self.is_calibration_active(side):
                return

            trim_step = -0.01 if self.angle_trim_degs < 0.0 else 0.01
            cmd_trim = self.move_elevon_to_angle(side, output_function,
                                                 self.angle_trim_degs, trim_step)

            if not self.is_calibration_active(side) or cmd_trim is None:
                print("%s automatic calibration stopped before trim completed" % side)
                return

            print("%s automatic calibration loading Trim[%.2f]" % (side, cmd_trim))
            self.set_side_param_and_refresh(side, trim_param, float, cmd_trim)

            print("%s automatic calibration finished" % side)
        finally:
            self.set_calibration_active(side, False)
            self.drone_interface.command_elevon(output_function, 0.0)
    # def

    def _left_calibration_worker(self):
        self._calibration_worker("LEFT")
    # def

    def _right_calibration_worker(self):
        self._calibration_worker("RIGHT")
    # def

    def _start_left_calibration_worker(self):
        if self.left_cal_thread is not None and self.left_cal_thread.is_alive():
            print("Left automatic calibration already running")
            return

        self.left_cal_active = True
        self.left_cal_thread = threading.Thread(
            target=self._left_calibration_worker,
            daemon=True
        )
        self.left_cal_thread.start()
    # def

    def _start_right_calibration_worker(self):
        if self.right_cal_thread is not None and self.right_cal_thread.is_alive():
            print("Right automatic calibration already running")
            return

        self.right_cal_active = True
        self.right_cal_thread = threading.Thread(
            target=self._right_calibration_worker,
            daemon=True
        )
        self.right_cal_thread.start()
    # def

    def start_left_calibration(self):
        self.zero_side_slider("LEFT")
        self._start_left_calibration_worker()
    # def

    def start_right_calibration(self):
        self.zero_side_slider("RIGHT")
        self._start_right_calibration_worker()
    # def

    def stop_left_calibration(self):
        if self.left_cal_active:
            print("Stopping LEFT automatic calibration...")
        else:
            print("LEFT automatic calibration is not running")
        self.left_cal_active = False
        try:
            self.drone_interface.command_elevon(self.LEFT_OUTPUT_FUNCTION, 0.0)
        except Exception as e:
            print("Failed to stop LEFT actuator command: %s" % str(e))
    # def

    def stop_right_calibration(self):
        if self.right_cal_active:
            print("Stopping RIGHT automatic calibration...")
        else:
            print("RIGHT automatic calibration is not running")
        self.right_cal_active = False
        try:
            self.drone_interface.command_elevon(self.RIGHT_OUTPUT_FUNCTION, 0.0)
        except Exception as e:
            print("Failed to stop RIGHT actuator command: %s" % str(e))
    # def

    def clear_left(self):
        self.start_side_clear("LEFT")
    # def

    def clear_right(self):
        self.start_side_clear("RIGHT")
    # def

    def start_side_clear(self, side):
        """Reset one side's min/max/trim. Three writes, so up to 15 s - off-thread."""
        if self.is_calibration_active(side):
            print("%s is calibrating - not clearing" % side)
            return

        if side == "LEFT":
            keys = ["left_min", "left_max", "left_trim"]
            params = (self.LEFT_MIN_PARAM, self.LEFT_MAX_PARAM, self.LEFT_TRIM_PARAM)
            slider = self.left_pos
        else:
            keys = ["right_min", "right_max", "right_trim"]
            params = (self.RIGHT_MIN_PARAM, self.RIGHT_MAX_PARAM, self.RIGHT_TRIM_PARAM)
            slider = self.right_pos

        for key in keys:
            self._param_write_seq[key] = self._param_write_seq.get(key, 0) + 1

        self.set_param_widgets_enabled(keys, False)
        slider.set(0.0)

        threading.Thread(
            target=self._side_clear_worker,
            args=(side, keys, params),
            daemon=True
        ).start()
    # def

    def _side_clear_worker(self, side, keys, params):
        min_param, max_param, trim_param = params
        clear_min = 900
        clear_max = 2100
        clear_trim = 0.0

        results = {}
        try:
            if self.drone_interface.set_param_value(min_param, int, clear_min):
                results[keys[0]] = str(clear_min)
            if self.drone_interface.set_param_value(max_param, int, clear_max):
                results[keys[1]] = str(clear_max)
            if self.drone_interface.set_param_value(trim_param, float, clear_trim):
                results[keys[2]] = "%.3f" % clear_trim

            print("%s cleared" % side)
        except Exception as e:
            print("Failed to clear %s: %s" % (side, str(e)))

        def finish():
            if self._closing:
                return
            self.set_param_widgets_enabled(keys, True)
            for key, text in results.items():
                entry_var = self._param_text_var(key)
                if entry_var is not None:
                    entry_var.set(text)
        # def

        self.post_to_gui(finish)
    # def

    def _param_text_var(self, key):
        return {
            "left_min": self.left_min_var,
            "left_max": self.left_max_var,
            "left_trim": self.left_trim_var,
            "right_min": self.right_min_var,
            "right_max": self.right_max_var,
            "right_trim": self.right_trim_var,
        }.get(key)
    # def

    def set_param_widgets_enabled(self, keys, enabled):
        """Tk thread. Lock a field while its write is in flight."""
        for key in keys:
            widgets = self._param_widgets.get(key)
            if widgets is None:
                continue
            entry, btn = widgets
            # readonly rather than disabled: the value stays legible.
            entry.config(state="normal" if enabled else "readonly")
            btn.config(state="normal" if enabled else "disabled",
                       text="Set" if enabled else "...")
    # def

    def start_param_write(self, key, side, label, param_name, py_type, value, text_var, fmt):
        """GUI thread: lock the field and hand the blocking write to a worker.

        set_param_value retries to a 5 s deadline and the readback adds another,
        so doing this inline froze the whole window for up to 10 s per press.
        """
        if self.is_calibration_active(side):
            print("%s is calibrating - not writing %s" % (side, param_name))
            return

        print("%s [%s]" % (label, fmt % value))

        self._param_write_seq[key] = self._param_write_seq.get(key, 0) + 1
        seq = self._param_write_seq[key]

        self.set_param_widgets_enabled([key], False)

        threading.Thread(
            target=self._param_write_worker,
            args=(key, param_name, py_type, value, text_var, fmt, seq),
            daemon=True
        ).start()
    # def

    def _param_write_worker(self, key, param_name, py_type, value, text_var, fmt, seq):
        confirmed = None
        try:
            if self.drone_interface.set_param_value(param_name, py_type, value):
                confirmed = self.drone_interface.get_param(param_name, py_type)
        except Exception as e:
            print("Failed to set %s: %s" % (param_name, str(e)))

        def finish():
            if self._closing:
                return
            # Re-enable unconditionally, or a superseded write leaves the field
            # locked forever. Only the write-back is gated on the generation.
            self.set_param_widgets_enabled([key], True)
            if confirmed is not None and seq == self._param_write_seq.get(key):
                text_var.set(fmt % confirmed)
        # def

        self.post_to_gui(finish)
    # def

    def apply_left_min(self):
        try:
            value = self.parse_int_param_entry(self.left_min_var)
            self.start_param_write("left_min", "LEFT", "Left-min",
                                   self.LEFT_MIN_PARAM, int, value, self.left_min_var, "%d")
        except Exception as e:
            print("Failed to set left min: %s" % str(e))
    # def

    def apply_left_max(self):
        try:
            value = self.parse_int_param_entry(self.left_max_var)
            self.start_param_write("left_max", "LEFT", "Left-max",
                                   self.LEFT_MAX_PARAM, int, value, self.left_max_var, "%d")
        except Exception as e:
            print("Failed to set left max: %s" % str(e))
    # def

    def apply_left_trim(self):
        try:
            value = self.parse_trim_param_entry(self.left_trim_var)
            self.start_param_write("left_trim", "LEFT", "Left-trim",
                                   self.LEFT_TRIM_PARAM, float, value, self.left_trim_var, "%.3f")
        except Exception as e:
            print("Failed to set left trim: %s" % str(e))
    # def

    def apply_right_min(self):
        try:
            value = self.parse_int_param_entry(self.right_min_var)
            self.start_param_write("right_min", "RIGHT", "Right-min",
                                   self.RIGHT_MIN_PARAM, int, value, self.right_min_var, "%d")
        except Exception as e:
            print("Failed to set right min: %s" % str(e))
    # def

    def apply_right_max(self):
        try:
            value = self.parse_int_param_entry(self.right_max_var)
            self.start_param_write("right_max", "RIGHT", "Right-max",
                                   self.RIGHT_MAX_PARAM, int, value, self.right_max_var, "%d")
        except Exception as e:
            print("Failed to set right max: %s" % str(e))
    # def

    def apply_right_trim(self):
        try:
            value = self.parse_trim_param_entry(self.right_trim_var)
            self.start_param_write("right_trim", "RIGHT", "Right-trim",
                                   self.RIGHT_TRIM_PARAM, float, value, self.right_trim_var, "%.3f")
        except Exception as e:
            print("Failed to set right trim: %s" % str(e))
    # def

    def on_left_pos(self, value):
        value = float(value)
        self.left_pos_changed(value)
    # def

    def on_right_pos(self, value):
        value = float(value)
        self.right_pos_changed(value)
    # def

    def left_pos_changed(self, value):
        print("Left-pos [%.3f]" % value)
    # def

    def right_pos_changed(self, value):
        print("Right-pos [%.3f]" % value)
    # def

    def get_left_value(self):
        avg = self.position_reader.get_average_position_nonblocking("LEFT")
        if avg is None:
            return None
        return self.position_reader.position_to_degrees("LEFT", avg)
    # def

    def get_right_value(self):
        avg = self.position_reader.get_average_position_nonblocking("RIGHT")
        if avg is None:
            return None
        return self.position_reader.position_to_degrees("RIGHT", avg)
    # def

    def update_labels(self):
        if self._closing:
            return

        left_val = self.get_left_value()
        right_val = self.get_right_value()

        if left_val is None:
            self.left_label.config(text="Left: --")
        else:
            self.left_label.config(text="Left: %.1f deg" % left_val)

        if right_val is None:
            self.right_label.config(text="Right: --")
        else:
            self.right_label.config(text="Right: %.1f deg" % right_val)

        self.update_pico_rate_label()

        self.root.after(100, self.update_labels)
    # def

    def update_expected_pwm(self):
        if self._closing:
            return

        try:
            left_min = self.get_int_var(self.left_min_var, 1000)
            left_max = self.get_int_var(self.left_max_var, 2000)
            left_trim = self.get_float_var(self.left_trim_var, 0.0)
            left_cmd = float(self.left_pos.get())
            left_rev = self.is_main_channel_reversed(5)
            left_pwm = self.expected_pwm(left_cmd, left_min, left_max, left_trim, left_rev)
            self.left_pwm_label.config(
                text="PWM exp: %d%s" % (left_pwm, " R" if left_rev else "")
            )

            right_min = self.get_int_var(self.right_min_var, 1000)
            right_max = self.get_int_var(self.right_max_var, 2000)
            right_trim = self.get_float_var(self.right_trim_var, 0.0)
            right_cmd = float(self.right_pos.get())
            right_rev = self.is_main_channel_reversed(6)
            right_pwm = self.expected_pwm(right_cmd, right_min, right_max, right_trim, right_rev)
            self.right_pwm_label.config(
                text="PWM exp: %d%s" % (right_pwm, " R" if right_rev else "")
            )

        except Exception as e:
            print("Expected PWM update error: %s" % str(e))

        self.root.after(100, self.update_expected_pwm)
    # def

    def update_actuators(self):
        if self._closing:
            return

        try:
            if not self.left_cal_active and not self.sweep_active \
                    and not self.measure_active:
                left_cmd = float(self.left_pos.get())
                self.drone_interface.command_elevon(self.LEFT_OUTPUT_FUNCTION, left_cmd)

            if not self.right_cal_active and not self.sweep_active \
                    and not self.measure_active:
                right_cmd = float(self.right_pos.get())
                self.drone_interface.command_elevon(self.RIGHT_OUTPUT_FUNCTION, right_cmd)

        except Exception as e:
            print("Actuator update error: %s" % str(e))

        self.root.after(100, self.update_actuators)
    # def

    def update_log_window(self):
        if self._closing:
            return

        lines = INSTRUMENTATION_LOG.drain()
        if lines:
            self.log_text.insert(tk.END, "".join(lines))

            # Trim from the top, or a long calibration session grows the widget
            # without limit.
            line_count = int(self.log_text.index("end-1c").split(".")[0])
            if line_count > MAX_LOG_LINES:
                self.log_text.delete("1.0", "%d.0" % (line_count - MAX_LOG_LINES + 1))

            self.log_text.see(tk.END)
        self.root.after(100, self.update_log_window)
    # def

    def on_close(self):
        """Window close: stop workers, centre the elevons, release both ports."""
        if self._closing:
            return
        self._closing = True

        print("Shutting down...")

        self.left_cal_active = False
        self.right_cal_active = False
        self.sweep_active = False
        self.measure_active = False

        for thread in (self.left_cal_thread, self.right_cal_thread,
                       self.sweep_thread, self.measure_thread):
            if thread is not None and thread.is_alive():
                thread.join(2.0)

        try:
            if self.drone_interface.is_connected():
                self.drone_interface.command_elevon(self.LEFT_OUTPUT_FUNCTION, 0.0)
                self.drone_interface.command_elevon(self.RIGHT_OUTPUT_FUNCTION, 0.0)
        except Exception as e:
            print("Failed to centre elevons on exit: %s" % str(e))

        self.position_reader.stop()
        self.drone_interface.close()

        self.root.destroy()
    # def
# class


if __name__ == "__main__":
    root = tk.Tk()

    position_reader = PositionReader()   # port chosen by auto-detect / the UI
    drone_interface = DroneInterface()

    app = FourSliderGUI(root, position_reader, drone_interface)

    root.protocol("WM_DELETE_WINDOW", app.on_close)

    # Detection probes serial ports and can take a second or two, so it runs on
    # a worker thread after the window is up rather than blocking startup.
    root.after(200, app.autodetect_and_connect)

    root.mainloop()
# if
