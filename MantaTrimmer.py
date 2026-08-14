try:
    import tkinter as tk
    from tkinter import ttk
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

from manta_common import (
    APP_DIR,
    IS_LINUX,
    IS_WINDOWS,
    SERIAL_BAUD,
    PICO_VIDS,
    PICO_VID_PIDS,
    FCU_VID_HINTS,
    POSITION_REGEX,
    PortCandidate,
    describe_serial_error,
    find_candidate,
    list_serial_ports,
    position_to_degrees,
    probe_pico,
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
POSITION_HISTORY = 200          # ~20 s of backlog at 10 Hz

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

    def command_elevon(self, output_function, value):
        if not self.is_connected():
            return

        # Test escape hatch: exercise the whole app without moving surfaces.
        if os.environ.get("MANTA_NO_ACTUATE"):
            return

        MAV_CMD_ACTUATOR_TEST = 310
        timeout_s = 60.0

        with self._mav_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                MAV_CMD_ACTUATOR_TEST,
                0,
                float(value),
                float(timeout_s),
                0,
                0,
                float(output_function),
                0,
                0,
            )
    # def

# class


class PositionReader:
    def __init__(self, port=None):
        self.port = port
        self.baud = SERIAL_BAUD
        self.timeout = 1.0

        # (monotonic_timestamp, raw_value) pairs. Read non-destructively by any
        # number of consumers; maxlen bounds the backlog.
        self._queue_left = deque(maxlen=POSITION_HISTORY)
        self._queue_right = deque(maxlen=POSITION_HISTORY)
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()

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
                self.connected = True
                self.last_error = None

                while not self._stop.is_set():
                    line = ser.readline()
                    if not line:
                        continue

                    text = line.decode("ascii", errors="ignore").strip()
                    if not text:
                        continue

                    m = POSITION_REGEX.search(text)
                    if m:
                        try:
                            p1 = int(m.group("position1"))
                            p2 = int(m.group("position2"))
                        except ValueError:
                            continue

                        now = time.monotonic()
                        with self._lock:
                            self._queue_left.append((now, p1))
                            self._queue_right.append((now, p2))
                            self._last_sample_time = now

        except (serial.SerialException, OSError) as e:
            self.last_error = describe_serial_error(port, e)
            print(self.last_error)

        finally:
            self.connected = False
            print("Position stream on %s closed" % port)
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

    def stop(self, join_timeout=2.0):
        self._stop.set()

        t = self._thread
        if t is not None and t.is_alive():
            # readline() has a 1s timeout, so this returns promptly.
            t.join(join_timeout)

        self._thread = None
        self.connected = False
    # def

    def set_port(self, port):
        if port == self.port and self._thread is not None and self._thread.is_alive():
            return

        self.stop()
        self.port = port
        self.last_error = None
        self._last_sample_time = 0.0
        self.clear_queues()

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

        self.drone_name_var = tk.StringVar(value="")
        self.folding_var = tk.StringVar(value="")

        self.angle_neg_degs = self.position_reader.angle_neg_degs
        self.angle_pos_degs = self.position_reader.angle_pos_degs
        self.angle_trim_degs = self.position_reader.angle_trim_degs

        self.angle_neg_var = tk.StringVar(value="%.1f" % self.angle_neg_degs)
        self.angle_pos_var = tk.StringVar(value="%.1f" % self.angle_pos_degs)
        self.angle_trim_var = tk.StringVar(value="%.1f" % self.angle_trim_degs)

        main_frame = tk.Frame(root)
        main_frame.pack(padx=20, pady=20)

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
        angle_group = tk.LabelFrame(main_frame, text="Settings and Control", padx=10, pady=10)
        angle_group.pack(side=tk.LEFT, padx=10, anchor="n", fill=tk.Y)

        self.build_connection_panel(angle_group)

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
        left_group = tk.LabelFrame(main_frame, text="Left", padx=10, pady=10)
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
            relief="solid",
            bd=2,
            width=16,
            anchor="center"
        )
        self.left_pwm_label.pack(pady=(10, 4))

        self.left_label = tk.Label(
            left_group,
            text="Left: --",
            relief="solid",
            bd=2,
            width=16,
            anchor="center"
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
        right_group = tk.LabelFrame(main_frame, text="Right", padx=10, pady=10)
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
            relief="solid",
            bd=2,
            width=16,
            anchor="center"
        )
        self.right_pwm_label.pack(pady=(10, 4))

        self.right_label = tk.Label(
            right_group,
            text="Right: --",
            relief="solid",
            bd=2,
            width=16,
            anchor="center"
        )
        self.right_label.pack(pady=(0, 10))

        right_center_btn = tk.Button(
            right_group,
            text="Zero angle",
            width=10,
            command=self.centre_right
        )
        right_center_btn.pack(pady=(0, 5))

        # Instrumentation panel
        log_group = tk.LabelFrame(main_frame, text="Instrumentation", padx=10, pady=10)
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
            csv_filename = os.path.join(APP_DIR, "%s_%s.csv" % (safe_name, timestamp))

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
        group = tk.LabelFrame(parent, text="Connection", padx=6, pady=6)
        group.pack(anchor="w", pady=(0, 8), fill=tk.X)

        self.drone_port_var = tk.StringVar(value="")
        self.pico_port_var = tk.StringVar(value="")
        self._port_by_label = {}

        drone_row = tk.Frame(group)
        drone_row.pack(anchor="w", pady=2, fill=tk.X)
        tk.Label(drone_row, text="Drone", width=6, anchor="w").pack(side=tk.LEFT)
        self.drone_combo = ttk.Combobox(drone_row, textvariable=self.drone_port_var, width=30)
        self.drone_combo.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.drone_status_label = tk.Label(group, text="Drone: not connected", anchor="w", fg="black")
        self.drone_status_label.pack(anchor="w", padx=(46, 0))

        pico_row = tk.Frame(group)
        pico_row.pack(anchor="w", pady=(6, 2), fill=tk.X)
        tk.Label(pico_row, text="Pico", width=6, anchor="w").pack(side=tk.LEFT)
        self.pico_combo = ttk.Combobox(pico_row, textvariable=self.pico_port_var, width=30)
        self.pico_combo.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.pico_status_label = tk.Label(group, text="Pico: not connected", anchor="w", fg="black")
        self.pico_status_label.pack(anchor="w", padx=(46, 0))

        button_row = tk.Frame(group)
        button_row.pack(anchor="w", pady=(8, 0), fill=tk.X)

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
        colour = "black" if ok is None else ("dark green" if ok else "red")
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
            self.set_conn_status("pico", "Pico: not found", False)
        if result["drone"] is None:
            self.set_conn_status("drone", "Drone: no heartbeat found", False)

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
            self.set_conn_status("drone", "Drone: connecting...", None)
        if pico_device:
            self.set_conn_status("pico", "Pico: opening...", None)

        threading.Thread(
            target=self._connect_worker,
            args=(drone_device, pico_device),
            daemon=True
        ).start()
    # def

    def _connect_worker(self, drone_device, pico_device):
        """Worker thread: open both links, then report status back to Tk."""
        drone_text = "Drone: no port selected"
        drone_ok = False

        try:
            if drone_device:
                print("Connecting MAVLink on %s..." % drone_device)
                if self.drone_interface.reconnect(drone_device):
                    drone_ok = True
                    drone_text = "Drone: connected (system %s)" % self.drone_interface.master.target_system
                    # Remember the port before the long param refresh, so a
                    # mid-refresh failure doesn't lose a known-good port.
                    self.position_reader.set_remembered_ports(drone_port=drone_device)
                    self.refresh_params_from_drone(clear_name=True)
                elif self.drone_interface.last_error_kind == "open":
                    drone_text = "Drone: cannot open %s" % drone_device
                else:
                    drone_text = "Drone: no heartbeat on %s" % drone_device

        except Exception as e:
            drone_text = "Drone: connect failed"
            print("MAVLink connect failed: %s" % str(e))

        pico_text = "Pico: no port selected"
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
                    pico_text = "Pico: streaming on %s" % pico_device
                    self.position_reader.set_remembered_ports(pico_port=pico_device)
                elif self.position_reader.last_error is not None:
                    pico_text = "Pico: cannot open %s" % pico_device
                else:
                    pico_text = "Pico: no data on %s" % pico_device

        except Exception as e:
            pico_text = "Pico: open failed"
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
        ok = self.drone_interface.set_param_value(param_name, py_type, value)
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

    def move_elevon_to_angle(self, side, output_function, target_angle_deg, inc_angle_deg):
        print("%s calibration move: centering elevon to 0.0 command" % side)
        self.drone_interface.command_elevon(output_function, 0.0)
        time.sleep(1.0)

        cmd = 0.0
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

            self.drone_interface.command_elevon(output_function, cmd)

            if (inc_angle_deg < 0.0 and cmd <= -1.0) or (inc_angle_deg > 0.0 and cmd >= 1.0):
                print("%s calibration move: hit command limit before reaching target" % side)
                return None

            time.sleep(0.25)

        print("%s calibration move: timed out before reaching target" % side)
        return None
    # def

    def _left_calibration_worker(self):
        self.left_cal_active = True
        side = "LEFT"
        output_function = self.LEFT_OUTPUT_FUNCTION
        min_param = self.LEFT_MIN_PARAM
        max_param = self.LEFT_MAX_PARAM
        trim_param = self.LEFT_TRIM_PARAM
        try:
            print("%s automatic calibration started" % side)

            self.set_side_param_and_refresh(side, min_param, int, 900)
            self.set_side_param_and_refresh(side, max_param, int, 2100)
            self.set_side_param_and_refresh(side, trim_param, float, 0.0)

            if not self.left_cal_active:
                print("%s automatic calibration stopped" % side)
                return

            cmd_neg = self.move_elevon_to_angle(side, output_function, self.angle_neg_degs, -0.01)
            if not self.left_cal_active or cmd_neg is None:
                print("%s automatic calibration stopped before negative endpoint completed" % side)
                return
            pwm_neg = self.get_side_expected_pwm(side, cmd_neg)

            cmd_pos = self.move_elevon_to_angle(side, output_function, self.angle_pos_degs, 0.01)
            if not self.left_cal_active or cmd_pos is None:
                print("%s automatic calibration stopped before positive endpoint completed" % side)
                return
            pwm_pos = self.get_side_expected_pwm(side, cmd_pos)

            print("%s automatic calibration loading Min[%s] Max[%s]" %
                  (side, str(pwm_pos), str(pwm_neg)))
            self.set_side_param_and_refresh(side, min_param, int, pwm_pos)
            self.set_side_param_and_refresh(side, max_param, int, pwm_neg)

            trim_step = -0.01 if self.angle_trim_degs < 0.0 else 0.01
            cmd_trim = self.move_elevon_to_angle(side, output_function, self.angle_trim_degs, trim_step)

            if not self.left_cal_active or cmd_trim is None:
                print("%s automatic calibration stopped before trim completed" % side)
                return

            print("%s automatic calibration loading Trim[%.2f]" % (side, cmd_trim))
            self.set_side_param_and_refresh(side, trim_param, float, cmd_trim)

            print("%s automatic calibration finished" % side)
        finally:
            self.left_cal_active = False
            self.drone_interface.command_elevon(output_function, 0.0)
    # def

    def _right_calibration_worker(self):
        self.right_cal_active = True
        side = "RIGHT"
        output_function = self.RIGHT_OUTPUT_FUNCTION
        min_param = self.RIGHT_MIN_PARAM
        max_param = self.RIGHT_MAX_PARAM
        trim_param = self.RIGHT_TRIM_PARAM
        try:
            print("%s automatic calibration started" % side)

            self.set_side_param_and_refresh(side, min_param, int, 900)
            self.set_side_param_and_refresh(side, max_param, int, 2100)
            self.set_side_param_and_refresh(side, trim_param, float, 0.0)

            if not self.right_cal_active:
                print("%s automatic calibration stopped" % side)
                return

            cmd_neg = self.move_elevon_to_angle(side, output_function, self.angle_neg_degs, -0.01)
            if not self.right_cal_active or cmd_neg is None:
                print("%s automatic calibration stopped before negative endpoint completed" % side)
                return
            pwm_neg = self.get_side_expected_pwm(side, cmd_neg)

            cmd_pos = self.move_elevon_to_angle(side, output_function, self.angle_pos_degs, 0.01)
            if not self.right_cal_active or cmd_pos is None:
                print("%s automatic calibration stopped before positive endpoint completed" % side)
                return
            pwm_pos = self.get_side_expected_pwm(side, cmd_pos)

            print("%s automatic calibration loading Min[%s] Max[%s]" %
                  (side, str(pwm_neg), str(pwm_pos)))
            self.set_side_param_and_refresh(side, min_param, int, pwm_neg)
            self.set_side_param_and_refresh(side, max_param, int, pwm_pos)

            trim_step = -0.01 if self.angle_trim_degs < 0.0 else 0.01
            cmd_trim = self.move_elevon_to_angle(side, output_function, self.angle_trim_degs, trim_step)

            if not self.right_cal_active or cmd_trim is None:
                print("%s automatic calibration stopped before trim completed" % side)
                return

            print("%s automatic calibration loading Trim[%.2f]" % (side, cmd_trim))
            self.set_side_param_and_refresh(side, trim_param, float, cmd_trim)

            print("%s automatic calibration finished" % side)
        finally:
            self.right_cal_active = False
            self.drone_interface.command_elevon(output_function, 0.0)
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
            if not self.left_cal_active and not self.sweep_active:
                left_cmd = float(self.left_pos.get())
                self.drone_interface.command_elevon(self.LEFT_OUTPUT_FUNCTION, left_cmd)

            if not self.right_cal_active and not self.sweep_active:
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

        for thread in (self.left_cal_thread, self.right_cal_thread, self.sweep_thread):
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
