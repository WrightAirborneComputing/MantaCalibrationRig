import tkinter as tk
import threading
import time
import re
import struct
import builtins
import queue
from collections import deque
from pymavlink import mavutil
import json
import os
import serial
import csv
from datetime import datetime


class InstrumentationLog:
    def __init__(self):
        self._queue = queue.Queue()
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


class DroneInterface:

    def __init__(self, port):
        self.port = port
        self.baud = 115200
        self.timeout = 5.0
        self.master = None

        self._mav_lock = threading.Lock()
        self._param_cache = {}
    # def

    def connect(self):
        print("Connecting to %s at %d baud..." % (self.port, self.baud))
        self.master = mavutil.mavlink_connection(self.port, self.baud)

        print("Waiting for heartbeat from PX4...")
        self.master.wait_heartbeat()
        print("Heartbeat from system %s component %s" % (
            self.master.target_system,
            self.master.target_component
        ))
    # def

    def reconnect(self):
        try:
            if self.master is not None:
                close_fn = getattr(self.master, "close", None)
                if callable(close_fn):
                    close_fn()
        except Exception as e:
            print("Previous MAVLink close failed: %s" % str(e))

        self.master = None
        self._param_cache.clear()
        self.connect()
    # def

    def get_id(self, timeout=5.0):

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


POSITION_REGEX = re.compile(r"\[(?P<position1>-?\d+)\s*/\s*(?P<position2>-?\d+)\]")


class PositionReader:
    def __init__(self, port):
        self.port = port
        self.baud = 115200
        self.timeout = 1.0
        self.num_samples = 10

        self._queue_left = deque()
        self._queue_right = deque()
        self._lock = threading.Lock()
        self._thread = None

        self.calibration_file = "settings.json"

        self.drone_name = ""

        self.left_offset = -75.68
        self.left_scaler = 0.0042

        self.right_offset = +117.75
        self.right_scaler = 0.0045

        self.angle_neg_degs = -33.0
        self.angle_pos_degs = 35.0
        self.angle_trim_degs = -5.0

        self.load_calibration()
    # def

    def load_calibration(self):
        if not os.path.exists(self.calibration_file):
            print("Calibration file %s not found, using defaults" % self.calibration_file)
            return

        try:
            with open(self.calibration_file, "r", encoding="utf-8") as f:
                data = json.load(f)

            self.drone_name = str(data.get("drone_name", ""))

            left = data.get("LEFT", {})
            right = data.get("RIGHT", {})
            angles = data.get("ANGLES", {})

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
                "Loaded calibration from %s: drone_name=%s, LEFT scaler=%.6f offset=%.6f, RIGHT scaler=%.6f offset=%.6f, angles=(%.2f, %.2f, %.2f)"
                % (
                    self.calibration_file,
                    self.drone_name,
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
            print("Failed to load calibration from %s: %s" % (self.calibration_file, str(e)))
    # def

    def save_calibration(self):
        data = {
            "drone_name": self.drone_name,
            "LEFT": {
                "scaler": self.left_scaler,
                "offset": self.left_offset,
            },
            "RIGHT": {
                "scaler": self.right_scaler,
                "offset": self.right_offset,
            },
            "ANGLES": {
                "angle_neg_degs": self.angle_neg_degs,
                "angle_pos_degs": self.angle_pos_degs,
                "angle_trim_degs": self.angle_trim_degs,
            },
        }

        try:
            with open(self.calibration_file, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=4)

            print(
                "Saved calibration to %s: drone_name=%s, LEFT scaler=%.6f offset=%.6f, RIGHT scaler=%.6f offset=%.6f, angles=(%.2f, %.2f, %.2f)"
                % (
                    self.calibration_file,
                    self.drone_name,
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

    def set_drone_name(self, drone_name):
        self.drone_name = str(drone_name)
        self.save_calibration()
    # def

    def set_angle_settings(self, angle_neg_degs=None, angle_pos_degs=None, angle_trim_degs=None):
        if angle_neg_degs is not None:
            self.angle_neg_degs = float(angle_neg_degs)
        if angle_pos_degs is not None:
            self.angle_pos_degs = float(angle_pos_degs)
        if angle_trim_degs is not None:
            self.angle_trim_degs = float(angle_trim_degs)

        self.save_calibration()
    # def

    def position_to_degrees(self, side, raw_position):
        if side == "LEFT":
            return (self.left_scaler * raw_position) + self.left_offset
        elif side == "RIGHT":
            return -(self.right_scaler * raw_position) + self.right_offset
        return None
    # def

    def set_center(self, side):
        raw = self.get_average_position_nonblocking(side)
        if raw is None:
            print("No data for centering %s" % side)
            return

        if side == "LEFT":
            self.left_offset = -(self.left_scaler * raw)
            print("Left centred. Scaler=%.4f Offset = %.4f" % (self.left_scaler, self.left_offset))
            self.save_calibration()

        elif side == "RIGHT":
            self.right_offset = (self.right_scaler * raw)
            print("Right centred. Scaler=%.4f Offset = %.4f" % (self.right_scaler, self.right_offset))
            self.save_calibration()
    # def

    def set_scaler_and_offset(self, side, scaler=None, offset=None):
        if side == "LEFT":
            if scaler is not None:
                self.left_scaler = float(scaler)
            if offset is not None:
                self.left_offset = float(offset)

            print("LEFT calibration set. Scaler=%.6f Offset=%.6f" % (self.left_scaler, self.left_offset))
            self.save_calibration()

        elif side == "RIGHT":
            if scaler is not None:
                self.right_scaler = float(scaler)
            if offset is not None:
                self.right_offset = float(offset)

            print("RIGHT calibration set. Scaler=%.6f Offset=%.6f" % (self.right_scaler, self.right_offset))
            self.save_calibration()
    # def

    def _position_reader_loop(self):
        print("Opening position stream on %s at %d baud..." % (self.port, self.baud))
        try:
            with serial.Serial(self.port, baudrate=self.baud, timeout=self.timeout) as ser:
                while True:
                    line = ser.readline()
                    if not line:
                        continue

                    try:
                        text = line.decode("ascii", errors="ignore").strip()
                    except Exception:
                        print("No decode")
                        continue

                    if not text:
                        print("Not text")
                        continue

                    m = POSITION_REGEX.search(text)
                    if m:
                        try:
                            p1 = int(m.group("position1"))
                            p2 = int(m.group("position2"))

                            with self._lock:
                                self._queue_left.append(p1)
                                self._queue_right.append(p2)

                                if len(self._queue_left) > 500:
                                    self._queue_left.popleft()
                                if len(self._queue_right) > 500:
                                    self._queue_right.popleft()

                        except ValueError:
                            pass

        except serial.SerialException as e:
            print("Error opening/reading %s: %s" % (self.port, e))
    # def

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return

        t = threading.Thread(target=self._position_reader_loop, daemon=True)
        self._thread = t
        t.start()
    # def

    def clear_queues(self):
        with self._lock:
            self._queue_left.clear()
            self._queue_right.clear()
    # def

    def get_average_position_nonblocking(self, side):
        samples = []

        with self._lock:
            if side == "LEFT":
                queue_ref = self._queue_left
            elif side == "RIGHT":
                queue_ref = self._queue_right
            else:
                return None

            while queue_ref and len(samples) < self.num_samples:
                samples.append(queue_ref.popleft())

        if not samples:
            return None

        return sum(samples) / float(len(samples))
    # def
# class


class FourSliderGUI:
    def __init__(self, root, position_reader, drone_interface):
        self.root = root
        self.root.title("Manta Trimmer")

        self.position_reader = position_reader
        self.drone_interface = drone_interface

        self.calibration_log_file = "calibration_log.csv"

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

        self.drone_name_var = tk.StringVar(value=self.position_reader.drone_name)

        self.angle_neg_degs = self.position_reader.angle_neg_degs
        self.angle_pos_degs = self.position_reader.angle_pos_degs
        self.angle_trim_degs = self.position_reader.angle_trim_degs

        self.angle_neg_var = tk.StringVar(value="%.1f" % self.angle_neg_degs)
        self.angle_pos_var = tk.StringVar(value="%.1f" % self.angle_pos_degs)
        self.angle_trim_var = tk.StringVar(value="%.1f" % self.angle_trim_degs)

        main_frame = tk.Frame(root)
        main_frame.pack(padx=20, pady=20)

        ident = drone_interface.get_id()
        self.uid_var = tk.StringVar(value="--" if ident is None else str(ident))

        left_min_param = drone_interface.get_param(self.LEFT_MIN_PARAM, int)
        left_max_param = drone_interface.get_param(self.LEFT_MAX_PARAM, int)
        left_trim_param = drone_interface.get_param(self.LEFT_TRIM_PARAM, float)
        right_min_param = drone_interface.get_param(self.RIGHT_MIN_PARAM, int)
        right_max_param = drone_interface.get_param(self.RIGHT_MAX_PARAM, int)
        right_trim_param = drone_interface.get_param(self.RIGHT_TRIM_PARAM, float)
        main_rev_param = drone_interface.get_param("PWM_MAIN_REV", int)

        if left_min_param is None:
            left_min_param = 1000
        if left_max_param is None:
            left_max_param = 2000
        if left_trim_param is None:
            left_trim_param = 0.0

        if right_min_param is None:
            right_min_param = 1000
        if right_max_param is None:
            right_max_param = 2000
        if right_trim_param is None:
            right_trim_param = 0.0

        if main_rev_param is None:
            main_rev_param = 0
        self.main_rev = int(main_rev_param)

        print("PX4 UID = %s" % str(ident))
        print("PWM_MAIN_REV = %d (0x%X)" % (self.main_rev, self.main_rev))
        print("MAIN5 reversed = %s" % str(((self.main_rev >> 4) & 1) != 0))
        print("MAIN6 reversed = %s" % str(((self.main_rev >> 5) & 1) != 0))

        # Angle configuration panel
        angle_group = tk.LabelFrame(main_frame, text="Settings and Control", padx=10, pady=10)
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

        connect_btn = tk.Button(
            angle_group,
            text="Connect",
            width=18,
            command=self.connect_mavlink
        )
        connect_btn.pack(pady=(8, 2), anchor="w")

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

        log_cal_btn = tk.Button(
            angle_group,
            text="Log calibration",
            width=18,
            command=self.log_calibration
        )
        log_cal_btn.pack(pady=(8, 0), anchor="w")

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

        self.create_param_entry(left_params, "Left-min", self.left_min_var, self.apply_left_min)
        self.create_param_entry(left_params, "Left-max", self.left_max_var, self.apply_left_max)
        self.create_param_entry(left_params, "Left-trim", self.left_trim_var, self.apply_left_trim)

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

        self.create_param_entry(right_params, "Right-min", self.right_min_var, self.apply_right_min)
        self.create_param_entry(right_params, "Right-max", self.right_max_var, self.apply_right_max)
        self.create_param_entry(right_params, "Right-trim", self.right_trim_var, self.apply_right_trim)

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

        self.update_labels()
        self.update_actuators()
        self.update_expected_pwm()
        self.update_log_window()
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

    def connect_mavlink(self):
        try:
            print("Reconnecting MAVLink...")
            self.drone_interface.reconnect()

            ident = self.drone_interface.get_id()
            self.uid_var.set("--" if ident is None else str(ident))

            self.drone_name_var.set("")
            self.position_reader.set_drone_name("")
            print("Drone name cleared after connect")

            left_min_param = self.drone_interface.get_param(self.LEFT_MIN_PARAM, int)
            left_max_param = self.drone_interface.get_param(self.LEFT_MAX_PARAM, int)
            left_trim_param = self.drone_interface.get_param(self.LEFT_TRIM_PARAM, float)
            right_min_param = self.drone_interface.get_param(self.RIGHT_MIN_PARAM, int)
            right_max_param = self.drone_interface.get_param(self.RIGHT_MAX_PARAM, int)
            right_trim_param = self.drone_interface.get_param(self.RIGHT_TRIM_PARAM, float)
            main_rev_param = self.drone_interface.get_param("PWM_MAIN_REV", int)

            if left_min_param is not None:
                self.left_min_var.set(str(int(left_min_param)))
            if left_max_param is not None:
                self.left_max_var.set(str(int(left_max_param)))
            if left_trim_param is not None:
                self.left_trim_var.set("%.3f" % float(left_trim_param))

            if right_min_param is not None:
                self.right_min_var.set(str(int(right_min_param)))
            if right_max_param is not None:
                self.right_max_var.set(str(int(right_max_param)))
            if right_trim_param is not None:
                self.right_trim_var.set("%.3f" % float(right_trim_param))

            if main_rev_param is not None:
                self.main_rev = int(main_rev_param)

            print("Reconnect complete")
            print("PWM_MAIN_REV = %d (0x%X)" % (self.main_rev, self.main_rev))
            print("MAIN5 reversed = %s" % str(((self.main_rev >> 4) & 1) != 0))
            print("MAIN6 reversed = %s" % str(((self.main_rev >> 5) & 1) != 0))

        except Exception as e:
            print("Reconnect failed: %s" % str(e))
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
            self.position_reader.set_drone_name(name)
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

            uid = self.uid_var.get().strip()

            angle_neg = float(self.angle_neg_var.get().strip())
            angle_pos = float(self.angle_pos_var.get().strip())
            angle_trim = float(self.angle_trim_var.get().strip())

            left_min = int(self.left_min_var.get().strip())
            left_max = int(self.left_max_var.get().strip())
            left_trim = float(self.left_trim_var.get().strip())

            right_min = int(self.right_min_var.get().strip())
            right_max = int(self.right_max_var.get().strip())
            right_trim = float(self.right_trim_var.get().strip())

            file_exists = os.path.exists(self.calibration_log_file)

            with open(self.calibration_log_file, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)

                if not file_exists:
                    writer.writerow([
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
                    ])

                writer.writerow([
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
                ])

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

        self.root.after(0, do_update)
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

    def create_param_entry(self, parent, label, text_var, apply_callback):
        row = tk.Frame(parent)
        row.pack(anchor="w", pady=2)

        lbl = tk.Label(row, text=label, width=8, anchor="w")
        lbl.pack(side=tk.LEFT, padx=(0, 5))

        entry = tk.Entry(row, textvariable=text_var, width=8)
        entry.pack(side=tk.LEFT)
        entry.bind("<Return>", lambda event: apply_callback())

        btn = tk.Button(row, text="Set", width=5, command=apply_callback)
        btn.pack(side=tk.LEFT, padx=(5, 0))

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

    def get_side_expected_pwm(self, side, cmd):
        if cmd is None:
            return None

        if side == "LEFT":
            pwm_min = self.get_int_var(self.left_min_var, 1000)
            pwm_max = self.get_int_var(self.left_max_var, 2000)
            trim = self.get_float_var(self.left_trim_var, 0.0)
            rev = self.is_main_channel_reversed(5)
        else:
            pwm_min = self.get_int_var(self.right_min_var, 1000)
            pwm_max = self.get_int_var(self.right_max_var, 2000)
            trim = self.get_float_var(self.right_trim_var, 0.0)
            rev = self.is_main_channel_reversed(6)

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
            elif target_angle_deg == 0.0 and abs(angle_deg) <= 0.5:
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
        try:
            left_min = 900
            left_max = 2100
            left_trim = 0.0

            ok_min = self.drone_interface.set_param_value(self.LEFT_MIN_PARAM, int, left_min)
            ok_max = self.drone_interface.set_param_value(self.LEFT_MAX_PARAM, int, left_max)
            ok_trim = self.drone_interface.set_param_value(self.LEFT_TRIM_PARAM, float, left_trim)

            if ok_min:
                self.left_min_var.set(str(left_min))
            if ok_max:
                self.left_max_var.set(str(left_max))
            if ok_trim:
                self.left_trim_var.set("%.3f" % left_trim)

            self.left_pos.set(0.0)

            print("Left cleared")
        except Exception as e:
            print("Failed to clear left: %s" % str(e))
    # def

    def clear_right(self):
        try:
            right_min = 900
            right_max = 2100
            right_trim = 0.0

            ok_min = self.drone_interface.set_param_value(self.RIGHT_MIN_PARAM, int, right_min)
            ok_max = self.drone_interface.set_param_value(self.RIGHT_MAX_PARAM, int, right_max)
            ok_trim = self.drone_interface.set_param_value(self.RIGHT_TRIM_PARAM, float, right_trim)

            if ok_min:
                self.right_min_var.set(str(right_min))
            if ok_max:
                self.right_max_var.set(str(right_max))
            if ok_trim:
                self.right_trim_var.set("%.3f" % right_trim)

            self.right_pos.set(0.0)

            print("Right cleared")
        except Exception as e:
            print("Failed to clear right: %s" % str(e))
    # def

    def apply_left_min(self):
        try:
            value = self.parse_int_param_entry(self.left_min_var)
            print("Left-min [%d]" % value)
            ok = self.drone_interface.set_param_value(self.LEFT_MIN_PARAM, int, value)
            if ok:
                confirmed = self.drone_interface.get_param(self.LEFT_MIN_PARAM, int)
                if confirmed is not None:
                    self.left_min_var.set(str(confirmed))
        except Exception as e:
            print("Failed to set left min: %s" % str(e))
    # def

    def apply_left_max(self):
        try:
            value = self.parse_int_param_entry(self.left_max_var)
            print("Left-max [%d]" % value)
            ok = self.drone_interface.set_param_value(self.LEFT_MAX_PARAM, int, value)
            if ok:
                confirmed = self.drone_interface.get_param(self.LEFT_MAX_PARAM, int)
                if confirmed is not None:
                    self.left_max_var.set(str(confirmed))
        except Exception as e:
            print("Failed to set left max: %s" % str(e))
    # def

    def apply_left_trim(self):
        try:
            value = self.parse_trim_param_entry(self.left_trim_var)
            print("Left-trim [%.3f]" % value)
            ok = self.drone_interface.set_param_value(self.LEFT_TRIM_PARAM, float, value)
            if ok:
                confirmed = self.drone_interface.get_param(self.LEFT_TRIM_PARAM, float)
                if confirmed is not None:
                    self.left_trim_var.set("%.3f" % confirmed)
        except Exception as e:
            print("Failed to set left trim: %s" % str(e))
    # def

    def apply_right_min(self):
        try:
            value = self.parse_int_param_entry(self.right_min_var)
            print("Right-min [%d]" % value)
            ok = self.drone_interface.set_param_value(self.RIGHT_MIN_PARAM, int, value)
            if ok:
                confirmed = self.drone_interface.get_param(self.RIGHT_MIN_PARAM, int)
                if confirmed is not None:
                    self.right_min_var.set(str(confirmed))
        except Exception as e:
            print("Failed to set right min: %s" % str(e))
    # def

    def apply_right_max(self):
        try:
            value = self.parse_int_param_entry(self.right_max_var)
            print("Right-max [%d]" % value)
            ok = self.drone_interface.set_param_value(self.RIGHT_MAX_PARAM, int, value)
            if ok:
                confirmed = self.drone_interface.get_param(self.RIGHT_MAX_PARAM, int)
                if confirmed is not None:
                    self.right_max_var.set(str(confirmed))
        except Exception as e:
            print("Failed to set right max: %s" % str(e))
    # def

    def apply_right_trim(self):
        try:
            value = self.parse_trim_param_entry(self.right_trim_var)
            print("Right-trim [%.3f]" % value)
            ok = self.drone_interface.set_param_value(self.RIGHT_TRIM_PARAM, float, value)
            if ok:
                confirmed = self.drone_interface.get_param(self.RIGHT_TRIM_PARAM, float)
                if confirmed is not None:
                    self.right_trim_var.set("%.3f" % confirmed)
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
        try:
            if not self.left_cal_active:
                left_cmd = float(self.left_pos.get())
                self.drone_interface.command_elevon(self.LEFT_OUTPUT_FUNCTION, left_cmd)

            if not self.right_cal_active:
                right_cmd = float(self.right_pos.get())
                self.drone_interface.command_elevon(self.RIGHT_OUTPUT_FUNCTION, right_cmd)

        except Exception as e:
            print("Actuator update error: %s" % str(e))

        self.root.after(100, self.update_actuators)
    # def

    def update_log_window(self):
        lines = INSTRUMENTATION_LOG.drain()
        if lines:
            self.log_text.insert(tk.END, "".join(lines))
            self.log_text.see(tk.END)
        self.root.after(100, self.update_log_window)
    # def
# class


if __name__ == "__main__":
    drone_interface = DroneInterface("COM20")  # COM4 on FS PC
    drone_interface.connect()

    position_reader = PositionReader("COM5")  # COM9 on FS PC
    position_reader.start()

    root = tk.Tk()
    app = FourSliderGUI(root, position_reader, drone_interface)
    root.mainloop()
# if
