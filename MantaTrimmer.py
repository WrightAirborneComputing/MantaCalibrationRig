import tkinter as tk
import threading
import time
import re
import struct
import builtins
import queue
from collections import deque
from pymavlink import mavutil

import serial


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

    def _handle_incoming_msg(self, msg):
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
                self._handle_incoming_msg(msg)

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

        self.left_offset = -75.68
        self.left_scaler = 0.0042

        self.right_offset = +117.75
        self.right_scaler = 0.0045
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

        elif side == "RIGHT":
            self.right_offset = (self.right_scaler * raw)
            print("Right centred. Scaler=%.4f Offset = %.4f" % (self.right_scaler, self.right_offset))
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
                queue = self._queue_left
            elif side == "RIGHT":
                queue = self._queue_right
            else:
                return None

            while queue and len(samples) < self.num_samples:
                samples.append(queue.popleft())

        if not samples:
            return None

        return sum(samples) / float(len(samples))
    # def


class FourSliderGUI:
    def __init__(self, root, position_reader, drone_interface):
        self.root = root
        self.root.title("Manta Trimmer")

        self.position_reader = position_reader
        self.drone_interface = drone_interface

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

        main_frame = tk.Frame(root)
        main_frame.pack(padx=20, pady=20)

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

        print("PWM_MAIN_REV = %d (0x%X)" % (self.main_rev, self.main_rev))
        print("MAIN5 reversed = %s" % str(((self.main_rev >> 4) & 1) != 0))
        print("MAIN6 reversed = %s" % str(((self.main_rev >> 5) & 1) != 0))

        # LEFT group
        left_group = tk.Frame(main_frame)
        left_group.pack(side=tk.LEFT, padx=20)

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
        left_cal_btn.pack(pady=(0, 10))

        left_params = tk.Frame(left_group)
        left_params.pack(pady=(0, 10))

        self.left_min_var = tk.StringVar(value=str(left_min_param))
        self.left_max_var = tk.StringVar(value=str(left_max_param))
        self.left_trim_var = tk.StringVar(value="%.3f" % left_trim_param)

        self.create_param_entry(left_params, "Left-min", self.left_min_var, self.apply_left_min)
        self.create_param_entry(left_params, "Left-max", self.left_max_var, self.apply_left_max)
        self.create_param_entry(left_params, "Left-trim", self.left_trim_var, self.apply_left_trim)

        left_pos_frame = tk.Frame(left_group)
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
        right_group = tk.Frame(main_frame)
        right_group.pack(side=tk.LEFT, padx=20)

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
        right_cal_btn.pack(pady=(0, 10))

        right_params = tk.Frame(right_group)
        right_params.pack(pady=(0, 10))

        self.right_min_var = tk.StringVar(value=str(right_min_param))
        self.right_max_var = tk.StringVar(value=str(right_max_param))
        self.right_trim_var = tk.StringVar(value="%.3f" % right_trim_param)

        self.create_param_entry(right_params, "Right-min", self.right_min_var, self.apply_right_min)
        self.create_param_entry(right_params, "Right-max", self.right_max_var, self.apply_right_max)
        self.create_param_entry(right_params, "Right-trim", self.right_trim_var, self.apply_right_trim)

        right_pos_frame = tk.Frame(right_group)
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
        log_group = tk.Frame(main_frame)
        log_group.pack(side=tk.LEFT, padx=20, fill=tk.BOTH, expand=True)

        log_label = tk.Label(log_group, text="Instrumentation")
        log_label.pack(anchor="w")

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

    def move_elevon_to_angle(self, side, output_function, target_angle_deg, inc_angle_deg):
        print("%s calibration move: centering elevon to 0.0 command" % side)
        self.drone_interface.command_elevon(output_function, 0.0)
        time.sleep(1.0)

        cmd = 0.0
        deadline = time.time() + 60.0

        while time.time() < deadline:
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
            # if

            if reached:
                print("%s calibration move: target reached, cmd=%.3f" % (side, cmd))
                return cmd
            # if

            if(abs(target_angle_deg - angle_deg) > 20.0):
                cmd += (inc_angle_deg * 3.0)
            else:
                cmd += inc_angle_deg
            # if

            if cmd < -1.0:
                cmd = -1.0
            if cmd > 1.0:
                cmd = 1.0
            # if

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

            cmd_neg35 = self.move_elevon_to_angle(side, output_function, -33.0, -0.01)
            pwm_neg35 = self.get_side_expected_pwm(side, cmd_neg35)

            cmd_pos35 = self.move_elevon_to_angle(side, output_function, 35.0, 0.01)
            pwm_pos35 = self.get_side_expected_pwm(side, cmd_pos35)

            print("%s automatic calibration loading Min[%s] Max[%s]" %
                  (side, str(pwm_pos35), str(pwm_neg35)))
            self.set_side_param_and_refresh(side, min_param, int, pwm_pos35)
            self.set_side_param_and_refresh(side, max_param, int, pwm_neg35)

            cmd_neg10 = self.move_elevon_to_angle(side, output_function, -5.0, -0.01)

            print("%s automatic calibration loading Trim[%.2f]" % (side, cmd_neg10))
            self.set_side_param_and_refresh(side, trim_param, float, cmd_neg10)

            print("%s automatic calibration finished" % side)
        finally:
            self.left_cal_active = False
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

            cmd_neg35 = self.move_elevon_to_angle(side, output_function, -33.0, -0.01)
            pwm_neg35 = self.get_side_expected_pwm(side, cmd_neg35)

            cmd_pos35 = self.move_elevon_to_angle(side, output_function, 35.0, 0.01)
            pwm_pos35 = self.get_side_expected_pwm(side, cmd_pos35)

            print("%s automatic calibration loading Min[%s] Max[%s]" %
                  (side, str(pwm_neg35), str(pwm_pos35)))
            self.set_side_param_and_refresh(side, min_param, int, pwm_neg35)
            self.set_side_param_and_refresh(side, max_param, int, pwm_pos35)

            cmd_neg10 = self.move_elevon_to_angle(side, output_function, -5.0, -0.01)

            print("%s automatic calibration loading Trim[%.2f]" % (side, cmd_neg10))
            self.set_side_param_and_refresh(side, trim_param, float, cmd_neg10)

            print("%s automatic calibration finished" % side)
        finally:
            self.right_cal_active = False
    # def

    def start_left_calibration(self):
        if self.left_cal_thread is not None and self.left_cal_thread.is_alive():
            print("Left automatic calibration already running")
            return

        self.left_cal_thread = threading.Thread(
            target=self._left_calibration_worker,
            daemon=True
        )
        self.left_cal_thread.start()
    # def

    def start_right_calibration(self):
        if self.right_cal_thread is not None and self.right_cal_thread.is_alive():
            print("Right automatic calibration already running")
            return

        self.right_cal_thread = threading.Thread(
            target=self._right_calibration_worker,
            daemon=True
        )
        self.right_cal_thread.start()
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


if __name__ == "__main__":
    drone_interface = DroneInterface("COM4")
    drone_interface.connect()

    position_reader = PositionReader("COM9")
    position_reader.start()

    root = tk.Tk()
    app = FourSliderGUI(root, position_reader, drone_interface)
    root.mainloop()
# if
