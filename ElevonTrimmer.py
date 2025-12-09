from pymavlink import mavutil
import time
import struct
import threading
import re
import serial  # for COM5 position stream
from collections import deque

# Regex to parse lines like: [123/456]
POSITION_REGEX = re.compile(r"\[(?P<position1>-?\d+)\s*/\s*(?P<position2>-?\d+)\]")

# ======================================================================
# DroneInterface: handles interaction with drone via MAVLink
# ======================================================================
class DroneInterface:
    
    def __init__(self, port):
        self.port    = port
        self.baud    = 115200
        self.timeout = 5.0
        self.master  = None
    # def

    def connect(self):
        print("Connecting to %s at %d baud..." % (self.port, self.baud))
        self.master = mavutil.mavlink_connection(self.port, self.baud)

        print("Waiting for heartbeat from PX4...")
        self.master.wait_heartbeat()
        print("Heartbeat from system %s component %s" % (self.master.target_system, self.master.target_component))
    # def

    def clean_param_id(self,raw):
        if isinstance(raw, bytes):
            return raw.decode("ascii", errors="ignore").rstrip("\x00")
        else:
            return str(raw).rstrip("\x00")
        # if
    # def

    def get_param(self, param_name, py_type, timeout=5.0):

        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode("ascii"),
            -1,  # use param_id instead of index
        )

        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue

            if self.clean_param_id(msg.param_id) != param_name:
                continue

            raw_value = msg.param_value  # float32 as delivered by MAVLink

            # Integer-like params: reinterpret bits
            if py_type in (int, bool):
                packed = struct.pack("<f", raw_value)
                int_val = struct.unpack("<i", packed)[0]
                if py_type is bool:
                    return bool(int_val)
                return int_val

            # Float params: use as-is
            if py_type is float:
                return float(raw_value)

            # Fallback: just return the raw float
            return float(raw_value)

        print("Timed out waiting for current value of %s." % param_name)
        return None
    # def

    def set_param_value(self, param_name, py_type, value):

        # print("Setting parameter %s to %s (type=%s)" % (param_name, str(value), getattr(py_type, "__name__", str(py_type))))

        # Decide encoding based on py_type
        if py_type in (int, bool):
            if isinstance(value, bool):
                target_int = 1 if value else 0
            elif isinstance(value, (int, float)):
                target_int = int(round(value))
            else:
                raise ValueError("Cannot set INT param %s from value %r" %
                                 (param_name, value))

            # Encode int32 -> float bits
            packed = struct.pack("<i", int(target_int))
            wire_value = struct.unpack("<f", packed)[0]
            want_int = True
            target_val = target_int
            send_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32

        elif py_type is float:
            target_val = float(value)
            wire_value = target_val
            want_int = False
            send_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

        else:
            raise ValueError("Unsupported py_type %r for param %s" %
                             (py_type, param_name))

        # Send to FCU
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode("ascii"),
            wire_value,
            send_type,
        )

        # WAit for value to write
        time.sleep(0.2)

    # def

    def command_elevon(self, output_function, value):

        MAV_CMD_ACTUATOR_TEST = 310
        timeout_s = 5.0  # PX4's actuator test timeout; not strictly critical here

        # 1. Send actuator test command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            MAV_CMD_ACTUATOR_TEST,
            0,                   # confirmation
            float(value),        # param1: value (-1..1 or NaN)
            float(timeout_s),    # param2: timeout [0..3]
            0,                   # param3 (unused)
            0,                   # param4 (unused)
            float(output_function),  # param5: ACTUATOR_OUTPUT_FUNCTION
            0,                   # param6 (unused)
            0,                   # param7 (unused)
        )

        # Wait for the elevon to move into position
        time.sleep(0.2)

    # def

# class

# ======================================================================
# PositionReader: handles background serial reading + averaging
# ======================================================================
class PositionReader:
    def __init__(self, port):
        self.port = port
        self.baud = 115200
        self.timeout = 1.0
        self.num_samples=10

        self._queue_left = deque()
        self._queue_right = deque()
        self._lock = threading.Lock()
        self._thread = None
    # def

    def position_to_degrees(self,side,raw_position):
        if(side=="LEFT"):
            return (-0.0043 * raw_position) + 155.45
        elif(side=="RIGHT"):
            return (0.0039 * raw_position) - 115.8
        # if
    # def

    # Internal thread loop
    def _position_reader_loop(self):
        print(f"Opening position stream on {self.port} at {self.baud} baud...")
        try:
            with serial.Serial(self.port, baudrate=self.baud, timeout=self.timeout) as ser:
                while True:
                    line = ser.readline()
                    if not line:
                        continue
                    try:
                        text = line.decode("ascii", errors="ignore").strip()
                    except Exception:
                        continue

                    if not text:
                        continue

                    m = POSITION_REGEX.search(text)
                    if m:
                        try:
                            # Parse two integer fields; for now we use position1 as the sample.
                            p1 = int(m.group("position1"))
                            p2 = int(m.group("position2"))  # available if you need it

                            with self._lock:
                                self._queue_left.append(p1)
                                self._queue_right.append(p2)
                                # Keep queues bounded so it doesn't grow forever
                                if len(self._queue_left) > 500:
                                    self._queue_left.popleft()
                                if len(self._queue_right) > 500:
                                    self._queue_right.popleft()

                        except ValueError:
                            # Bad number format; ignore this line
                            pass
        except serial.SerialException as e:
            print(f"Error opening/reading {self.port}: {e}")
    # def

    def start(self):
        # Start the position reader in a daemon thread.
        if self._thread is not None and self._thread.is_alive():
            return

        t = threading.Thread(target=self._position_reader_loop, daemon=True)
        self._thread = t
        t.start()
    # def

    def clear_queues(self):
        # Clear any previously collected position samples.
        with self._lock:
            self._queue_left.clear()
            self._queue_right.clear()
    # def

    def get_average_position(self,side):

        samples = []
        deadline = time.time() + self.timeout

        if(side=="LEFT"):
            queue = self._queue_left
        elif(side=="RIGHT"):
            queue = self._queue_right
        # if

        while len(samples) < self.num_samples and time.time() < deadline:
            with self._lock:
                while queue and (len(samples) < self.num_samples):
                    samples.append(queue.popleft())
                # while
            # with
            if len(samples) < self.num_samples:
                time.sleep(0.01)
            # if

        if not samples:
            return None
        # if

        return sum(samples) / len(samples)
    # def

# class PositionReader

class Calibrator:
    def __init__(self, side, drone_interface, position_reader):
        self.side            = side
        self.drone_interface = drone_interface
        self.position_reader = position_reader

        # Select the parameter to hijack the side control
        if self.side == "LEFT":
            self.min_param  = "PWM_MAIN_MIN5"
            self.max_param  = "PWM_MAIN_MAX5"
            self.trim_param = "CA_SV_CS0_TRIM"
            self.output_function = 1201 # LEFT_ELEVON
        elif self.side == "RIGHT":
            self.min_param       = "PWM_MAIN_MIN6"
            self.max_param       = "PWM_MAIN_MAX6"
            self.trim_param      = "CA_SV_CS1_TRIM"
            self.output_function = 1202 # RIGHT_ELEVON
        else:
            print(f"Unsupported side position [{self.side}]")
        # if

    # def

    def cmd_get_deg(self, value):

        # Move the elevon
        self.drone_interface.command_elevon(self.output_function, value)

        # Clear queue and gather 10 fresh samples
        self.position_reader.clear_queues()
        avg_pos = self.position_reader.get_average_position(self.side)

        # Translate to degrees
        avg_deg = self.position_reader.position_to_degrees(self.side, avg_pos)
        # print("Elevon[%d] Pos[%.1f] Angle[%.1f]" % (self.output_function,avg_pos,avg_deg))
        return avg_deg
    # def

    def old_calibrate_servo(self):

        print("")
        print("*** Calibrating %s elevon ***" % (self.side))

        # Desired FINAL angles at the normalized command endpoints (u = ±1)
        DESIRED_FULL_POS_DEGS = -55.0
        DESIRED_FULL_NEG_DEGS =  55.0
        MAX_ABS_DEGS          =  55.0
        DESIRED_TRIM_DEGS     =  10.0

        TEST_POS_DEGS = -45.0
        TEST_NEG_DEGS =  45.0

        INITIAL_CENTRE_PWM = 1500
        INITIAL_SWING_PWM  =  500
        INITIAL_MIN_PWM    = INITIAL_CENTRE_PWM - INITIAL_SWING_PWM  # 1000
        INITIAL_MAX_PWM    = INITIAL_CENTRE_PWM + INITIAL_SWING_PWM  # 2000
        INITIAL_RANGE_PWM  = INITIAL_MAX_PWM - INITIAL_MIN_PWM        # 1000

        # Command magnitude that corresponds to ±45° in the controller's internal model
        TEST_POS_CMD =  TEST_POS_DEGS / DESIRED_FULL_POS_DEGS
        TEST_NEG_CMD = -TEST_POS_CMD

        # Set datum config
        print("*** Clearing down to datum ***")
        self.drone_interface.set_param_value("VT_ELEV_MC_LOCK", bool, True)
        self.drone_interface.set_param_value(self.min_param,  int,   INITIAL_MIN_PWM)
        self.drone_interface.set_param_value(self.max_param,  int,   INITIAL_MAX_PWM)
        self.drone_interface.set_param_value(self.trim_param, float, 0.0)
        self.drone_interface.command_elevon(self.output_function, 0.0)

        # Initial sweep
        test_degs = -45.0
        test_cmd = -test_degs / MAX_ABS_DEGS # +ve cmd => -ve degrees
        print("*** Testing (%.1f / %.3f) ***" % (test_degs,test_cmd))
        fullPosDegs = self.cmd_get_deg( test_cmd)
        zeroDegs    = self.cmd_get_deg( 0.0)     
        fullNegDegs = self.cmd_get_deg(-test_cmd)
        print("Pre-scale degs[%.1f, %.1f, %.1f]" % (fullPosDegs, zeroDegs, fullNegDegs))

        # Calculate correction
        error_max_deg = fullPosDegs - TEST_POS_DEGS
        error_min_deg = fullNegDegs - TEST_NEG_DEGS
        pwm_min_per_deg = (float(INITIAL_SWING_PWM) * -test_cmd) / (zeroDegs - fullNegDegs)
        pwm_max_per_deg = (float(INITIAL_SWING_PWM) * -test_cmd) / (fullPosDegs - zeroDegs)
        pwm_per_deg     = (float(INITIAL_RANGE_PWM) *  test_cmd) / (fullNegDegs - fullPosDegs)
        min_pwm_offset = int(error_min_deg * pwm_min_per_deg)
        max_pwm_offset = int(error_max_deg * pwm_max_per_deg)

        # Load correction
        corrected_min = INITIAL_MIN_PWM + min_pwm_offset
        corrected_max = INITIAL_MAX_PWM + max_pwm_offset
        print("*** Loading [%s] [%s:%d] [%s:%d] ***" % (self.side,self.min_param,corrected_min,self.max_param,corrected_max))
        self.drone_interface.set_param_value(self.min_param,  int,  corrected_min)
        self.drone_interface.set_param_value(self.max_param,  int,  corrected_max)

        self.drone_interface.set_param_value(self.min_param,  int,  corrected_min)
        self.drone_interface.set_param_value(self.max_param,  int,  corrected_max)

        # Retest
        print("*** Retesting (%.1f / %.3f) ***" % (test_degs,test_cmd))
        fullPosDegs = self.cmd_get_deg( test_cmd)
        fullNegDegs = self.cmd_get_deg(-test_cmd)
        zeroDegs    = self.cmd_get_deg( 0.0)     
        print("Post-scale degs[%.1f, %.1f, %.1f]" % (fullPosDegs, zeroDegs, fullNegDegs))

    # def

    def calibrate_trim(self,param_name,des_degs,direction):

        param_val  = 0.0
        act_degs = self.cmd_get_deg(0.0)
        error = des_degs-act_degs

        while(abs(error) > 0.5):
            act_degs = self.cmd_get_deg(0.3) # Move out to avoid backlash errors
            act_degs = self.cmd_get_deg(0.0)
            error = des_degs-act_degs
            unit_per_deg = 1.0 / 55.0
            increment = error * unit_per_deg
            param_val -= increment
            print("Diff[%.1f] %s[%0.2f]" % (error,param_name,param_val))
            self.drone_interface.set_param_value(param_name, float, param_val)
        # while
    # def

    def calibrate_centre(self,min_param_name,initial_min_param_val,max_param_name,initial_max_param_val,direction):
        min_param_val  = initial_min_param_val
        max_param_val  = initial_max_param_val
        des_degs = 0.0

        act_degs = self.cmd_get_deg(0.0)
        error = des_degs-act_degs
        while(abs(error) > 0.5):
            act_degs = self.cmd_get_deg(0.3) # Move out to avoid backlash errors
            act_degs = self.cmd_get_deg(0.0)
            error = des_degs-act_degs
            pwm_per_deg = 500.0 / 55.0
            increment = error * pwm_per_deg * direction
            min_param_val -= increment
            max_param_val -= increment
            print("Error[%.1f] %s[%d] %s[%d]" % (error,min_param_name,min_param_val,max_param_name,max_param_val))
            self.drone_interface.set_param_value(min_param_name, int, min_param_val)
            self.drone_interface.set_param_value(max_param_name, int, max_param_val)
        # while
        return(min_param_val,max_param_val)
    # def

    def calibrate_min_max(self,des_degs,max_des_degs,param_name,initial_param_val,direction):
        param_val  = initial_param_val
        cmd = -des_degs / max_des_degs # +ve cmd => -ve degrees
        act_degs = self.cmd_get_deg(cmd)
        error = des_degs-act_degs
        while(abs(error) > 0.5):
            act_degs = self.cmd_get_deg(0.0) # Move back to start point to avoid backlash errors
            act_degs = self.cmd_get_deg(cmd)
            error = des_degs-act_degs
            pwm_per_deg = 500.0 / 55.0
            increment = error * pwm_per_deg * direction
            param_val -= increment
            print("Error[%.1f] %s[%d]" % (error,param_name,param_val))
            self.drone_interface.set_param_value(param_name, int, param_val)
        # while
        return(param_val)
    # def

    def tramline_params(self,max_param_val,min_param_val):

        if(max_param_val < 1700):
            print("Too low max value [%d]" % (max_param_val))
            max_param_val = 1700
        elif(max_param_val > 2300):
            print("Too high max value [%d]" % (max_param_val))
            max_param_val = 2300
        # if

        if(min_param_val < 700):
            print("Too low min value [%d]" % (min_param_val))
            min_param_val = 700
        elif(min_param_val > 1300):
            print("Too high min value [%d]" % (min_param_val))
            min_param_val = 1300
        # if

        return(max_param_val,min_param_val)
    # def

    def calibrate_servo(self):

        print("")
        print("*** Calibrating %s elevon ***" % (self.side))

        INITIAL_CENTRE_PWM = 1500
        INITIAL_SWING_PWM  =  500
        INITIAL_MIN_PWM    = INITIAL_CENTRE_PWM - INITIAL_SWING_PWM  # 1000
        INITIAL_MAX_PWM    = INITIAL_CENTRE_PWM + INITIAL_SWING_PWM  # 2000
        INITIAL_RANGE_PWM  = INITIAL_MAX_PWM - INITIAL_MIN_PWM
        TEST_DEGS          =  30.0
        MAX_ABS_DEGS       =  55.0

        # Set datum config
        print("*** Clearing down to datum ***")
        self.drone_interface.set_param_value("VT_ELEV_MC_LOCK", bool, True)
        self.drone_interface.set_param_value(self.trim_param, float, 0.0)
        self.drone_interface.command_elevon(self.output_function, 0.0)

        # Select increment
        if(self.side=="LEFT"):
            direction =  -1.0 # Left has range reversed
        elif(self.side=="RIGHT"):
            direction =  1.0
        # if

        # Initialise
        iteration = 1
        cmd = TEST_DEGS / MAX_ABS_DEGS
        min_param_val = INITIAL_MIN_PWM
        max_param_val = INITIAL_MAX_PWM
        diff          = TEST_DEGS
        neg_undershoot    = TEST_DEGS
        pos_undershoot    = TEST_DEGS

        print("*** Balancing ***")
        while(abs(diff)>0.3):
            print("Iteration [%d] [%d/%d]" % (iteration,min_param_val,max_param_val))
            self.drone_interface.set_param_value(self.min_param,  int,   min_param_val)
            self.drone_interface.set_param_value(self.max_param,  int,   max_param_val)
            fullPosDegs = self.cmd_get_deg( cmd)
            fullNegDegs = self.cmd_get_deg(-cmd)
            deg_range = fullNegDegs - fullPosDegs
            diff       = (fullNegDegs + fullPosDegs) / 2.0
            pwm_per_deg = (float(INITIAL_RANGE_PWM) * cmd) / deg_range

            print("Degs[%.1f, %.1f] Range[%.1f] Diff[%.1f]" % (fullPosDegs, fullNegDegs, deg_range, diff))

            # Iterate settings
            min_param_val += (diff * pwm_per_deg * direction)
            max_param_val += (diff * pwm_per_deg * direction)

            # Tramline
            (max_param_val,min_param_val) = self.tramline_params(max_param_val,min_param_val)

            iteration += 1
        # while

        iteration = 1

        print("*** Scaling ***")
        while( (abs(diff)>0.5) or (abs(neg_undershoot)>0.5) or (abs(pos_undershoot)>0.5) ):
            print("Iteration [%d] [%d/%d]" % (iteration,min_param_val,max_param_val))
            self.drone_interface.set_param_value(self.min_param,  int,   min_param_val)
            self.drone_interface.set_param_value(self.max_param,  int,   max_param_val)
            fullPosDegs = self.cmd_get_deg( cmd)
            fullNegDegs = self.cmd_get_deg(-cmd)
            deg_range = fullNegDegs - fullPosDegs
            diff       = (fullNegDegs + fullPosDegs) / 2.0
            pos_undershoot =  fullPosDegs + TEST_DEGS 
            neg_undershoot =  TEST_DEGS - fullNegDegs
            pwm_per_deg = (float(INITIAL_RANGE_PWM) * cmd) / deg_range

            print("Degs[%.1f, %.1f] Range[%.1f] Diff[%.1f] Under[%.1f/%.1f]" % (fullPosDegs, fullNegDegs, deg_range, diff, pos_undershoot, neg_undershoot))

            # Iterate settings
            max_param_val += (pos_undershoot * pwm_per_deg)
            min_param_val -= (neg_undershoot * pwm_per_deg)

            # Tramline
            (max_param_val,min_param_val) = self.tramline_params(max_param_val,min_param_val)

            iteration += 1
        # while

        print("*** Trimming ***")
        self.calibrate_trim(self.trim_param, 10.0, direction)

        # Post-test
        print("*** Post-trim testing ***")
        fullPosDegs = self.cmd_get_deg( cmd)
        zeroDegs    = self.cmd_get_deg( 0.0)     
        fullNegDegs = self.cmd_get_deg(-cmd)
        print("Post-trim degs[%.1f, %.1f, %.1f]" % (fullPosDegs, zeroDegs, fullNegDegs))

        # Release lock
        print("*** Lock release ***")
        self.drone_interface.set_param_value("VT_ELEV_MC_LOCK", bool, False)
    # def

# class Calibrator


# ======================================================================
# Main script usage
# ======================================================================

if __name__ == "__main__":
    # MAVLink on COM3
    drone_interface = DroneInterface("COM20")
    drone_interface.connect()

    # Start reading position from COM5 in the background
    position_reader = PositionReader("COM5")
    position_reader.start()

    # Calibrator that uses that position reader
    left_calibrator  = Calibrator("LEFT",  drone_interface, position_reader)
    right_calibrator = Calibrator("RIGHT", drone_interface, position_reader)

    # Run calibration using COM5 as the resolver port
    left_calibrator.calibrate_servo()
    right_calibrator.calibrate_servo()

    print("Closing MAVLink connection...")
    drone_interface.master.close()
