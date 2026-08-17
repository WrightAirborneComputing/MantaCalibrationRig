#!/usr/bin/env python3
"""Two-stage elevon endpoint calibration - an offline experiment. Not the GUI.

The rig's current calibration measures the elevon angle *while creeping* toward
it in 6 us steps, converts the command it stopped at into a PWM value, and
writes that as the endpoint. Measured on this airframe, a crept-to position and
a rapidly-driven position at the same PWM differ by around 2.4 deg, so the
endpoint that gets written is not the one the surface reaches in service.

This script closes that loop the way the manual procedure did:

  Stage 1  creep to each target with the range wide open, purely to get close.
           Nothing measured here is trusted or committed.
  Stage 2  command the endpoint, let the servo slew to it in one motion, and
           measure. Correct the PWM, then evaluate the *other* end - the
           traverse between ends is itself the next rapid approach, so
           alternating MAX, MIN, MAX, MIN costs nothing extra.

An endpoint is set when both hold:
  - no hard stop: 50 us or less of inward travel is enough to move the elevon
  - the angle on the rapid approach is within 0.5 deg of target

Trim is deliberately out of scope: stage 2 runs at trim 0 and the trim question
(what MIN/MAX should be once a nonzero trim clamps the command) is unresolved.

    python3 endpoint_cal.py set --side LEFT       # runs both stages, WRITES params
    python3 endpoint_cal.py verify --side LEFT    # read-only re-measurement

`set` writes PWM_MAIN_MIN/MAX and restores the values it started with if it
cannot converge. `verify` writes nothing.
"""

import argparse
import json
import os
import struct
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink is not installed: pip install -r requirements.txt")

import servo_probe
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
    clamp_endpoint,
    command_delta_for_pwm,
    correction_us,
    endpoint_accepted,
    endpoint_command,
    hard_stop_verdict,
    inward_sign,
)
from manta_common import find_pico_port

APP_DIR = os.path.dirname(os.path.abspath(__file__))
SETTINGS_PATH = os.path.join(APP_DIR, "settings.json")

# ------------------------------------------------------------------ plumbing


def load_targets():
    with open(SETTINGS_PATH, "r") as handle:
        angles = json.load(handle)["ANGLES"]
    return float(angles["angle_neg_degs"]), float(angles["angle_pos_degs"])
# def


def set_param(fcu, name, py_type, value, verify_timeout=5.0):
    """param_set with readback verification. The only writer in this project
    outside MantaTrimmer, kept here so servo_probe.py stays read-only."""
    if py_type is int:
        target = int(round(value))
        wire = struct.unpack("<f", struct.pack("<i", target))[0]
        send_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    else:
        target = float(value)
        wire = target
        send_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    deadline = time.time() + verify_timeout
    while time.time() < deadline:
        fcu.master.mav.param_set_send(
            fcu.master.target_system, fcu.master.target_component,
            name.encode("ascii"), wire, send_type)

        readback = fcu.get_param(name, timeout=1.0)
        if readback is not None:
            if py_type is int and int(readback) == target:
                return True
            if py_type is float and abs(float(readback) - target) < 1e-6:
                return True
        time.sleep(0.2)

    print("  FAILED to set %s = %s" % (name, value))
    return False
# def


class Rig:
    """The FC, the Pico and the one side under calibration."""

    def __init__(self, fcu, pico, side):
        self.fcu = fcu
        self.pico = pico
        self.side = side
        self.function = servo_probe.SIDES[side]["function"]
        self.min_param = servo_probe.SIDES[side]["min_param"]
        self.max_param = servo_probe.SIDES[side]["max_param"]
        self.trim_param = servo_probe.SIDES[side]["trim_param"]

    def params(self):
        return servo_probe.read_side_params(self.fcu, self.side)
    # def

    def measure_at(self, cmd, dwell_s=ENDPOINT_DWELL_S):
        """Drive to a command in one motion, dwell, and read the angle.

        Short by design: lingering on a possible hard stop is what the back-off
        probe exists to avoid. The dwell is one averaging window plus margin,
        which is the floor for a reading that means anything.
        """
        self.fcu.actuator_test(self.function, cmd)
        time.sleep(dwell_s)
        angle = self.pico.angle(self.side)

        # Refresh before the shell read, or the override lapses mid-check.
        self.fcu.actuator_test(self.function, cmd, wait_ack=False)
        pwm = servo_probe.read_side_pwm(self.fcu, self.side)
        return angle, pwm
    # def

    def centre(self):
        self.fcu.actuator_test(self.function, 0.0, wait_ack=False)
    # def
# class


def probe_breakaway(rig, params, which, cmd_endpoint, angle_at_endpoint):
    """Back off inward until the elevon moves, and report how far that took.

    Returns (breakaway_us, angle_when_it_moved). Movement is measured against
    the angle at the endpoint, so a surface pinned on a mechanical stop simply
    never registers one.
    """
    backed = 0.0
    while backed < BACKOFF_CEILING_US:
        backed += BACKOFF_STEP_US
        delta_cmd = command_delta_for_pwm(
            inward_sign(which) * backed, params["pwm_min"], params["pwm_max"],
            params["rev"])
        cmd = servo_probe.clamp(cmd_endpoint + delta_cmd, -1.0, 1.0)

        angle, _ = rig.measure_at(cmd)
        if angle is None:
            return None, None

        if abs(angle - angle_at_endpoint) >= MOVEMENT_THRESHOLD_DEG:
            return backed, angle

    return None, None
# def


def evaluate_endpoint(rig, which, target_deg, attempt):
    """One rapid approach: measure, probe for a hard stop, and decide.

    Returns a dict describing what happened; the caller writes the new value.
    """
    params = rig.params()
    if params is None:
        return None

    pwm_now = params["pwm_max"] if which == "MAX" else params["pwm_min"]
    cmd = endpoint_command(which, params["rev"])

    angle, pwm_read = rig.measure_at(cmd)
    if angle is None:
        print("  no angle from the rig")
        return None

    ok, actual, expected = servo_probe.check_authority(rig.fcu, rig.side, params, cmd)
    if not ok:
        print("  lost command authority (read %s, expected %d)" % (actual, expected))
        return None

    breakaway, angle_backed = probe_breakaway(rig, params, which, cmd, angle)
    is_hard_stop, pull_in_us = hard_stop_verdict(breakaway)
    gain = (None if breakaway is None
            else angle_gain_per_us(angle, angle_backed, breakaway, which))

    result = {
        "which": which, "attempt": attempt, "pwm": pwm_now, "pwm_read": pwm_read,
        "cmd": cmd, "angle_deg": angle, "target_deg": target_deg,
        "error_deg": angle - target_deg, "breakaway_us": breakaway,
        "hard_stop": is_hard_stop, "gain_deg_per_us": gain,
        "accepted": endpoint_accepted(angle, target_deg, is_hard_stop),
        "new_pwm": None,
    }

    if result["accepted"]:
        return result

    if is_hard_stop:
        result["new_pwm"] = clamp_endpoint(pwm_now + inward_sign(which) * pull_in_us)
        result["reason"] = "hard stop, pulling in %.0f us" % pull_in_us
        return result

    shift = correction_us(angle, target_deg, gain)
    if shift is None:
        result["reason"] = "no usable gain from the probe"
        return result

    result["new_pwm"] = clamp_endpoint(pwm_now + shift)
    result["reason"] = "%.2f deg out, shifting %+.0f us" % (result["error_deg"], shift)
    return result
# def


def describe(result):
    parts = ["  %s attempt %d: pwm %d, angle %+.2f (target %+.2f)"
             % (result["which"], result["attempt"], result["pwm"],
                result["angle_deg"], result["target_deg"])]
    if result["breakaway_us"] is None:
        parts.append("no breakaway within %.0f us" % BACKOFF_CEILING_US)
    else:
        parts.append("breakaway %.0f us" % result["breakaway_us"])
        if result["gain_deg_per_us"]:
            parts.append("gain %+.4f deg/us" % result["gain_deg_per_us"])
    if result["accepted"]:
        parts.append("ACCEPTED")
    elif result.get("reason"):
        parts.append(result["reason"])
    return ", ".join(parts)
# def


def run_coarse_stage(rig, target_neg, target_pos, step_us):
    """Creep to both targets with the range wide open. Nothing here is trusted.

    Returns {"MAX": (pwm, target_deg), "MIN": (pwm, target_deg)} - which end
    carries which angle target is taken from the measurement, not inferred from
    the reversal bit.
    """
    print("\nStage 1: coarse creep")
    if not set_param(rig.fcu, rig.min_param, int, COARSE_MIN):
        return None
    if not set_param(rig.fcu, rig.max_param, int, COARSE_MAX):
        return None
    if not set_param(rig.fcu, rig.trim_param, float, 0.0):
        return None

    params = rig.params()
    if params is None:
        return None

    found = {}
    for target in (target_pos, target_neg):
        print("  creeping to %+.1f deg ..." % target)
        result = servo_probe.sweep_to_threshold(
            rig.fcu, rig.pico, rig.side, params, target, step_us,
            servo_probe.SWEEP_PERIOD_S, servo_probe.SWEEP_WINDOW_S, 240.0)
        if result is None or result["stop_pwm"] is None:
            print("  coarse creep to %+.1f deg failed" % target)
            return None
        print("    stopped at %d us (cmd %+.3f, reported %+.2f deg)"
              % (result["stop_pwm"], result["stop_cmd"], result["reported_deg"]))
        found[target] = result["stop_pwm"]
        rig.centre()
        time.sleep(1.0)

    hi_target = max(found, key=lambda t: found[t])
    lo_target = min(found, key=lambda t: found[t])
    return {"MAX": (found[hi_target], hi_target), "MIN": (found[lo_target], lo_target)}
# def


def run_refine_stage(rig, endpoints, rows):
    """Alternate MAX, MIN, MAX, MIN until both are set or one runs out of tries."""
    print("\nStage 2: rapid-approach refinement")

    # Every measured approach has to be a full-span traverse. The verify pass
    # showed MAX reading -36.04 when approached from centre but -37.7 twice
    # running when approached from MIN, and the alternation only guarantees a
    # full-span run-up from the *second* evaluation onward. So prime it with one
    # unmeasured traverse to the far end, and the first MAX reading matches
    # every later one.
    primer = rig.params()
    if primer is not None:
        rig.measure_at(endpoint_command("MIN", primer["rev"]))

    attempts = {"MAX": 0, "MIN": 0}
    accepted = {"MAX": False, "MIN": False}

    for which in alternating_order(MAX_ATTEMPTS):
        if accepted["MAX"] and accepted["MIN"]:
            break
        if accepted[which]:
            continue

        attempts[which] += 1
        if attempts[which] > MAX_ATTEMPTS:
            print("  %s did not converge in %d attempts" % (which, MAX_ATTEMPTS))
            return False

        result = evaluate_endpoint(rig, which, endpoints[which][1], attempts[which])
        if result is None:
            return False

        rows.append(result)
        print(describe(result))

        if result["accepted"]:
            accepted[which] = True
            continue

        if result["new_pwm"] is None:
            return False

        param = rig.max_param if which == "MAX" else rig.min_param
        if not set_param(rig.fcu, param, int, result["new_pwm"]):
            return False

    if not (accepted["MAX"] and accepted["MIN"]):
        print("  ran out of attempts with MAX=%s MIN=%s"
              % (accepted["MAX"], accepted["MIN"]))
        return False
    return True
# def


def run_set(args):
    rig, fcu, pico, restore = None, None, None, None
    try:
        fcu, pico = connect(args)
        if fcu is None:
            return 2
        rig = Rig(fcu, pico, args.side)

        before = rig.params()
        if before is None:
            print("Could not read the side's parameters")
            return 3
        restore = (before["pwm_min"], before["pwm_max"], before["trim"])
        print("Starting values: MIN=%d MAX=%d TRIM=%+.4f REV=%s"
              % (restore[0], restore[1], restore[2], before["rev"]))

        target_neg, target_pos = load_targets()
        print("Targets: %+.1f / %+.1f deg" % (target_neg, target_pos))

        endpoints = run_coarse_stage(rig, target_neg, target_pos, args.step_us)
        if endpoints is None:
            restore_params(rig, restore)
            return 4

        print("  coarse: MAX=%d (%+.1f deg), MIN=%d (%+.1f deg)"
              % (endpoints["MAX"][0], endpoints["MAX"][1],
                 endpoints["MIN"][0], endpoints["MIN"][1]))
        if not set_param(fcu, rig.max_param, int, endpoints["MAX"][0]):
            restore_params(rig, restore)
            return 4
        if not set_param(fcu, rig.min_param, int, endpoints["MIN"][0]):
            restore_params(rig, restore)
            return 4

        rows = []
        converged = run_refine_stage(rig, endpoints, rows)
        rig.centre()

        after = rig.params()
        if converged:
            print("\n%s endpoints set: MIN=%d MAX=%d"
                  % (args.side, after["pwm_min"], after["pwm_max"]))
        else:
            print("\nCalibration failed - restoring the starting values")
            restore_params(rig, restore)

        if rows:
            print("Wrote %s" % servo_probe.write_rows(rows, RESULT_COLUMNS,
                                                      "endpoint_cal.csv"))
        return 0 if converged else 5
    finally:
        shutdown(fcu, pico, rig)
# def


def restore_params(rig, restore):
    if restore is None:
        return
    pwm_min, pwm_max, trim = restore
    set_param(rig.fcu, rig.min_param, int, pwm_min)
    set_param(rig.fcu, rig.max_param, int, pwm_max)
    set_param(rig.fcu, rig.trim_param, float, trim)
    print("Restored MIN=%d MAX=%d TRIM=%+.4f" % (pwm_min, pwm_max, trim))
# def


def run_verify(args):
    """Re-measure the endpoints at whatever MIN/MAX are currently set. No writes."""
    rig, fcu, pico = None, None, None
    try:
        fcu, pico = connect(args)
        if fcu is None:
            return 2
        rig = Rig(fcu, pico, args.side)

        params = rig.params()
        if params is None:
            return 3
        target_neg, target_pos = load_targets()
        targets = {"MAX": None, "MIN": None}

        # Which end carries which target follows from the reversal, and is
        # confirmed by the measurement itself below.
        if params["rev"]:
            targets["MAX"], targets["MIN"] = target_neg, target_pos
        else:
            targets["MAX"], targets["MIN"] = target_pos, target_neg

        print("\n%s  MIN=%d MAX=%d TRIM=%+.4f REV=%s"
              % (args.side, params["pwm_min"], params["pwm_max"], params["trim"],
                 params["rev"]))
        if abs(params["trim"]) > 1e-6:
            print("  WARNING: trim is %+.4f, so cmd +/-1 no longer reaches both\n"
                  "  endpoints - one end will read short by |trim|/2 * span.\n"
                  "  These numbers only mean what they look like at trim 0."
                  % params["trim"])

        print("  round  end   pwm    angle   target    error")

        rows = []
        for index, which in enumerate(alternating_order(args.rounds)):
            cmd = endpoint_command(which, params["rev"])
            angle, pwm = rig.measure_at(cmd)
            if angle is None:
                print("  no angle from the rig")
                continue
            error = angle - targets[which]
            rows.append({"which": which, "attempt": index // 2 + 1, "pwm": pwm,
                         "cmd": cmd, "angle_deg": round(angle, 2),
                         "target_deg": targets[which], "error_deg": round(error, 2),
                         "accepted": abs(error) <= ENDPOINT_TOLERANCE_DEG})
            print("  %5d  %-4s %5s  %+7.2f  %+7.2f  %+7.2f%s"
                  % (index // 2 + 1, which, pwm, angle, targets[which], error,
                     "" if abs(error) <= ENDPOINT_TOLERANCE_DEG else "   OUT"))

        rig.centre()
        if rows:
            print("\nWrote %s" % servo_probe.write_rows(rows, RESULT_COLUMNS,
                                                        "endpoint_verify.csv"))
        return 0
    finally:
        shutdown(fcu, pico, rig)
# def


RESULT_COLUMNS = ["which", "attempt", "pwm", "pwm_read", "cmd", "angle_deg",
                  "target_deg", "error_deg", "breakaway_us", "hard_stop",
                  "gain_deg_per_us", "accepted", "new_pwm", "reason"]


def connect(args):
    fcu = servo_probe.Fcu(args.port or servo_probe.default_fcu_port())
    if fcu.device is None or not fcu.connect():
        return None, None

    if not fcu.open_shell():
        fcu.close()
        return None, None
    fcu.output_instance = fcu.find_live_instance()
    if fcu.output_instance is None:
        print("actuator_outputs is not published; nothing to read back")
        fcu.close()
        return None, None

    pico_port = args.pico_port or find_pico_port()
    if pico_port is None:
        print("This needs the Pico for angles; none found")
        fcu.close()
        return None, None

    pico = servo_probe.PicoAngles(pico_port, servo_probe.load_scalers())
    pico.start()
    return fcu, pico
# def


def shutdown(fcu, pico, rig):
    try:
        if rig is not None:
            rig.centre()
    except Exception:
        pass
    if pico is not None:
        pico.stop()
    if fcu is not None:
        fcu.close()
# def


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("mode", choices=("set", "verify"), nargs="?", default="verify")
    parser.add_argument("--side", choices=("LEFT", "RIGHT"), default="LEFT")
    parser.add_argument("--port", help="MAVLink device or connection string")
    parser.add_argument("--pico-port", help="Pico serial device")
    parser.add_argument("--step-us", type=float, default=servo_probe.SWEEP_STEP_US,
                        dest="step_us", help="coarse creep step")
    parser.add_argument("--rounds", type=int, default=3,
                        help="verify: rapid approaches per end")
    parser.add_argument("--yes", action="store_true", help="skip the prompt")
    args = parser.parse_args(argv)

    if not args.yes:
        if args.mode == "set":
            print("This MOVES %s through both end stops and WRITES PWM_MAIN_MIN/MAX. "
                  "The starting values are restored if it cannot converge." % args.side)
        else:
            print("This moves %s to both end stops. It writes nothing." % args.side)
        if input("Continue? [y/N] ").strip().lower() not in ("y", "yes"):
            return 1

    return run_set(args) if args.mode == "set" else run_verify(args)
# def


if __name__ == "__main__":
    sys.exit(main())
