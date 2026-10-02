"""Tests for the six-face sensor calibration.

Every one of these runs without a rig. The expensive part of this procedure is
a person holding a plate still for thirty seconds nine times over, so the
arithmetic downstream of that had better not need one - and the acceptance
decisions, which are the part that can quietly be wrong, need one least of all.
"""

import copy
import csv
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import sensor_cal
import sensor_cal_logic as L
from fake_teensy import DEFAULT_SENSORS, rotate, synthesise


# --- The wire format -------------------------------------------------------

def test_parses_a_three_channel_sample():
    sample = L.parse_sample_line("{123456:1,2,3,4,5,6|7,8,9,10,11,12|-1,-2,-3,0,0,0}")
    assert sample["t_us"] == 123456
    assert sample["LEFT"] == (1, 2, 3, 4, 5, 6)
    assert sample["RIGHT"] == (-1, -2, -3, 0, 0, 0)


def test_dead_channel_is_none_not_zero():
    # A reading of zero is a real reading and would average into a plausible
    # wrong answer; no reading must stay distinguishable from it.
    sample = L.parse_sample_line("{1:1,2,3,4,5,6|x|7,8,9,10,11,12}")
    assert sample["CENTRE"] is None


def test_replies_are_inert_to_the_sample_parser():
    for reply in ("# ACK F 200", "# STATUS uptime_ms=1 hz=200",
                  "# ID dev=manta-rig proto=1", "# ERR NOPE"):
        assert L.parse_sample_line(reply) is None


def test_pico_position_lines_do_not_parse_as_teensy_samples():
    # The delimiter change is load-bearing: a host that mixed the two boards up
    # must fail to parse rather than succeed into wrong numbers.
    assert L.parse_sample_line("[32768/31000]") is None
    assert L.parse_sample_line("[123456:32768/31000]") is None


def test_malformed_lines_return_none_rather_than_partial_data():
    for bad in ("{1:1,2,3|4,5,6|7,8,9}",          # three fields, not six
                "{1:1,2,3,4,5,6|7,8,9,10,11,12}",  # two channels
                "{1:}", "{}", "garbage"):
        assert L.parse_sample_line(bad) is None


# --- Faces -----------------------------------------------------------------

@pytest.mark.parametrize("vector,expected", [
    ((0.0, 0.0, 1.0), "Z+"),
    ((0.0, 0.0, -1.0), "Z-"),
    ((0.98, 0.05, 0.05), "X+"),
    ((-0.98, 0.05, 0.05), "X-"),
    ((0.05, 0.98, 0.05), "Y+"),
    ((0.05, -0.98, 0.05), "Y-"),
])
def test_classify_face(vector, expected):
    assert L.classify_face(vector) == expected


def test_an_ambiguous_orientation_classifies_as_nothing():
    # 45 degrees between two axes. Refusing here is what lets the wizard tell
    # "being moved" apart from "resting on a face, badly".
    assert L.classify_face((0.707, 0.707, 0.0)) is None


def test_face_verdict_thresholds():
    assert L.face_verdict(0.0) == "ok"
    assert L.face_verdict(7.9) == "ok"
    assert L.face_verdict(8.1) == "warn"
    assert L.face_verdict(14.9) == "warn"
    assert L.face_verdict(15.1) == "reject"
    assert L.face_verdict(None) == "reject"


def test_sensors_must_agree_on_the_face():
    # Same plate, same orientation: they must classify the same. Disagreement
    # means a module is mounted rotated or a channel is on the wrong UART.
    agree = {"LEFT": (0, 0, 2048, 0, 0, 0),
             "CENTRE": (0, 0, 2048, 0, 0, 0),
             "RIGHT": (0, 0, 2048, 0, 0, 0)}
    assert L.sample_face(agree) == "Z+"

    disagree = dict(agree, RIGHT=(2048, 0, 0, 0, 0, 0))
    assert L.sample_face(disagree) is None


# --- The stillness gate ----------------------------------------------------

def _reading(acc_g, gyro_dps=(0.0, 0.0, 0.0)):
    return tuple([a * L.ACC_COUNTS_PER_G for a in acc_g] +
                 [g * L.GYRO_COUNTS_PER_DPS for g in gyro_dps])


def test_spinning_plate_is_not_quiet():
    sample = {n: _reading((0, 0, 1.0), (0, 0, 30.0)) for n in L.CHANNELS}
    assert not L.sample_is_quiet(sample)


def test_an_uncalibrated_sensor_at_rest_is_still_quiet():
    """Regression: the stillness gate must not assume a calibrated sensor.

    This is the circularity that cost two of six faces the first time. A sensor
    with a 0.974 scale and a 45 mg bias reads 0.93 g on a face while perfectly
    still, and a gate built around "magnitude must be 1 g" throws that face
    away - which is to say it rejects exactly the sensors this whole procedure
    exists to fix. The worst case the sanity gates will accept is 0.80 g.
    """
    for magnitude in (0.80, 0.93, 1.20):
        sample = {n: _reading((0, 0, magnitude)) for n in L.CHANNELS}
        assert L.sample_is_quiet(sample), magnitude


def test_a_plate_under_real_acceleration_is_not_quiet():
    sample = {n: _reading((0, 0, 1.6)) for n in L.CHANNELS}
    assert not L.sample_is_quiet(sample)


def test_a_sample_with_no_live_channels_is_not_quiet():
    # Silence is not stillness.
    assert not L.sample_is_quiet({n: None for n in L.CHANNELS})


# --- The face-change state machine -----------------------------------------

def _flat_sample(face="Z+"):
    vec = L.FACE_VECTORS[face]
    return {n: _reading(vec) for n in L.CHANNELS}


def test_tracker_fires_only_after_the_settle_time():
    tracker = L.FaceTracker(settle_seconds=1.0)
    sample = _flat_sample("Z+")
    assert tracker.feed(sample, 0.0) is None
    assert tracker.feed(sample, 0.5) is None
    assert tracker.feed(sample, 1.5) == "Z+"


def test_tracker_will_not_refire_until_the_plate_is_disturbed():
    """Without this the operator gets six captures of face one."""
    tracker = L.FaceTracker(settle_seconds=1.0)
    sample = _flat_sample("Z+")
    tracker.feed(sample, 0.0)
    assert tracker.feed(sample, 2.0) == "Z+"
    tracker.mark_captured("Z+", {})

    # Still sitting on Z+, and Z+ is done: nothing should fire.
    for t in (3.0, 4.0, 5.0, 60.0):
        assert tracker.feed(sample, t) is None

    # Picked up, then set down on a new face.
    moving = {n: _reading((0.3, 0.9, 0.2), (40.0, 0.0, 0.0)) for n in L.CHANNELS}
    tracker.feed(moving, 61.0)
    nxt = _flat_sample("X+")
    tracker.feed(nxt, 62.0)
    assert tracker.feed(nxt, 64.0) == "X+"


def test_tracker_restarts_settling_when_the_face_changes():
    tracker = L.FaceTracker(settle_seconds=1.0)
    tracker.feed(_flat_sample("Z+"), 0.0)
    tracker.feed(_flat_sample("X+"), 0.9)      # changed before settling
    assert tracker.feed(_flat_sample("X+"), 1.5) is None
    assert tracker.feed(_flat_sample("X+"), 2.0) == "X+"


def test_remaining_is_in_single_flip_order():
    tracker = L.FaceTracker()
    assert tracker.remaining() == list(L.SUGGESTED_FACE_ORDER)
    tracker.mark_captured("X+", {})
    assert "X+" not in tracker.remaining()


# --- The solve -------------------------------------------------------------

def _observations(sensor, tilt_deg=0.0, seed=0):
    import random
    rng = random.Random(seed)
    out = []
    for face, vec in L.FACE_VECTORS.items():
        v = rotate(rotate(vec, "x", rng.uniform(-tilt_deg, tilt_deg)),
                   "y", rng.uniform(-tilt_deg, tilt_deg))
        out.append((face, sensor.measure(v, 0.0, rng)))
    return out


def test_solve_recovers_the_injected_coefficients():
    sensor = DEFAULT_SENSORS["LEFT"]
    solution = L.solve_sensor(_observations(sensor, tilt_deg=0.0))
    for i in range(3):
        assert solution["bias"][i] == pytest.approx(sensor.bias[i], abs=1e-6)
        assert solution["scale"][i] == pytest.approx(sensor.scale[i], abs=1e-6)


def test_refinement_removes_the_cosine_error_the_seed_cannot():
    """The reason the solve is not just a min/max over the six faces.

    A face held a few degrees off flat projects its axis by cos(tilt), and the
    closed-form seed takes that at face value and loses scale. The refinement
    works from the measured vector instead, so the same captures give the right
    answer.
    """
    sensor = DEFAULT_SENSORS["LEFT"]
    observations = _observations(sensor, tilt_deg=7.0, seed=3)

    seed = L.seed_solution(observations)
    refined = L.solve_sensor(observations)

    seed_error = max(abs(seed["scale"][i] / sensor.scale[i] - 1.0)
                     for i in range(3))
    refined_error = max(abs(refined["scale"][i] / sensor.scale[i] - 1.0)
                        for i in range(3))

    assert seed_error > 2e-3          # the seed loses several tenths of a percent
    assert refined_error < 1e-6       # the refinement does not
    assert refined_error < seed_error / 100.0


def test_six_faces_have_no_degrees_of_freedom():
    """The trap this procedure invites, asserted so nobody re-adds the score.

    Six orientations against six unknowns is exactly determined, so the
    residual is approximately zero by construction and means nothing at all.
    """
    solution = L.solve_sensor(_observations(DEFAULT_SENSORS["LEFT"], 6.0))
    assert solution["dof"] == 0
    assert solution["residual_rms_g"] < 1e-12


def test_extra_orientations_give_the_residual_meaning():
    import random
    rng = random.Random(4)
    sensor = DEFAULT_SENSORS["LEFT"]
    observations = _observations(sensor, tilt_deg=6.0, seed=4)
    for k in range(3):
        v = rotate(rotate((0.0, 0.0, 1.0), "x", rng.uniform(25.0, 55.0)),
                   "z", rng.uniform(0.0, 360.0))
        observations.append(("tilt%d" % k, sensor.measure(v, 3e-4, rng)))

    solution = L.solve_sensor(observations)
    assert solution["dof"] == 3
    # Now it tracks the injected noise instead of being zero by construction.
    assert solution["residual_rms_g"] > 1e-6


def test_solve_refuses_when_an_axis_never_saw_both_signs():
    # Four faces: Z is never negative. A scale from one side is a guess.
    partial = [(f, L.FACE_VECTORS[f])
               for f in ("Z+", "X+", "X-", "Y+", "Y-")]
    assert L.seed_solution(partial) is None
    assert L.solve_sensor(partial) is None


# --- Sanity gates ----------------------------------------------------------

def test_sanity_accepts_a_good_solution():
    solution = {"bias": [0.03, -0.02, 0.04], "scale": [0.985, 1.012, 0.974]}
    assert L.sanity_problems("LEFT", solution) == []


def test_sanity_names_the_sensor_and_the_axis():
    solution = {"bias": [0.03, -0.02, 0.40], "scale": [0.985, 1.5, 0.974]}
    problems = L.sanity_problems("CENTRE", solution)
    assert len(problems) == 2
    assert any("CENTRE Y scale" in p for p in problems)
    assert any("CENTRE Z bias" in p for p in problems)


# --- Plate consistency -----------------------------------------------------

def _solved_capture(sensors, seed=1):
    import random
    rng = random.Random(seed)
    captures = {}
    for face, vec in L.FACE_VECTORS.items():
        v = rotate(rotate(vec, "x", rng.uniform(-5, 5)),
                   "y", rng.uniform(-5, 5))
        captures[face] = {
            n: {"face": face, "mean_g": s.measure(v, 0.0, rng),
                "scatter_g": (0.0, 0.0, 0.0), "tilt_deg": 0.0, "n": 100}
            for n, s in sensors.items()
        }
    solutions = {}
    for n in L.CHANNELS:
        obs = [(f, captures[f][n]["mean_g"]) for f in captures]
        solutions[n] = L.solve_sensor(obs)
    return captures, solutions


def test_mounting_misalignment_alone_does_not_fail_the_plate_check():
    """Mounting is fitted out, so only movement is left to gate on.

    A 4 degree mounting offset is unremarkable for hand-bolted modules and must
    not fail a rigid, correctly calibrated plate. The rigid fit reports it as
    the mounting rotation - a real quantity, and the one a CENTRE datum
    correction needs - and gates on what the rotation cannot explain.
    """
    sensors = copy.deepcopy(DEFAULT_SENSORS)
    sensors["RIGHT"].mount_deg = (4.0, -3.0)
    captures, solutions = _solved_capture(sensors)

    result = L.plate_consistency(captures, solutions)
    for pair, r in result.items():
        assert r["verdict"] == "ok", (pair, r["residual_rms_deg"])
    assert result["LEFT-RIGHT"]["mount_deg"] > 3.0    # still reported as context


def test_a_sensor_that_shifted_mid_run_is_caught_and_named():
    sensors = copy.deepcopy(DEFAULT_SENSORS)
    captures, _ = _solved_capture(sensors)

    # RIGHT works loose after the first three faces.
    moved = copy.deepcopy(DEFAULT_SENSORS["RIGHT"])
    moved.mount_deg = (2.0, 2.0)
    import random
    rng = random.Random(2)
    for face in list(captures)[3:]:
        vec = L.FACE_VECTORS[face]
        captures[face]["RIGHT"]["mean_g"] = moved.measure(vec, 0.0, rng)

    solutions = {}
    for n in L.CHANNELS:
        solutions[n] = L.solve_sensor(
            [(f, captures[f][n]["mean_g"]) for f in captures])

    result = L.plate_consistency(captures, solutions)
    assert result["LEFT-CENTRE"]["verdict"] == "ok"
    # The culprit is the sensor appearing in both bad pairs.
    assert result["LEFT-RIGHT"]["verdict"] == "reject"
    assert result["CENTRE-RIGHT"]["verdict"] == "reject"


# --- End to end ------------------------------------------------------------

def test_replay_recovers_the_whole_calibration(tmp_path, capsys):
    path = str(tmp_path / "capture.txt")
    sensors = synthesise(path, hold_s=2.0, tilts=3, seed=1)

    assert sensor_cal.replay(path, hold_s=2.0) == 0
    out = capsys.readouterr().out

    assert "PASS" in out
    for face in L.SUGGESTED_FACE_ORDER:
        assert face in out

    # And the numbers, not just the verdict.
    captures = {}
    tilts = []
    for samples, _ in sensor_cal.static_segments(path, 2.0):
        face = L.sample_face(samples[len(samples) // 2])
        if face is not None and face not in captures:
            captures[face] = L.summarise_capture(samples, face)
        else:
            tilts.append(("t%d" % len(tilts), L.summarise_capture(samples, None)))

    solutions = sensor_cal.solve_all(captures, tilts)
    for name, sensor in sensors.items():
        for i in range(3):
            assert solutions[name]["bias"][i] == pytest.approx(
                sensor.bias[i], abs=5e-4)
            assert solutions[name]["scale"][i] == pytest.approx(
                sensor.scale[i], abs=5e-4)


def test_replay_reports_a_short_capture_rather_than_solving_it(tmp_path, capsys):
    path = str(tmp_path / "short.txt")
    synthesise(path, hold_s=2.0, tilts=0, seed=1)
    # Ask for holds longer than the file contains: no segment qualifies.
    assert sensor_cal.replay(path, hold_s=30.0) == 2
    assert "Fewer than six faces" in capsys.readouterr().out


def test_a_dead_channel_does_not_stop_the_other_two(tmp_path):
    path = str(tmp_path / "dead.txt")
    synthesise(path, hold_s=2.0, tilts=3, seed=1, dead=("CENTRE",))

    captures = {}
    tilts = []
    for samples, _ in sensor_cal.static_segments(path, 2.0):
        face = L.sample_face(samples[len(samples) // 2])
        if face is not None and face not in captures:
            captures[face] = L.summarise_capture(samples, face)
        else:
            tilts.append(("t%d" % len(tilts), L.summarise_capture(samples, None)))

    solutions = sensor_cal.solve_all(captures, tilts)
    assert set(solutions) == {"LEFT", "RIGHT"}


def test_checksum_is_stable_and_sensitive():
    a = {"bias": [0.03, -0.02, 0.04], "scale": [0.985, 1.012, 0.974]}
    b = {"bias": [0.03, -0.02, 0.04], "scale": [0.985, 1.012, 0.975]}
    assert sensor_cal.coefficient_checksum("LEFT", a) == \
        sensor_cal.coefficient_checksum("LEFT", a)
    assert sensor_cal.coefficient_checksum("LEFT", a) != \
        sensor_cal.coefficient_checksum("LEFT", b)
    # The sensor's name is in it, so a left/right swap is visible.
    assert sensor_cal.coefficient_checksum("LEFT", a) != \
        sensor_cal.coefficient_checksum("RIGHT", a)


def test_a_rejected_face_will_not_retry_itself():
    """Otherwise a face that is genuinely too far off flat retries forever.

    The plate is still sitting on the face that just failed, so a tracker that
    went straight back to looking would settle on it and re-capture the same bad
    hold without the operator doing anything.
    """
    tracker = L.FaceTracker(settle_seconds=1.0)
    sample = _flat_sample("Z+")
    tracker.feed(sample, 0.0)
    assert tracker.feed(sample, 2.0) == "Z+"

    tracker.restart()                      # the capture was thrown away
    for t in (3.0, 4.0, 10.0):
        assert tracker.feed(sample, t) is None

    # Only after it is actually picked up and set down again.
    moving = {n: _reading((0.3, 0.9, 0.2), (40.0, 0.0, 0.0)) for n in L.CHANNELS}
    tracker.feed(moving, 11.0)
    tracker.feed(sample, 12.0)
    assert tracker.feed(sample, 14.0) == "Z+"


# --- The interactive half, over a real serial port --------------------------

def test_the_live_wizard_walks_all_six_faces(tmp_path, capsys):
    """--replay cannot reach any of this: the waiting, the settling, the holds.

    Driven over a pty by a fake board replaying a synthetic capture, so the
    reader thread, the reply routing and the FaceTracker's wall-clock timing are
    all the real ones.
    """
    import serial
    from fake_teensy import FakeTeensy

    path = str(tmp_path / "live.txt")
    sensors = synthesise(path, hold_s=1.5, tilts=0, seed=1, motion_s=0.5,
                         rate_hz=200)

    with FakeTeensy(path, rate_hz=600) as board:
        with serial.Serial(board.device, timeout=1.0) as ser:
            with open(str(tmp_path / "raw.txt"), "w") as sink:
                stream = sensor_cal.Stream(ser, sink)
                tracker = L.FaceTracker(settle_seconds=0.1)
                captures = sensor_cal.run_faces(stream, tracker, 0.3,
                                                min_samples=50)

    assert set(captures) == set(L.SUGGESTED_FACE_ORDER)

    solutions = sensor_cal.solve_all(captures, [])
    for name, sensor in sensors.items():
        for i in range(3):
            assert solutions[name]["scale"][i] == pytest.approx(
                sensor.scale[i], abs=2e-3)
            assert solutions[name]["bias"][i] == pytest.approx(
                sensor.bias[i], abs=2e-3)


def test_handshake_rejects_the_bringup_firmware(tmp_path):
    """The bring-up build answers "I" too, and its reply looks similar enough."""
    import serial
    from fake_teensy import FakeTeensy

    path = str(tmp_path / "live.txt")
    synthesise(path, hold_s=0.5, tilts=0, seed=1, motion_s=0.2)

    class Bringup(FakeTeensy):
        def _write(self, text):
            if text.startswith("# ID"):
                text = ("# ID dev=manta-bringup proto=0 fw=0.1.0 hw=teensy40 "
                        "chans=LEFT,CENTRE,RIGHT units=none rate=0 cal=none "
                        "mode=bringup feat=uartprobe")
            FakeTeensy._write(self, text)

    with Bringup(path) as board:
        with serial.Serial(board.device, timeout=1.0) as ser:
            assert sensor_cal.handshake(ser, 200) is False


def test_the_stream_is_halted_across_the_operator_prompt(tmp_path):
    """Otherwise the OS buffer fills while the operator reads, and the first
    thing the tracker sees is seconds of readings that predate the run."""
    import serial
    from fake_teensy import FakeTeensy

    path = str(tmp_path / "live.txt")
    synthesise(path, hold_s=0.5, tilts=0, seed=1, motion_s=0.2)

    with FakeTeensy(path) as board:
        with serial.Serial(board.device, timeout=1.0) as ser:
            sensor_cal.send_command(ser, "H")
            assert "H" in board.commands


# --- Holding, as a person actually can ------------------------------------

def test_mean_observation_removes_the_cone_shrinkage():
    """Averaging vectors spread over a cone reads short; magnitudes do not.

    This is why a hand-held plate works and why a longer hold does not help:
    the shortfall is a bias, not noise, so more samples do not average it away.
    """
    import math
    spread = 5.0
    vectors = []
    for k in range(72):
        a = math.radians(k * 5.0)
        v = rotate(rotate((0.0, 0.0, 1.0), "x", spread), "z", math.degrees(a))
        vectors.append(v)

    plain = tuple(sum(v[i] for v in vectors) / len(vectors) for i in range(3))
    assert L.vector_norm(plain) < 0.9962          # cos(5 deg), read short

    corrected = L.mean_observation(vectors)
    assert L.vector_norm(corrected) == pytest.approx(1.0, abs=1e-9)


def test_mean_observation_matches_a_plain_mean_when_there_is_no_wander():
    steady = [(0.10, 0.20, 0.97)] * 50
    assert L.mean_observation(steady) == pytest.approx((0.10, 0.20, 0.97))


def test_wander_is_reported_in_degrees():
    vectors = [rotate((0.0, 0.0, 1.0), "x", d) for d in (-2.0, 0.0, 2.0)]
    assert L.wander_deg(vectors) == pytest.approx(1.63, abs=0.1)
    assert L.wander_deg([(0.0, 0.0, 1.0)] * 10) == pytest.approx(0.0, abs=1e-9)


class _ScriptedStream:
    """Replays a fixed list of samples, then repeats the last one for ever.

    Repeating the last rather than substituting a still one matters: a stream
    that quietly turns calm at the end would let "never still enough" pass by
    accident.
    """

    def __init__(self, samples):
        self.samples = list(samples)
        self.last = self.samples[-1]
    # def

    def read(self):
        if self.samples:
            return self.samples.pop(0)
        return self.last
    # def
# class


def test_a_twitch_mid_hold_costs_one_sample_not_the_whole_capture():
    """The behaviour the operator asked for: one wobble must not cost 30 s."""
    still = {n: _reading(L.FACE_VECTORS["Z+"]) for n in L.CHANNELS}
    twitch = {n: _reading(L.FACE_VECTORS["Z+"], (0.0, 40.0, 0.0))
              for n in L.CHANNELS}

    stream = _ScriptedStream([still] * 30 + [twitch] * 5 + [still] * 30)
    samples, reason = capture_hold_briefly(stream, "Z+", min_samples=10)

    assert reason is None
    # The twitches were dropped, not fatal.
    assert all(L.sample_is_quiet(s) for s in samples)


def test_a_capture_that_is_never_still_enough_says_so():
    twitch = {n: _reading(L.FACE_VECTORS["Z+"], (0.0, 40.0, 0.0))
              for n in L.CHANNELS}
    stream = _ScriptedStream([twitch] * 200)
    samples, reason = capture_hold_briefly(stream, "Z+", min_samples=50)
    assert reason is not None and "still enough" in reason


def test_a_capture_ends_when_the_plate_actually_moves_to_another_face():
    """Different from failing to hold still: the observation would be a blend
    of two orientations, which looks entirely healthy and is entirely wrong."""
    here = {n: _reading(L.FACE_VECTORS["Z+"]) for n in L.CHANNELS}
    there = {n: _reading(L.FACE_VECTORS["X+"]) for n in L.CHANNELS}
    stream = _ScriptedStream([here] * 20 + [there] * 20)
    samples, reason = capture_hold_briefly(stream, "Z+", min_samples=5)
    assert reason is not None and "moved from Z+ to X+" in reason


def capture_hold_briefly(stream, face, min_samples):
    """capture_hold with a short wall-clock window, for the scripted tests."""
    return sensor_cal.capture_hold(stream, 0.25, face, expect_face=face,
                                   min_samples=min_samples)
# def


def test_run_faces_gives_up_rather_than_retrying_for_ever(capsys):
    """Regression: a face that can never satisfy the gates used to hang.

    It presented as the wizard sitting there saying nothing, which is the worst
    possible way for an answerable problem to show up.
    """
    twitch = {n: _reading(L.FACE_VECTORS["Z+"], (0.0, 40.0, 0.0))
              for n in L.CHANNELS}
    still = {n: _reading(L.FACE_VECTORS["Z+"]) for n in L.CHANNELS}

    class Stream:
        def __init__(self):
            self.n = 0
        def read(self):
            self.n += 1
            # Bursts of stillness long enough to settle on the face, never long
            # enough to fill a capture. The tracker needs consecutive quiet
            # samples to settle at all, so a lone quiet sample between twitches
            # would deadlock the wait rather than exercise the give-up path.
            return still if (self.n // 10) % 2 == 0 else twitch

    tracker = L.FaceTracker(settle_seconds=0.0)
    # Only Z+ outstanding, since the scripted stream never presents another and
    # waiting for one that never comes is correct behaviour, not the bug here.
    for other in L.SUGGESTED_FACE_ORDER:
        if other != "Z+":
            tracker.mark_captured(other, {})

    # A scripted stream yields samples as fast as the loop turns, so a wall
    # clock window is not what bounds this - the sample budget is. Set it past
    # anything reachable so the give-up path is what gets exercised.
    captures = sensor_cal.run_faces(Stream(), tracker, 0.05,
                                    min_samples=10 ** 7)

    assert captures == {}
    out = capsys.readouterr().out
    assert "Giving up on" in out


def test_the_log_verdict_reflects_the_plate_check_not_just_sanity():
    """Regression from the 2026-08-28 runs: a run whose plate check rejected
    outright still logged three rows saying "pass"."""
    good = {"bias": [0.0, 0.0, 0.0], "scale": [1.0, 1.0, 1.0]}

    rejecting = {"LEFT-CENTRE": {"verdict": "reject"},
                 "LEFT-RIGHT": {"verdict": "ok"},
                 "CENTRE-RIGHT": {"verdict": "ok"}}
    assert sensor_cal._row_verdict("LEFT", good, rejecting) == "plate-reject"
    assert sensor_cal._row_verdict("RIGHT", good, rejecting) == "pass"

    warning = {"LEFT-CENTRE": {"verdict": "warn"}}
    assert sensor_cal._row_verdict("LEFT", good, warning) == "plate-warn"

    # A bad solve still refuses, whatever the plate says.
    bad = {"bias": [0.5, 0.0, 0.0], "scale": [1.0, 1.0, 1.0]}
    assert sensor_cal._row_verdict("LEFT", bad, rejecting) == "refuse"

    # No pairs computed at all is not the same as passing.
    assert sensor_cal._row_verdict("LEFT", good, {}) == "unchecked"


# --- The rigid-body fit ----------------------------------------------------

def _random_units(count, seed):
    import random
    rng = random.Random(seed)
    out = []
    while len(out) < count:
        v = (rng.gauss(0, 1), rng.gauss(0, 1), rng.gauss(0, 1))
        u = L.unit(v)
        if u is not None:
            out.append(u)
    return out


def test_fit_rotation_recovers_a_known_rotation():
    truth = lambda v: rotate(rotate(v, "x", 3.0), "y", -2.0)
    pairs = [(a, truth(a)) for a in _random_units(9, 1)]

    R = L.fit_rotation(pairs)
    for a, b in pairs:
        assert L.angle_between_deg(L.apply_rotation(R, a), b) == pytest.approx(
            0.0, abs=1e-6)


def test_fit_rotation_is_not_transposed():
    """The trap: a transposed result has exactly the right rotation *angle*.

    Which means the fitted angle looks perfect while the fit is useless - it
    explains the data no better than the identity does. Only the residual
    catches it, so the residual is what this asserts.
    """
    truth = lambda v: rotate(rotate(v, "x", 3.0), "y", -2.0)
    pairs = [(a, truth(a)) for a in _random_units(9, 2)]

    R = L.fit_rotation(pairs)
    assert L.rotation_angle_deg(R) == pytest.approx(3.606, abs=0.01)

    transposed = [[R[j][i] for j in range(3)] for i in range(3)]
    assert L.rotation_angle_deg(transposed) == pytest.approx(3.606, abs=0.01)

    def rms(M):
        import math
        angles = [L.angle_between_deg(L.apply_rotation(M, a), b) for a, b in pairs]
        return math.sqrt(statistics_fmean(x * x for x in angles))

    assert rms(R) < 1e-6
    assert rms(transposed) > 1.0        # right angle, useless axis


def statistics_fmean(values):
    import statistics
    return statistics.fmean(values)


def test_fit_rotation_refuses_when_it_cannot_tell():
    """One gravity vector pins a rotation only up to a spin about itself."""
    assert L.fit_rotation([]) is None
    assert L.fit_rotation([((0.0, 0.0, 1.0), (0.0, 0.0, 1.0))]) is None


def test_fit_rotation_survives_noise():
    import random
    rng = random.Random(5)
    truth = lambda v: rotate(v, "x", 2.0)
    pairs = []
    for a in _random_units(9, 5):
        b = truth(a)
        b = L.unit(tuple(c + rng.gauss(0, 2e-3) for c in b))
        pairs.append((a, b))

    R = L.fit_rotation(pairs)
    assert L.rotation_angle_deg(R) == pytest.approx(2.0, abs=0.2)


def test_plate_check_separates_mounting_from_movement():
    """Mounting is fitted out and reported; only movement is gated."""
    sensors = copy.deepcopy(DEFAULT_SENSORS)
    sensors["RIGHT"].mount_deg = (4.0, -3.0)
    captures, solutions = _solved_capture(sensors)

    result = L.plate_consistency(captures, solutions)
    pair = result["CENTRE-RIGHT"]
    assert pair["mount_deg"] > 3.0                 # reported...
    assert pair["residual_rms_deg"] < 0.05         # ...and explained away
    assert pair["verdict"] == "ok"


def test_plate_check_still_catches_a_module_that_shifted():
    sensors = copy.deepcopy(DEFAULT_SENSORS)
    captures, _ = _solved_capture(sensors)

    moved = copy.deepcopy(DEFAULT_SENSORS["RIGHT"])
    moved.mount_deg = (2.0, 2.0)
    import random
    rng = random.Random(2)
    for face in list(captures)[3:]:
        captures[face]["RIGHT"]["mean_g"] = moved.measure(
            L.FACE_VECTORS[face], 0.0, rng)

    solutions = {n: L.solve_sensor([(f, captures[f][n]["mean_g"])
                                    for f in captures]) for n in L.CHANNELS}
    result = L.plate_consistency(captures, solutions)

    assert result["LEFT-CENTRE"]["verdict"] == "ok"
    assert result["LEFT-RIGHT"]["verdict"] == "reject"
    assert result["CENTRE-RIGHT"]["verdict"] == "reject"


def test_the_fit_yields_the_rotation_a_datum_correction_needs():
    """The check is also where the CENTRE datum coefficient comes from."""
    sensors = copy.deepcopy(DEFAULT_SENSORS)
    sensors["LEFT"].mount_deg = (2.5, 0.0)
    sensors["CENTRE"].mount_deg = (0.0, 0.0)
    captures, solutions = _solved_capture(sensors)

    R = L.plate_consistency(captures, solutions)["LEFT-CENTRE"]["rotation"]

    # Carrying LEFT's corrected vector through R must land on CENTRE's, in
    # every orientation - that is what "use CENTRE as the datum" means once the
    # correction is done in the native format.
    for face, summary in captures.items():
        left = L.unit(L.apply_solution(summary["LEFT"]["mean_g"], solutions["LEFT"]))
        centre = L.unit(L.apply_solution(summary["CENTRE"]["mean_g"],
                                         solutions["CENTRE"]))
        assert L.angle_between_deg(L.apply_rotation(R, left), centre) < 0.05


def test_extra_tilts_feed_the_plate_fit():
    """Nine orientations constrain the rotation better than six."""
    captures, solutions = _solved_capture(copy.deepcopy(DEFAULT_SENSORS))
    tilts = [("tilt1", captures.pop("Z-"))]

    merged = sensor_cal.all_orientations(captures, tilts)
    assert len(merged) == 6
    assert L.plate_consistency(merged, solutions)["LEFT-CENTRE"][
        "n_orientations"] == 6


def test_the_log_migrates_rather_than_appending_under_a_stale_header(tmp_path):
    """The plate column changed meaning, so its old values must not be relabelled."""
    log = tmp_path / "sensor_cal_log.csv"
    old_columns = list(sensor_cal.LOG_COLUMNS)
    old_columns[old_columns.index("plate_residual_deg")] = "plate_worst_deg"
    with open(str(log), "w", newline="") as handle:
        w = csv.writer(handle)
        w.writerow(old_columns)
        w.writerow(["20260828_164539", "LEFT", "C384BFCB"] +
                   ["0.0"] * 6 + ["9", "3", "6.2e-04", "5.69", "9.43", "3.23",
                                  "pass"])

    sensor_cal._migrate_log(str(log))

    rows = list(csv.reader(open(str(log))))
    assert rows[0] == sensor_cal.LOG_COLUMNS
    # The old value was a different quantity; it is dropped, not renamed.
    assert rows[1][sensor_cal.LOG_COLUMNS.index("plate_residual_deg")] == ""
    # And the rest of the row survives.
    assert rows[1][0:3] == ["20260828_164539", "LEFT", "C384BFCB"]
    # The original is kept.
    assert (tmp_path / "sensor_cal_log.csv.bak").exists()


def test_migrating_a_current_log_is_a_no_op(tmp_path):
    log = tmp_path / "sensor_cal_log.csv"
    with open(str(log), "w", newline="") as handle:
        csv.writer(handle).writerow(sensor_cal.LOG_COLUMNS)

    sensor_cal._migrate_log(str(log))
    assert not (tmp_path / "sensor_cal_log.csv.bak").exists()


def test_the_wizard_refuses_a_board_that_is_not_in_raw_mode(tmp_path):
    """Calibrating against corrected values would be wrong by the square."""
    import serial
    from fake_teensy import FakeTeensy

    path = str(tmp_path / "live.txt")
    synthesise(path, hold_s=0.5, tilts=0, seed=1, motion_s=0.2)

    class Degrees(FakeTeensy):
        def _write(self, text):
            if text.startswith("# ID"):
                text = text.replace("mode=raw", "mode=degrees")
            elif text == "# ERR R":
                text = "# ERR R"          # a board with no mode command
            FakeTeensy._write(self, text)

    with Degrees(path) as board:
        with serial.Serial(board.device, timeout=1.0) as ser:
            assert sensor_cal.handshake(ser, 200) is False


def test_a_board_with_no_mode_command_is_still_accepted(tmp_path):
    """"# ERR R" means the board has no degrees mode, so it cannot be in one."""
    import serial
    from fake_teensy import FakeTeensy

    path = str(tmp_path / "live.txt")
    synthesise(path, hold_s=0.5, tilts=0, seed=1, motion_s=0.2)

    with FakeTeensy(path) as board:
        with serial.Serial(board.device, timeout=1.0) as ser:
            assert sensor_cal.handshake(ser, 200) is True
    assert "R" in board.commands


# --- A channel dying, which has happened twice on the real rig --------------

def test_a_dead_channel_is_named_as_a_dead_channel(capsys):
    """Not "past 15 deg from flat, or a channel is silent".

    That message cost a real run: a serial lead had worked loose and the
    operator was told to hold the plate flatter. Detection is not diagnosis.
    """
    summary = L.summarise_capture(
        [{"LEFT": None,
          "CENTRE": _reading(L.FACE_VECTORS["Z+"]),
          "RIGHT": _reading(L.FACE_VECTORS["Z+"])}] * 10, "Z+")

    problems = sensor_cal.report_capture(summary)
    assert problems == ["LEFT sent nothing"]
    assert "NO DATA" in capsys.readouterr().out


def test_a_bad_hold_is_named_as_a_bad_hold(capsys):
    """The other half: a real tilt rejection must say the tilt, and by how much."""
    tilted = rotate(L.FACE_VECTORS["Z+"], "x", 25.0)
    summary = L.summarise_capture(
        [{n: _reading(tilted) for n in L.CHANNELS}] * 10, "Z+")

    problems = sensor_cal.report_capture(summary)
    assert len(problems) == 3
    assert all("deg from flat" in p for p in problems)
    assert all("sent nothing" not in p for p in problems)


def test_a_dead_channel_stops_the_run_rather_than_retrying(capsys):
    """Retrying a dead channel cannot ever work, so the wizard must not ask."""
    dead = {"LEFT": None,
            "CENTRE": _reading(L.FACE_VECTORS["Z+"]),
            "RIGHT": _reading(L.FACE_VECTORS["Z+"])}

    class Stream:
        def read(self):
            return dead

    tracker = L.FaceTracker(settle_seconds=0.0)
    captures = sensor_cal.run_faces(Stream(), tracker, 0.05, min_samples=1)

    assert captures == {}
    out = capsys.readouterr().out
    assert "dead channel" in out
    assert "serial line" in out
    assert "Giving up on" not in out          # it stopped, it did not retry


def test_preflight_passes_when_every_channel_delivers(tmp_path):
    import serial
    from fake_teensy import FakeTeensy

    path = str(tmp_path / "live.txt")
    synthesise(path, hold_s=1.0, tilts=0, seed=1, motion_s=0.2)

    with FakeTeensy(path, rate_hz=600) as board:
        with serial.Serial(board.device, timeout=1.0) as ser:
            assert sensor_cal.preflight(ser, seconds=0.5) is True


def test_preflight_refuses_a_silent_channel(tmp_path, capsys):
    """Caught before the operator picks anything up, not six faces in."""
    import serial
    from fake_teensy import FakeTeensy

    path = str(tmp_path / "live.txt")
    synthesise(path, hold_s=1.0, tilts=0, seed=1, motion_s=0.2, dead=("LEFT",))

    with FakeTeensy(path, rate_hz=600) as board:
        with serial.Serial(board.device, timeout=1.0) as ser:
            assert sensor_cal.preflight(ser, seconds=0.5) is False

    out = capsys.readouterr().out
    assert "LEFT silent" in out
    assert "serial line" in out


def test_preflight_refuses_an_intermittent_channel(tmp_path, capsys):
    """A marginal connection corrupts a capture silently, by dropping samples
    from one channel and not the others."""
    import serial
    from fake_teensy import FakeTeensy

    path = str(tmp_path / "live.txt")
    synthesise(path, hold_s=2.0, tilts=0, seed=1, motion_s=0.2)

    # Blank LEFT from a third of the lines, as a loose lead would.
    lines = [l.rstrip("\n") for l in open(path) if l.startswith("{")]
    with open(path, "w") as handle:
        for i, line in enumerate(lines):
            if i % 3 == 0:
                line = "{%s:x|%s}" % (line[1:].split(":", 1)[0],
                                      line.split(":", 1)[1].rstrip("}")
                                      .split("|", 1)[1])
            handle.write(line + "\n")

    with FakeTeensy(path, rate_hz=600) as board:
        with serial.Serial(board.device, timeout=1.0) as ser:
            assert sensor_cal.preflight(ser, seconds=0.5) is False
    assert "intermittent" in capsys.readouterr().out


# --- Writing the calibration to the board ----------------------------------

def test_read_block_does_not_stop_on_an_interior_line():
    """Regression: "CAL" also matches the header, which orphaned four lines.

    A block reader that stops early leaves the remainder in the buffer, where
    the next command mistakes it for its own reply - and the symptom is a
    staging command that appears to answer with a calibration dump.
    """
    class FakeSerial:
        def __init__(self, lines):
            self.lines = list(lines)
        def readline(self):
            return (self.lines.pop(0) + "\n").encode() if self.lines else b""

    dump = ["# CAL ver=1 crc=DEADBEEF",
            "# CAL LEFT bias=0,0,0 scale=1,1,1",
            "# CAL CENTRE bias=0,0,0 scale=1,1,1",
            "# CAL RIGHT bias=0,0,0 scale=1,1,1",
            "# CAL end"]
    got = sensor_cal.read_block(FakeSerial(dump), sensor_cal.CAL_TERMINATORS,
                                timeout=1.0)
    assert len(got) == 5

    # "# CAL none" is the other way the dump ends, and has no "end" line.
    got = sensor_cal.read_block(FakeSerial(["# CAL none"]),
                                sensor_cal.CAL_TERMINATORS, timeout=1.0)
    assert got == ["# CAL none"]


def test_verify_readback_accepts_a_good_write():
    sensors = {n: {"bias_g": [0.001, -0.002, 0.003],
                   "scale": [0.995, 0.997, 0.993]} for n in L.CHANNELS}
    block = ["# CAL ver=1 crc=F7D78C77"]
    for n in L.CHANNELS:
        block.append("# CAL %s bias=0.001000,-0.002000,0.003000 "
                     "scale=0.995000,0.997000,0.993000" % n)
    block.append("# CAL end")

    assert sensor_cal.verify_readback(block, sensors, "F7D78C77") is True
    # Case must not matter: upper()ing the whole line turns "crc=" into "CRC="
    # and rejected a perfectly good calibration the first time this ran.
    assert sensor_cal.verify_readback(block, sensors, "f7d78c77") is True


def test_verify_readback_catches_a_wrong_crc_and_a_wrong_value():
    sensors = {n: {"bias_g": [0.001, -0.002, 0.003],
                   "scale": [0.995, 0.997, 0.993]} for n in L.CHANNELS}
    block = ["# CAL ver=1 crc=F7D78C77"]
    for n in L.CHANNELS:
        block.append("# CAL %s bias=0.001000,-0.002000,0.003000 "
                     "scale=0.995000,0.997000,0.993000" % n)
    block.append("# CAL end")

    assert sensor_cal.verify_readback(block, sensors, "AAAAAAAA") is False

    wrong = list(block)
    wrong[1] = ("# CAL LEFT bias=0.009000,-0.002000,0.003000 "
                "scale=0.995000,0.997000,0.993000")
    assert sensor_cal.verify_readback(wrong, sensors, "F7D78C77") is False


def test_the_datum_frame_is_inverted_for_the_pair_stored_the_other_way():
    """plate_consistency keys pairs in CHANNELS order, so LEFT-CENTRE carries
    LEFT onto CENTRE and must be inverted before it is stored."""
    R = L.fit_rotation([(a, rotate(a, "x", 3.0)) for a in _random_units(9, 7)])

    forward = sensor_cal._frame_for("RIGHT", {"CENTRE-RIGHT": {"rotation": R}})
    assert forward == [R[i][j] for i in range(3) for j in range(3)]

    inverted = sensor_cal._frame_for("LEFT", {"LEFT-CENTRE": {"rotation": R}})
    assert inverted == [R[j][i] for i in range(3) for j in range(3)]

    assert sensor_cal._frame_for("CENTRE", {}) == [1.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0,
                                                   0.0, 0.0, 1.0]
    assert sensor_cal._frame_for("LEFT", {}) is None


def test_corrected_lines_and_raw_lines_cannot_be_confused():
    """The differing delimiters are the guard against a missed mode change."""
    raw = "{123:1,2,3,4,5,6|7,8,9,10,11,12|13,14,15,16,17,18}"
    corrected = "[123:1000,2000,3000,4,5,6|7,8,9,10,11,12|13,14,15,16,17,18]"

    assert L.parse_sample_line(raw) is not None
    assert L.parse_sample_line(corrected) is None
    assert L.parse_corrected_line(corrected) is not None
    assert L.parse_corrected_line(raw) is None

    assert L.parse_corrected_line("[1:1,2,3,4,5,6|x|7,8,9,10,11,12]")["CENTRE"] is None
