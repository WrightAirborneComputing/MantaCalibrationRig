"""The driver contract, tested against every driver that claims to implement it.

The point of this file is the parameterisation. Each contract test runs twice -
once against `PicoDriver` talking to a fake Pico over a real pty, once against
`FakeDriver` with no hardware at all - and asserts only what the contract
promises. A test that needs to know which driver it has is either testing an
implementation detail, in which case it belongs in the per-driver section
below, or it has found a hole in the contract, which is the interesting case.

When the Teensy driver arrives it should pass this file unmodified. If it
cannot, the contract was wrong and it is better to learn that from a diff to
these assertions than from a rig that reads plausible nonsense.

    python3 -m pytest tests/test_sensor_driver.py -v
"""

import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fake_pico import FakePico

import sensor_driver as SD
import MantaTrimmer as MT


# The fake Pico wobbles its counts by up to 8, which at 0.0042 deg/count is
# about 0.034 deg. Anything tighter than this would be testing the wobble.
POT_TOLERANCE_DEG = 0.05

# Known scaler and tare for the pot rig under test, set on the instance rather
# than through set_scaler_and_offset() so the real settings.json is never
# touched by a test run.
TEST_LEFT_SCALER = 0.0042
TEST_LEFT_OFFSET = -75.0
TEST_RIGHT_SCALER = 0.0045
TEST_RIGHT_OFFSET = +117.0


def wait_until(predicate, timeout=5.0, interval=0.02):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)

    return False
# def


class Rig(object):
    """A driver plus whatever it takes to put a known angle in front of it.

    Placing an angle is the one thing that cannot be uniform - the fake is told
    a number, the Pico has to be fed counts and given time to average them - so
    it is hidden here and every test above stays driver-agnostic.
    """

    def __init__(self, driver, place, tolerance, teardown):
        self.driver = driver
        self.place = place
        self.tolerance = tolerance
        self._teardown = teardown
    # def

    def channels(self):
        return self.driver.channels()
    # def

    def close(self):
        self._teardown()
    # def
# class


def _fake_rig():
    driver = SD.FakeDriver()
    driver.start()

    for channel in driver.channels():
        driver.set_angle(channel, 0.0)

    def place(channel, degrees):
        driver.set_angle(channel, degrees)
        return True
    # def

    return Rig(driver, place, tolerance=1e-9, teardown=lambda: driver.stop())
# def


def _pico_rig():
    pico = FakePico().start()

    reader = MT.PositionReader()
    reader.left_scaler = TEST_LEFT_SCALER
    reader.left_offset = TEST_LEFT_OFFSET
    reader.right_scaler = TEST_RIGHT_SCALER
    reader.right_offset = TEST_RIGHT_OFFSET

    driver = SD.PicoDriver(reader)
    driver.set_port(pico.device)

    assert wait_until(driver.is_streaming), "pico driver never started streaming"

    def place(channel, degrees):
        # Invert position_to_degrees to get the counts that produce this angle.
        # LEFT is (scaler * raw) + offset, RIGHT is -(scaler * raw) + offset.
        if channel == SD.CHANNEL_LEFT:
            pico.base_left = int(round((degrees - TEST_LEFT_OFFSET) / TEST_LEFT_SCALER))
        elif channel == SD.CHANNEL_RIGHT:
            pico.base_right = int(round((TEST_RIGHT_OFFSET - degrees) / TEST_RIGHT_SCALER))
        else:
            return False

        # The averaging window still holds pre-move samples; drop them and wait
        # for it to refill, exactly as a caller stepping a servo would have to.
        driver.clear()

        return wait_until(
            lambda: driver.get_angle(channel) is not None
            and abs(driver.get_angle(channel) - degrees) <= POT_TOLERANCE_DEG,
            timeout=5.0,
        )
    # def

    def teardown():
        driver.stop()
        pico.stop()
    # def

    return Rig(driver, place, tolerance=POT_TOLERANCE_DEG, teardown=teardown)
# def


RIG_BUILDERS = {"fake": _fake_rig, "pico": _pico_rig}


@pytest.fixture(params=sorted(RIG_BUILDERS))
def rig(request):
    built = RIG_BUILDERS[request.param]()
    try:
        yield built
    finally:
        built.close()
# def


# -- the contract ---------------------------------------------------------

def test_identity_is_a_usable_name(rig):
    """Exports record which sensor produced them, so this has to be printable."""
    identity = rig.driver.identity()

    assert isinstance(identity, str)
    assert identity.strip()
# def


def test_capabilities_are_self_consistent(rig):
    caps = rig.driver.capabilities()

    assert caps.channels == rig.driver.channels()
    assert caps.channels, "a driver with no channels cannot be read"

    for channel in caps.channels:
        assert channel in SD.ALL_CHANNELS, "unknown channel name %r" % channel

    assert caps.max_sample_rate_hz > 0
    assert caps.resolution_deg > 0
    assert caps.describe()
# def


def test_capabilities_answer_rate_questions_honestly(rig):
    """The whole reason capabilities exist: ask before you move."""
    caps = rig.driver.capabilities()

    assert caps.supports_rate(caps.max_sample_rate_hz)
    assert not caps.supports_rate(caps.max_sample_rate_hz + 1)

    # A device may accept more than it can sustain, never less.
    assert caps.max_requestable_rate_hz >= caps.max_sample_rate_hz
# def


def test_every_advertised_channel_reads_degrees(rig):
    for channel in rig.channels():
        assert rig.driver.capabilities().has_channel(channel)

        value = rig.driver.get_angle(channel)
        assert value is None or isinstance(value, float)
# def


def test_an_unknown_channel_reads_none_rather_than_raising(rig):
    """The GUI polls on a timer; an exception there takes the window down."""
    assert rig.driver.get_angle("NOSUCHCHANNEL") is None
# def


def test_placed_angles_come_back(rig):
    """Degrees in, degrees out, on every channel that is also a side."""
    for channel, target in ((SD.CHANNEL_LEFT, 4.25), (SD.CHANNEL_RIGHT, -3.5)):
        if channel not in rig.channels():
            continue

        assert rig.place(channel, target), "could not place %s at %s" % (channel, target)

        got = rig.driver.get_angle(channel)
        assert got is not None
        assert abs(got - target) <= rig.tolerance
# def


def test_channels_are_independent(rig):
    """Placing one side must not move the reading of the other."""
    if not set(SD.SIDES).issubset(set(rig.channels())):
        pytest.skip("driver does not offer both sides")

    assert rig.place(SD.CHANNEL_LEFT, 6.0)
    assert rig.place(SD.CHANNEL_RIGHT, -6.0)

    left = rig.driver.get_angle(SD.CHANNEL_LEFT)
    right = rig.driver.get_angle(SD.CHANNEL_RIGHT)

    assert abs(left - 6.0) <= rig.tolerance
    assert abs(right - (-6.0)) <= rig.tolerance
# def


def test_streaming_and_connected_are_different_questions(rig):
    assert rig.driver.is_connected()
    assert rig.driver.is_streaming()

    assert rig.driver.last_error() is None
# def


def test_stopping_ends_the_stream(rig):
    rig.driver.stop()

    assert not rig.driver.is_streaming()
    assert not rig.driver.is_connected()
# def


def test_sample_rate_reports_what_was_granted(rig):
    """Never more than advertised, and the driver's own view agrees."""
    caps = rig.driver.capabilities()

    achieved = rig.driver.set_sample_rate(caps.max_sample_rate_hz)

    if achieved is None:
        pytest.skip("driver does not negotiate a rate")

    assert achieved <= caps.max_sample_rate_hz
    assert rig.driver.sample_rate() == achieved
    assert caps.supports_rate(achieved)
# def


def test_asking_beyond_the_ceiling_is_granted_but_never_called_sustainable(rig):
    """The one place a driver is allowed to hand back more than it advertises.

    pico/sampler.py sets MAX_HZ deliberately above the sustainable rate so that
    asking for too much produces an honest overrun report rather than silent
    drift, and PositionReader faithfully reports the rate the board ACKed. That
    is a feature, so the contract must permit it - but only on these terms: the
    driver may never exceed what it says it will accept, and it must not claim
    the over-rate is sustainable. A measurement that asks supports_rate() gets
    the truth even while running at a rate it should not trust.
    """
    caps = rig.driver.capabilities()

    achieved = rig.driver.set_sample_rate(caps.max_requestable_rate_hz * 10)

    if achieved is None:
        pytest.skip("driver does not negotiate a rate")

    assert achieved <= caps.max_requestable_rate_hz

    if achieved > caps.max_sample_rate_hz:
        assert not caps.supports_rate(achieved)
# def


def test_capture_round_trips(rig):
    if not rig.driver.capabilities().raw_capture:
        pytest.skip("driver does not capture raw")

    assert not rig.driver.is_capturing()

    rig.driver.start_capture()
    assert rig.driver.is_capturing()

    rig.place(SD.CHANNEL_LEFT, 1.0)
    rig.place(SD.CHANNEL_LEFT, 2.0)

    samples, truncated = rig.driver.stop_capture()

    assert not rig.driver.is_capturing()
    assert isinstance(samples, list)
    assert truncated in (True, False)
    assert samples, "capture recorded nothing while samples were arriving"
# def


def test_a_command_round_trip_answers_or_says_nothing(rig):
    reply = rig.driver.send_command("V", timeout=0.5, attempts=1)

    assert reply is None or isinstance(reply, str)
# def


def test_clear_discards_what_came_before(rig):
    """Used before a measurement so the window cannot hold pre-move samples."""
    rig.place(SD.CHANNEL_LEFT, 5.0)
    assert rig.driver.get_angle(SD.CHANNEL_LEFT) is not None

    rig.driver.clear()

    assert rig.driver.get_angle(SD.CHANNEL_LEFT) is None
# def


# -- PicoDriver: the arithmetic must not have moved -------------------------

def test_the_seam_did_not_change_the_pot_arithmetic():
    """Characterisation. These numbers are the whole safety net.

    The migration's promise is that a calibration run through the driver
    produces the same figures as one taken before it existed, so the conversion
    is pinned here against hand-computed values rather than against itself.
    LEFT is (scaler * raw) + offset and RIGHT is -(scaler * raw) + offset,
    opposite because the pots are mounted mirrored.
    """
    reader = MT.PositionReader()
    reader.left_scaler = 0.0042
    reader.left_offset = -75.0
    reader.right_scaler = 0.0045
    reader.right_offset = +117.0

    driver = SD.PicoDriver(reader)

    # 0.0042 * 31400 - 75.0
    assert driver.reader.position_to_degrees("LEFT", 31400) == pytest.approx(56.88)
    # -(0.0045 * 34180) + 117.0
    assert driver.reader.position_to_degrees("RIGHT", 34180) == pytest.approx(-36.81)

    # A tare of zero leaves the raw linear term, and the signs stay mirrored.
    reader.left_offset = 0.0
    reader.right_offset = 0.0
    assert driver.reader.position_to_degrees("LEFT", 1000) == pytest.approx(4.2)
    assert driver.reader.position_to_degrees("RIGHT", 1000) == pytest.approx(-4.5)
# def


def test_the_pico_driver_reports_degrees_the_old_path_would_have():
    """Same counts through both paths, one answer.

    Guards the case the contract tests cannot see: that PicoDriver.get_angle()
    is the old two-step read-then-convert and not a re-derivation that happens
    to look right at zero.
    """
    pico = FakePico(base_left=30000, base_right=33000).start()

    reader = MT.PositionReader()
    reader.left_scaler = TEST_LEFT_SCALER
    reader.left_offset = TEST_LEFT_OFFSET
    reader.right_scaler = TEST_RIGHT_SCALER
    reader.right_offset = TEST_RIGHT_OFFSET

    driver = SD.PicoDriver(reader)
    driver.set_port(pico.device)

    try:
        assert wait_until(driver.is_streaming)
        assert wait_until(lambda: driver.get_angle(SD.CHANNEL_LEFT) is not None)

        for channel in (SD.CHANNEL_LEFT, SD.CHANNEL_RIGHT):
            raw = reader.get_average_position_nonblocking(channel)
            expected = reader.position_to_degrees(channel, raw)

            # Re-read through the seam. The counts wobble between the two reads,
            # so compare within one count's worth of degrees rather than exactly.
            assert driver.get_angle(channel) == pytest.approx(expected, abs=POT_TOLERANCE_DEG)
    finally:
        driver.stop()
        pico.stop()
# def


def test_the_pico_driver_offers_no_centre_channel():
    """The pots cannot see the airframe, and must not pretend otherwise."""
    caps = SD.PicoDriver(MT.PositionReader()).capabilities()

    assert SD.CHANNEL_CENTRE not in caps.channels
    assert not caps.reference_channel
# def


# -- FakeDriver: the parts that exist to make the contract testable ---------

def test_the_fake_clamps_to_its_advertised_ceiling():
    driver = SD.FakeDriver(max_sample_rate_hz=200)
    driver.start()

    assert driver.set_sample_rate(1000) == 200
    assert driver.set_sample_rate(50) == 50
# def


def test_the_fake_reads_none_when_it_is_not_running():
    driver = SD.FakeDriver()

    assert driver.get_angle(SD.CHANNEL_LEFT) is None
    assert not driver.is_streaming()
    assert driver.set_sample_rate(100) is None
# def


def test_the_fake_can_simulate_a_dead_link():
    driver = SD.FakeDriver()
    driver.start()
    driver.set_angle(SD.CHANNEL_LEFT, 5.0)

    assert driver.get_angle(SD.CHANNEL_LEFT) == pytest.approx(5.0)

    driver.fail("cable pulled")

    assert driver.get_angle(SD.CHANNEL_LEFT) is None
    assert driver.last_error() == "cable pulled"
    assert not driver.is_connected()
# def


def test_the_fake_refuses_a_channel_it_does_not_have():
    """A typo in a test must fail loudly here, not read as a silent zero."""
    driver = SD.FakeDriver(channels=(SD.CHANNEL_LEFT, SD.CHANNEL_RIGHT))

    with pytest.raises(KeyError):
        driver.set_angle(SD.CHANNEL_CENTRE, 1.0)
# def


def test_a_two_channel_fake_reports_no_reference():
    """reference_channel follows the channel list rather than being asserted."""
    both = SD.FakeDriver(channels=(SD.CHANNEL_LEFT, SD.CHANNEL_RIGHT))
    three = SD.FakeDriver()

    assert not both.capabilities().reference_channel
    assert three.capabilities().reference_channel
# def


# -- conversion through the seam -------------------------------------------

def test_native_values_convert_through_the_driver(rig):
    """The capture path's escape hatch, so nothing has to borrow a scaler.

    Only the shape is asserted here - what a native value *means* is the
    driver's business and is pinned per-driver above.
    """
    driver = rig.driver

    assert driver.to_degrees("NOSUCHCHANNEL", 1.0) is None

    for channel in rig.channels():
        assert driver.to_degrees(channel, None) is None

        converted = driver.to_degrees(channel, 1000)
        assert isinstance(converted, float)
# def


def test_a_degrees_native_driver_converts_by_identity():
    driver = SD.FakeDriver()

    assert driver.to_degrees(SD.CHANNEL_LEFT, 12.5) == pytest.approx(12.5)
    assert driver.to_degrees(SD.CHANNEL_CENTRE, -3.0) == pytest.approx(-3.0)
# def


def test_the_pico_driver_converts_counts_the_same_way_the_reader_does():
    reader = MT.PositionReader()
    reader.left_scaler = 0.0042
    reader.left_offset = -75.0
    reader.right_scaler = 0.0045
    reader.right_offset = +117.0

    driver = SD.PicoDriver(reader)

    assert driver.to_degrees(SD.CHANNEL_LEFT, 31400) == pytest.approx(56.88)
    assert driver.to_degrees(SD.CHANNEL_RIGHT, 34180) == pytest.approx(-36.81)
    assert driver.to_degrees(SD.CHANNEL_CENTRE, 1000) is None
# def
