"""Tests for how a serial port is described in the picker.

Written against the Windows case, which is the one that broke: both the Pico
and the flight controller bind to the inbox usbser.sys CDC driver, and pyserial
reads manufacturer from that driver's registry entry rather than from the USB
descriptor. Every port came back as "COMx  -  Microsoft", so the manual picker
could not tell the two boards apart - the fallback you reach for precisely when
auto-detect has missed.

The USB ID is the field that does differ, and the code already reads it to
identify the boards for connection. These tests pin it into the label too.

    python3 -m pytest tests/test_port_labels.py -v
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from manta_common import PortCandidate


class FakePortInfo:
    """What serial.tools.list_ports.comports() hands back, minus the hardware."""

    def __init__(self, device, description="n/a", hwid="n/a", vid=None,
                 pid=None, manufacturer=None, product=None,
                 serial_number=None):
        self.device = device
        self.description = description
        self.hwid = hwid
        self.vid = vid
        self.pid = pid
        self.manufacturer = manufacturer
        self.product = product
        self.serial_number = serial_number
    # def
# class


def windows_pico(device="COM5"):
    """A Pico as Windows reports it: driver manufacturer, no product."""
    return PortCandidate(FakePortInfo(
        device=device,
        description="USB Serial Device (%s)" % device,
        hwid="USB VID:PID=2E8A:0005 SER=E66038B713849C31 LOCATION=1-2",
        vid=0x2E8A, pid=0x0005,
        manufacturer="Microsoft", product=None,
        serial_number="E66038B713849C31"))
# def


def windows_fcu(device="COM7"):
    """A PX4 board as Windows reports it. Same driver, same manufacturer."""
    return PortCandidate(FakePortInfo(
        device=device,
        description="USB Serial Device (%s)" % device,
        hwid="USB VID:PID=3185:0038 SER=0 LOCATION=1-4:x.0",
        vid=0x3185, pid=0x0038,
        manufacturer="Microsoft", product=None))
# def


def linux_pico(device="/dev/ttyACM0"):
    """The same board on Linux, where the descriptors come through."""
    return PortCandidate(FakePortInfo(
        device=device,
        description="Board in FS mode",
        hwid="USB VID:PID=2E8A:0005 SER=E66038B713849C31 LOCATION=1-2:1.0",
        vid=0x2E8A, pid=0x0005,
        manufacturer="MicroPython", product="Board in FS mode"))
# def


def test_windows_ports_are_distinguishable():
    """The bug: two boards, two identical labels."""
    pico = windows_pico().label()
    fcu = windows_fcu().label()

    assert pico != fcu
    assert "Microsoft" not in pico
    assert "Microsoft" not in fcu
# def


def test_windows_label_names_the_board_and_the_id():
    label = windows_pico().label()

    assert label.startswith("COM5  -  ")
    assert "Pico" in label
    assert "2E8A:0005" in label
# def


def test_windows_fcu_label_names_the_board_and_the_id():
    label = windows_fcu().label()

    assert "FCU" in label
    assert "3185:0038" in label
# def


def test_friendly_name_does_not_repeat_the_port():
    """Windows' friendly name ends in "(COM9)"; the label already opens with it."""
    unknown = PortCandidate(FakePortInfo(
        device="COM9",
        description="USB Serial Device (COM9)",
        hwid="USB VID:PID=0403:6001",
        vid=0x0403, pid=0x6001,
        manufacturer="Microsoft"))

    label = unknown.label()

    assert label.count("COM9") == 1
    assert "USB Serial Device" in label
    assert "0403:6001" in label
# def


def test_linux_detail_still_used():
    """Where the descriptors are real, they stay in the label."""
    label = linux_pico().label()

    assert label.startswith("/dev/ttyACM0  -  ")
    assert "Pico" in label
    assert "MicroPython" in label
    assert "2E8A:0005" in label
# def


def test_port_with_nothing_known_is_just_the_device():
    bare = PortCandidate(FakePortInfo(device="/dev/ttyS0"))

    assert bare.label() == "/dev/ttyS0"
# def


def test_device_survives_the_split_the_gui_falls_back_to():
    """populate_port_combos() recovers a hand-typed device with split(" ")[0]."""
    for candidate in (windows_pico(), windows_fcu(), linux_pico()):
        assert candidate.label().split(" ")[0] == candidate.device
# def


def test_role_is_only_claimed_when_the_id_says_so():
    unknown = PortCandidate(FakePortInfo(
        device="COM9", description="USB Serial Device (COM9)",
        vid=0x0403, pid=0x6001, manufacturer="Microsoft"))

    assert unknown.role() == ""
    assert windows_pico().role() == "Pico"
    assert windows_fcu().role() == "FCU"
# def
