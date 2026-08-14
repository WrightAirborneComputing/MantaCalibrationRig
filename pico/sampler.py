"""Dual-rate MicroPython firmware for the Manta calibration rig Pico.

Copy this to the Pico's filesystem **as `main.py`** so it runs at power-on:

    mpremote connect auto fs cp pico/sampler.py :main.py + reset

It replaces `pico/main.py`, which is kept in the repo as the legacy 10 Hz-only
firmware.

Two presets, switched over USB at runtime:

    slow (boot default)   10 Hz   "[<left_u16>/<right_u16>]"
    fast                 500 Hz   "[<ticks_us>:<left_u16>/<right_u16>]"

Slow mode is byte-identical to the legacy firmware, so plugging the board into
any terminal gives readable output and the existing host tools keep working
untouched. Fast mode exists to resolve an elevon transit (150-400 ms) into 100+
samples, which is what measuring travel range and slew rate needs.

The ceiling is the per-line cost of formatting and printing over USB CDC, not
the ADC and not SERIAL_BAUD - this is a USB CDC virtual port, so the baud rate
the host asks for is never applied to anything.

Measured on an RP2040 at MicroPython 1.19.1:

    ADC pair (both channels)          25 us
    "%" formatting of the line       377 us     <- dominant cost
    plus print() to USB CDC          510 us

The whole loop - ADC, formatting, print, scheduling and the command poll - comes
to ~660 us, and the board sustains rates up to about 1500 Hz. Asking for 2000 Hz
yields ~1508 Hz, i.e. it saturates rather than keeps up. 500 Hz spends a third of
its 2000 us budget, leaving ~3x headroom, which is why it is the fast preset.

MAX_HZ is left above the sustainable ceiling on purpose: asking for more produces
an honest "could not sustain" report from pico_monitor rather than silent drift.

The formatting choice is worth keeping: on this board "%" costs 377 us where
"{}".format() costs 1423 us and "+"-concatenation of str() costs 2999 us. The
legacy firmware used .format(), so this loop is cheaper than the 10 Hz one it
replaces despite doing more.

The microsecond stamp in fast mode is taken next to the ADC read, so timing is
immune to USB's bursty 1 ms-frame delivery - and it is what lets the host prove
the board is *sampling* at 500 Hz rather than merely that lines are arriving.
ticks_us() wraps at 2^30 us (~17.9 min); the host unwraps it.

Command protocol - newline-terminated ASCII, case-insensitive, tolerant of CRLF:

    S        slow preset: 10 Hz, no timestamp      -> "# ACK S 10"
    F        fast preset: 500 Hz, timestamp on     -> "# ACK F 500"
    F<hz>    fast at a given rate, e.g. "F1000"    -> "# ACK F 1000"
    G        collect garbage now                   -> "# ACK G"
    ?        status                                -> "# STATUS mode=slow hz=10 ts=0"
    other                                          -> "# ERR <text>"

`G` exists because MicroPython's garbage collector stalls the sample loop for
~7.7 ms whenever it runs, which at 500 Hz is roughly every 4.3 seconds and costs
three consecutive samples. Measured on this board, a collect costs ~4.5 ms even
on an almost-empty heap, so the cost tracks heap size rather than garbage volume
and collecting *more often* would only stall more. Instead the host calls G
immediately before a capture, paying the stall at a moment when nothing is being
measured and buying a clean window of a few seconds after it.

Every board-to-host reply is prefixed "#", which cannot match the sample regex,
so replies are inert to every parser on the host side. Commands are short enough
to type by hand into a serial terminal, which is the point - the console is the
debugging tool.

Ctrl-C is deliberately left enabled: it is the escape hatch when the board is
firehosing at 500 Hz. The host must therefore never send a raw 0x03, which costs
nothing since every command is letters and digits.
"""

from machine import Pin, ADC
import gc
import select
import sys
import utime

SLOW_HZ = 10
FAST_HZ = 500

# Below 1 Hz the period arithmetic degenerates; above ~2-3 kHz the print cost
# exceeds the sample period and the loop silently stops keeping time.
MIN_HZ = 1
MAX_HZ = 2000

# Longest command we will buffer. Anything longer is truncated and then fails to
# parse, which is the correct outcome for line noise.
MAX_COMMAND_LEN = 16

LEFT_ADC_PIN = 26    # ADC0, GPIO26
RIGHT_ADC_PIN = 27   # ADC1, GPIO27


def clamp_hz(hz):
    if hz < MIN_HZ:
        return MIN_HZ
    if hz > MAX_HZ:
        return MAX_HZ
    return hz
# def


def initial_state():
    return {"mode": "slow", "hz": SLOW_HZ, "ts": False}
# def


def _all_digits(text):
    # str.isdigit() exists in MicroPython but its unicode behaviour varies by
    # port; an explicit ASCII check is predictable everywhere.
    if not text:
        return False
    for ch in text:
        if ch < "0" or ch > "9":
            return False
    return True
# def


def apply_command(state, text):
    """Parse one command line. Returns (state, reply, action).

    Pure: touches no hardware and never raises, so the host-side test suite can
    drive it under CPython. Side effects are *named* rather than performed -
    `action` is None or "collect" - which keeps the parser testable while the
    caller does the part that needs the runtime.

    A returned state that *is* the input object means nothing changed; a new
    dict means the caller must re-derive its timing. A reply of None means there
    is nothing to send.
    """
    text = text.strip().upper()

    if text == "":
        return state, None, None

    if text == "?":
        return state, "# STATUS mode=%s hz=%d ts=%d" % (
            state["mode"], state["hz"], 1 if state["ts"] else 0
        ), None

    if text == "S":
        return {"mode": "slow", "hz": SLOW_HZ, "ts": False}, "# ACK S %d" % SLOW_HZ, None

    if text == "G":
        return state, "# ACK G", "collect"

    if text[0] == "F":
        rest = text[1:]

        if rest == "":
            hz = FAST_HZ
        elif _all_digits(rest):
            hz = clamp_hz(int(rest))
        else:
            return state, "# ERR %s" % text, None

        return {"mode": "fast", "hz": hz, "ts": True}, "# ACK F %d" % hz, None

    return state, "# ERR %s" % text, None
# def


def derive_timing(hz):
    """Loop constants for a sample rate: (period_us, cmd_interval, blink_reload).

    Command polling is decimated rather than run every sample so the hot loop
    stays lean: 20 ms of switch latency at 500 Hz, and every sample at 10 Hz
    where there is budget to spare. Blinking is decimated to ~2-4 Hz so the LED
    reads as "alive" instead of a blur.
    """
    period_us = 1000000 // hz

    cmd_interval = hz // 50
    if cmd_interval < 1:
        cmd_interval = 1

    blink_reload = hz // 4
    if blink_reload < 1:
        blink_reload = 1

    return period_us, cmd_interval, blink_reload
# def


def _service_stdin(poller, pending, state):
    """Drain whatever stdin has, without ever blocking.

    Returns (pending, state, disturbed). Only reads while poll() reports data
    ready, so a board with no host attached costs one syscall per check.

    "disturbed" means the caller must re-derive its timing and re-anchor its
    deadline - either the rate changed, or we just spent milliseconds in a
    garbage collection.
    """
    disturbed = False

    while poller.poll(0):
        ch = sys.stdin.read(1)

        if not ch:
            break

        if ch == "\n" or ch == "\r":
            if pending:
                new_state, reply, action = apply_command(state, pending)
                pending = ""

                if reply is not None:
                    print(reply)

                if new_state is not state:
                    state = new_state
                    disturbed = True

                if action == "collect":
                    # Deliberate: pay the ~5-7 ms stall here, on request, rather
                    # than have it land in the middle of someone's measurement.
                    gc.collect()
                    disturbed = True

        elif len(pending) < MAX_COMMAND_LEN:
            pending += ch

    return pending, state, disturbed
# def


def run():
    led = Pin("LED", Pin.OUT)
    adc_left = ADC(LEFT_ADC_PIN)
    adc_right = ADC(RIGHT_ADC_PIN)

    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)

    state = initial_state()
    period_us, cmd_interval, blink_reload = derive_timing(state["hz"])

    print("# MANTA sampler ready mode=%s hz=%d ts=%d" %
          (state["mode"], state["hz"], 1 if state["ts"] else 0))

    pending = ""
    cmd_countdown = cmd_interval
    blink_countdown = blink_reload
    next_us = utime.ticks_us()

    while True:
        # Absolute-deadline scheduling, so a slow iteration does not accumulate
        # drift into the sample interval.
        delay_us = utime.ticks_diff(next_us, utime.ticks_us())

        if delay_us > 0:
            utime.sleep_us(delay_us)
        elif delay_us < -period_us:
            # More than a whole period late - a host stall or a rate change.
            # Re-anchor instead of sprinting to work off a backlog of missed
            # deadlines, which would emit a burst at the wrong cadence.
            next_us = utime.ticks_us()

        next_us = utime.ticks_add(next_us, period_us)

        t_us = utime.ticks_us()
        raw_left = adc_left.read_u16()
        raw_right = adc_right.read_u16()

        # %-formatting rather than .format(): measured at 377 us against 1423 us
        # on this board. This is the line that sets the rate ceiling, so the
        # difference is the difference between ~1960 Hz and ~700 Hz.
        if state["ts"]:
            print("[%d:%d/%d]" % (t_us, raw_left, raw_right))
        else:
            print("[%d/%d]" % (raw_left, raw_right))

        blink_countdown -= 1
        if blink_countdown <= 0:
            blink_countdown = blink_reload
            led.toggle()

        cmd_countdown -= 1
        if cmd_countdown <= 0:
            cmd_countdown = cmd_interval

            pending, state, disturbed = _service_stdin(poller, pending, state)

            if disturbed:
                period_us, cmd_interval, blink_reload = derive_timing(state["hz"])
                cmd_countdown = cmd_interval
                blink_countdown = blink_reload
                next_us = utime.ticks_us()
# def


if __name__ == "__main__":
    run()
# if
