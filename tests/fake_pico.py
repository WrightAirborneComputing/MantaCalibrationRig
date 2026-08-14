"""A fake Pico on a pty, running pico/sampler.py's real command parser.

Lets the host-side serial code - PositionReader's reader thread, reply routing,
rate negotiation, capture - be tested end to end over a genuine serial port with
no board attached. It does not prove MicroPython can hit 500 Hz; only hardware
does that. It proves the protocol and everything above it.

    with FakePico() as pico:
        reader.set_port(pico.device)
"""

import os
import sys
import threading
import time
import tty
import types

PICO_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "pico")

TICKS_MODULO = 1 << 30


def load_sampler():
    """Import pico/sampler.py under CPython with its MicroPython imports stubbed."""
    machine = types.ModuleType("machine")
    machine.Pin = object
    machine.ADC = object

    utime = types.ModuleType("utime")
    utime.ticks_us = lambda: 0
    utime.ticks_add = lambda a, b: a + b
    utime.ticks_diff = lambda a, b: a - b
    utime.sleep_us = lambda us: None

    saved = {name: sys.modules.get(name) for name in ("machine", "utime")}
    sys.modules["machine"] = machine
    sys.modules["utime"] = utime
    sys.path.insert(0, PICO_DIR)

    try:
        import sampler
        return sampler
    finally:
        sys.path.remove(PICO_DIR)
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module
# def


class FakePico(object):
    """Streams samples over a pty and answers commands like the real firmware.

    rate_ceiling caps what it can actually achieve regardless of what it is
    asked for, standing in for a board whose print cost has caught up with its
    sample period. drop_every discards every Nth line *after* stamping it, which
    is what losing a line in transport looks like.
    """

    def __init__(self, rate_ceiling=None, drop_every=0, base_left=31400, base_right=34180):
        self.sampler = load_sampler()
        self.rate_ceiling = rate_ceiling
        self.drop_every = drop_every
        self.base_left = base_left
        self.base_right = base_right

        self._master_fd, self._slave_fd = os.openpty()
        tty.setraw(self._master_fd)
        tty.setraw(self._slave_fd)
        os.set_blocking(self._master_fd, False)

        self.device = os.ttyname(self._slave_fd)

        self._stop = threading.Event()
        self._thread = None
        self.state = self.sampler.initial_state()
    # def

    def _period(self, hz):
        if self.rate_ceiling:
            hz = min(hz, self.rate_ceiling)
        return 1.0 / float(hz)
    # def

    def _serve(self):
        pending = b""
        emitted = 0
        period = self._period(self.state["hz"])
        next_t = time.monotonic()

        while not self._stop.is_set():
            while not self._stop.is_set():
                remaining = next_t - time.monotonic()
                if remaining <= 0:
                    break

                try:
                    chunk = os.read(self._master_fd, 256)
                except (BlockingIOError, OSError):
                    chunk = b""

                if chunk:
                    pending += chunk

                    while b"\n" in pending or b"\r" in pending:
                        cut = min(pending.index(c) for c in (b"\n", b"\r") if c in pending)
                        line, pending = pending[:cut], pending[cut + 1:]

                        text = line.decode("ascii", errors="ignore")
                        if not text.strip():
                            continue

                        new_state, reply, _action = self.sampler.apply_command(self.state, text)

                        if reply is not None:
                            self._write(reply + "\r\n")

                        if new_state is not self.state:
                            self.state = new_state
                            period = self._period(self.state["hz"])
                            next_t = time.monotonic()
                            remaining = 0

                if remaining <= 0:
                    break

                time.sleep(min(remaining, 0.0005))

            next_t += period

            now = time.monotonic()
            raw_l = self.base_left + (int(now * 7) % 9)
            raw_r = self.base_right + (int(now * 5) % 7)

            if self.state["ts"]:
                t_us = int(now * 1000000) % TICKS_MODULO
                line = "[%d:%d/%d]\r\n" % (t_us, raw_l, raw_r)
            else:
                line = "[%d/%d]\r\n" % (raw_l, raw_r)

            emitted += 1
            if self.drop_every and emitted % self.drop_every == 0:
                continue

            self._write(line)
    # def

    def _write(self, text):
        try:
            os.write(self._master_fd, text.encode("ascii"))
        except (BlockingIOError, OSError):
            pass
    # def

    def start(self):
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        return self
    # def

    def stop(self):
        self._stop.set()

        if self._thread is not None:
            self._thread.join(2.0)
            self._thread = None

        for fd in (self._master_fd, self._slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass
    # def

    def __enter__(self):
        return self.start()
    # def

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False
    # def
# class
