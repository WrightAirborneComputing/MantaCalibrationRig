# Manta Calibration Rig

Tooling for calibrating the elevon control surfaces of the Manta drone.

The rig is a Raspberry Pi Pico with a potentiometer on each elevon. The Pico
reports both pot positions over USB; `MantaTrimmer.py` converts those to angles
and drives the flight controller's PWM min/max/trim parameters over MAVLink
until each surface hits its target deflection.

Plug the Pico into one USB port and the flight controller into another. Both
ports are detected automatically — see [Port detection](#port-detection).

## Layout

| Path | What it is |
|---|---|
| `MantaTrimmer.py` | The desktop GUI. This is the tool you run. |
| `manta_common.py` | Shared constants and helpers (wire format, USB IDs, port discovery, angle maths). No tkinter or pymavlink, so it imports on a headless box. |
| `pico_monitor.py` | CLI diagnostic: streams the Pico's raw pot data with the conversion shown at every stage. |
| `range_test.py` | The range & rate test from the console. Also the analysis the GUI tab imports. |
| `pico/sampler.py` | MicroPython firmware, dual-rate (10 Hz / 1000 Hz). Copy to the Pico as `main.py`. |
| `pico/main.py` | The legacy 10 Hz-only firmware, kept as a fallback. |
| `settings.json` | Persisted pot scalers/offsets, target angles, and last-used ports. |
| `calibration_log.csv` | Append-only record of every logged calibration. Tracked on purpose, so it stays at the top level rather than in `reports/`. |
| `reports/` | Per-run CSV artefacts: sweep captures, range/rate summaries and sample dumps. Gitignored — clear it out whenever you like. |
| `tests/` | Host-side tests. No hardware needed: `python3 -m pytest tests/` — see [Tests](#tests). |
| `SERVO_SETTING.md` | The end stop setting procedure, and the measurements behind it. |
| `endpoint_cal.py` | Implements that procedure offline. `set` writes parameters; `verify` does not. |
| `servo_probe.py` | Read-only PWM diagnostic. A bolt-on - delete it when the investigation closes. |
| `ISSUES.md` | Known defects with proposed fixes. |

## Setup

Runs on Linux and Windows. Python 3.8+.

```bash
pip install -r requirements.txt
```

### Linux (Debian/Ubuntu)

`tkinter` is not bundled with the system Python and must be installed separately:

```bash
sudo apt install python3-tk
```

Serial access needs membership of the `dialout` group. Check with `id -nG`; if
`dialout` is missing:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in for it to take effect.

### Windows

Install Python from python.org with the Tcl/Tk option enabled (it is on by
default), then `pip install -r requirements.txt`. No driver setup is needed —
both devices enumerate as USB CDC.

### Tests

No hardware needed — the Pico and the flight controller are both faked:

```bash
python3 -m pytest tests/
```

The GUI tests build real Tk windows and pump the event loop, so a few dozen
windows flash up over the run and steal focus while they do. On Linux, hand them
a throwaway X server instead of yours:

```bash
xvfb-run -a python3 -m pytest tests/
```

Same tests, same result, nothing on screen. `-a` picks a free display number, so
parallel runs do not collide. On Debian/Ubuntu it comes from `sudo apt install
xvfb`. Unsetting `DISPLAY` is not the same thing: the
windowed tests fail rather than skip, because `tkinter` imports fine and only
`tk.Tk()` needs the server.

## Running

```bash
python3 MantaTrimmer.py
```

The window opens immediately, then scans for hardware in the background. If a
device is missing, the app stays usable — plug it in, hit **Refresh ports**,
then **Connect**.

Connection state and the instrumentation log sit outside the tabs, so the link
and the sample rate stay visible whichever tab you are on.

### Trim tab

The per-side PWM min/max/trim controls, the position sliders, and auto
calibration. This is the original UI.

### Range & rate tab

Drives the elevons hard over from −1 to +1 and back while capturing at 1000 Hz,
and reports how far they moved and how fast. The default is both together over 30
cycles. Running one side at a time was how this started — the point was to expose
coupling through shared power or linkage — and it measured the same travel and
rate either way, so the single-side phases are still there for isolating one
servo but are no longer part of a routine run. Set the phases and the cycle
count, then press **Run test**. It asks before moving anything.

The headline is the 10–90% transit time, with rate derived from it. Endpoints are
where a servo is slowest and least repeatable, so 10–90% is both the standard
measure and the stable one. Results are reported as mean ± sample standard
deviation across the cycles.

Alongside range, the table and both CSVs carry the **max and min settled angle**,
each with its own standard deviation. Range is their difference and conceals
them: a surface whose whole travel has shifted a few degrees reports an unchanged
range while both endpoints have moved, and an endpoint that wanders between
cycles shows up in its sd even when travel does not. The endpoints are clustered
by direction and then ordered by value, not by direction name, because which way
round they fall depends on the side's sign convention.

Two export buttons sit side by side. **Save CSV** writes the summary — one row
per phase/side/direction, mean ± sd. **Save samples** writes every captured
sample, one row per reading, in the same schema as `range_test.py --samples-csv`
so the same plotting scripts read both. Both files carry the run's timestamp
rather than the moment you pressed the button, so they pair up on disk.

Reach for the samples dump when the summary raises a question the aggregate
cannot settle. The worked example: a run whose travel sd is four times the usual
is either drifting steadily across the run or throwing a few bad cycles, and the
mean ± sd is identical either way — but settled position plotted against cycle
number separates them immediately. Samples are retained for every run, so the
button can be pressed after the fact; a run past `MAX_EXPORT_SAMPLES` keeps the
first rows and says so in the note line rather than quietly holding a partial
record.

Watch the trace, not just the table: a servo at its mechanical limit shows a
rounded start as it accelerates and a settling approach at the end, while a
slew-limited command tracks a straight line and stops crisply. That difference is
how you tell whether the autopilot is limiting the surface or the servo is.

The same test is available from the console, which also writes every captured
sample for plotting. Both files land in `reports/`:

```bash
python3 range_test.py --name manta --samples-csv
```

## Port detection

The **Connection** panel at the top left shows what was found and lets you
override it.

- **Pico** — identified by its USB vendor ID (`0x2E8A`, Raspberry Pi). If that
  fails, each remaining port is opened and checked for the Pico's `[n/n]` data
  format.
- **Drone** — identified by an actual MAVLink heartbeat, which is authoritative.
  Ports with a known flight-controller vendor ID are probed first; the port
  already claimed by the Pico is never probed.

Both dropdowns are editable, so an unlisted device (`/dev/ttyUSB0`, `COM7`) can
be typed in directly. Successful ports are remembered in `settings.json` and
tried first next time.

Legacy motherboard serial ports (`/dev/ttyS*`, `COM1`) are filtered out of the
list.

### If a port won't open

- **Permission denied** (Linux) — you are not in the `dialout` group; see above.
- **Device or resource busy** — something else has the port. On Ubuntu,
  ModemManager grabs `/dev/ttyACM*` for a few seconds after plug-in, so waiting
  and hitting **Connect** again usually works. Otherwise check for a running
  QGroundControl, serial monitor, or second copy of this tool.

## Pico firmware

The firmware is MicroPython, not desktop Python — running it with CPython fails
on `import machine`. Copy it to the Pico's filesystem **as `main.py`** and it
runs automatically at power-on:

```bash
mpremote connect auto fs cp pico/sampler.py :main.py + reset
```

(`pip install mpremote` first; Thonny and `rshell` work too.)

It samples ADC0 (GPIO26, left) and ADC1 (GPIO27, right), and the onboard LED
blinks a few times a second as a heartbeat.

### Two sample modes

`pico/sampler.py` has two presets, switched over USB at runtime:

| Mode | Rate | Line format | For |
|---|---|---|---|
| slow (boot default) | 10 Hz | `[<left_u16>/<right_u16>]` | Reading on a console. Byte-identical to the legacy firmware. |
| fast | 1000 Hz | `[<ticks_us>:<left_u16>/<right_u16>]` | Resolving elevon travel and slew rate. |

It boots slow, so plugging the board into any terminal gives readable output and
the GUI keeps working unchanged. Fast mode exists because a 10 Hz stream covers a
~100 ms elevon transit in one sample; 1000 Hz covers it in about a hundred.

The microsecond stamp is taken next to the ADC read, so timing does not depend on
when USB happened to deliver the line — USB CDC arrives in bursts on a 1 ms frame
boundary. It also lets the host prove the board is *sampling* at 1000 Hz rather
than merely that lines are turning up. The counter wraps every ~17.9 minutes and
the host unwraps it.

Note that `SERIAL_BAUD` is not a real constraint anywhere here: this is a USB CDC
virtual serial port, so the requested baud rate is never applied. The ceiling is
the per-line cost of formatting and printing. Measured on the rig's RP2040:

| | |
|---|---|
| ADC pair (both channels) | 25 µs |
| `%` formatting of the line | 377 µs |
| plus `print()` to USB CDC | 510 µs |
| whole loop, incl. scheduling | **~660 µs** |

Measured sustained rates on the rig (20 s each, board's own `ticks_us`):

| Requested | Sustained |
|---|---|
| 500 Hz | 499.5 Hz |
| 750 Hz | 749.1 Hz |
| 1000 Hz | 998.0 Hz |
| 1250 Hz | 1228.5 Hz |
| 1500 Hz | 1519.8 Hz — already free-running, no sleep margin left |
| 1750 Hz | 1524.4 Hz — saturated |
| 2000 Hz | 1512.9 Hz — saturated |

So the board free-runs at about **1520 Hz** and cannot go faster. Nothing was ever
lost in transport at any of these rates — every gap was the collector — so the
ceiling is the loop cost, not the link.

1000 Hz is the fast preset: two thirds of the ceiling, so a slow iteration still
has somewhere to go. It was 500 Hz, which was needlessly conservative.

### The one wrinkle: collector stalls

MicroPython's garbage collector stops the sample loop for ~7 ms whenever it runs,
which at 1000 Hz is every ~2.5 s and costs seven consecutive samples (0.28% of
the stream). It is not tunable away — a collect costs ~4.5 ms even on an empty heap,
because the cost tracks heap size rather than garbage, so collecting *more* often
only stalls more.

What you can do is choose *when* it happens. Send `G` immediately before a
capture and the stall is paid up front, buying a clean window of ~2.5 s — a
little longer than the ~2.4 s leg the range/rate test captures, so the margin is
thin and an occasional gap late in a leg is expected rather than alarming. `pico_monitor.py` does this
automatically after any mode switch.

Because every sample is timestamped, a stall is always visible in the data rather
than silently distorting it, and `--stats` tells a stall apart from genuine line
loss: a lost line leaves a hole of exactly N sample periods, while a stall leaves
an arbitrary one (measured at ~3.8 periods).

### Commands

Newline-terminated, case-insensitive, short enough to type into any serial
terminal:

| Command | Effect |
|---|---|
| `S` | slow: 10 Hz, no timestamp |
| `F` | fast: 1000 Hz, timestamped |
| `F<hz>` | fast at a given rate, e.g. `F1500` (clamped 1–2000) |
| `G` | collect garbage now — see below |
| `?` | report current mode |

Replies are prefixed `#` (`# ACK F 1000`), which cannot match the sample regex, so
they are ignored by every parser. Ctrl-C still drops to the REPL, which is the
escape hatch when the board is streaming at 1000 Hz.

`pico_monitor.py` drives all of this:

```bash
python3 pico_monitor.py --mode fast --quiet --seconds 10 --stats
```

That switches the board, reads for ten seconds, and reports the achieved rate two
independent ways — the host's line count, and the board's own interval timing.
The second is what distinguishes "the board could not keep up" (every interval is
long) from "lines were lost in transport" (most intervals nominal, a few are
exact multiples). Without the timestamp both look the same.

The legacy `pico/main.py` never reads commands; pointing `--mode` at a board
running it prints a notice and carries on at 10 Hz.
