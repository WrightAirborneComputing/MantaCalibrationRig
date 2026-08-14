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
| `pico/main.py` | MicroPython firmware for the Pico. Copy to the Pico as `main.py`. |
| `settings.json` | Persisted pot scalers/offsets, target angles, and last-used ports. |
| `calibration_log.csv` | Append-only record of every logged calibration. |
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

## Running

```bash
python3 MantaTrimmer.py
```

The window opens immediately, then scans for hardware in the background. If a
device is missing, the app stays usable — plug it in, hit **Refresh ports**,
then **Connect**.

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

`pico/main.py` is MicroPython, not desktop Python — running it with CPython
fails on `import machine`. Copy it to the Pico's filesystem as `main.py`
(Thonny, `mpremote`, or `rshell`) and it runs automatically at power-on.

It samples ADC0 (GPIO26, left) and ADC1 (GPIO27, right) and prints one line
per sample at 10 Hz:

```
[<left_u16>/<right_u16>]
```

The onboard LED toggles each cycle as a heartbeat.
