# Known issues

Defects found during the code review that preceded the `feature/cross-platform-rework`
branch. Everything below is **unfixed** unless marked otherwise; each entry carries a
proposed fix so the work can be picked up without re-deriving the analysis.

Line references point at `MantaTrimmer.py` as of that branch. Issues in the deleted
`ElevonTrimmer.py` are excluded.

---

## 1. Tk variables are read from worker threads — High

**Location:** `_sweep_to_csv_worker` reads `drone_name_var.get()` at
[MantaTrimmer.py:1311](MantaTrimmer.py#L1311); the calibration workers reach
`get_int_var`/`get_float_var` via `get_side_expected_pwm` at
[MantaTrimmer.py:2053-2063](MantaTrimmer.py#L2053).

**Symptom:** Tk is not thread-safe. Reading a `StringVar` off the main thread can
corrupt interpreter state or raise `RuntimeError: main thread is not in main loop`.
It is intermittent and load-dependent, so it presents as a random freeze or crash
during a sweep or auto-calibration rather than an obvious bug.

**Proposed fix:** Snapshot every needed variable on the GUI thread *before* starting
the worker and pass the values in as arguments. For values the worker must re-read
later, push a request through `post_to_gui` and have the callback deliver the result
back over a `queue.Queue`.

`post_to_gui` / `_drain_gui_queue` were added on this branch and are the correct
mechanism — note that the older `root.after(0, ...)`-from-a-worker pattern (used by
`_set_side_param_vars_on_gui_thread` and `_sweep_to_csv_worker`) is itself unsafe:
`after()` registers a Tcl command and must only be called from the Tk thread. Those
remaining call sites should be migrated to `post_to_gui` as part of this fix.

---

## 2. Blocking MAVLink calls run on the GUI thread — High

**Location:** `apply_left_min` at [MantaTrimmer.py:2346](MantaTrimmer.py#L2346) and its
five sibling apply callbacks.

**Symptom:** `set_param_value` retries until a 5 s verify deadline. The entire UI
freezes for up to 5 s per button press, and longer on a flaky link. The operator
cannot tell a slow write from a hung app.

**Proposed fix:** Run each apply in a short-lived worker thread; disable the button
and show a pending state until a `post_to_gui` completion callback re-enables it.
`_connect_worker` is the shape to copy.

---

## 3. Position samples are destructively consumed — High

**Location:** `get_average_position_nonblocking` at
[MantaTrimmer.py:932](MantaTrimmer.py#L932) *pops* up to `num_samples` entries;
`update_labels` [MantaTrimmer.py:2456](MantaTrimmer.py#L2456) calls it at 10 Hz, and so
do the calibration workers via `get_side_angle`.

**Symptom:** The Pico produces ~10 samples/s and the UI drains at 10 Hz, so each call
typically averages **one** sample — there is no smoothing at all despite
`num_samples = 10`, and the displayed angle is noisy. Worse, the UI steals samples
from the calibration workers, which then receive `None` and stall in the retry at
[MantaTrimmer.py:2096](MantaTrimmer.py#L2096), making auto-calibration slow and erratic.

**Proposed fix:** Store `(timestamp, value)` pairs and make the getter *peek* a time
window (e.g. the last 300 ms) instead of popping, trimming entries by age. Multiple
consumers then coexist and the average is genuinely over N samples. Pairs with
issue 9.

---

## 4. `calibration_log.csv` column count mismatch — Medium

**Location:** header written at [MantaTrimmer.py:1793](MantaTrimmer.py#L1793), row
written at [MantaTrimmer.py:1809](MantaTrimmer.py#L1809).

**Symptom:** The existing file on disk has a 14-column header ending in `Folding?`
(hand-added in commit `a096ee3`), but the code writes 13 values per row and would
write a 13-column header to a fresh file. Every appended row leaves `Folding?` empty,
and a newly created log silently loses the column entirely.

**Proposed fix:** Add `Folding?` to both the header list and the row tuple, sourced
from a new UI field (or an empty string while it stays a manual annotation). Assert
`len(row) == len(header)` before writing so the two cannot drift again.

---

## 5. Instrumentation log grows without bound — Medium

**Location:** `InstrumentationLog._queue` at [MantaTrimmer.py:35](MantaTrimmer.py#L35);
`update_log_window` at [MantaTrimmer.py:2526](MantaTrimmer.py#L2526) only ever inserts.

**Symptom:** Both the queue and the `Text` widget grow without limit. A multi-hour
calibration session degrades progressively and eventually exhausts memory.

**Proposed fix:** Bound the queue (`queue.Queue(maxsize=...)`; `write` already swallows
the resulting exception, so full means drop) and trim the widget after each insert:
`self.log_text.delete("1.0", "end-%dl" % MAX_LINES)`.

---

## 6. `set_drone_name` never persists the name — Low

**Location:** [MantaTrimmer.py:754](MantaTrimmer.py#L754) calls `save_calibration()`,
but `drone_name` is absent from the dict built at
[MantaTrimmer.py:712](MantaTrimmer.py#L712).

**Symptom:** Setting the drone name rewrites `settings.json` with unchanged content.
The name is lost on restart, and the write is pure overhead.

**Proposed fix:** Add `"drone_name"` to the saved dict, read it back in
`load_calibration`, and seed `drone_name_var` from it at startup. Note that
`refresh_params_from_drone(clear_name=True)` deliberately clears the name on connect —
decide whether persistence should survive a reconnect or only apply across restarts.

---

## 7. `_mav_lock` contention causes UI stutter — Low

**Location:** `update_actuators` [MantaTrimmer.py:2507](MantaTrimmer.py#L2507) needs the
lock every 100 ms; `get_param`'s receive loop
[MantaTrimmer.py:511](MantaTrimmer.py#L511) holds it in 0.2 s slices for up to its full
timeout.

**Symptom:** Sliders and labels visibly hitch whenever a parameter is being read or
verified, which is constantly during auto-calibration.

**Proposed fix:** Hold the lock only around the actual `mav.*_send` / `recv_match`
call rather than across the whole cache-check block, and have the actuator tick skip
rather than block when the lock is contended
(`self._mav_lock.acquire(blocking=False)`).

---

## 8. Float equality on a target angle — Low

**Location:** `target_angle_deg == 0.0` at
[MantaTrimmer.py:2107](MantaTrimmer.py#L2107).

**Symptom:** Works today only because `0.0` is passed as a literal. Any computed trim
target can miss the branch, fall through to the `< 0.0` / `> 0.0` comparisons, and
never register as reached — the move then runs to its 60 s timeout.

**Proposed fix:** `elif abs(target_angle_deg) < 1e-6:`.

---

## 9. Pico firmware sample rate limits angle resolution — Informational

**Location:** `utime.sleep(0.1)` in [pico/main.py:21](pico/main.py#L21).

**Symptom:** 10 Hz output with no averaging on the Pico side. Combined with issue 3,
each displayed angle is a single unfiltered ADC reading, so the value visibly jitters.

**Proposed fix (firmware):** Oversample inside the Pico loop — average e.g. 16
`read_u16()` calls per transmitted line. This costs nothing at the 10 Hz output rate
and removes most of the noise before it reaches the host.

---

## Fixed on `feature/cross-platform-rework`

Recorded for context; no action needed.

- **Hardcoded Windows COM ports.** `DroneInterface("COM20")` / `PositionReader("COM5")`
  in the old entrypoint made the tool Windows-only and machine-specific. Replaced by
  USB-ID and heartbeat-based auto-detection with an editable UI override.
- **`wait_heartbeat()` had no timeout.** With no flight controller attached the GUI
  thread hung unrecoverably. Now bounded, and `connect()` returns a bool.
- **Position reader thread died permanently.** Any `SerialException` killed the thread
  with no restart path, so angle readouts silently sat at `--` forever. The reader is
  now stoppable and restartable via `set_port`, driven by Refresh/Connect. *A full
  auto-reconnect supervisor loop is still worth adding* — the operator currently has
  to click Connect again after a USB glitch.
- **Nothing was closed on exit.** No `WM_DELETE_WINDOW` handler; the MAVLink link and
  serial port leaked and the reader thread was killed mid-`readline()`. Now handled by
  `on_close`.
- **Blocking MAVLink calls in `FourSliderGUI.__init__`.** Seven parameter round-trips
  ran before any widget existed — up to ~35 s of dead time on a bad link. Moved into
  `refresh_params_from_drone()`, called after connection.
- **Config and log paths resolved against the current working directory.** Launching
  from another directory silently started from default calibration and wrote logs
  elsewhere. All three paths are now anchored to `APP_DIR`.
- **Reader loop caught only `serial.SerialException`.** An `OSError` from a yanked USB
  device escaped and killed the thread with a bare traceback that bypassed the
  instrumentation pane. Now catches `(serial.SerialException, OSError)` and routes
  through `describe_serial_error`.
- **`_param_cache.clear()` ran outside `_mav_lock`** while worker threads could be
  reading it. Now inside the lock, in `close()`.
- **No safe way to update widgets from a worker thread.** Added `post_to_gui` and the
  `_drain_gui_queue` poller. All new worker code uses it; see issue 1 for the older
  call sites that still need migrating.
