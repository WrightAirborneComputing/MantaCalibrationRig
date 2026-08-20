# Sensors: how the rig measures an elevon

The rig has always measured elevon deflection one way — a rotary potentiometer
per elevon, read as raw ADC counts by a Pico. It is about to have a second way:
three inclinometers on a Teensy, one on each elevon and one on the airframe.

Both have to work, from the same laptop software, indefinitely. This document is
the architecture that makes that possible, and the reasoning behind it. The
procedure the sensors serve is documented in `SERVO_SETTING.md`; this is about
the measuring, not the calibrating.

## The two sensor sets

|  | Pot rig | Inclinometer rig |
|---|---|---|
| Board | Raspberry Pi Pico, MicroPython | Teensy 4.0, C++ |
| Sensor | 2 rotary potentiometers | 3 DFRobot 6-axis accelerometers |
| Measures | shaft angle, directly | the gravity vector, per sensor |
| Native units | 16-bit ADC counts | degrees |
| Channels | LEFT, RIGHT | LEFT, CENTRE, RIGHT |
| Sample ceiling | ~1520 Hz free-running, 1000 Hz preset | to be measured; likely 200 Hz |
| Airframe reference | none | CENTRE |
| Hard calibration | host, `settings.json` | rig, Teensy EEPROM |

The pot measures the thing you want. The inclinometer measures gravity and
infers the thing you want, which is a weaker position in three specific ways
that the rest of this document keeps having to account for:

- **It is only an inclinometer at rest.** Any real acceleration adds to gravity
  and corrupts the angle.
- **It only sees rotation about a horizontal axis.** An elevon whose hinge line
  has tilted towards vertical is measured with a cosine penalty.
- **It moves with the airframe.** Which is what CENTRE is for.

## Sides are not channels

The most important distinction in the design, and the one that makes a third
sensor possible without a rename across the whole file.

- An **elevon side** is `LEFT` or `RIGHT`. It owns an output function, PWM
  min/max/trim parameters, a calibration worker and a flow chart. There are
  exactly two and there always will be.
- A **sensor channel** is whatever the attached backend provides. The pots give
  `LEFT, RIGHT`. The inclinometers give `LEFT, CENTRE, RIGHT`.

CENTRE is a channel and never a side. It has no output function, no PWM
parameters, and no row in the Trim tab. This is why the dozens of existing
`if side == "LEFT"` branches stay correct and stay put: they are all in
elevon-land, and CENTRE never reaches them. Only the handful of sites that
actually touch a sensor become channel-aware.

Resist the urge to introduce a `Side` class. Strings work in this codebase, and
polymorphising them is a several-hundred-site diff with no behavioural payoff.

## The backend contract

A backend owns its transport, its wire format and its native units, and
advertises what it can do rather than being asked to guess. It provides:

- its identity and its channel list;
- its capabilities — sustainable sample rate, angular resolution, whether it
  stamps samples on the device, whether it can capture raw, whether it has an
  airframe reference channel, whether it can say "I was moving";
- connect, disconnect, start, stop, sample-rate negotiation, a command
  round-trip, and raw capture;
- a windowed average per channel, **in degrees**.

Units conversion lives behind that boundary. The pot backend applies its scaler
internally; the inclinometer backend receives degrees and does nothing.
`position_to_degrees()` stops being global truth and becomes a pot-backend
implementation detail.

Averaging lives *above* it, in one shared place, deliberately. A stiction figure
measured with pots and one measured with inclinometers must be averaged
identically or the comparison between them means nothing — the same argument
`range_test.settled_angle` already makes about not subtracting two numbers
estimated different ways.

### `get_side_angle()` is the seam

`FourSliderGUI.get_side_angle(side)` is already the de facto sensor seam:
`rig_sync.RigArbiter`'s read callback is bound to it, and the calibration tests
fake sensors by monkeypatching it. It stays exactly where it is, with the same
signature and the same "degrees or `None`" contract. Everything new lives
strictly below it.

It must stay an **ordinary method** — not a property, not a
constructor-injected attribute. The tests assign over it on the instance, and
that only works over a plain method. This is the single constraint that lets the
whole migration be verified step by step against the existing suite.

## Where calibration lives

The governing rule: **hard calibration specific to the rig hardware lives on the
rig; per-event tares live in the Python app.**

|  | Pot rig | Inclinometer rig |
|---|---|---|
| Hard calibration | `settings.json` scaler | Teensy EEPROM, six-face bias and scale |
| Per-event tare | `settings.json` | `settings.json` |
| Airframe correction | not available | host, per frame, from CENTRE |

The pot row is an honest exception rather than an oversight: the pot rig has
nowhere to put a calibration, and the Pico is not being reflashed for this.

### The tare is not the offset, but it is today

`set_center()` currently writes the tare into `offset`. That conflates a
per-event zero with a hardware transfer function, and it is why `clear_center()`
has to exist at all.

Splitting them is lossless, which is worth knowing before anyone worries about
the migration. `set_center()` sets `left_offset = -(scaler x raw)` and
`right_offset = +(scaler x raw)`; feed either back through
`position_to_degrees()`, whose LEFT and RIGHT branches carry opposite signs
because the pots are mounted mirrored, and the result is exactly 0.0. So after
any centring the linear intercept is structurally zero and `offset` carries
nothing but the tare. Reading an old file is a rename plus a zero, not a
re-derivation.

Three consequences worth stating plainly:

- **The tare is keyed by backend.** A zero measured with pots is meaningless for
  inclinometers. This is what makes switching sensor sets safe rather than
  quietly wrong, and `clear_center()` should fire on a backend switch for the
  same reason it fires on a drone connect.
- **The default tare is 0.0.** The built-in defaults scattered through
  `MantaTrimmer.py`, `pico_monitor.py` and `servo_probe.py` are not neutral
  values — they are somebody's zero, measured against an airframe that has long
  since left the rig, and they already disagree with each other and with the
  live file. One of them silently reports 0.0 degrees when the settings file is
  missing. Defaults belong in exactly one place, and the honest default for a
  tare is no tare.
- **The tare does not go in EEPROM.** The EEPROM is right there and it is
  tempting, but a tare is a property of the airframe currently bolted to the rig.
  Storing it on the rig would make the rig's persistent state depend on which
  drone was last mounted, which breaks the swap-and-re-zero workflow the rule
  exists to protect.

## The angle pipeline

```
native frame value
  -> backend converts to degrees      pot: scaler x counts (mirrored sign)
                                      incl: already degrees
  -> subtract CENTRE                  only if the backend has a reference
  -> subtract the host tare           per channel, per backend
  = the degrees every caller already expects
```

### One frame, all channels

All three inclinometer channels arrive on **one line**, with one device
timestamp taken next to the reads. Three independent streams would push
triplet alignment into every consumer — the GUI, `range_test.py`,
`pico_monitor.py`, `servo_probe.py` — and would need time alignment exactly when
it is least reliable.

For the same reason the CENTRE subtraction is **per frame, not per window**:
left minus centre, sample by sample. Subtracting window means would cancel a
static airframe tilt but not airframe *motion* during a transit, and airframe
motion during a transit is the entire reason CENTRE exists.

This puts a real obligation on the firmware. Three UART modules free-run
independently, so a naive round-robin read produces a "frame" whose values are
not simultaneous. The firmware keeps one slot per channel, stamps each on
arrival, and emits the current contents together with each one's age. A channel
with no fresh data emits a sentinel that **cannot parse as a number**, so a host
that forgot to handle it fails loudly at the parse rather than quietly consuming
a magic value.

The host rule that follows: **no silent degradation when CENTRE is missing.**
Report it. Never fall back to un-subtracted angles as though nothing happened.

### Knowing when not to believe a reading

An accelerometer at rest reads gravity; an accelerometer in motion reads
gravity plus whatever else is happening. The pot this replaces had no such
failure mode, so swapping the sensor removes a property the calibration
currently depends on unless something replaces it.

`rig_sync.py` establishes that this class of contamination is already
unacceptable on this rig: *"The rig is one frame. Sweeping an elevon shakes it,
and that shake arrives at the opposite potentiometer as angle."* With a pot that
is mechanical coupling. With an accelerometer it is also direct acceleration
sensing, so the problem gets strictly worse.

The board should therefore say how much to trust each frame. Be honest about
which signal does what:

- **Vector magnitude departure from 1 g** is a *second-order* detector of the
  case that hurts most. A horizontal 0.1 g tilts the apparent vertical by
  `atan(0.1)` = 5.7 degrees while changing the magnitude by only 0.5%. It is a
  good detector of vertical acceleration and of knocks, and a weak one of
  horizontal acceleration. On its own it is a false comfort.
- **Gyro magnitude is first-order in motion** and is by a wide margin the better
  static gate. These modules have a gyro; use it as the primary signal.
- **Host-side variance across the averaging window** is the third leg, and the
  host can compute it from the series it already holds.

Emit what the board can, name each honestly, and let the host combine them.

## Capabilities, and honest degradation

The pot rig sustains 1000 Hz. The inclinometer rig will not — the modules cap
out somewhere near 200 Hz. Measurements that depend on rate must therefore
declare what they need and be told whether they can have it, **before anything
moves**:

- **ok** — run.
- **degraded** — run, but stamp the results. At 150 Hz a ~100 ms transit
  resolves into ~15 samples: travel is still sound, transit and rate are roughly
  +/-10%.
- **refuse** — say which sensor set is attached, what rate it offers and what
  was needed.

Two rules about where that check lives:

**The refusal belongs in the analysis layer, not the UI.** `analyse_leg()`
should take the achieved rate and decline to report a transit figure, with a
stated reason, rather than compute one from eight samples. It already returns
`{"ok": False, "reason": ...}` and already guards on sample count, so this fits
the existing idiom rather than inventing a mechanism. UI-only gating would cover
neither `range_test.py`'s CLI path nor `servo_probe.py`.

**Every export records which sensor produced it.** A stiction CSV from pots and
one from inclinometers are not comparable, and today nothing on the row says
which you are reading. `calibration_log.csv` is append-only against a tracked
file — append columns, never reorder them.

### Resolution may bind before rate does

Stiction is a difference between two settled angles. `endpoint_logic.py` sets
`MOVEMENT_THRESHOLD_DEG = 0.25`, explicitly "above the 0.23-0.27 deg hold noise
floor measured", with `ENDPOINT_TOLERANCE_DEG = 0.5`. Those numbers come from
pots resolving about 0.004 degrees per count.

If an inclinometer's static noise after averaging lands near the top of its
plausible range, no sample rate rescues it and the end-stop procedure itself
needs retuning. **Measure the noise floor before committing to the parts.** It
is the one unknown that can invalidate the hardware change rather than merely
complicate it.

## The six-face calibration

Three sensors bolted to one flat plate, moved through six faces, calibrated in
one pass. The plate is the fixture; gravity is the only reference.

### The model

Per sensor, per axis, bias and scale:

```
measured = scale x true + bias        so   true = (measured - bias) / scale
```

For any static orientation the corrected vector must have magnitude *g*. Each
captured orientation therefore contributes one scalar residual, and six
orientations give six equations for six unknowns.

Seed it closed-form from the two faces where each axis reads plus and minus *g*
— bias is the mean of the pair, scale is half their difference — then refine
against all captured orientations using their *measured* vectors rather than
their nominal ones. That refinement is what removes the cosine error from a face
that was not perfectly flat.

No numpy. The refinement is a small normal-equations solve, and
`range_test._solve` is already a general Gaussian elimination with partial
pivoting. `range_test.fit_polynomial`'s docstring states the standing policy
outright: numpy is not in `requirements.txt`, the README commits to a stock
Windows install, and a calibration cannot be the thing that adds a build
dependency.

### What six faces cannot do

Six orientations give six scalar equations. A full model — a 3x3 matrix plus
three biases — has twelve unknowns, so the system is **rank-deficient by six**.
Misalignment cannot be solved this way, no matter how carefully the plate is
held. It is a counting argument, not a precision problem.

Worth stating because "six-face calibration" is widely written about as though
it does solve misalignment; the versions that do are drawing constraints from
somewhere else, such as a turntable or known reference angles. Solve bias and
diagonal scale, and reserve room in the stored record for misalignment terms
written as identity, so adding them later is a version bump rather than a format
break.

### Judging the result

Six faces and six unknowns is exactly determined, so the residuals are
approximately zero **by construction**. Reporting that as a quality score would
be a fit statistic with zero degrees of freedom, and it is the trap this
procedure invites. Quality comes from three other places:

1. **Tilt from nominal, per face, in degrees.** The direct measurement of
   whether the face was flat.
2. **Per-face scatter** during the hold, plus gyro magnitude if available.
   Bounds precision.
3. **Extra orientations.** Three arbitrary tilts on top of the six faces turn
   nine equations against six unknowns into three genuine degrees of freedom,
   and only then does a residual mean anything. This is the largest quality win
   available for the least operator effort.

Then sanity-gate the answer itself. A scale factor outside roughly 0.90 to 1.10,
or a bias beyond about 100 mg, means an axis is mislabelled or a face was badly
wrong. Refuse to write, and say which sensor and which axis.

### The plate-consistency check

The payoff for calibrating all three sensors at once, and something a
single-sensor calibration cannot do at all.

All three are bolted to one rigid plate, so **the angle between any two
sensors' corrected gravity vectors must be the same in every orientation**.
Compute it per face for each of the three pairs. A spread across faces of more
than a degree or two means the plate flexed, a sensor shifted, or one sensor's
solve is bad — and the pattern of which pairs disagree names the culprit.

It is cheap, hardware-free and fully testable. Put it in front of the operator.

### Guiding the operator

Order the faces so that consecutive ones are a **single 90-degree flip** rather
than a re-orientation. Faster, and much harder to get wrong.

Accept a face at under about 8 degrees from nominal, warn between 8 and 15, and
reject past 15. The justification is in the numbers already on the rig: an 8
degree tilt costs 1.0% of scale under a naive min/max solve, but the refinement
uses the measured vector, so what survives is well under 0.1 degrees — far below
the 0.23-0.27 degree hold noise floor recorded in `endpoint_logic.py`. Past 15
degrees the face classification itself becomes ambiguous, so rejecting is a
correctness gate rather than a quality one. This is what "or close enough"
means, quantified.

Name the failing sensor: "LEFT is 14 degrees off" beats "a sensor is off" when
you are holding a plate still with both hands.

And tell the operator once why both faces of each axis are needed, because it is
not obvious: bias error dominates near level (5 mg is about 0.29 degrees at
zero), scale error dominates at the extremes (1% is about 0.35 degrees at 35),
and only both faces separate the two.

### Writing it to the rig

Stage each sensor, then commit the whole set in one operation. A
one-command-per-sensor write leaves the store holding one sensor's new
coefficients and two sensors' old ones if the cable comes out mid-sequence.

Read it back and check **both** the values and the checksum the write reported —
the checksum catches a transcription error the value comparison would miss. Then
hold the plate flat once more and confirm all three read within about half a
degree of each other. Ten seconds, and it catches a whole class of "the write
landed but the wrong way round" faults.

An empty or corrupt store means the board **refuses to emit degrees** and says
so on every identity reply. The alternative — emitting with a flag — loses
because flags get ignored, and the consequence here is a drone trimmed against
wrong angles. A board with no calibration has exactly one useful thing it can
do, which is be calibrated, and the raw mode the wizard needs is available
regardless.

Log the pass to `reports/`. It does **not** belong in `calibration_log.csv`,
which is a fixed-schema, append-only record of per-drone elevon calibrations
keyed on drone name and UID; a rig sensor calibration has neither key and is a
different kind of event. Give it its own tracked top-level log.

Then stamp the calibration's checksum into every run artefact the rig produces
afterwards. That one field is what lets you answer "which calibration were those
forty drones measured against" six months later, and it is the strongest
argument for the readback command existing at all.

## Firmware and wire protocol

Firmware lives in `teensy/`, mirroring `pico/`. Match the documentation style of
`pico/sampler.py`: measured numbers, and the reasoning behind every choice.

Two invariants carry over from the Pico unchanged, because both are load-bearing:

- **Every board-to-host reply is prefixed `#`**, so replies are inert to every
  sample parser.
- **Every command is short enough to type by hand** into a serial terminal,
  because the console is the debugging tool.

### Identifying which board is attached

The host sends an identity command on connect. The Teensy answers with its
channel list, units, rates and features. The Pico answers anything it does not
recognise with `# ERR`, and the legacy 10 Hz firmware never reads stdin and
answers nothing at all — so all three are distinguishable, and **no change to
`pico/sampler.py` is required**. That matters: the pot rig must keep working
exactly as it does today.

Adding an identity command to the Pico too is still worth doing, because
self-description beats inference, but it is not a prerequisite.

Carry a protocol version and refuse politely on an unknown one, rather than
parsing a format that might not mean what it appears to.

### The frame

Keep the bracketed grammar so the reply-prefix invariant holds, generalised to
carry three channels and a device timestamp. The existing two-channel regex
cannot match a three-channel line, so the formats are safely distinguishable —
but the new format needs its own pattern rather than a widened shared one.

Consider giving raw mode a **different delimiter** from degrees mode. A host that
missed the mode-change acknowledgement — reconnected mid-run, restarted, lost
the reply — then physically cannot mistake raw counts for degrees. It costs one
character, and the failure it prevents is writing garbage angles into a drone's
calibration, which is both unrecoverable and invisible.

**Units on the wire: signed integer millidegrees.** Plus or minus 35 degrees is
plus or minus 35000, comfortably in range. It keeps the grammar integer-only and
parsing cheap, which matters because the parse runs in Python on every line
while the board formats at 600 MHz and does not care. Integers also parse
exactly, which a reproducible log wants. And floats admit `nan`, which would
parse successfully and propagate a sensor fault silently — with integers a fault
must use the explicit sentinel and cannot masquerade as a number.

### Storage

A record with a magic number, a version, per-sensor bias and scale, reserved
misalignment terms, and a checksum over the whole. Teensy 4.0 has no true EEPROM
— it emulates about a kilobyte in flash, which is ample — but flash writes have
finite endurance, so nothing writes except an explicit operator-initiated
commit. No per-boot write, no remembered mode, no counters. Skip the write
entirely when the staged record matches what is already stored, so re-running a
calibration and getting the same answer costs nothing.

Store each sensor's name alongside its coefficients even though the index
implies it. It costs a couple of dozen bytes and buys a dump you can read plus a
check that the stored record matches the wiring the firmware expects. Left/right
swaps are exactly the class of fault that produces a plausible-looking and
completely wrong calibration.

### Two hazards worth naming in advance

`Serial.print` on a Teensy will **block** if the USB endpoint backs up while a
host is attached but not reading, and that stall blocks the UART drain loop and
drops sensor frames. Check writability and drop the sample line instead — a
missing line is visible in the timestamps, whereas a stalled UART is silent data
loss that looks like a flaky sensor.

Host-side, `send_command()` returns exactly **one** line, so any multi-line reply
needs a block-reading variant before it works at all. And any unsolicited
`#`-prefixed warning the board emits lands in the same reply queue and can be
mistaken for a command reply, so the reply matcher must skip them.

## Testing

**The pots keep their property entirely.** `tests/fake_pico.py` imports the real
`pico/sampler.py` under stubbed `machine` and `utime` and runs its actual
command parser over a pty. Nothing about that changes, and all existing tests
keep their exact coverage. It is a large part of why the pot backend is migrated
first.

The Teensy cannot share source with Python the same way, so the boundary moves
from shared code to shared spec plus conformance:

1. **One pure-Python protocol module as the written authority** — grammar,
   command table, reply prefix, field order. The host parses with it and the
   fake generates with it, so the two cannot drift.
2. **A ctypes shim over the firmware's pure headers.** Keep everything decidable
   in headers that never include `Arduino.h`, taking `now_us` as a parameter
   (exactly as `cal_flow.SideFlow` takes `now`) and *naming* actions rather than
   performing them (exactly as `sampler.apply_command` returns `"collect"`).
   A small `extern "C"` wrapper compiled at test time then lets the fake Teensy
   serve a pty running the **real C++ parser and formatter**. Gate it on a
   compiler being present and skip cleanly otherwise, keep a pure-Python
   fallback so host-side tests run everywhere, and add one conformance test
   asserting both produce identical replies over a corpus of command lines —
   that test is what stops the fallback becoming a second source of truth.
3. **A recorded real trace, checked in**, replayed through the real parser. The
   only thing that proves the compiled firmware agrees with the specification.

Not worth it: a second test runner. `python3 -m pytest tests/` is the one command
in the README and should stay so.

A **contract test parameterised over every backend** gives any future backend its
tests for free — connect and stream, rate negotiation within advertised rates,
capture width matching the channel list, timestamps present only if advertised,
clean disconnect.

## Migration order

Incremental, with the suite green at every step. Ordered so the risky extraction
lands before the work that would otherwise duplicate it.

0. **Characterisation tests, no production change.** Pin the arithmetic before
   touching it. The existing suite covers threading but nothing pins the
   *numbers*. This is the safety net for everything after.
1. **Unify the three side-to-channel tables** into one module. Pure data move;
   keep the old attribute names as aliases so existing tests do not move.
2. **Extract settings persistence** out of `PositionReader`. Preserve the
   section-merge semantics verbatim — its docstring explains why, and it is what
   lets two processes share the file.
3. **Extract the transport.** `PositionReader` becomes composition and keeps
   every public method. Acceptance: the capture and GUI-shell tests pass with
   zero edits. Highest-risk step, because it is threading over a real pty.
4. **The backend contract, the pot backend, and the angle pipeline.** The pot is
   the first implementation, not a special case.
5. **Normalise the sample record** to named channels, retiring the positional
   four-tuple that is unpacked by position in three places. The one unavoidable
   signature change.
6. **Teensy firmware and backend.**
7. **Runtime backend selection** and per-backend port memory.
8. **Collapse the duplicate transports — three of the four.**

Do not ship steps 3 and 6 together. A transport extraction and a new backend
failing at the same time is undebuggable.

### One transport stays duplicated on purpose

`pico_monitor.py` keeps its own reader loop. It is the low-level diagnostic
whose entire value is that it shares no code with the application: when the
abstraction is broken, `pico_monitor` is how you find out. Folding it in is the
tidy answer and the wrong one.

## What the modules actually do

Measured on the bench 2026-08-20 with `teensy/bringup`, two modules attached
and lying flat. All frame checksums valid.

### The wire format is WitMotion

Eleven-byte frames, `55 <type> <8 data> <checksum>`, checksum being the low byte
of the sum of the first ten. Four frame types arrive in a repeating cycle:

| Type | Contents | Notes |
|---|---|---|
| `0x54` | magnetic | all zeros - this is a 6-axis part with no magnetometer |
| `0x51` | **acceleration** | int16 per axis, plus die temperature |
| `0x52` | angular velocity | int16 per axis |
| `0x53` | attitude angle | int16 per axis, plus a version word |

So the cycle is **44 bytes per sample set**, of which 11 carry nothing at all.

Acceleration scales as `raw / 32768 x 16 g`, angle as `raw / 32768 x 180 deg`.

**Raw per-axis acceleration is available.** This was the question that decided
whether the six-face calibration was possible at all, and the answer is yes: the
`0x51` frame is unfiltered accelerometer output, and the module's own fused
angle arrives separately in `0x53` as a free cross-check. Decoded together while
lying flat, the accelerometer gave a roll of -2.18 degrees where the module's
own filter reported -2.21, which agrees closely enough to trust both.

The gyro reads exactly zero on all three axes while stationary, which is what
makes it usable as the first-order motion gate described above.

### The modules were reconfigured, and now run at 200 Hz

As shipped they streamed 441 bytes/s - one 44-byte sample set per 100 ms, so
**10.0 Hz** - and the 9600 baud default could not have carried much more than
21.8 Hz whatever the module was asked for. The link was the constraint, not the
part.

Three changes, applied to all three modules and saved to their flash, in this
order and for this reason - free capacity first, then raise demand:

| Register | Set to | Effect |
|---|---|---|
| `0x02` RSW | `0x000E` | accel + gyro + angle; drops the magnetic frame |
| `0x04` BAUD | `0x0006` | 115200 |
| `0x03` RRATE | `0x000B` | 200 Hz |

The magnetic frame was 11 of every 44 bytes carrying nothing but zeros - this is
a 6-axis part with no magnetometer - so dropping it recovered a quarter of the
link for free and took the sample set from 44 bytes to 33. Measured effect was
exactly the predicted ratio: 443 B/s to 332 B/s.

Measured after all three changes, all channels, no bad checksums:

| Channel | rate | of the 115200 link | sample rate |
|---|---|---|---|
| LEFT | 6621 B/s | 57.5% | 200.6 Hz |
| CENTRE | 6625 B/s | 57.5% | 200.7 Hz |
| RIGHT | 6606 B/s | 57.3% | 200.2 Hz |

200 Hz is the part's rated ceiling, so this is as fast as these modules go. The
link still has 42% spare, which is deliberate - the Pico rig's own notes make
the case for not running a stream at its limit, and the headroom is there if a
fourth frame type is ever wanted back.

**The rate increase was free in noise terms.** Re-measured at 200 Hz, the static
scatter was 0.0128 / 0.0142 / 0.0111 degrees against 0.0129 / 0.0142 / 0.0134 at
10 Hz - identical within the measurement. A higher output rate often costs
noise, because there is less internal averaging behind each sample. Here it did
not, so there is no rate-versus-resolution trade to make: take the 200 Hz.

Note the USB side was never the constraint and still is not. The Teensy's link
to the host is CDC at 480 Mbit, and three channels at 200 Hz is about 20 kB/s.
This is a reversal of the Pico rig, where the board's own `print()` cost set the
ceiling.

### What 200 Hz means for the rate-dependent measurements

The rig's elevon transit is about 100 ms. At the Pico's 1000 Hz that resolved
into roughly 100 samples; at 200 Hz it is about 20. Travel and settled angles
are unaffected - they are averages of a stationary surface. What degrades is
anything reading the *shape* of the edge: the 10-90% crossings behind
`transit_ms` and `rate_deg_s`.

So the three-outcome rule above is not hypothetical, and the numbers to gate on
are now real rather than assumed. 20 samples across an edge is a usable but
noticeably coarser measurement, and it should be reported as degraded rather
than either refused or quietly presented as equivalent to a pot run.

### The static noise floor is far better than the pots

The measurement that could have invalidated the whole sensor change, and it
comes out decisively the right way.

All three sensors undisturbed on the bench, individual unaveraged `0x51`
readings, tilt scatter taken about each sensor's own mean gravity direction:

| Channel | n | mean magnitude | angle scatter (sd) | worst | die temp |
|---|---|---|---|---|---|
| LEFT | 78 | 1.0122 g | 0.0129 deg | 0.064 deg | 29.1 C |
| CENTRE | 79 | 1.0005 g | 0.0142 deg | 0.066 deg | 25.8 C |
| RIGHT | 66 | 1.0120 g | 0.0134 deg | 0.059 deg | 24.9 C |

Against `MOVEMENT_THRESHOLD_DEG = 0.25`, sitting just above the pots' measured
0.23-0.27 degree hold noise floor, that is roughly **twenty times quieter** -
and it is quieter *before* any of the 0.5 s averaging the app already applies.
The end-stop procedure's existing tolerances are in no danger.

Be honest about what this measurement is not: a couple of hundred samples over
a few minutes, on a bench, with nothing running. It says nothing about drift
over the length of a real run, or what the noise looks like with a powered
airframe on the rig. Re-measure in place before relying on it. But the failure
mode that was worth worrying about is not present, and it is not close.

### The three sensors disagree by 1.2%, which is the point

They report gravity as 1.0122, 1.0005 and 1.0120 g. They cannot all be right;
gravity is gravity. That 1.17% spread is scale-factor error, and left
uncorrected it is worth **0.41 degrees at 35 degrees of deflection** - the same
order as the trim resolution the whole procedure is chasing - while being
**exactly zero at the centre position**, where a naive zeroing would be done.

This is precisely the error the six-face calibration exists to remove, measured
on the actual parts before writing a line of it. It is also why a single
flat-surface zeroing would not have been enough: it would have made all three
read zero at the centre and left the 1.2% untouched at the extremes, which is
where the endpoints are set.

Worth noting what the spread is *not*: LEFT sat 4.2 C warmer than RIGHT and the
two agree to within 0.02%, while CENTRE, sitting between them in temperature, is
the one that differs. So this is unit-to-unit variation rather than thermal
drift - which is the good case, because unit-to-unit variation is a constant the
calibration can store, and thermal drift would not be.

## What is still not known

- **Noise on a live rig**, as opposed to a quiet bench.
- **That the module configuration survives a power cycle.** The changes were
  written to module flash with a save command and verified live, but the modules
  have not been powered down since. Confirm on next power-up; if they revert to
  9600 and 10 Hz, the save did not take and the host will need to apply the
  configuration at connect instead.
- **Drift** over the length of a real calibration run, and with temperature. The
  modules report die temperature in every `0x51` frame, so this is measurable
  without extra hardware.

### A wiring note, since it cost a session to find

On first power-up Serial1 (LEFT) received **zero bytes** over a ten-second
window while Serial2 and Serial3 each took in about 4,500. Not a slow channel or
a garbled one - nothing at all. It was miswired, and re-terminating it fixed it
completely: all three now stream identically at 10.0 Hz with no bad checksums.

Recorded because it is the class of fault worth checking first and in this
order - module powered, its TX to the Teensy's RX rather than TX, the pin pair
actually being the one assumed, then the module itself - and because a
byte-counting probe found it in seconds where a frame parser would have reported
something more ambiguous. That is the argument for the bring-up firmware
counting bytes before it understands them.

A second lesson from the same session, recorded because it nearly became a
false finding: a byte counter that is reset by a command is only as trustworthy
as the reset. One `Z` was silently swallowed by leftover bytes in the board's
command buffer, and the resulting "rate" was five times the real one and
physically impossible at 9600 baud - which is the only reason it was caught.
**Measure rates as a delta between two reads, never against a reset**, and
sanity-check any rate against the link's capacity before believing it.

## Related

- `SERVO_SETTING.md` — the procedure these sensors serve.
- `pico/sampler.py` — the current firmware, its protocol and its measured limits.
- `teensy/bringup/` — the bring-up firmware these measurements came from.
- `rig_sync.py` — why one elevon's motion contaminates the other's reading.
- `ISSUES.md` — including issue 9, on sample rate and angle resolution.
