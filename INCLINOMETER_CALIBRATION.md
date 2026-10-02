# Inclinometer calibration

How the rig's three inclinometer modules are calibrated, what the procedure
measures, and what it measured on 2026-08-28.

`SENSORS.md` carries the architecture and the reasoning behind every threshold.
This document is the procedure and its results.

## What is being calibrated, and what is not

Three DFRobot 6-axis modules — `LEFT`, `CENTRE`, `RIGHT` — read by a Teensy 4.0
over three hardware UARTs. Each reports a gravity vector, and the rig infers
angles from it.

Three separate coefficients come out, and keeping them apart is most of the
point:

| Coefficient | What it fixes | Where it is measured | Where it lives |
|---|---|---|---|
| **bias**, per axis | a sensor reading non-zero under no acceleration | six faces | Teensy EEPROM |
| **scale**, per axis | a sensor reading 1.014 g when it means 1.000 | six faces | Teensy EEPROM |
| **rotation**, per sensor pair | the modules not being quite parallel | the plate's rigid fit | Teensy EEPROM |
| *tare* | the airframe currently on the rig | operator, per session | `settings.json` |

The tare is not part of this procedure and is not affected by it. It is a
property of the drone bolted to the rig, and it is zeroed per session exactly as
it has always been.

## The method

The plate is presented on each of its six faces. Gravity is the only reference,
and for any static orientation the corrected vector must have magnitude *g*, so
each orientation contributes one scalar residual.

Seeded closed-form from the two faces where each axis reads plus and minus *g* —
bias is the mean of the pair, scale is half their difference — then refined by
Gauss-Newton against every captured orientation, using their **measured**
vectors rather than their nominal ones. That refinement is what removes the
cosine error from a face that was not perfectly flat, and it is not optional:
with faces held up to 7 degrees off, the closed-form seed loses up to 0.85% of
scale and the refinement recovers the injected coefficients to within 0.03%.

Three extra orientations at arbitrary awkward angles follow the six faces. Six
faces against six unknowns is exactly determined, so its residual is zero by
construction and means nothing; the extra three make it nine equations against
six unknowns and only then does a residual say anything. Measured: 8e-17 g with
six faces, 5.2e-5 g with nine orientations.

No numpy. The refinement is a small normal-equations solve and the rotation fit
is Davenport's q-method with power iteration, both in `sensor_cal_logic.py`,
which imports nothing.

## Running it

```bash
python3 sensor_cal.py
```

Requires `teensy/rig` flashed. The wizard identifies the board, asserts raw
mode, and pre-flights every channel over the live stream before you pick
anything up.

Then present the plate on each face in turn. It detects each face itself and
starts capturing once the plate has been steady for about a second; each hold is
three seconds. Faces are accepted **in whatever order they arrive** — there is
nothing to get right about the sequence.

### You do not need to hold it still, or for long

Both were requirements in an early version and neither survived contact with the
measurement:

- **Duration.** Against the modules' real noise floor, 50 samples pin bias to
  0.03 mg and scale to 0.003%. Six thousand samples reach 0.003 mg. The sanity
  gates care about 100 mg and 10%. Three seconds is ample.
- **Stillness.** Five degrees of hand wander costs 0.0036% of scale, which at 35
  degrees of elevon is 0.0013 degrees — two orders below the rig's own
  0.23-0.27 degree hold noise floor.
- **A twitch costs one sample, not the capture.** Disturbed samples are dropped
  and the hold continues. Only the plate arriving on a *different* face ends a
  capture early.

Holding *longer* is in fact slightly worse when hand-held: averaging vectors
spread over a cone shortens the resultant, so the mean reads short and scale
comes out low. It is a bias rather than noise, so more time accumulates it
rather than averaging it away. The fix is that a sample's magnitude does not
care which way the plate points, so the observation takes its direction from the
mean vector and its length from the mean of the individual lengths.

### If a channel dies

A serial lead working loose has cost two runs on this rig. The pre-flight
refuses to start on a silent or intermittent channel, and a channel that dies
mid-run stops the wizard rather than being retried — retrying a dead channel
cannot work, and an earlier version reported it as "past 15 degrees from flat",
which sent the operator to check their hands instead of the cable.

The board's `?` reply gives per-channel frame counts, bad-checksum counts and
the age of the last frame. `age_ms` is live; `hz:` and `sensors=` are measured
once at boot and do not refresh, so a reconnected channel still reads `hz:0`.

### Options

| Option | Effect |
|---|---|
| `--tilts 0` | six faces only; gives up the only real quality statistic |
| `--seconds 10` | longer holds; buys almost nothing |
| `--replay FILE` | re-solve a saved raw capture, no board needed |
| `--write JSON` | write a completed run to the board and verify the read-back |
| `--check` | show live how well the three sensors agree |

Every run writes its raw stream to `reports/`, a full record as JSON, and one
row per sensor to the tracked `sensor_cal_log.csv`.

## Judging a run

The residuals cannot be the quality score. Four things can:

1. **Run-to-run agreement.** Two independent runs of the same hardware. This is
   the strongest statement available, because it is one physical thing measured
   twice rather than a fit reporting on itself.
2. **Tilt from nominal, per face.** Accept under 8 degrees, warn to 15, reject
   past it — where the face classification itself turns ambiguous.
3. **Per-face scatter and wander.** Bounds precision.
4. **The plate's rigid fit.** Below.

Then the sanity gates: a scale outside 0.90 to 1.10 or a bias past 100 mg means
an axis is mislabelled or a face was badly wrong. The wizard refuses to write and
names the sensor and the axis.

### The plate check

All three modules share one rigid plate, so **one rotation should carry any
sensor's corrected vectors onto another's in every orientation at once**. Fit
that rotation across all nine orientations and inspect what is left over.

This separates two things that a cruder check conflates. The **mounting
rotation** is how far apart the modules are bolted: a real quantity, not a
fault, and the coefficient a datum correction needs. The **residual** is whether
they stayed that way, and it is the only part gated on — warn at 0.25 degrees,
reject at 0.40.

Which pairs fail names the culprit: the sensor appearing in *both* bad pairs is
the sensor.

## Writing it to the board

```bash
python3 sensor_cal.py --write reports/sensor_cal_<stamp>.json
```

Every sensor is staged and the set committed in one operation, because a
per-sensor write leaves the store holding one sensor's new coefficients and two
sensors' old ones if the cable comes out mid-sequence. The commit returns the
CRC it wrote; the host then reads the record back and checks the values, the
rotations **and** the CRC. The CRC catches a transcription error the value
comparison would miss, and the value comparison catches a CRC computed over the
wrong thing.

A commit matching what is already stored is skipped, so re-running costs no
flash endurance. With no valid record the board refuses to emit corrected values
at all and says so on every identity reply — a flag would get ignored, and the
consequence is a drone trimmed against wrong angles.

## The results, 2026-08-28

### The modules

Measured through `teensy/rig` at 200 Hz, resting on the desk:

| | |
|---|---|
| sample rate | 200.1 Hz host-side; board timestamps exactly 5000 µs apart, no gaps |
| dropped lines | 0 with a host reading |
| bad checksums | 1 in ~149,000 |
| per-axis scatter | X 0.48, Y 0.39, Z 1.33 mg — 0.073 to 0.083 degrees |
| gyro at rest | exactly 0.000 dps on two channels, 0.244 dps peak on the third |

The raw accelerometer is about six times noisier than the 0.0128 degree figure
recorded for these modules elsewhere — that one is the module's own *fused*
attitude output, and this is the unfiltered accelerometer the calibration
consumes. Z is consistently about three times noisier than X and Y on all three.

### The coefficients

From the run of 17:31, committed as CRC `F7D78C77`:

| Sensor | bias X, Y, Z (mg) | scale X, Y, Z |
|---|---|---|
| `LEFT` | +6.601, −10.827, +21.683 | 0.99421, 0.99704, 0.99184 |
| `CENTRE` | −3.162, −13.185, +6.926 | 0.99588, 0.99767, 0.99309 |
| `RIGHT` | −0.360, −17.712, +18.442 | 0.99611, 0.99640, 0.99302 |

### Repeatability

Three runs, all hand-held, the plate fully re-handled between each:

| | worst bias difference | worst scale difference |
|---|---|---|
| 16:45 vs 16:53 | 0.260 mg | 0.0452% |
| 16:45 vs 17:31 | 0.328 mg | 0.0417% |
| 16:53 vs 17:31 | 0.557 mg | 0.0521% |

0.05% of scale is 0.016 degrees at 35 degrees of elevon — about fifteen times
better than the hold noise floor the rig already lives with. The coefficients are
not the weak part of this measurement.

### What the calibration achieves

Live off the board, before and after:

| | `LEFT` | `CENTRE` | `RIGHT` |
|---|---|---|---|
| raw magnitude | 1.01406 g | 1.00054 g | 1.01219 g |
| corrected magnitude | 1.00021 g | 1.00018 g | 1.00006 g |

A 1.4% magnitude error becomes 0.02%. Then, with the stored rotation applied to
bring each outboard sensor into `CENTRE`'s frame:

| | before | after |
|---|---|---|
| `LEFT` to `CENTRE` | 1.078° | **0.012°** |
| `RIGHT` to `CENTRE` | 0.436° | **0.026°** |

## Two results that look wrong and are not

### The angles between sensors get bigger before they get smaller

Raw, the three sensors sat 0.708 / 0.171 / 0.566 degrees apart. Corrected, 1.078
/ 0.648 / 0.436. Two of the three grew.

The raw figures were flattered. Each sensor's bias and scale errors tilt its
apparent gravity vector, and those tilts partly cancelled between sensors — so
the uncorrected readings agreed better than the sensors did. What remains after
correction is the **true** mounting misalignment, which the plate's rigid fit
independently measured at 1.49 / 0.92 / 0.67 degrees. Each observed angle is at
or below its fitted rotation, exactly as the geometry requires: a single
orientation shows the angle between **g** and *R* **g**, which reaches the full
rotation angle only when gravity is perpendicular to *R*'s axis.

The corrected numbers are honest and the raw ones were not.

### A rigid plate whose sensors are a degree apart

The obvious worry is that a degree of disagreement between modules bolted to one
flat plate means either the calibration is wrong or the plate is not rigid.
Neither follows, and the data settles it.

**The plate is rigid.** That is what the rigid fit's residual measures, and it
came in at 0.091 degrees. When a module genuinely *was* loose — the 16:45 run,
where a serial lead had also been disturbed — the same number read 1.04 to 1.32
degrees. A factor of eleven.

**The rotations are real.** Between the two clean runs, fully re-handled, they
reproduce to 0.18 to 0.37 degrees. A misalignment that repeats like that is a
fixed physical quantity, not an artefact.

**The plate's flatness says nothing about the silicon.** The accelerometer die
is soldered to a PCB, the PCB sits in a housing, and the housing bolts to the
plate. Solder reflow alone typically leaves 0.5 to 2 degrees of die-to-package
rotation. A degree between three separate modules is unremarkable, and it is
inside them.

**And no six-face calibration could ever have fixed it.** The method's only
constraint is that corrected gravity has magnitude 1 *g*, and magnitude is
invariant under rotation — so every misalignment fits the data equally
perfectly and the solve has no basis to prefer one. It is a symmetry rather than
a shortage of equations. `SENSORS.md` carries the derivation.

Which is why the rotation is measured by comparing sensors to each other, and
removed by applying it as a rotation. That is what the datum is for, and it takes
1.078 degrees to 0.012.

### A check that does not work

The three fitted rotations compose exactly — `LEFT`→`CENTRE` then
`CENTRE`→`RIGHT` equals `LEFT`→`RIGHT` to 0.0000 degrees — which looks like a
strong rigidity test and is not one. The 16:45 run, with a module genuinely
loose, also composed to 0.0046 degrees: when a single sensor moves, the
compromise fit propagates through the composition consistently and cancels.

Recorded so nobody reaches for it as a fault check. The residual is the one that
works.

## Related

- `SENSORS.md` — the architecture, and the reasoning behind every threshold here.
- `sensor_cal.py` — the wizard, the writer and the checker.
- `sensor_cal_logic.py` — the method, with nothing attached to it.
- `teensy/rig/` — the firmware, its calibration store and its wire formats.
- `SERVO_SETTING.md` — the procedure these sensors ultimately serve.
