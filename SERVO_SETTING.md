# Servo end stop setting procedure

How the elevon PWM end stops are set on the Manta rig, and why the procedure is
built the way it is. This describes the two-stage method implemented in
`endpoint_cal.py`. It replaces the single-pass creep in `MantaTrimmer.py`, which
is still in place and still has the fault described below.

Trim is deliberately out of scope. See [Trim](#trim-is-not-covered).

---

## The fault this procedure exists to fix

The original calibration creeps the actuator command toward a target in 6 µs
steps, watches the elevon angle, and the moment the angle crosses the target it
converts the command it stopped at into a PWM value and writes that as the end
stop.

Every part of that is sound except the measurement. **The angle a surface holds
while being crept up to is not the angle that PWM produces when the surface is
driven there normally.**

Measured on this airframe, left elevon, at a single fixed PWM of 1926 µs:

| how the surface got to 1926 µs | settled angle |
|---|---:|
| crept in 6 µs steps at 0.25 s per step | −33.6° |
| single full-span traverse from the far end | −37.7° |

Both readings are stable — held for six seconds they move less than 0.3° — and
consecutive full-span traverses repeat to within 0.07°. The 4.1° between them is
real, repeatable, and entirely invisible to a procedure that only ever creeps.

So the end stop written by the old method is out by that margin. A verify pass
against the values it left on the rig measured the left MAX end at **−37.7°
against a −33.0° target: 4.7° of error**.

Two further facts shape the procedure:

- **The PWM itself is not in doubt.** A sweep comparing commanded against actual
  servo output, read from `actuator_outputs`, matched the tool's command-to-PWM
  model to **0 µs at every command on both channels**. The arithmetic is right;
  only the angle measurement is wrong.
- **Run-up distance matters.** The same end stop reads −36.0° when approached
  from centre and −37.7° when approached from the opposite end. Every
  measurement in this procedure is therefore taken off a full-span traverse, so
  they are all comparable to each other.

---

## Preconditions

- Vehicle disarmed, safety switch off, `COM_MOT_TEST_EN = 1`.
- Servos powered, and the Pico streaming positions.
- Both pots centred and scaled — the procedure trusts the angle readings
  absolutely, so a stale scaler puts the error straight into the parameters.
- Target angles set in `settings.json` under `ANGLES`.

The actuator test override that drives the surfaces **expires after about two
seconds** on this board regardless of the timeout requested, after which PX4
takes the outputs back. Every wait longer than that is spent re-sending the same
command, and every measurement is checked against the commanded PWM before it is
believed. A reading taken during a lapse is discarded, not reported.

---

## Stage 1 — coarse creep

Purpose: get roughly into range. **Nothing measured here is trusted or
committed.**

1. Open the range: `PWM_MAIN_MIN = 900`, `PWM_MAIN_MAX = 2100`, trim `0`.
2. Creep to the positive target, then the negative one, using the same step
   logic as the existing calibration — 6 µs per step, 0.25 s per step, against a
   0.5 s trailing mean of the angle.
3. Record the PWM each creep stopped at.
4. Write the higher PWM as `MAX`, then the lower as `MIN`.

Which end carries which angle target is taken from the measurement, not inferred
from `PWM_MAIN_REV`. On a reversed channel the negative angle sits at the *high*
PWM end, and assuming otherwise is a silent way to mirror a calibration.

---

## Stage 2 — rapid-approach refinement

Purpose: set each end stop by measuring it the way the surface is actually used.
This is the automated form of the manual procedure that always worked: command a
PWM, let the servo slew to it in one motion, read the angle, adjust.

Before the first measurement, park at the far end. That way the opening traverse
is full-span like every one after it.

Then, repeatedly:

1. **Traverse** to the end stop under test in a single command.
2. **Measure** after 0.8 s — one 0.5 s averaging window plus margin. Do not
   linger: if the surface is jammed against a mechanical stop, the servo is
   stalled and holding it there is what the next step exists to detect.
3. **Probe for a hard stop** — but only when it can tell you something. A
   surface that landed *past* its target by more than the acceptance band
   plainly has travel to spare, so it is corrected on the average gain and the
   probe is skipped; the attempt that follows lands near target and measures
   properly. Otherwise back the output off inward in 10 µs steps, waiting one
   dwell after each, until the elevon moves by at least 0.25° — comfortably
   above the 0.23–0.27° noise floor measured during steady holds.
4. **Correct**, then evaluate **the opposite end**. The traverse across to it is
   itself the next rapid approach, so alternating MAX, MIN, MAX, MIN costs
   nothing.

### Acceptance

An end stop is set when **both** hold:

- **No hard stop** — 50 µs or less of inward travel is enough to move the
  elevon. Beyond that the servo is pushing into a mechanical limit and the PWM
  value is meaningless, since a range of PWM values all produce the same angle.
- **Within 0.5°** of the target angle, measured on the rapid approach.

### Corrections

| condition | action |
|---|---|
| hard stop | pull the end stop inward by the breakaway distance + 10 µs |
| outside 0.5° | shift the end stop by `−error ÷ gain` |

The gain is deg/µs, and it comes free from the back-off probe: that probe
already moved the output a known number of microseconds and measured the
resulting angle change. Its sign falls out of the measurement, so nothing has to
assume the linkage direction or the reversal bit. On this airframe the slope is
negative — the angle rises as the PWM falls.

Because the rapid approach overshoots where the creep stopped, the corrections
normally pull both ends **inward** and the span tightens. That is expected, but
the sign is not forced: an end stop that falls short is pushed outward instead.

### Failure

Ten attempts per end. Beyond that the procedure stops, reports the error, and
**restores the `MIN`, `MAX` and trim it started with**. A partially corrected
pair is worse than the values you began with, because it looks calibrated.

---

## Verification

`endpoint_cal.py verify` re-measures both ends at whatever `MIN`/`MAX` are
currently set and reports the error against target. It writes nothing, so it is
safe to run at any time — including against a calibration produced by the old
method, which is how the 4.7° figure above was obtained.

Run it after any calibration, and treat it as the real acceptance test: it is an
independent measurement rather than the calibration marking its own work.

Two cautions when reading its output:

- Its first round approaches from wherever the surface was parked, so round 1
  will differ from rounds 2 and 3 by around 1.7°. Use the later rounds.
- If a non-zero trim is set, `cmd ±1` no longer reaches both ends and one will
  read short by `|trim| ÷ 2 × span`. It prints a warning when this applies.

---

## Values

| quantity | value | where it came from |
|---|---:|---|
| acceptance band | 0.5° | operational requirement |
| movement threshold | 0.25° | above the 0.23–0.27° hold noise floor |
| back-off step | 10 µs | |
| hard stop threshold | 50 µs | |
| hard stop pull-in | breakaway + 10 µs | |
| endpoint dwell | 0.8 s | one 0.5 s averaging window plus margin |
| attempts per end | 10 | |
| coarse range | 900–2100 µs | wide enough to bracket any endpoint |
| coarse creep step | 6 µs | matches the existing calibration |

---

## Trim is not covered

Stage 2 runs at trim 0, so it sets the end stops reached by `cmd ±1` with no
trim applied. Writing a non-zero trim afterwards changes what those commands
reach, because PX4 computes `clamp(cmd + trim, −1, +1)`.

Measured on both channels, the effect is exact:

- throw from the trim point to `cmd +1` is always **half the span**
- throw from the trim point to `cmd −1` is **`(1 − |trim|) ÷ 2 × span`**

So a −0.29 trim removes 117 µs of an 810 µs span from one end — 14.4% — and
creates a dead band where the last 29% of the command range on the other side
produces no movement at all.

That the full range is not reached after trimming is expected. What has **not**
been settled is whether the end stops should be set before the trim, after it,
or solved algebraically so the post-trim positions land on target. Until that is
decided, set the end stops with this procedure at trim 0 and treat the trim as a
separate, later question.

---

## Related

- `endpoint_cal.py` — implements this procedure. `set` writes parameters,
  `verify` does not.
- `servo_probe.py` — read-only diagnostic. Confirms commanded PWM against what
  the FC actually drives, and can replay the old creep at varying step periods.
  A bolt-on: delete it and its test when the investigation is closed.
- The Measure tab in `MantaTrimmer.py` — range and rate, and the creep-versus-
  swing stiction number quoted above. Both share one set of hard-overs: a
  range/rate cycle *is* the swing half of the stiction comparison.
- `ISSUES.md` — the wider defect list.
