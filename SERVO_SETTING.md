# Servo end stop setting procedure

How the elevon PWM end stops and the trim point are set on the Manta rig, and
why the procedure is built the way it is. It replaces the single-pass creep the
calibration used to do, which has the fault described below.

Stages 1 and 2 set the end stops and run in both the trim tab of
`MantaTrimmer.py` and the `endpoint_cal.py` diagnostic, from the same decision
logic in `endpoint_logic.py`. [Stage 3](#stage-3--the-trim-point) sets the trim
point and is the trim tab only, from `trim_logic.py`.

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
2. Creep to 2° short of the positive target, then 2° short of the negative one,
   using the same step logic as the existing calibration — 6 µs per step, 0.25 s
   per step, against a 0.5 s trailing mean of the angle.
3. Record the PWM each creep stopped at.
4. Write the higher PWM as `MAX`, then the lower as `MIN`.

The creep is deliberately sent after less than stage 2 will set. It does not
reach as far as the rapid approach does — the same effect stage 2 exists to
correct for — so a creep asked for the full target runs out of command, and the
run fails, on servos whose refined end stops would have made that target
comfortably. Only the creep is short: the targets stage 2 refines against are
unchanged.

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
4. **Correct**, re-command the surface (a parameter write takes longer than the
   actuator override survives, so the FC parks the surface part way through and
   the next move must not start from wherever it left it), then evaluate **the
   opposite end**.

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

#### The probed gain is checked before it is used

The probe measures over a single 10 µs step taken 0.8 s after a full-span slam
into the end stop — while the surface is still creeping back from the overshoot.
When the creep is larger than the step, the subtraction returns the creep. On a
RIGHT run it returned it **with the wrong sign**, at a plausible magnitude:

| attempt | MIN read | error vs −33.0° | shift applied | implied gain |
|---|---:|---:|---:|---:|
| 1 | −31.18° | +1.82° | +21 µs | −0.087 deg/µs |
| 2 | −29.42° | +3.58° | +138 µs | −0.026 |
| 3 | −17.18° | +15.82° | +323 µs | −0.049 |
| 4 | +11.65° | +44.65° | −1348 µs | +0.033 |

The true gain there is +0.10 deg/µs (68° over a 1213–1893 span). Every
correction pushed MIN further from its target until it clamped, crossed MAX, and
drove the servo into its stop hard enough to brown out the flight controller.

So the probed gain is now believed only when it agrees with the geometry: same
sign as the nominal gain, and within 0.3× to 3× of it. The linkage gain was
measured to vary about 2:1 end to end, which that band holds comfortably.
Anything outside it is settling drift, and the correction falls back to the
nominal gain — crude, but right about which way the surface moves.

#### Bounds on every correction

Four of them, narrowest wins:

- **200 µs per attempt.** A gain that passes the check above can still be off by
  a factor of two, and another attempt costs one traverse.
- **900–2100 µs**, the same range the coarse stage opens up to before it
  creeps — an endpoint outside it is one the creep could not have found. A
  servo limit, not a PX4 one: `PWM_MAIN_MIN` accepts 800 quite happily, and
  writing it is what browned out the FC.
- **±300 µs of the coarse creep's finding.** The creep is rough but measured; an
  endpoint 300 µs away from it is a symptom, not a correction.
- **MIN and MAX stay 200 µs apart.** PX4 swaps them at param load when
  `MIN > MAX`, so a crossed pair does not fail — it calibrates on silently
  against a range nobody chose, and every reading after it is meaningless.

### Failure

Ten attempts per end. Beyond that the procedure stops, reports the error, and
**restores the `MIN`, `MAX` and trim it started with**. A partially corrected
pair is worse than the values you began with, because it looks calibrated.

It also stops early, on the same terms, when:

- **The error grew on two consecutive attempts at the same end.** A correction
  that leaves the end stop further out than it found it is pushing the wrong
  way, and applying it again only makes it worse. One growth can be hold noise
  on a surface that is nearly there; two in a row is not. On the RIGHT run above
  this ends it on attempt 3, with MIN 15° out instead of 44°.
- **A correction is pinned against its own bound** and the end stop is still out
  of tolerance. There is nowhere left to move it, so spending the remaining
  attempts driving the surface into its stop achieves nothing.

---

## Stage 3 — the trim point

The trim used to be set by the same creep as the old end stops, and inherited
the same fault: about 4° between the angle a surface holds while being crept up
to and the angle that command produces when it is driven there.

It is now found the same way the end stops are — driven to, from a known
direction, and measured settled.

### One direction, not two

**The trim point is defined as the angle reached arriving from `cmd +1`.**

Requiring both approach directions to land on target was tried and cannot be
met: there is more backlash in these servos than any band worth accepting could
close, so a two-sided test loops until it runs out of attempts and writes
nothing. The opposite approach is measured once, at the end, and **reported as
backlash** — it is a property of the servo and the linkage, and no trim value
makes it smaller.

### The procedure

1. Runs with **trim 0 still in the flight controller**. Once a trim is written
   `cmd ±1` no longer reaches the end stops, and the parks this depends on stop
   being parks. The command is carried through the search and only written when
   it is accepted.
2. The **starting estimate** comes from the two accepted end stops — they are
   known (command, angle) pairs at `cmd −1` and `cmd +1`, so the trim command
   falls straight out of them. No creep needed to find it.
3. Each attempt: traverse out to `cmd +1`, then drive to the trim command in
   one move and measure it settled.
4. Correct by `−error ÷ gain`. The first attempt uses the gain implied by the
   two end stops; after that the **secant** between consecutive attempts, which
   measures the gain where the trim actually sits. Both readings come from the
   same approach, so the lash is common to them and cancels in the difference.
   The secant is checked against the nominal gain exactly as the back-off
   probe's is — same sign, within 0.3× to 3×.
5. **Accepted at 0.25°** on that approach. Then one traverse to `cmd −1` and
   back to the same command to measure the backlash.

### Failure

Eight attempts. It also stops early when the error grows on two consecutive
attempts, or when the correction is pinned against its bound and the trim is
still out. Nothing is written in either case.

### What the trim costs

PX4 computes `clamp(cmd + trim, −1, +1)`, so a written trim always costs
travel, and the calibration prints what it cost before verifying:

- the end at the far side of the trim **falls short by `|trim| ÷ 2 × span`**
- the end the trim leans toward is reached early, so the last `|trim|` of
  command travel there **produces no movement at all**

A −0.29 trim on an 810 µs span gives up 117 µs — 14.4% — at one end and
deafens the last 29% of the command range at the other.

Both ends are then **verified a second time with the trim applied**. One of
them is expected to read `OUT` by exactly the shortfall printed above. That is
the trim being paid for, not the end stop being wrong.

### Backlash is logged

The measured backlash goes into `calibration_log.csv` as `left_backlash_deg`
and `right_backlash_deg`. A log file written before those columns existed is
widened once on the next write — current header, existing rows padded, the
original kept as `.bak`. A blank means no calibration in that session measured
one, which is not the same claim as a backlash of zero.

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
| consecutive growing errors allowed | 1 | two in a row is a runaway, not noise |
| max shift per attempt | 200 µs | |
| endpoint band | 900–2100 µs | the coarse range; 800 µs browned out the FC |
| max drift from the coarse value | 300 µs | |
| minimum MIN–MAX separation | 200 µs | PX4 swaps a crossed pair silently |
| probed gain accepted within | 0.3×–3× nominal | linkage gain varies about 2:1 end to end |
| coarse range | 900–2100 µs | wide enough to bracket any endpoint |
| coarse creep step | 6 µs | matches the existing calibration |
| coarse creep backoff | 2° | the creep reaches less than the rapid approach |
| trim approach | `cmd +1` | the direction the trim point is defined from |
| trim acceptance band | 0.25° | on that approach only |
| max trim shift per attempt | 0.15 cmd | about 5° on this airframe |
| trim attempts | 8 | |
| trim command limit | ±0.9 | beyond it the trim eats half the travel |

---

## Related

- `endpoint_logic.py`, `trim_logic.py` — the decision logic, with nothing
  attached to it. Every number in this document is a constant in one of them.
- The trim tab in `MantaTrimmer.py` — runs all three stages against the rig.
- `endpoint_cal.py` — stages 1 and 2 as a standalone diagnostic. `set` writes
  parameters, `verify` does not.
- `servo_probe.py` — read-only diagnostic. Confirms commanded PWM against what
  the FC actually drives, and can replay the old creep at varying step periods.
  A bolt-on: delete it and its test when the investigation is closed.
- The Measure tab in `MantaTrimmer.py` — range and rate, and the creep-versus-
  swing stiction number quoted above. Both share one set of hard-overs: a
  range/rate cycle *is* the swing half of the stiction comparison.
- `ISSUES.md` — the wider defect list.
