# Elevon hard stops, measured with the inclinometers

Where each elevon's mechanical hard stops are, measured on 2026-10-02 with
the three calibrated inclinometer modules rather than the pots. Two full
sweeps, neutral to positive stop to negative stop, both sides at once.

**Airframe: not recorded at the time of the measurement.** Add it here.

`INCLINOMETER_CALIBRATION.md` covers the sensors themselves. This document
is what they measured once mounted.

## The result

The RIGHT elevon travels about **25 degrees less** than the LEFT in total, and
it is short at both ends rather than at one:

| | + travel | − travel | **full range** |
|---|---|---|---|
| `LEFT` | +51.7 / +53.1 | −58.7 / −56.3 | **110.4 / 109.4** |
| `RIGHT` | +42.0 / +42.2 | −43.0 / −42.6 | **85.0 / 84.8** |

Degrees from neutral, second sweep / first sweep.

RIGHT reaches about 79% of LEFT's travel going positive and 76% going
negative, and splits almost evenly about neutral (+42/−43). A shortfall that
is proportional at both ends points at whatever sets the range as a whole on
that side — stop geometry, linkage or servo horn — rather than one end being
obstructed.

It is not the sensors. For a sensor to read 20-25% short, its scale would need
to be off by that much, and its corrected magnitude would then sit well away
from 1 g. Every reading below is between 0.998 and 1.001 g.

### Which side repeats

- **RIGHT's stops are solid.** Its positive stop repeated to 0.1 degrees between
  sweeps, its negative to 0.3, and its full range to 0.2.
- **LEFT's positive stop repeats; its negative stop gives.** LEFT read +47.18 at
  the positive stop on the first sweep, and +47.29 with the sensor still
  mounted upright before the remount below. The negative stop went 2.1 degrees
  further on the second sweep with nothing touched and the airframe steady. Something on
  that side flexes or has play at the negative end, so LEFT's negative
  range depends on how hard it is pushed.

LEFT's +46.05 on the second sweep is about 1.1 degrees under its other two
positive readings. That hold was shaky (below), and LEFT probably came off
the stop for part of it. On that reading its full range is nearer 111.5.

## The mounting

`CENTRE` is on the airframe body, X up, Y along the hinge line towards the LEFT
elevon. It is the check that the airframe did not move: it stayed within 0.13
degrees of its neutral at every pose of both sweeps.

`RIGHT` is mounted the same way as `CENTRE`, on the elevon.

**`LEFT` is mounted X-down**, because the wiring fouled the elevon at the
negative end the other way up. A half-turn always reverses two axes, never
one, and this one is **about Z**: X and Y are reversed, Z is unchanged, so
LEFT's +Y points away from its own elevon.

The data decides it. LEFT's Z stayed negative at neutral after the flip. A
half-turn about Y would have reversed it and put the elevon at +5.9 at
neutral, 11 degrees from where it had sat a minute before. And read as a
Z flip, LEFT reaches +47.18 at the positive stop, against +47.29 upright —
the same stop to 0.1 degrees.

So **anything reading LEFT must reverse its X and Y first.** Without that it
reads about 174 degrees out and turns the wrong way.

Bias and scale belong to each module and survive the remount. LEFT's stored
plate rotation does not, since it was measured with the module upright, so
`sensor_cal.py --check` gives wrong numbers for any comparison involving LEFT.
Nothing below applies a stored rotation.

## The method

Corrected vectors straight off the board (`D` mode, calibration `F7D78C77`),
200 Hz. Ten seconds at neutral, five at each stop.

- **Elevon angle**, about Y: `atan2(z, x)`, after LEFT's X and Y are reversed.
  Positive is towards the airframe's upper surface, PX4's positive.
- **Out of plane**: `asin(y / |a|)`. How far the up vector leaves the plane
  the elevon swings in.

`atan2(y, x)` looks like the natural out-of-plane measure and is not one. It
divides by x, which shrinks as the elevon swings, so it overstates the tilt
at large angles: at 60 degrees, by a factor of two.

**Travel is measured from that sweep's own neutral.** The neutral moves by a
few tenths of a degree between sweeps as the elevons settle, so a stop is
never compared against another sweep's neutral.

The sign was checked on the airframe before the LEFT remount. Raising the left wing
sent Y positive on all three modules (+7.4 degrees, agreeing to 0.04);
raising the right wing sent it negative (−4.8, agreeing to 0.1).

## The sweeps

### Sweep 1

Neutral at 18:14:41, saved as `reports/sensor_ref_20261002_181441.json` with
its raw stream. The two stop readings were not saved to disk; the figures
below are as printed during the session.

| | `LEFT` | `RIGHT` | `CENTRE` |
|---|---|---|---|
| neutral | −5.931 | −5.176 | +0.229 |
| positive stop | +47.180 (+46.94 to +47.34) | +36.981 (+36.77 to +37.15) | +0.196 |
| negative stop | −62.242 (−62.43 to −61.99) | −47.806 (−48.12 to −47.46) | +0.107 |
| travel + / − | +53.111 / −56.311 | +42.157 / −42.631 | |
| full range | **109.422** | **84.787** | |

Out of plane, neutral / + / −: LEFT +1.32 / +1.12 / −1.57, RIGHT
+0.73 / −0.16 / +1.41. Magnitudes 0.998 to 1.001 g.

### Sweep 2

All four poses in `reports/sensor_sweep_20261002_181901.json`. The first
neutral was taken while the airframe was still being handled — 3-7 mg of noise
against a resting 0.3, the body swinging ±1.6 degrees — so it was retaken.
Both are in the file; the first is labelled `neutral_disturbed` and nothing
is measured from it.

| | `LEFT` | `RIGHT` | `CENTRE` |
|---|---|---|---|
| neutral (18:19:51) | −5.660 (−5.70 to −5.58) | −5.122 (−5.16 to −5.07) | +0.223 |
| positive stop (18:20:19) | +46.051 (+44.55 to +47.56) | +36.876 (+35.73 to +38.34) | +0.168 |
| negative stop (18:21:10) | −64.341 (−64.50 to −64.18) | −48.084 (−48.30 to −47.90) | +0.100 |
| travel + / − | +51.711 / −58.681 | +41.998 / −42.962 | |
| full range | **110.392** | **84.960** | |

Out of plane, neutral / + / −: LEFT +1.28 / +1.36 / −2.02, RIGHT
+0.75 / −0.15 / +1.44.

The positive hold is the noisy one: 4-9 mg, with the body moving by up to 3
degrees. Holding the elevons on the stop by hand shakes the airframe. Bracing
it would firm up that row.

### Out of plane

Both elevons leave their swing plane by 1-3 degrees across full travel, LEFT
more than RIGHT. That is consistent with each module's Y sitting a degree or
two off the true hinge line, and it moves the elevon angle by a small fraction
of a degree. It is not a fault.

## The soft stops, under command

`inclinometer_sweep.py slam`, 18:31: both elevons driven between −1 and +1
by `MAV_CMD_ACTUATOR_TEST`, ten cycles, 200 Hz, the last 0.75 s of each
1.5 s hold. A command of ±1 lands on PX4's configured output limits, so
these are the soft stops. Data in `reports/elevon_20261002_183129_slam.json`,
with the full stream beside it.

Angles are each elevon **relative to `CENTRE`**, subtracted sample by sample,
so airframe movement during a hold cancels. Mean ± sd over the ten cycles:

| | +1 | −1 | **range** | command 0 |
|---|---|---|---|---|
| `LEFT` | +24.548 ± 0.012 | −45.367 ± 0.021 | **69.92** | −14.06 |
| `RIGHT` | +21.658 ± 0.048 | −38.508 ± 0.021 | **60.17** | −12.22 |

The asymmetry is the same shape as at the hard stops, smaller. RIGHT
covers 86% of LEFT's commanded range (77% at the hard stops), and the
shortfall sits mostly at the negative end: 6.9 degrees there, 2.9 at the
positive. Watching the run confirmed it by eye — the negative extents are
plainly different, the positive ones a little. They should not differ at
either end.

**Repeatability is 0.01 to 0.05 degrees** cycle to cycle, an order better than
the pot rig's 0.23-0.27 degree hold noise floor.

**The correction against `CENTRE` works.** On cycle 10's −1 hold the airframe
was jolted and `CENTRE` swung through 3.2 degrees; both elevons still came
in within 0.02 of their means.

**Every soft stop is inside its hard stop.** Against sweep 2, both relative to
`CENTRE`:

| | + hard | + margin | − hard | − margin |
|---|---|---|---|---|
| `LEFT` | +45.9 | 21.3 | −64.4 | 19.1 |
| `RIGHT` | +36.7 | 15.1 | −48.2 | **9.7** |

**Command 0 is not the hand neutral.** It sits about 8 degrees (LEFT) and 7
(RIGHT) below it, of the order of the −7.5 degree `angle_trim_degs` in
`settings.json`. It also depends slightly on the side it is approached from:
LEFT read −14.31 from rest and −13.81 coming back from −1. Two samples is
not enough to call that backlash, but it is the right shape for it.

The tool marks a hold unsettled when its tail spans more than 0.5 degrees.
Most holds tripped it at an sd of 0.07 to 0.18: that is servo buzz at the
stop, not a surface still moving, and 0.5 is too tight a threshold for it.

The flight controller sent no `COMMAND_ACK` for any actuator test. The
surfaces moved regardless; the tool warns and continues, as the GUI does.

## Related

- `INCLINOMETER_CALIBRATION.md` — the calibration these readings depend on.
- `SERVO_SETTING.md` — the end stop procedure these stops bound.
