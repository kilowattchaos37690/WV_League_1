# Pinpoint Turn Tuning Guide

## Robot Configuration
- GoBilda chassis with mecanum wheels
- GoBilda Pinpoint odometry computer (4-bar pods)
- REV Control Hub
- Pinpoint convention: **+H = CCW (left)**, +X = forward, +Y = left

---

## Fixes Applied (Nov 2026)

### Fix 1 — TURN_CMD_SIGN: 1.0 → -1.0 (Critical)

**Symptom**: X (-90 CCW) spun 270-360° the wrong way. B (+90 CW) went the wrong
direction but safety catches masked it.

**Root cause**: The mecanum mix uses positive `turn` = CW. The Pinpoint uses +H = CCW.
When `angleErrorDeg` returned positive (need CCW), the PID produced a positive command,
which drove the robot CW — the opposite direction. Both turns went the wrong way.

**Files**: `RobotPinpointMyBlocks.java` line 95, `RobotPinpointMyBlocks_diag.java` line 95

### Fix 2 — TURN_CW_SCALE / TURN_CCW_SCALE: reset to 1.0

**Symptom**: Asymmetric turn results.

**Root cause**: These were tuned to compensate for the wrong turn direction. With
TURN_CMD_SIGN fixed, the compensation made things worse.

**Rule**: Only adjust these AFTER everything else is tuned. They compensate for
physical asymmetry (one direction has more friction), not PID issues.

### Fix 3 — Removed `no_reverse` safety check in turnDegreesOdo

**Symptom**: B overshoots by 30-35°, X undershoots by 30-35° — symmetric and opposite.

**Root cause**: The D term produces a braking force that can flip the sign of the total
PID output near the target. The `no_reverse` check interpreted this as "trying to go
backwards" and killed power immediately — while the robot was still 30-35° away. CW
momentum carried it past; CCW momentum was insufficient to reach the target.

**What still protects against runaway**: `settle` (within tolerance), `crossed_target`
(error sign flip after 85% progress), `passed_target` (120% of commanded angle).

### Fix 4 — Approach scale applied to clip limit, not PID output

**Symptom**: Persistent 10-30° overshoot regardless of DECEL_HEADING_DEG.

**Root cause**: The approach scale multiplied the raw PID output, which is much larger
than TURN_MAX_POWER for most of the turn. The `Range.clip` then capped it to pMax,
rendering the approach scale invisible until the last ~11°. The robot ran at full power
until 11° from target — not enough distance to brake.

**Before** (approach ineffective until close):
```java
cmd = TURN_CMD_SIGN * PID * approach;
lastTurn = Range.clip(cmd, -pMax, pMax);
```

**After** (approach directly caps speed from DECEL_HEADING_DEG out):
```java
cmd = TURN_CMD_SIGN * PID;
double limit = pMax * approach;
lastTurn = Range.clip(cmd, -limit, limit);
```

**Files**: Both `turnDegreesOdo` and `turnToHeadingOdo` in both Pinpoint files.

### Fix 5 — HOLD_HEADING / HOLD_CROSS_AXIS: false → true

**Symptom**: Robot rotated during straight-line drives in SamplePinpointKids.

**Root cause**: Without heading hold, uneven motor power/friction caused the robot to
drift rotationally during forward/strafe moves. These flags were intentionally off
during turn debugging (the debug ladder) and needed to be re-enabled once
TURN_CMD_SIGN was correct.

### Fix 6 — bindFromLinearOpMode calls in Java OpModes

**Symptom**: Erratic timing, telemetry not updating, control loop not yielding.

**Root cause**: Blocks-to-Java exports don't include `bindFromLinearOpMode(this)`.
Without it, `linearOpMode` is null, so `opModeIsActiveCompat()` always returns true
and `idleCompat()` does nothing. The control loop never yields to the FTC scheduler.

**Rule**: Every Java `@Autonomous` or `@TeleOp` that uses `RobotMyBlocks` or
`RobotPinpointMyBlocks` must call both bind methods before init:
```java
RobotMyBlocks.bindFromLinearOpMode(this);
RobotPinpointMyBlocks.bindFromLinearOpMode(this);
```
Blocks OpModes do NOT need this — the Blocks runtime handles it automatically.

### Fix 7 — telemetry.setMsTransmissionInterval(100)

**Symptom**: Driver Station display slow to update, values took time to settle.

**Root cause**: FTC default is 250ms. Setting to 100ms makes updates 2.5x faster.

**Where**: Add in `runOpMode()` after init, before `waitForStart()`.

---

## Final Tuned Constants

```
TURN_CMD_SIGN       = -1.0      // positive PID error → CCW motor command
TURN_MAX_POWER      =  0.10     // hard cap on turn speed
HEAD_P              =  0.035    // proportional gain
HEAD_I              =  0.0      // no integral
HEAD_D              =  0.001    // light damping
HEAD_F              =  0.0      // no feedforward
DECEL_HEADING_DEG   = 45.0      // start speed reduction at 45° from target
HEADING_TOLERANCE_DEG = 5.0     // stop when within 5°
TURN_CW_SCALE       =  1.0     // no directional trim
TURN_CCW_SCALE      =  1.0     // no directional trim
HOLD_HEADING        = true      // correct rotation during forward/strafe
HOLD_CROSS_AXIS     = true      // correct lateral drift during forward/strafe
```

**Result**: +90° lands at +83°, -90° lands at -85° (5-7° accuracy).

---

## Tuning Guide for Future Changes

### If you increase TURN_MAX_POWER (faster turns)

More power = more momentum = more coast at the end.

| TURN_MAX_POWER | DECEL_HEADING_DEG | Notes |
|---|---|---|
| 0.10 | 45 | Current — slow but accurate |
| 0.12-0.13 | 50-55 | Moderate speed, good for competition |
| 0.15 | 60 | Fast, may need SCALE factor trim |
| 0.20+ | 70+ | Aggressive, expect 5-10° overshoot |

**Rule of thumb**: when doubling power, add 50% to DECEL_HEADING_DEG.

HEAD_P and HEAD_D do NOT need to change when you change power — they scale with
heading error, not motor power.

### If turns overshoot (goes past target)

Try in this order:
1. **Lower TURN_MAX_POWER** (most effective — less momentum)
2. **Raise DECEL_HEADING_DEG** (start braking earlier)
3. **Raise HEAD_D** to 0.002 (more active braking — risk: oscillation if too high)

### If turns undershoot (stops short)

Try in this order:
1. **Raise TURN_MAX_POWER** slightly (more power to overcome friction)
2. **Lower DECEL_HEADING_DEG** (brake later, carry more speed)
3. **Raise HEAD_P** to 0.04 (stronger drive toward target)

### If turns oscillate (wheels jitter back and forth near target)

The D term is too strong relative to P:
1. **Lower HEAD_D** (try halving it)
2. If HEAD_D is already 0, lower HEAD_P

### If one direction consistently over/undershoots more than the other

This is physical asymmetry (friction, weight, motor differences):
1. If CW overshoots: lower TURN_CW_SCALE (e.g. 0.95)
2. If CCW undershoots: raise TURN_CCW_SCALE (e.g. 1.05)
3. Adjust in 0.02-0.05 increments, test 3-4 times per change

### If robot drifts sideways or rotates during forward/strafe

- Verify HOLD_HEADING = true and HOLD_CROSS_AXIS = true
- If still drifting, the heading PID may need stronger response:
  raise HEAD_P or check pod offsets (X_POD_OFFSET_MM, Y_POD_OFFSET_MM)

### If small-angle turns (< 20°) don't move

The approach scale reduces power to near-zero at small angles.
Add a minimum power floor in turnDegreesOdo:
```java
double limit = Math.max(0.03, pMax * approach);
```

### If battery level affects turn accuracy

Low battery = less torque = undershoot. Full battery = more torque = overshoot.
This is normal. SCALE factors can partially compensate, but the best fix is to
always run autonomous with a battery above 12.5V.

---

## Constants NOT changed (still at original values)

```
FWD_P / STR_P       = 0.05     // forward/strafe PID — were working fine
FWD_D / STR_D       = 0.002    // forward/strafe damping
STRAFE_CMD_SIGN     = -1.0     // strafe direction was correct
X_POD_OFFSET_MM     = 5        // pod geometry — measure on your robot
Y_POD_OFFSET_MM     = -150     // pod geometry — measure on your robot
POS_TOLERANCE_IN    = 1.5      // forward/strafe stop tolerance
DEFAULT_TIMEOUT_SEC = 8.0      // safety timeout
```
