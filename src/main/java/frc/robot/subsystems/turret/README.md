# Turret Subsystem — Design Notes & Learnings

This document captures how the turret works and the hard-won lessons from tuning it (2026-07-03/04).
Read this before changing gains, the homing logic, or the shoot-on-the-move / feed path.

---

## 1. Overview

- **Actuator:** Kraken/TalonFX, `SensorToMechanismRatio = 41.666667` (rotor rot per turret rot).
- **Control:** field-relative aiming. The turret holds a *field* angle (`goalAngle`); each loop the
  robot-relative setpoint is `goalAngle - robotHeading`, wrapped into the mechanical range and
  clamped. So the turret counter-rotates as the robot turns.
- **Units:** mechanism rotations internally; the code works in radians and converts.
- **"Home" / "zero" / "stow"** all mean the same physical spot: turret pointing straight forward,
  encoder = 0, `stowed = true` holds 0° robot-relative.

---

## 2. Two different "offsets" — do not conflate

| Concept | What it is | When it applies | Field |
|---|---|---|---|
| **Aiming trim** | Operator fudge to the shot aim | **Only when shooting/tracking** | `turretOffset` |
| **Zero / home** | The mechanical encoder reference | Always | encoder position + `homingIndexRad` |

- `turretOffset` (`offsetLeft/Right`) is added to the aim target **inside the shoot commands only**.
  It is deliberately **not** in `getTurretAngle()` and **not** touched by homing.
- `getTurretAngle()` returns the **true mechanical angle** (`getPosition()`), no trim. Earlier it
  added `turretOffset`, which **double-counted** the trim into the shot calculator (the turret
  physically moves to `aim+trim`, so `getPosition()` already reflects it). That's fixed — the shot
  calculator must see the real angle.
- **Zero jog** (`nudgeZeroLeft/Right`, `Turret/NudgeZeroStepDeg`) shifts the *encoder reference* to
  correct drift while stowed. It's separate from `turretOffset` and also trims `homingIndexRad` so a
  later auto-home reproduces the corrected zero instead of undoing it.

---

## 3. Homing / drift correction (the big one)

### The sensor
- A **magnet** rides on the moving turret; a **digital hall-effect sensor** is fixed on the frame at
  home. It is wired into **roboRIO analog channel 3** (`HOMING_ANALOG_CHANNEL`) purely for wiring
  convenience — it is **not** an analog encoder. Output is binary: ~0 V or ~4.5 V.
- **It is ACTIVE-LOW.** Voltage drops to ~0 V when the magnet is over it (at home), ~4.5 V otherwise.
  `TurretHomingSensor.ACTIVE_HIGH = false`. **This bit us hard:** it was initially assumed active-high,
  so the index read "true" everywhere *except* home, and the auto-home fired at wrong positions and
  actively *created* drift. Always confirm polarity against a log: voltage LOW ⇔ near home.

### Why level-snapping alone is imprecise
The digital sensor only tells you "in band" or "out of band," and the detection **band is wider than
the turret's settling precision**. The turret can't hold 0 tightly (see stiction, §4), so it parks a
few degrees off *inside* the band. Snapping "you're at 0" from a stiction-offset settle **writes that
offset into the zero and walks the aim over a match.**

### Two homing paths (both active)
1. **Edge-midpoint homing (primary, precise) — `updateHomingEdges()`**
   When the turret *sweeps fully through* the sensor, capture the position at the **rising edge**
   (enter band) and **falling edge** (exit band). Their **midpoint = the magnet's true center**,
   independent of stiction and sweep direction. Shift the encoder frame so midpoint = `homingIndexRad`.
   Runs passively any time the turret slews across home — no routine needed. Guards: plausible band
   width (`EdgeBandMin/MaxDeg`), quick crossing (`EdgeMaxCrossSeconds`), capped correction
   (`EdgeMaxCorrectionDeg`).
   **Edge latency compensation (2026-07-05):** an edge is only observed one loop late — up to
   `velocity × loop-time` past the true transition, biased in the sweep direction. At overrun loop
   times (~43 ms) this produced alternating ±5–15° "drift corrections" that oscillated the zero
   (seen as `EdgeDriftDeg` flipping sign every crossing and `EdgeBandDeg` varying 1–20°). Each edge
   is now estimated as the midpoint of the previous and current loop's positions, which removes the
   directional bias to first order. If `EdgeDriftDeg` still alternates sign with large magnitude,
   suspect loop overrun first.
2. **Level-snap (fallback, ±3° guarded) — `updateHomingRezero()`**
   When the turret just *parks* at home without a full crossing: after `StowSettleSeconds` stowed +
   sensor triggered + nearly stationary + **already within `RezeroMaxErrorDeg` of home**, snap to
   `homingIndexRad`. The near-home guard is what stops stiction-offset settles from poisoning the zero.

### Manual homing
- `homeCommand()` / `Turret/Home` dashboard button: drive to stow, wait for the sensor, re-zero.
- `homingIndexRad` starts at `HOMING_INDEX_POSITION_RAD` (0 = magnet center is true forward). If the
  magnet is mounted a few degrees off, one `NudgeZero` at start sets it and all homing respects it.

### Useful log signals
`Turret/Homing/Voltage`, `Triggered`, `AtIndex`, `Present`, `RezeroCount`, `HomeErrorDeg`,
`EdgeHomeCount`, `EdgeDriftDeg` (real drift caught per crossing), `EdgeBandDeg` (measured band width).

---

## 4. The stiction deadband (root cause of several symptoms)

`kS = 1.90 V` is large. When holding near a target, the correcting voltage is `kP·error`; a small
error (e.g. 4°) produces ~0.3 V, **far below `kS`**, so the motor can't break static friction and the
turret parks a few degrees short. Consequences seen:
- ~−4° residual when returning to zero (and the "it doesn't return to exactly 0" question).
- ±5–7° wander around home while stowed → looked like drift, and poisoned level-snap homing.

It's approach-direction dependent (backlash adds to it). Not fully "fixable" without a bounded
integral term or mechanical friction reduction; **edge-midpoint homing sidesteps it** by measuring
home while moving. Leave it be unless it becomes the limiting factor.

---

## 5. Startup lunge (fixed)

On the first enabled loop the turret used to lunge then settle home. Cause: `goalAngle` defaults to
field 0°, and the stow default command only overwrites it *after* the first `periodic()`. So for one
loop it tried to point at absolute field zero → a large robot-relative swing. **Fix:** while
disabled/not-zeroed, `periodic()` pins `goalAngle = robotHeading` (hold home) and syncs
`lastGoalAngle` to the current position, so the first enabled setpoint is "hold."

---

## 6. PID / Motion Magic tuning journey

Tuned from AdvantageKit logs with ClaudeScope. Progression (all on the real robot):

| Symptom | Finding | Fix |
|---|---|---|
| 33% overshoot, ~2 s ring | Cruise (2.4) == free speed → no headroom + max momentum into target | Cruise → **1.8** |
| Overshoot unchanged | Mechanism ran ~30° **ahead** of the profile (`ClosedLoop/Error` ≈ −30°) → **kV too high** | kV 4.2 → **3.4** (measured `(V−kS)/vel` on a cruise plateau) |
| Small-step overshoot | Light turret blasted through the short profile | Accel 12 → **8**, kD 1.5 → **3.0** |
| Residual ~24° spike | Turret lunged ahead during the abrupt accel onset | **Jerk limit = 80** |

**Current Slot0 gains:** `kP=20, kI=0.002, kD=3.0, kS=1.90, kV=3.4, kA=0.15`
**Motion Magic:** cruise `1.8` rot/s, accel `8.0` rot/s², jerk `80` rot/s³.

Lessons:
- **Get feedforward right before touching kP/kD.** Tuning kP/kD against a wrong kV never converges —
  that's why pass 1 (cruise+kD) did nothing until kV was fixed.
- **kV is measurable:** on a constant-velocity segment, `kV ≈ (V − kS) / velocity`.
- Real free speed ≈ 3.0 rot/s (not the 2.4 the ratio implied). Aiming works, so the ratio is fine and
  the nameplate free-speed estimate was optimistic. A turret **SysId** routine would nail kV/kA/kS/ratio.
- The `SENSOR_TO_MECHANISM_RATIO` calibration flow (`Turret/RunTest` + `Turret/Test/MeasuredDeg` →
  `SuggestedRatio`) exists for validating the gear ratio.

---

## 7. Shoot-on-the-move (SOTM)

Two independent problems, both fixed:

### a) Tracking lag (the turret couldn't keep up)
Field-relative setpoint = `goalAngle − robotHeading`, so its required velocity is
`goalVelocity − robotYawRate`. The code fed only `goalVelocity` (and even that was disabled), so **when
the robot rotated, nothing told the turret to counter-rotate** → it lagged **49–111°** when slewing fast.

**Fix:** compute `turretVelFF = goalVelocity − robotYawRate` (yaw rate from
`drivetrain.getState().Speeds.omegaRadiansPerSecond`) and feed it forward. **Hybrid control:**
- error > `TrackingFeedforwardErrorDeg` (45°): **Motion Magic** (acquiring — keeps the smooth profile).
- error < threshold: **position + velocity feedforward** (tracking — slews *with* the target).

The threshold was 20° first, but that dropped out of feedforward into Motion Magic *during* fast
rotation (exactly when it's needed) → raised to 45°. Signals: `Turret/Tracking`, `TurretVelFF`,
`RobotYawRate`, `OnTargetErrorDeg`.

### b) Feed gate blocked feeding while moving
The feed required `isAtSpeed && isOnTarget` true **continuously for 0.12 s** (a rising debouncer).
While moving, both flicker, so the debouncer kept resetting and never latched → fed fine standing
still, never while moving.

**Fix: hysteretic feed latch (`SuperSystem.updateFeedReady()`)**
- **ARM** (`FeedSettleSeconds` 0.12 s): sustained on-target+at-speed needed to start (stops feeding
  mid-slew during a big turn).
- **HOLD** (`FeedHoldSeconds` 0.30 s): once feeding, tolerate brief flickers; drop only after the
  conditions are lost for the hold time. This keeps feeding through continuous tracking motion.

Computed once per loop in `SuperSystem.periodic()` so both feeders (roller floor + chute) share
identical timing. Signals: `SuperSystem/FeedRaw` (instant) vs `SuperSystem/FeedReady` (latched).

> Caveat: with feedforward tracking working, `isOnTarget` is meaningful again. If it still won't feed
> while spinning hard, check `OnTargetErrorDeg` — residual lag means bump the velocity-path effort,
> not loosen the gate.

### c) Cap robot yaw rate while shooting (so the turret *can* keep up)
Feedforward (§7a) only helps up to the turret's own top slew rate. The turret cruises at ~648°/s
(1.8 rot/s); the robot can yaw at 495°/s normally and ~990°/s in FAST mode — so at high yaw the required
turret rate physically exceeds what it can deliver, and no amount of feedforward wins.

**Fix:** while shooting, the drivetrain caps yaw rate hard — the same mechanism that already limits
translation. In `DriveConstants.teleopRequestUpdater`, when `shootingSpeedLimited` is set (engaged on the
right-trigger / right-bumper shoot in `ControlSubsystem`, *before* the shoot delay), rotation is scaled by
its **own** fraction, separate from and lower than the translation cap:
- `Drive/ShootingSpeedFraction` = 0.30 → translation = 0.30 × max linear speed.
- `Drive/ShootingRotationFraction` = 0.15 → yaw ≈ 0.15 × 495 ≈ **74°/s**, far inside the turret's ~648°/s.

Rotation gets a tighter cap than translation on purpose: yaw is the axis the turret can't out-slew, so it's
limited more. Raise `ShootingRotationFraction` toward 0.30 for more driver turn authority; lower it if aim
still lags. If lag persists even at a low fraction, it's not the yaw speed — it's the §7a 45° feedforward
dropout (spinning hard *before* the trigger, then shooting while already >45° off).

---

## 8. Neutral-zone flywheel spin-down

`ShooterSubsystem.runTrackTargetActiveShootingCommand()` spins the flywheel **down** while the robot
is in the central neutral zone (`FieldConstants.inNeutralZone(x)`; zone is `neutralZoneNear..Far`,
absolute field X, no alliance flip). Saves energy/wear when you can't score the hub from there.

Gotchas learned:
- The spin-down lives in the shooter's **default command**, which is *also* what spins the flywheel
  for a real shot. So it must be suppressed while shooting → `activelyShooting` flag (set by `Shoot()`
  via `beforeStarting`/`finallyDo`). Without it, shooting from the neutral zone "fought itself."
- **Passing is unaffected** — pass commands own the shooter directly and override the default.
- Pose jitter at the zone boundary chattered the flywheel on/off → **debounced** (`kBoth`,
  `NeutralZoneDebounceSec` 0.15 s).

---

## 9. Feed gate & turret readiness reference

`Shoot()` feeds only when `SuperSystem/FeedReady` (see §7b). `turret.isOnTarget()` = `|position −
lastClampedAngle| ≤ OnTargetToleranceDeg` and zeroed. Big overshoot naturally fails this during a
slew, so a ball isn't fed mid-turn.

---

## 10. Closed-loop tuning telemetry

For PID/Motion Magic tuning, the TalonFX closed-loop signals are logged under
`Turret Motor/ClosedLoop/*`: `Reference` (profiled target), `ReferenceSlope` (profiled velocity),
`Error`, `Output`, `Proportional`, `Derivative`, `FeedForward`. Published at **100 Hz** — opt-in per
motor via `MotorIOTalonFXConfig.closedLoopUpdateHz` (only the turret enables it, to avoid CAN load).
`MotorSubsystem.recordMotorTelemetry(name, io)` is the reusable per-motor logger (also used by
`ChuteSubsystem`, which holds MotorIOs directly instead of extending `MotorSubsystem`).

To read profile-following vs steady-state: `Reference` vs actual position. To validate feedforward:
`ReferenceSlope` vs `VelocityRPS`. The P/D/FF split shows which term is doing the work.

---

## 10b. Show mode: field lock and the 360° unwrap

Show mode has two opposite holds, and it is worth being clear which is which:

- **`runRobotRelativeHoldCommand`** — hold an angle *off the chassis*. The turret rides with the
  robot; the yaw-rate feedforward is zeroed because the robot-relative setpoint is constant.
- **`runFieldLockCommand`** (left bumper) — hold a *bearing in the room*. The turret counter-rotates
  as the driver spins the chassis, so it stays aimed at one spot. The bearing is captured once at
  button-down from `gyro yaw + turret mechanism angle` (plus `Show/FieldLockOffsetDeg`); nothing
  reads the pose estimator, so it works with no AprilTags. Gyro drift shows up as the aim slowly
  walking — irrelevant over a demo-length hold. The yaw-rate FF *does* apply here: the setpoint's
  true derivative is `-robotYawRate`, which is exactly the case §7a's feedforward was written for.

**The wrap.** Travel is ±180°, exactly one turn. Spin the chassis far enough one way and the
field-locked setpoint walks into a limit; the same bearing then exists one full turn the other way,
so the turret unwraps 360° and carries on. That is what keeps the cable alive when the robot is spun
in one direction all afternoon.

Because the window is exactly one turn, **only one wrap of a bearing is ever legal**, so the unwrap
can't be done early — it happens right at the boundary. The chatter guard is therefore in *time*,
not position (`fieldLockSetpoint`): a wrap is taken only if the setpoint is past the limit by more
than `Turret/FieldLock/WrapDeadbandDeg`, or if the last wrap was over
`Turret/FieldLock/WrapLockoutSec` ago. Otherwise it clamps to the stop — a few degrees of aim error
no spectator will notice — while `lastGoalAngle` keeps the *un-wrapped* branch, so a genuine
continued rotation still unwraps once instead of nudging back and forth. Watch
`Turret/FieldLock/WrapCount`: it should tick once per revolution of sustained spinning, never rapidly.

---

## 11. Tunables quick reference (current defaults)

**Control mode**
- `Turret/UseVelocityFeedforward` = 1.0 — 1 = velocity FF when tracking, 0 = always Motion Magic.
- `Turret/TrackingFeedforwardErrorDeg` = 45 — error below which we use velocity FF (else Motion Magic).

**Aim-while-moving (drivetrain, lives in `DriveConstants`)**
- `Drive/ShootingSpeedFraction` = 0.30 — translation cap while shooting.
- `Drive/ShootingRotationFraction` = 0.15 — yaw cap while shooting; lower than translation so the turret
  can keep up (see §7c). Raise for driver turn authority, lower if aim lags.

**Homing**
- `Turret/Homing/AutoRezeroEnabled` = 1.0
- `Turret/Homing/StowSettleSeconds` = 1.0 — settle time before level-snap rezero.
- `Turret/Homing/MaxVelRadPerSec` = 0.15 — max speed to allow level-snap.
- `Turret/Homing/RezeroMaxErrorDeg` = 3.0 — near-home guard for level-snap.
- `Turret/Homing/EdgeBandMinDeg` / `EdgeBandMaxDeg` = 1 / 30 — valid crossing band width.
- `Turret/Homing/EdgeMaxCrossSeconds` = 1.5 — max sweep duration for a valid crossing.
- `Turret/Homing/EdgeMaxCorrectionDeg` = 15 — cap on a single edge correction.
- `Turret/Homing/TriggerVolts` = 2.5 — digital high/low split (sensor is ~0/4.5 V).
- `Turret/Homing/DebounceSeconds` = 0.10
- `Turret/ZeroNudgeStepDeg` = 1.0 — manual zero-jog step.

**On target**
- `Turret/OnTargetToleranceDeg` = 10.5

**Show field lock (§10b)**
- `Show/FieldLockOffsetDeg` = 0 — bearing offset applied at capture; positive = left.
- `Turret/FieldLock/WrapDeadbandDeg` = 3.0 — overshoot past the limit needed to wrap during lockout.
- `Turret/FieldLock/WrapLockoutSec` = 1.0 — quiet period after a wrap before another is allowed.

**Feed (SuperSystem)**
- `SuperSystem/FeedSettleSeconds` = 0.12 — arm time.
- `SuperSystem/FeedHoldSeconds` = 0.30 — hold-through-flicker time.

**Neutral zone (Shooter)**
- `Shooter/OffInNeutralZone` = 1.0
- `Shooter/NeutralZoneDebounceSec` = 0.15

---

## 12. Debugging workflow (how these were found)

1. Enable closed-loop telemetry (§10) and drive the maneuver on the **real robot**.
2. Pull the `.wpilog` and analyze with **ClaudeScope** (`/scope`): load → `range`/`stats`/`find-bool`.
   AdvantageKit dedups unchanged values, so signals are event-based — reconstruct with zero-order hold.
3. Bucket errors by condition (e.g. turret error vs speed) — that's how the SOTM lag (49° when fast)
   and the homing polarity inversion (voltage-vs-angle correlation) were caught.
4. Change one thing, re-log, compare. Trust the data over assumptions (e.g. "active high" was wrong).
