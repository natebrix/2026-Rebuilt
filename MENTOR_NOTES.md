# Mentor Working Notes: 2026-Rebuilt Codebase

Working notes from an investigation of the Spartronics 4915 robot code, started 2026-09-25.
Purpose: get oriented in the codebase as a new programming mentor, record what has been
learned, and track concrete opportunities to contribute. Update as understanding deepens.

---

## 1. Codebase Overview

About 11,000 lines of Java under `src/main/java/com/spartronics4915/frc2026`. It is a
WPILib command-based robot built on CTRE Phoenix 6 swerve (`SwerveDrivetrain`), BLine and
PathPlanner path following, Phoenix 6 motor controllers, and a turreted ball shooter.
(The YAGSL vendordep and `src/main/deploy/swerve/` JSON are still in the repo but unused; see §2.)

**Design pattern.** Each mechanism is a small, state-holding subsystem that knows nothing
about the game. The intelligence lives in a few coordinator classes that read robot pose and
drive the mechanisms automatically.

### Entry and wiring
- `Main`, `Robot`, `RobotContainer`: standard WPILib scaffolding. `RobotContainer`
  instantiates every subsystem, the auto factories, and controller bindings (see README).
- `Constants.java` (~900 lines): all tuning values, motor IDs, PID gains, field geometry.

### Mechanisms (`subsystems/mechanisms`)
Each is a `SubsystemBase` with an enum of states or clamps, a SysId routine, and dashboard
buttons.

| Subsystem | Role | Control type |
|---|---|---|
| `IntakeSubsystem` | Roller intake | Velocity (torque-current) |
| `PivotSubsystem` | Arm that swings the intake out and back | Position (torque-current) |
| `IndexerSubsystem` | Spindexer, first stage of ball path | Velocity (torque-current) |
| `FeederSubsystem` | Feeds balls into the shooter | Velocity (torque-current) |
| `ShooterSubsystem` | Flywheel, lead + follower motor | Velocity (voltage) |
| `TurretSubsystem` | Rotates the shooter head; publishes timestamped angle to vision | Position (torque-current) |
| `HoodSubsystem` | Sets shot pitch | Position (torque-current) |
| `ClimberSubsystem` | Present but commented out, marked `!CLIMBER!` | Position (voltage) |

### Swerve (`subsystems/swerve`)
- `SwerveSubsystem` wraps CTRE Phoenix 6 `SwerveDrivetrain` (not YAGSL, despite the
  vendordep). Module config is Java: the `SwerveConfigurations` enum and `compChassisFactory()`
  in `Constants.java`. Owns the pose estimator. Path following uses BLine `FollowPath`.
  The YAGSL JSON under `src/main/deploy/swerve/` is never loaded (dead config).
- `util/swerve/SlipDetector` flags wheel slip or collisions by comparing commanded and
  measured module velocities.

### Vision (`subsystems/vision`)
The most layered part of the code.
- Two camera backends, `LimelightProcessor` and `PhotonProcessor`, behind a common
  `ProcessorInterface`.
- Results pass through `filters/`, get weighted by `StdDevCalculator`, and are fused by
  `PoseFusionEngine` (written for zero allocation in the periodic loop) into one AprilTag
  pose estimate that feeds the swerve pose estimator.

### Control layer (`subsystems/control`) — the interesting part
- `AutoAimController` runs every loop tick and calls the `AutoAim` solver
  (`util/control/AutoAim.java`). `TurretController` converts the solver's field-relative yaw
  into a turret setpoint with wrap handling and hysteresis.
- `Superstructure` divides the field into zones (alliance zone, trench, bump, tower, neutral
  zone, opponent zone) via `FieldZoneMap`. It uses WPILib Triggers to schedule
  `SuperstructureCommands` automatically as the robot crosses zones, for example lowering
  the hood before entering the trench based on velocity-projected position. Also reads a
  LaserCAN sensor for ball detection.

### Autonomous (`autos`)
Composable segments rather than fixed routines. Factories: `ZoneTransition`, `DriveToPOI`,
`NeutralZoneAutos`, `PreAlignment`. `ComplexAutoChooser` exposes them on the Elastic
dashboard as a chain of steps where each step constrains valid next steps. `Autos` has a
survey mode that renders the planned path on a Field2d for verification.

### Utilities (`util`)
Per-mode speed limiting, a mode-switch handler so subsystems reset on enable, `BumpSim`
for simulating the field bump, `TimeVarianceAuthority`, and a large vendored
`LimelightHelpers`.

**Best files to read first:** `AutoAim.java` (ballistics) and `Superstructure.java`
(zone-driven automation).

---

## 2. PID Calibration

### The concept
PID is the feedback loop that makes a mechanism hold a target. Each tick, error = setpoint
minus measurement, and output is a weighted sum:
- **P**: push harder the further from target. Too high overshoots and oscillates.
- **I**: accumulate error to remove persistent offset. Usually zero in FRC (windup risk).
- **D**: react to rate of change of error; damps oscillation.

**Feedforward** predicts the needed output from a physics model so PID only cleans up
residuals:

```
output = kS * sign(v) + kV * v + kA * a  (+ kG for gravity on arms)
```

**Calibration** means finding these constants for the physical mechanism:
1. Manual tuning: raise P until oscillation, back off, add D.
2. System identification (SysId): drive with known profiles, log voltage/position/velocity/
   acceleration, fit kS/kV/kA by regression in the WPILib SysId desktop tool, which also
   suggests PID gains. Two test types per direction:
   - **Quasistatic**: slow voltage ramp, isolates kS and kV.
   - **Dynamic**: voltage step, isolates kA.

### How it appears in this codebase
- **Gains are hard-coded, not learned at runtime.** Each mechanism has a block in
  `Constants.java`, e.g. `ShooterConstants` with P=0.46, V=0.115, S=0.22, A=30000
  (the A value looks like a placeholder or unit mismatch; worth asking about).
- **Loops run on the motor controller.** Gains are Phoenix 6 `SlotConfigs` applied to
  TalonFX motors in each subsystem constructor. The roboRIO only sends setpoints; the
  motor firmware runs PID + feedforward at 1 kHz.
- **Swerve gains live in one place: `compChassisFactory()` in `Constants.java`** (steer
  P=110, D=5, kS=0.1, kV=2.49, voltage output; drive P=9, kS=1, kV=0.124, *TorqueCurrentFOC*
  output, so drive gains are in amps). These are Phoenix 6 `Slot0Configs` passed to CTRE's
  `SwerveModuleConstantsFactory`. The YAGSL `pidfproperties.json` files are dead (resolved
  2026-10-01).
- **Chassis pose loops on the roboRIO**: `translationPID`, `rotationPID`, `crossTrackPID`
  in `Constants.java` are WPILib `PIDController`s correcting pose error during PathPlanner
  following. Their output becomes velocity setpoints for the swerve loops.
- **SysId is fully wired.** Seven mechanisms each construct a `SysIdRoutine` and publish
  four dashboard buttons (Quasistatic/Dynamic × Forward/Reverse). An `isCharacterizing`
  flag suppresses the normal `periodic` loop during a test.
- **Unit caveat.** Mechanisms using `*TorqueCurrentFOC` control requests have gains in
  amps, not volts. Their P/kS/kV constants are not comparable to the shooter's, which uses
  `VelocityVoltage`. The shooter's SysId routine drives with `TorqueCurrentFOC` and logs
  torque current in the voltage field, so its fitted constants would be in amps. **Confirmed
  inconsistent (2026-10-01):** runtime is `VelocityVoltage` (gains in volts), SysId drives
  `TorqueCurrentFOC` with a 4 "V" step that is really 4 A. A SysId fit from this routine
  cannot be pasted into `ShooterConstants` as-is.
- **Shooter `A = 30000` is currently inert.** The code only calls
  `velocityVoltage.withVelocity(...)`, never `withAcceleration`, so the requested
  acceleration is 0 and kA contributes nothing. It becomes a trap the moment someone adds an
  acceleration feedforward (30000 V per rps² would saturate instantly). The setpoint is
  shaped instead by a `SlewRateLimiter` (decel limit only, `maxShooterDecel = -12` rps/s).

### The implied calibration workflow
1. Deploy code, open Elastic.
2. Press the four SysId buttons for a mechanism; data logs to a WPILog on the roboRIO.
3. Pull the log, load in the WPILib SysId tool, fit constants.
4. Enter results in `Constants.java`, redeploy, hand-tune P and D.

### What is being calibrated, and why it matters for shooting
Position loops: turret, hood, intake pivot, climber, four swerve steer motors.
Velocity loops: shooter flywheel, four swerve drive motors, intake rollers, indexer, feeder.
Pose loops: translation, rotation, cross-track for path following.

For shooting accuracy, four loops matter and accuracy is limited by the weakest:
1. Turret position (point the right way)
2. Hood position (right pitch)
3. Shooter velocity (right launch speed, fast recovery after each ball)
4. Swerve odometry feeding the solver, which depends on drive and steer loops tracking well

---

## 3. The AutoAim Solver

`util/control/AutoAim.java` solves an inverse ballistics problem: given robot state and a
target point, find launch yaw, pitch, and speed so a projectile from a moving turret lands on
target. Structure:

**Inner problem: static aim, closed form.** With a stationary robot, launch pitch θ at speed
v, horizontal distance x, height h satisfies a quadratic in tan θ:

```
(g x² / 2v²) tan²θ − x tanθ + (h + g x² / 2v²) = 0
```

Zero, one, or two roots (low and high arcs). Each is checked against hood angle bounds and
two collision maps (predicates over ground-frame pitch and speed encoding whether the shot
clears the hub; one padded 5 cm, one 20 cm). Among feasible roots the flattest arc is chosen.

**Grid search for recommended speed.** Independently, sweep pitch from 50° to 90° in 50
steps, compute minimum speed to hit target at each, take the first feasible. Provides a
flywheel speed setpoint, and a fallback when the quadratic has no feasible root at current
speed (flagged `requiresIdealSpeed`).

**Outer problem: moving robot, fixed-point iteration.** Aim at a virtual target displaced by
predicted robot motion plus projectile drift, resolve, recompute displacement with new time
of flight, repeat until change < 1 mm or 20 iterations.

*Why fixed-point iteration is adequate:* the map d → F(d) is a contraction with Lipschitz
constant L ≈ |v_robot| × |dT/dd|. Time of flight changes only mildly when the target shifts a
few centimeters over a several-meter shot, so L is well under 0.5 at FRC speeds. Error halves
or better each step; convergence in a handful of iterations. Newton's quadratic convergence
is unnecessary. (Would diverge only with L > 1, e.g. absurd robot speeds or 30-second shots.)

**Lookahead for derivative terms.** Solve once more at a slightly later predicted state
(20 ms) and finite-difference yaw and pitch to get angular velocity setpoints for the turret
and hood. This is where the solver output meets the PID feedforward.

**Pipeline per tick:** odometry + vision → pose/velocity → `AutoAimController` → solver →
yaw, pitch, speed and rates → turret/hood/shooter loops. The solver decides where to point;
PID makes the hardware get there.

**Character of the problem.** This is a feasibility problem with heuristic selection rules
("flattest feasible arc", "first feasible pitch in sweep"), not an optimization. No explicit
objective. Reasonable given the 20 ms compute budget.

---

## 4. Opportunities to Contribute

Ordered roughly by value. Items 1 through 6 are solver improvements; the rest are broader.

### Solver improvements
1. **Choose the arc that is most robust, not the flattest.** The flat arc has the lowest time
   of flight but enters the hub at the shallowest angle, so it is most sensitive to speed
   error. ∂x/∂v has a closed form for a parabola. Choose the arc minimizing landing
   sensitivity to flywheel error, weighted by measured flywheel variance. One line of algebra,
   and it directly connects PID calibration quality to the aiming decision. The data to
   justify it is already logged by the shooter's velocity and setpoint publishers.
   **This is the one to pitch to the team first.**
2. **Replace the pitch sweep with root finding.** The minimum-speed pitch has a closed form;
   if the collision constraint makes it infeasible, bisect on the constraint boundary. Exact
   answers instead of 0.8° granularity, and cheaper than 50 evaluations.
3. **Let flywheel speed float toward the recommended value.** Today yaw/pitch are solved for
   the current speed and the recommended speed is only a fallback. With hood and speed as
   two degrees of freedom for one target there is a one-parameter family of solutions.
   Choosing speed to minimize recovery time between shots, or to keep the hood mid-range,
   is a small problem that reduces to picking a point on a curve.
4. **Warm-start the fixed-point iteration** from the previous tick's displacement. The
   controller runs at 50 Hz and d* barely changes between ticks. Trivial, pure compute win.
5. **Damp the lookahead derivative.** yawOmega and pitchOmega come from a finite difference
   over 20 ms on a chain that includes noisy pose and velocity estimates. A first-order filter
   or longer horizon would smooth feedforward without meaningful lag.
6. **Signed-distance collision margin.** Replace two boolean predicates with a single signed
   distance function and prefer solutions with larger clearance. Turns a binary check into a
   graded quantity that can be traded off against other criteria.

### Calibration and consistency
7. **Remove dead YAGSL config.** Swerve is CTRE `SwerveDrivetrain`; the YAGSL vendordep and
   `deploy/swerve/*.json` are unused and mislead readers (they misled me). Small, safe
   cleanup PR, good first contribution.
8. **Fix shooter SysId units / zero out `A`.** Either make SysId drive `VoltageOut` and log
   motor voltage (matches the `VelocityVoltage` runtime), or switch runtime to
   `VelocityTorqueCurrentFOC` like the other mechanisms. Set kA to 0 or a fitted value.
9. **Document the calibration workflow** for students: which buttons, where logs land, how to
   use the SysId tool, where constants go.

### Mentoring angle
- The solver is a natural teaching vehicle for students: projectile physics, quadratic roots,
  fixed-point iteration, and the idea of an objective vs. a feasibility check.
- PID/SysId calibration is a repeatable, hands-on process students can own each season.
- Logged shooter data offers a real dataset for a small analysis project (flywheel variance,
  recovery time between shots).

---

## 5. Open Questions

- ~~Which swerve config path is active?~~ CTRE `SwerveDrivetrain`, gains in `Constants.java`.
- ~~Is the shooter's runtime control request consistent with its SysId units?~~ No (§2).
- ~~Do the other SysId routines have the volts/amps mislabeling?~~ All drive `TorqueCurrentFOC`
  and log amps in the volts field, but every other mechanism also *runs* torque-current, so
  units are consistent. The shooter is the only mismatch.
- Which do autos use, BLine `FollowPath` or PathPlanner, and when?
- What is the actual convergence count of the fixed-point loop in match logs? (Would confirm
  the contraction argument and justify the warm-start change.)
- How is the collision map derived? Empirical or geometric?
- What is the measured flywheel speed variance at shot time? Needed for item 1.

---

## 7. Reading Plan and Walkthroughs

**Order:** WPILib command-based docs → `Robot` → `RobotContainer` → `IntakeSubsystem` +
`MotorHelpers` → `PivotSubsystem` → `SuperstructureCommands`/`Superstructure`/`FieldZoneMap` →
`DriveCommand`/`SwerveSubsystem` → aim stack → vision → autos. `Constants` as reference only;
skip vendored `LimelightHelpers`. Desktop sim works (`./gradlew simulateJava`). No `src/test`
exists; `AutoAim` is pure math and the ideal first place for JUnit tests.

### Robot.java (walked 2026-10-02)
- `TimedRobot` calls mode hooks every 20 ms; mode `*Periodic` runs before `robotPeriodic`.
- Line 98 `CommandScheduler.run()` is the heartbeat for everything.
- `robotInit` starts `.wpilog` logging (real robot only) and publishes git SHA/branch/dirty
  metadata, so every log joins to a commit.
- `teleopPeriodic` encodes the 2026 hub-shift schedule from FMS game data into static
  `Robot.hubEnabled` / `Robot.timeUntilSwitch`. `AutoAimController.shouldAutoShoot` (line 360)
  pre-fires when `timeUntilSwitch < ToF`.
- Questions: (a) shoot condition is asymmetric: does not stop when hub is about to turn
  *off* (grace period in rules?); (b) statics are global mutable state, a pure `MatchState`
  function would be testable; (c) non-R/B game data makes `^=` flip every tick on Blue;
  (d) `configureStandardDevsForDisabled()` is never called.

### RobotContainer.java (walked 2026-10-02)
- Composition root: field initializers build subsystems in declaration order, wiring by
  constructor args and method-reference callbacks (`swerveSubsystem::addVisionMeasurement`,
  turret → turreted-camera angle observer, feeder ← distance supplier).
- Three Xbox controllers: driver, operator, debug (debug mostly duplicates the other two).
- Bindings vocabulary: `onTrue`/`onFalse` (edge), `whileTrue` (level, cancel on release),
  `Commands.run` (every tick) vs `runOnce`, `parallel`, `defer`, `setDefaultCommand`.
- Driver bumpers lock robot Y to a trench line (`hubPose ± trenchTransform`) via
  `setMovementOverride`; 0.0 is a sentinel for "off".
- Findings: (a) driver POV up/down nudges (lines 170-184) omit the `swerveSubsystem`
  requirement that left/right and all debug nudges have, so `DriveCommand` keeps running
  alongside them; works only because the nudge executes later in the tick. One-line fix each.
  (b) Both operator stick clicks call `pivotSubsystem.resetMechanism(0)`, which jumps the
  pivot setpoint and profile state to 0; stick clicks are easy to hit by accident.
  (c) `getAutonomousCommand` defers with requirements `{swerve}` only.

### IntakeSubsystem + MotorHelpers (walked 2026-10-02): the mechanism template
- Pattern is a reconciliation loop: commands are instant setters (`this.runOnce(...)`, which
  also claims the subsystem); the desired setpoint lives in a field; `periodic()` clamps it and
  pushes it to the motor every tick. State lives in the subsystem, not in a running command.
- Every mechanism has: a `LoggedTalonFX`, a control request object, a SysId routine plus four
  dashboard buttons with an `isCharacterizing` flag, NetworkTables publishers, a state enum,
  `setStateCommand`, and `onModeSwitch()` that turns it off on every enable/mode change
  (`ModeSwitchHandler`, built on `RobotModeTriggers`).
- `LoggedTalonFX` is a `Sendable`: `SmartDashboard.putData("Intake Motor", motor)` exposes live
  editable P/I/D/V/A/S, profile limits and a "goal" setpoint on Elastic. Live tuning without
  redeploy; edits are not saved, so they must be copied back to `Constants`.
- Intake `A = 75.05` is inert (only `.Velocity` is set; requested acceleration is 0), same as
  the shooter. Intake has no kS. Supply current limit disabled (`CURRENT_LIMIT_ENABLE = false`).
- `new VoltageOut(0.0)` allocated each tick when off (shooter preallocates; trivial).

### Friction / kS discussion (2026-10-02 to 10-04)
- Under torque-current control kS/kV model friction: steady current needed = s*sign(v) + c*v.
  Missing kS leaves error = shortfall / kP.
- **Single-speed velocity mechanisms (intake, indexer, feeder): kS = 0 is practically
  irrelevant.** At one operating point kV absorbs kS; kP covers the rest; spin-up is
  current-saturated; OFF is coast (`VoltageOut(0)`), not a zero-velocity hold.
- **Indexer defines `S = 1.53135` but `PID_CONFIG` never calls `.withKS(S)`.** Oversight;
  negligible effect, easy one-line fix.
- kS by mechanism: hood 37.5 A (applied), shooter 0.22 V, turret/pivot/intake/feeder none.
- **Turret (kP 3010 A/rot = 8.4 A/deg):** stuck/lag error ~ s/kP, e.g. 5 A -> 0.6 deg -> ~5 cm
  at 5 m. Firing gate `turretTolerance = 7.5 deg` (AutoAimController.isTurretReady) is far
  looser, so kS affects accuracy, not whether it fires. Data check: setpoint - position while
  tracking; plateau whose sign follows rotation direction = s/kP. Also ask why 7.5 deg.
- Fitting exercise (teaching): coast-down regression a = -(s/J) - (c/J) v from logged rps;
  steady-state current vs speed needs torque-current publisher and multiple speeds
  (±22 only is unidentifiable: one equation s + 22c).

### Indexer
- Ball path: intake -> indexer ("spindexer", hopper) -> feeder -> shooter. Indexer + feeder
  form the "pipeline" (`SuperstructureCommands.PipelineState`), turned on by
  `Trigger(controller::isReadyToShoot)` in `Superstructure` and gated by `isShooterReady`.
- Trap: `setPipelineState` mutates `currentPipelineState` at command *build* time.

### PivotSubsystem (walked 2026-10-04)
- Position mechanism: absolute CANcoder on arm axle fused with rotor (`FusedCANcoder`), seeded
  at boot so no homing. States READY -0.1 deg (deployed), SAFE 60, STOW 130; limits -2..132.
- Trapezoid profile computed on roboRIO each tick (`dtCalc` = measured dt), only `Position`
  sent to the Talon (profile velocity discarded). No kS, no kG, no kV.
- `onModeSwitch()` resets setpoint and profile to current position, so the arm holds where it
  is on enable (profile keeps running while disabled).
- Findings: (a) profile constraints 100 rot/s, 100 rot/s^2 in *mechanism* rotations: full
  134 deg travel takes ~0.12 s with a 6 rot/s peak, ~3x what a 50.6:1 Kraken can do
  (~2 rot/s). Profile is effectively a step; likely meant deg/s or never tuned.
  (b) `MAGNET_OFFSET = 1.283203` is outside Phoenix's documented [-1, 1) rotation range;
  check in Tuner X what the CANcoder actually reports (apply status unchecked).
  (c) `deltaSetpoint` (operator D-pad) bypasses the angle clamp and `periodic` never
  re-clamps, so holding D-pad can park the setpoint past a hard stop (stall current).
  (d) No gravity feedforward; Phoenix `GravityType.Arm_Cosine` + kG is the standard fix
  (needs 0 = horizontal, which READY ~ 0 suggests).

### Superstructure, SuperstructureCommands, FieldZoneMap (walked 2026-10-04)
- Two layers: `SuperstructureCommands` = vocabulary (pure factory of compound commands);
  `Superstructure` = when (zone -> command via Triggers). `FieldZoneMap` = ordered list of
  predicates, first match wins (TRENCH, BUMP, TOWER, NEUTRAL, ALLIANCE; default OPPONENT),
  evaluated at the *turret's* field position from the smoothed pose.
- Trench lookahead: in zone if current pose, pose + 0.5 s velocity, or the segment between
  them crosses the trench centerline. Lowers hood before the 32 in trench.
- Zone commands differ only in clamps and auto-shoot: hood RESTRICTED (0 deg) in trench;
  shooter RESTRICTED (35 rps) in tower/idle/stowed; auto-shoot on only in alliance zone.
- `conditional*` helpers no-op outside autonomous: in auto, zones drive pivot/intake/aim/shoot;
  in teleop, zones only set hardware clamps.
- Interlock: turret UNRESTRICTED only after `isPivotSafe` (pivot <= 100 deg, i.e. not stowed);
  `stowed()` restricts turret, waits until turret within +/-10 deg, then stows pivot.
- `getReturnToZoneCommand` defers with empty requirements so it can run alongside
  `intakeOn()` (operator left trigger).
- Pose sanity: if pose x < 0 or y outside 0..8.1, reset to vision pose (null-safe). No upper x.
- Findings: (a) **Zone.TOWER has no trigger**, and `getReturnToZoneCommand` maps it to
  `idle()`, which sets intake OFF after the pivot deploys and restricts shooter to 35 rps.
  Operator left trigger in the tower rectangle (~1 m x 1 m between wall and tower) likely
  starts then kills the intake. Verify in sim. (b) `tower()`, `climb()` unused.
  (c) LaserCAN ball detection computed and published but `isBallDetectedDebounced()` has no
  callers; invalid reading counts as "ball present". (d) Five near-identical zone commands:
  refactor to one builder parameterized by (hoodClamp, shooterClamp, autoShoot).

### DriveCommand + SwerveSubsystem (walked 2026-10-06)
- DriveCommand (default command on swerve): stick -> deadband (0.0!) -> |x|^1.5 curve -> scale
  (maxSpeed 9.12 m/s, 8 rad/s) -> field-centric request. Bumper "trench override" replaces vY
  with a trapezoid-profiled Y target tracked by a PathPlanner holonomic P controller.
- Effective behavior is plain field-centric driving: speed-limit modes (HUB/FERRY) commented
  out, heading lock disabled (`driverIsRotating = true`), align trigger hard-coded false.
- SwerveSubsystem wraps CTRE `SwerveDrivetrain`, which owns the pose estimator (odometry at
  120 Hz on CTRE's own thread + Pigeon + latency-compensated vision via
  `addVisionMeasurement`). `updateOdometry` runs on that thread (hence AtomicBoolean,
  ConcurrentTimeBuffer, volatile).
- Slip handling: SlipDetector compares module measured vs target speed; while slipping,
  odometry std devs go 0.05 -> 2.0 so vision dominates (inverse-variance weighting).
- `getRelativePose()`: all game logic works in blue-alliance coordinates, flipped for red.
- Watchdog: if no drive call for 0.1 s, command zero speeds.
- Findings: (a) **trench override dt bug**: `dtCalc.update()` only runs while overriding, so
  the first tick of each engagement gets dt = time since last override (seconds to minutes);
  profile jumps straight to the goal, controller sees full Y error (P=2 m/s per m): lateral
  lurch. Fix: reset dtCalc when engaging. Profile velocity also discarded (fieldSpeeds = 0).
  (b) **`MovingAveragePose(1.60)` is clamped to alpha = 1.0, i.e. no smoothing**; the
  "smoothed" pose used by AutoAim and zones is the raw pose. Comment says previously 0.30;
  maybe deliberate, but misleading. (c) `stickDeadband = 0.0`: stick rest offset -> creep.
  (d) maxSpeed 9.12 vs `SpeedAt12Volts` 4.32: with the 1.5 curve, top speed reached at ~61%
  stick. (e) Debug controller takes precedence over driver whenever connected.
  (f) `tiltThresholdDegrees = 1.0` gates auto-shoot (isFlat); very tight.

### Glossary and clarifications (2026-10-08)
- **CTRE**: Cross The Road Electronics, FRC hardware vendor (Talon FX controllers inside Kraken
  motors, CANcoder, Pigeon 2, CANivore). **Phoenix 6** is their Java SDK, incl. swerve.
- **Holonomic**: drivetrain that controls x, y and heading independently (swerve); a
  holonomic controller runs separate feedback on each and outputs chassis speeds.
- **Pigeon 2**: CTRE IMU (gyro + accelerometer); yaw = heading, pitch/roll feed `isFlat`.
- **Pose estimation**: wheel odometry (module distances + angles -> kinematics) with gyro heading,
  integrated at 120 Hz, drifts. Vision (AprilTags) = absolute, noisy, delayed. Fusion follows
  WPILib's estimator: per-axis fixed gain from state vs vision std devs, applied at the camera
  timestamp and replayed forward. Example: state 0.05, vision 0.5 -> ~9% pull per frame;
  in slip (state 2.0) -> ~80%.
- **Trench override bug**, clarified: triggered by the driver *pressing a bumper*, not by entering
  the trench zone. Profile is bypassed on every press (unless re-pressed within a fraction of a
  second), so the robot gets a step velocity command of 2 m/s per metre of offset instead of a
  3 m/s^2 ramp. Feel/jolt issue, possible wheel slip; not a safety hazard.

---

## 6. Log

- **2026-09-25**: Cloned repo (`stable` branch, HEAD 3977202). Surveyed source tree, mapped
  subsystems, studied PID/SysId setup and the AutoAim solver. Identified solver improvement
  opportunities. Created this file.
- **2026-10-01**: Moved to a cloud session (same commit 3977202). Corrected the swerve stack
  (CTRE, not YAGSL). Resolved two open questions: swerve gain source, shooter SysId units.
  Found shooter kA is inert.
- **2026-10-02**: Walked `Robot.java` and `RobotContainer.java` (§7).
- **2026-10-02**: Walked `IntakeSubsystem` and `MotorHelpers`. Resolved SysId units question.
- **2026-10-04**: kS discussion, indexer, turret friction analysis, walked `PivotSubsystem`.
- **2026-10-04**: Walked Superstructure layer; found TOWER-zone return-to-zone issue.
- **2026-10-06**: Walked DriveCommand and SwerveSubsystem.
