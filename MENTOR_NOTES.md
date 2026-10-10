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

## 4. Opportunities to Contribute (re-prioritized 2026-10-10)

Ranked by value x confidence / effort, after reading the full stack except vision and autos.
Criteria: can it be verified (sim or logs)? does it teach? does it need the team's intent first?

### Tier 1: Small verified fixes (first PRs, pair with a student; verify in sim)
1. **Tower zone kills the intake.** `getReturnToZoneCommand` maps TOWER to `idle()`, which
   turns the intake off (and limits the shooter to 35 rps) while the operator holds intake.
   Fix: map TOWER like its surrounding zone. Sim: drive to tower, hold LT, watch intake setpoint.
2. **Trench override profile bypassed.** `dtCalc` not reset on engage; robot lurches sideways
   on bumper press. One-line fix (+ pass profile velocity). Sim: watch `swerve/FieldRelativeSpeeds`.
3. **Driver POV up/down nudges missing `swerveSubsystem` requirement.** Two one-line fixes;
   good teaching example for requirements.
4. **Housekeeping bundle:** indexer `.withKS(S)` missing; `checkHubCollision` duplicates the
   shooter height literal; pivot `deltaSetpoint` can push past the hard stop (re-clamp in
   `periodic`); `setPipelineState` mutates state at build time; misleading pipeline
   "debounced" comment; remove dead YAGSL vendordep + `deploy/swerve/*.json`.

### Tier 2: Questions to ask before changing anything (intent unknown)
- Is pose smoothing meant to be off? (`MovingAveragePose(1.60)` clamps to alpha 1.0.)
- Why `turretTolerance = 7.5 deg`? Is turret lag while moving the reason?
- 1 deg flatness gate on auto-shoot: margin on flat ground?
- Passes keep feeding down to 30% flywheel speed: intended?
- Pivot profile 100 rot/s, 100 rot/s^2: units mix-up or deliberate?
- `stickDeadband = 0`; `maxSpeed` 9.12 vs 4.32 configured: driver preference?
- Why were heading lock, speed limits, align trigger disabled in DriveCommand?
- Debug controller overrides driver when plugged in: competition procedure?
- Pivot `MAGNET_OFFSET = 1.28` outside [-1, 1): what does Tuner X show?
- Shoot gate doesn't stop when hub is about to turn *off*: rules grace period?
- `configureStandardDevsForDisabled()` never called: dropped on purpose?
- Intake supply current limit disabled; kA values (shooter 30000, intake 75) inert.
- Both operator stick clicks snap pivot to 0.
- Photon tag area passed as percent to a calculator expecting fraction: were base std devs
  tuned with this in place? Should vision be gated on the 1 deg flatness check?

### Tier 3: Infrastructure (highest leverage; mentor-shaped)
5. **Unit tests** (none exist). Add JUnit + `src/test`; start with pure functions:
   `AutoAim` (shots land on target, convergence), `TurretController` (path choice, wrap),
   `FieldZoneMap` zones (table-driven), and an extracted `MatchState` (hub shift schedule
   from Robot.teleopPeriodic). Makes every later change safer.
6. **Log analysis toolkit.** Python `.wpilog` reader (robotpy-wpiutil) + notebook template.
   Add missing publishers first: turret setpoint, intake/turret torque current.
7. **Calibration runbook.** SysId buttons -> log -> SysId tool -> Constants; dashboard live
   tuning and the copy-back step; units (volts vs amps) per mechanism.

### Tier 4: Data-driven accuracy (season-long student projects; need Tier 3)
8. **Exit-speed calibration** `v_ball(rps)` (currently `rps*pi*1.92in*(1-0.107)`). Single biggest
   accuracy lever: every pitch the solver computes depends on it. Needs CAD wheel diameter.
9. **Turret friction / tracking error** from logs (s/kP plateau); add kS if > ~0.5 deg.
10. **Flywheel sag at ball exit** (raw vs median-filtered rps at shot time). Prerequisite for 11.
11. **Robust arc selection** (former #1): choose arc minimizing landing error under measured
    speed variance. Now sequenced after 8 and 10.
12a. **Vision std-dev model calibration.** Fix area units, then fit empirical error vs
    distance / tag count from logs (stationary at known poses, or vs odometry at low speed).
12. **Pivot gravity feedforward** (`Arm_Cosine` + kG via SysId). Needs CAD to confirm 0 = horizontal.

### Tier 5: Solver and code-quality improvements (after tests exist)
13. Replace pitch sweep with root finding (former #2); warm-start fixed point (former #4);
    damp lookahead derivatives (former #5); signed-distance collision margin (former #6).
    Former #3 (float speed) is half-done: hood already adapts to measured speed.
14. Refactors: one zone-command builder instead of five copies; `Optional`/feasibility flag
    instead of -1/null sentinels; fix pivot/trench profiles discarding velocity.

### Mentoring angle
- Tier 1 items are ideal "first PR with a student" exercises: small, explainable, verifiable.
- Tier 3-4 match my background: tests, logging, regression, experimental design
  (identifiability, observational vs designed data).
- Lead with questions (Tier 2) before proposing changes: the team knows the robot's history.

## 5. Open Questions

- ~~Which swerve config path is active?~~ CTRE `SwerveDrivetrain`, gains in `Constants.java`.
- ~~Is the shooter's runtime control request consistent with its SysId units?~~ No (§2).
- ~~Do the other SysId routines have the volts/amps mislabeling?~~ All drive `TorqueCurrentFOC`
  and log amps in the volts field, but every other mechanism also *runs* torque-current, so
  units are consistent. The shooter is the only mismatch.
- Which do autos use, BLine `FollowPath` or PathPlanner, and when?
- What is the actual convergence count of the fixed-point loop in match logs? (Would confirm
  the contraction argument and justify the warm-start change.)
- ~~How is the collision map derived?~~ Geometric: square hub, rim height + padding (see §7).
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

### Aiming stack (walked 2026-10-10): AutoAimController, TurretController, turret/hood/shooter
- Per tick (`AutoAimController.periodic`): finite-difference field accel (median-5 filtered) ->
  collision cache -> solver (`calculateDynamicAim`) with pose, speeds, accels, target, and
  *measured* flywheel speed (median-10 filtered, ~100 ms lag) -> apply.
- Speed vs pitch split: shooter setpoint = solver's recommendedShotSpeed (grid search);
  hood pitch = solved for current measured speed, so hood compensates during spin-up.
- Flywheel idles at 30 rps when setpoint 0; spins up only once firing conditions hold; decel
  slew-limited to 12 rps/s so it stays fast between shots.
- When not firing: hood flat, turret pre-aims at hub (solve with speed 0, no ToF compensation).
- Firing chain: meetsFiringConditions (valid speed, auto-shoot on, shouldAutoShoot: hub
  active or about to be, robot behind hub line, flat; or operator override; shooter
  UNRESTRICTED) -> isReadyToShoot (+ hood within 4 deg, turret within 7.5 deg, not wrapping,
  solution feasible at current speed; debounced falling 0.2 s) -> pipeline ON (waits for
  flywheel within 4 rps) -> keep feeding while flywheel >= 90% (manual) / 30% (passes).
- Targets: hub funnel bottom when behind hub line; else left/right pass point (z = 0), with
  robot velocity scaled by 0.5 (or 0 deep in opponent zone) and 1.05x speed.
- **Collision map answered: geometric.** Hub = 47 in square; distance from turret to near wall
  along line of sight; projectile height there vs rim + padding (5 cm / 20 cm). No drag.
- Exit speed model: v = rps * pi * 1.92 in * (1 - 0.10695). Linear, two fudge factors; a prime
  calibration target.
- TurretController: target yaw -> robot-relative -> two candidate paths (shortest and +/-360)
  within -180..230 deg; prefer path near last setpoint; in auto bias toward +/-90 deg to avoid
  later wraps; `isWrapping` (move > 180 deg) blocks firing.
- Findings: (a) solver gets flywheel speed through a 10-sample median filter: ignores sag at
  ball exit (ties to robust-arc idea #1). (b) shooterZ literal duplicated in
  `checkHubCollision` instead of `turretTranslation3D.getZ()`. (c) passes keep feeding down
  to 30% flywheel speed (`ferryingShooterLeniency = 0.7`): intended? (d) disabling aim
  mid-shot leaves shooter setpoint at last value. (e) sentinels everywhere (ToF -1, speed -1,
  null pitch). (f) Uses raw pose (smoothing clamped off) and 1 deg flatness gate.

### Vision (walked 2026-10-10)
- Active cameras: three fixed PhotonVision cameras (evan front, val back, daniil rio-side).
  Limelight turret camera (argos) commented out, so the turret-angle observer wired in
  RobotContainer currently has no consumer; MegaTag2 path dormant.
- Each PhotonProcessor runs on its own Notifier thread at 20 Hz: unread frames ->
  PhotonPoseEstimator (coprocessor multi-tag if >1 tag, else lowest-ambiguity single tag) ->
  std devs -> lock-free queue. VisionSubsystem.periodic drains queues -> filters (latency
  <= 90 ms; single-tag ambiguity <= 0.18 and distance <= 7 m; area 0.04..10; odometry
  outlier filter disabled) -> PoseFusionEngine (group within 20 ms, reject > 2 sigma from
  mean, inverse-variance weighted average) -> if robot flat, `swerve.addVisionMeasurement`.
- Std dev model (StdDevCalculator): base (0.41 m, 0.58 rad) x ambiguity^0.7 x area^0.6 x
  latency^0.2 x 1.4/ln(n+1). Hand-built heuristic.
- Findings: (a) **area unit mismatch**: calculator expects fraction of frame, Photon
  `getArea()` is percent (0-100); Limelight path divides by 100, Photon doesn't. Net: std devs
  ~4x smaller than designed (10^0.6), variance ~16x; tags >= 1% of frame all get the floor
  factor (distance sensitivity lost). Base constants may have been tuned around it.
  (b) Vision measurements dropped unless `isFlatDebounced` (1 deg gate) -> also affects
  localization. (c) `getVisionPose()` (used for resets) returns latest single-camera result,
  not the fused one; `primaryFused` list is actually filtered raw. (d) Fusion assumes
  independent camera errors; outlier test uses non-robust mean. (e) Ring-buffer result
  objects reused across threads (latent, low-probability race). (f) Turreted Photon path
  scales by 1/sqrt(n) on top of tag-count factor (double count; currently unused).

### Vision, deeper pass (2026-10-11, during Bordie Blast)
- Timestamps handled correctly: Photon FPGA time -> `Utils.fpgaToCurrentTime` (CTRE timebase)
  before `addVisionMeasurement`; turret yaw buffer uses FPGA time on both sides.
- Two-stage estimation: cameras pre-fused (inverse variance, independence assumed, stamped
  with the *latest* timestamp in a 20 ms group) then fused again with odometry. At 4 m/s the
  timestamp choice smears up to ~8 cm. Alternative: send each camera's measurement to the
  estimator separately (it already weights and latency-compensates); lose cross-camera
  outlier rejection. Trade-off worth testing.
- Single-tag frames use lowest-ambiguity PnP and ignore the gyro. PhotonLib 2026 should offer
  heading-assisted strategies (PnP distance trig solve / constrained solvePnP via heading
  data), the Photon analogue of Limelight MegaTag2. Likely accuracy win for single-tag.
- Vision heading is fused into the estimator (theta std ~0.4 rad per camera; ~5-8% pull per
  measurement). Turret aim = field yaw - robot heading, so heading bias = aim bias. Check:
  estimator heading vs raw Pigeon yaw over a match.
- Simulation has PhotonVision camera sim (50 fps, small calibration noise) with known true
  pose: a sandbox to prototype the outlier / std-dev analyses before real logs arrive.

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
- **2026-10-10**: Walked aiming stack. Collision-map open question resolved (geometric).
- **2026-10-10**: Re-prioritized contribution list (§4).
- **2026-10-10**: Walked vision. Found area-unit mismatch in std-dev model.
- **2026-10-11**: Vision deeper pass (timestamps, two-stage fusion, heading, sim sandbox).
