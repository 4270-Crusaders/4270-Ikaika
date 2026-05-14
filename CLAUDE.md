# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## FRC Team 4270 — Ikaika

WPILib 2026, Command-Based Java, CTRE Phoenix 6 (TalonFX/Pigeon2), AdvantageKit logging, Limelight vision, PathPlanner autonomous, Lombok.

## Build Commands

```bash
./gradlew build             # compile and check
./gradlew test              # run JUnit 5 tests
./gradlew simulateJava      # launch desktop simulation GUI
./gradlew deploy            # deploy to roboRIO over USB/WiFi
./gradlew replayWatch       # replay an AKit log file
```

To run a single test class: `./gradlew test --tests "frc.robot.SomeTest"`

## Architecture

### AdvantageKit IO Layer

Every hardware subsystem follows a three-file pattern:
- `*IO.java` — interface declaring `@AutoLog`-annotated `Inputs` inner class and update/command methods
- `*IOTalonFX.java` (or `*IOPigeon2`, `*IOSim`) — hardware implementation
- The subsystem itself receives an `*IO` in its constructor and calls `io.updateInputs(inputs)` in `periodic()`

`RobotContainer` switches implementations based on `Constants.currentMode` (REAL / SIM / REPLAY). In REPLAY mode all IO implementations are empty anonymous classes. **Never add hardware calls directly to a subsystem — they belong in an IO implementation.**

### RobotState Singleton

`RobotState.getInstance()` is the single source of truth for:
- `estimatedPose` — fused wheel + gyro + vision (Kalman filter via `addVisionObservation`)
- `odometryPose` — wheel-only pose
- `robotVelocity` / `robotAcceleration` — chassis speeds + finite-difference accel

`Drive` feeds `addOdometryObservation()`; `Vision` feeds `addVisionObservation()`. All consumers (ShooterCalculator, DriveCommands, etc.) read from RobotState — subsystems do not talk to each other directly.

### Shooter Pipeline (post-scheduler)

The shooter has its own coordination loop that runs **after** `CommandScheduler.run()` in `Robot.robotPeriodic()`:

1. Commands set `ShooterState.getInstance().setShooterMode(mode)` to select a target (HUB / PASS / IDLE / CUSTOM / POINT_3D).
2. `ShooterCalculator.coordinateAfterScheduler(flywheel, hood, turret)` solves the ballistic trajectory (drag + Magnus physics, moving-target lead), then calls `flywheel/hood/turret.applySetpointForOutput()`.
3. `ShooterState.isShooterReadyToShoot()` gates the indexer kicker in `Indexer.periodic()`.

The flywheel/hood/turret default commands call `runTrackTargetCommand()` which continuously reads `ShooterCalculator.getParameters()`.

### FullSubsystem

Subsystems that need hardware writes **after** commands execute extend `FullSubsystem` instead of `SubsystemBase`. They implement `periodicAfterScheduler()`. `FullSubsystem.runAllPeriodicAfterScheduler()` is called at the end of `robotPeriodic()` (disabled: skipped to reduce CAN load).

### robotPeriodic() Execution Order

```
CommandScheduler.run()                          // periodic() + command execute()
RobotContainer.runShooterCoordinationAfterScheduler()  // ballistic solve + mechanism setpoints
FullSubsystem.runAllPeriodicAfterScheduler()    // hardware output writes
```

### RobotStateCommands — Robot-Wide Modes

All high-level behaviors (INTAKE, TELE_SHOOT, OUTTAKE, etc.) are composed in `RobotStateCommands.commandFor(RobotState)`. OI bindings and PathPlanner named commands always go through this entry point. Use `Commands.defer(() -> RobotStateCommands.commandFor(state), Set.of())` when binding to OI buttons so each press builds a fresh command graph.

### LoggedTunableNumber

Use `LoggedTunableNumber` (not raw SmartDashboard) for all gains and setpoints. Keys follow `Subsystem/Component[/Category]/Name` (e.g. `Shooter/Flywheel/Gains/kP`). Values appear under `TunableNumbers/` in NetworkTables / AdvantageScope.

### Vision

Real robot uses Limelight (`VisionIOLimelight`); simulation uses PhotonVision (`VisionIOPhotonVisionSim`). All cameras push `RobotState.VisionObservation` records; `RobotState` handles timestamp-corrected fusion. Camera names and transforms live in `VisionConstants`.

### PathPlanner Auto

Auto paths are exported from PathPlanner GUI to `src/main/deploy/pathplanner/`. Named commands used inside paths are registered in `RobotContainer.registerNamedCommand()` — add new named commands there before referencing them in PathPlanner.

## Rules

- Subsystems use the AKit IO pattern; extend `FullSubsystem` when hardware writes must happen post-scheduler, `SubsystemBase` otherwise.
- Commands are static factory methods (see `DriveCommands`, `ShooterCommands`, `RobotStateCommands`) — avoid creating new command subclasses unless necessary.
- All robot-wide state changes go through `RobotStateCommands`.
- Never modify `RobotContainer` without confirming the full button-binding and named-command registration is correct.
- Tunable values use `LoggedTunableNumber`; hardware constants go in `Constants.java` or the relevant `*Constants.java`.
