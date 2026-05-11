# 4270 Ikaika — Robot Code

Java command-based robot program for **FIRST Robotics Competition Team 4270**, built on **WPILib 2026** with **AdvantageKit** logging, **CTRE Phoenix 6** devices, **PathPlanner** autos, and **PhotonVision** / **Limelight** vision (mode-dependent).

This repository is intended for public sharing: architecture follows common FRC patterns (subsystem + `*IO` hardware abstraction) with credit to **FRC 6328 Mechanical Advantage** templates and practices where noted in source headers.

---

## Robot overview

| Area | Description |
|------|-------------|
| **Drivetrain** | Swerve drive using generated `TunerConstants` (CTRE Swerve), `Drive`, per-module `ModuleIO` (TalonFX on real robot, simulation IO in sim/replay). |
| **Intake** | Roller + wrist; states composed in `RobotStateCommands` with the indexer. |
| **Indexer** | Four coordinated mechanisms: **rollers**, **conveyor**, **agitator**, **kicker** — each has tunable velocity goals and Phoenix gains in `IndexerConstants`. |
| **Shooter** | Flywheel, hood, turret; `ShooterCalculator` coordinates targets after the command scheduler each loop (`RobotContainer.runShooterCoordinationAfterScheduler`). |
| **Vision** | Real robot: Limelight IO for multiple cameras. Simulation: PhotonVision sim. Replay: stub `VisionIO`. Poses feed `RobotState` for odometry fusion. |
| **Autonomous** | PathPlanner named commands (`TRENCH`, `INTAKE`, `DEFAULT`, `HUB_FOCUS`, `HUB_SHOOT`, `PASS_FOCUS`, `PASS_SHOOT`, `AGITATE`) plus `LoggedDashboardChooser` auto selection. |

Field geometry and AprilTag layouts live in `FieldConstants` (blue-alliance frame convention per class Javadoc).

---

## Repository layout

| Path | Role |
|------|------|
| `src/main/java/frc/robot/Robot.java` | AdvantageKit setup (real / sim / replay), scheduler, post-scheduler hooks. |
| `src/main/java/frc/robot/RobotContainer.java` | Subsystem wiring for REAL vs SIM vs REPLAY, OI bindings, named auto commands. |
| `src/main/java/frc/robot/Constants.java` | `Mode` selection; **`simMode`** chooses desktop sim vs log replay when not on a roboRIO. |
| `src/main/java/frc/robot/commands/` | Drive commands, shooter commands, `RobotStateCommands` (high-level robot modes). |
| `src/main/java/frc/robot/subsystems/` | Mechanisms; each logical part typically has `*IO`, `*IOTalonFX`, and a coordinator class. |
| `src/main/java/frc/robot/generated/TunerConstants.java` | Swerve module constants (Phoenix Tuner / template generated). |
| `src/main/deploy/pathplanner/` | Autos and paths consumed by PathPlanner. |
| `Notes.md` | **CAN map**, hardware notes, and legacy / aspirational OI notes (verify against `RobotContainer` for truth). |
| `CodingStandard.md` | Team Java conventions. |

---

## Build and deploy

Prerequisites: **JDK 17**, WPILib / GradleRIO (vendordeps are committed under `vendordeps/`).

```bash
./gradlew compileJava
./gradlew deploy
```

Desktop simulation (when `Constants.currentMode` resolves to `SIM`):

```bash
./gradlew simulateJava
```

Log replay uses `Mode.REPLAY` and `Constants.simMode`; see `Robot.java` and AdvantageKit docs for log paths.

---

## Operator interface (as wired in code)

Bindings are defined in `RobotContainer.configureButtonBindings()`. Summary:

**Driver — Xbox (port 0)**

- Default: field-relative joystick drive (`DriveCommands.joystickDrive`).
- **D-pad down:** reset pose heading to zero (tare field-centric heading).
- **Left trigger:** `INTAKE` on press, `STOP_INTAKE` on release.
- **Right trigger (held):** `TELE_SHOOT`; on release, `STOP_SHOOT`.
- **D-pad right:** `OUTTAKE` on press, `STOP_INTAKE` on release.
- **A:** `AGITATE` on press, `UN_AGITATE` on release.

**Operator — Joystick (port 1)**

- **Buttons 3, 4, 5:** same agitate / un-agitate pairing as driver A.
- **Button 1:** `CUSTOM` on press, `STOP_SHOOT` on release.

If `Notes.md` disagrees with the table above, **trust the Java**; update `Notes.md` when hardware mapping catches up.

---

## Configuration tips

1. **Team number:** `.wpilib/wpilib_preferences.json` (or Gradle deploy flags).
2. **Desktop vs real:** `Constants.simMode` in `Constants.java` — on the roboRIO, mode is always `REAL`.
3. **Tunable numbers:** Many setpoints and gains use `LoggedTunableNumber` for AdvantageKit / AdvantageScope tuning without redeploying constants-only experiments.
4. **Phoenix:** Shared retry/timeouts in `Constants.TalonFxIo`.

---

## Third-party and attribution

- **WPILib** and vendor libraries listed in `build.gradle` / `vendordeps/`.
- **AdvantageKit** (`org.littletonrobotics.junction`) for logging and replay.
- **PathPlanner**, **Phoenix 6**, **PhotonLib** as pinned in `vendordeps/`.
- Source files may credit **FRC 6328 Mechanical Advantage**; retain those notices when redistributing.

---

## Contributing

Follow `CodingStandard.md`. Prefer small, reviewable changes; keep subsystem IO boundaries clear when adding hardware.

Questions about the robot or software stack are welcome via your team’s usual channels (issues/discussions if enabled on GitHub).
