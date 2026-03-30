// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;

/** Factory methods for shooter mode OI and auto helpers. */
public final class ShooterCommands {

  private ShooterCommands() {}

  public static Command getSetStateCommand(RobotState.ShooterMode mode, Flywheel flywheel) {
    return Commands.runOnce(() -> RobotState.getInstance().setShooterMode(mode), flywheel);
  }

  public static Command getTrackHubCommand(Flywheel flywheel) {
    return Commands.startEnd(
        () -> RobotState.getInstance().setShooterMode(RobotState.ShooterMode.HUB),
        () -> {
          RobotState.getInstance().setShooterMode(RobotState.ShooterMode.IDLE);
          ShooterCalculator.getInstance().clearShootingParameters();
        },
        flywheel);
  }

  public static Command getTrackPassCommand(Flywheel flywheel) {
    return Commands.startEnd(
        () -> RobotState.getInstance().setShooterMode(RobotState.ShooterMode.PASS),
        () -> {
          RobotState.getInstance().setShooterMode(RobotState.ShooterMode.IDLE);
          ShooterCalculator.getInstance().clearShootingParameters();
        },
        flywheel);
  }

  public static Command getManualCommand(Flywheel flywheel) {
    return Commands.startEnd(
        () -> RobotState.getInstance().setShooterMode(RobotState.ShooterMode.CUSTOM),
        () -> RobotState.getInstance().setShooterMode(RobotState.ShooterMode.CUSTOM),
        flywheel);
  }

  /** Optional: autonomous parallel tracks; {@link ShooterCalculator#coordinateAfterScheduler} refreshes the solve. */
  public static Command getTrackAllMechanismsCommand(Flywheel f, Hood h, Turret t) {
    return new ParallelCommandGroup(
        f.runTrackTargetCommand(), h.runTrackTargetCommand(), t.runTrackTargetCommand());
  }

  public static Command getAimAtTranslation3dCommand(
      Translation3d targetFieldBluePerspective, Flywheel flywheel) {
    return Commands.runOnce(
        () -> RobotState.getInstance().applyShooterPoint3dTargetBlue(targetFieldBluePerspective),
        flywheel);
  }
}
