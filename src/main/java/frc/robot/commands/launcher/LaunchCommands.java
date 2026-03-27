package frc.robot.commands.launcher;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;

/** Factory methods for launcher mode OI and auto helpers. */
public final class LaunchCommands {

  private LaunchCommands() {}

  public static Command getSetStateCommand(RobotState.LauncherMode mode, LaunchCoordinatorSubsystem coord) {
    return Commands.runOnce(() -> RobotState.getInstance().setLauncherMode(mode), coord);
  }

  public static Command getTrackHubCommand(LaunchCoordinatorSubsystem coord) {
    return Commands.startEnd(
        () -> RobotState.getInstance().setLauncherMode(RobotState.LauncherMode.HUB),
        () -> {
          RobotState.getInstance().setLauncherMode(RobotState.LauncherMode.IDLE);
          LaunchCalculator.getInstance().clearLaunchingParameters();
        },
        coord);
  }

  public static Command getTrackPassCommand(LaunchCoordinatorSubsystem coord) {
    return Commands.startEnd(
        () -> RobotState.getInstance().setLauncherMode(RobotState.LauncherMode.PASS),
        () -> {
          RobotState.getInstance().setLauncherMode(RobotState.LauncherMode.IDLE);
          LaunchCalculator.getInstance().clearLaunchingParameters();
        },
        coord);
  }

  public static Command getManualCommand(LaunchCoordinatorSubsystem coord) {
    return Commands.startEnd(
        () -> RobotState.getInstance().setLauncherMode(RobotState.LauncherMode.CUSTOM),
        () -> RobotState.getInstance().setLauncherMode(RobotState.LauncherMode.CUSTOM),
        coord);
  }

  /** Optional: autonomous parallel tracks; coordinator {@code periodic()} still refreshes the calculator. */
  public static Command getTrackAllMechanismsCommand(Flywheel f, Hood h, Turret t) {
    return new ParallelCommandGroup(
        f.runTrackTargetCommand(), h.runTrackTargetCommand(), t.runTrackTargetCommand());
  }

  public static Command getAimAtTranslation3dCommand(
      Translation3d targetFieldBluePerspective, LaunchCoordinatorSubsystem coord) {
    return Commands.runOnce(
        () -> RobotState.getInstance().applyLauncherPoint3dTargetBlue(targetFieldBluePerspective), coord);
  }
}
