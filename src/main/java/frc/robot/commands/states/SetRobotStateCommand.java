package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.INDEXER_STATE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.INTAKE_STATE;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_STATE;

/**
 * Runs one parallel step of per-subsystem {@code runOnce} commands. Each child only declares the
 * subsystem it touches, so e.g. OUTTAKE does not interrupt indexer/shooter like a single command
 * requiring all three would.
 */
public class SetRobotStateCommand extends ParallelCommandGroup {

  public enum ROBOT_STATE {
    DEFAULT,
    AUTODEFAULT,
    TRENCH,

    INTAKE,
    OUTTAKE,
    STOP_INTAKE,
    SPIT,
    AGITATE,
    UN_AGITATE,

    SHOOT,
    AIM,

    STOP_SHOOT,

    AUTO_SHOOT_HUB,
    AUTO_SHOOT_PASS,
    AUTO_AIM_HUB,
    AUTO_AIM_PASS,

    CUSTOM
  }

  public SetRobotStateCommand(ROBOT_STATE state) {
    switch (state) {
      case CUSTOM:
        addCommands(
            Shooter.getSetStateCommand(SHOOTER_STATE.CUSTOM, RobotContainer.shooter),
            Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer));
        break;
      case STOP_SHOOT:
        addCommands(
            Shooter.getSetStateCommand(SHOOTER_STATE.HOME, RobotContainer.shooter),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer));
        break;
      case STOP_INTAKE:
        addCommands(
            Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer));
        break;
      case DEFAULT:
        addCommands(
            Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.HOME, RobotContainer.shooter));
        break;
      case AUTODEFAULT:
        addCommands(
            Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AUTOHOME, RobotContainer.shooter));
        break;
      case TRENCH:
        addCommands(
            Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.HOME, RobotContainer.shooter));
        break;
      case INTAKE:
        addCommands(
            Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.INTAKE, RobotContainer.indexer));
        break;
      case OUTTAKE:
        addCommands(Intake.getSetStateCommand(INTAKE_STATE.OUTTAKE, RobotContainer.intake));
        break;
      case SPIT:
        addCommands(Indexer.getSetStateCommand(INDEXER_STATE.SPIT, RobotContainer.indexer));
        break;
      case AGITATE:
        addCommands(Intake.getSetStateCommand(INTAKE_STATE.AGITATE, RobotContainer.intake));
        break;
      case UN_AGITATE:
        addCommands(Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake));
        break;
      case AIM:
        addCommands(
            Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AIM, RobotContainer.shooter));
        break;
      case SHOOT:
        addCommands(
            Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AIM, RobotContainer.shooter));
        break;
      case AUTO_AIM_HUB:
        addCommands(
            Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AUTOAIMHUB, RobotContainer.shooter));
        break;
      case AUTO_SHOOT_HUB:
        addCommands(
            Indexer.getSetStateCommand(INDEXER_STATE.AUTOSHOOT, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AUTOAIMHUB, RobotContainer.shooter));
        break;
      case AUTO_AIM_PASS:
        addCommands(
            Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
            Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AUTOAIMPASS, RobotContainer.shooter));
        break;
      case AUTO_SHOOT_PASS:
        addCommands(
            Indexer.getSetStateCommand(INDEXER_STATE.AUTOSHOOT, RobotContainer.indexer),
            Shooter.getSetStateCommand(SHOOTER_STATE.AUTOAIMPASS, RobotContainer.shooter));
        break;
    }
  }
}
