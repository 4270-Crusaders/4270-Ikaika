package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.INDEXER_STATE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.INTAKE_STATE;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_STATE;

/** Composes mechanism state changes into robot-wide modes. */
public final class RobotStateCommands {

  /** High-level modes selected by OI or autonomous. */
  public enum RobotState {
    DEFAULT,
    AUTODEFAULT,
    TRENCH,
    INTAKE,
    OUTTAKE,
    SPIT,
    AGITATE,
    UN_AGITATE,
    AUTOSTARTSHOOT,
    HUB_FOCUS,
    HUB_SHOOT,
    /** Teleop: intake + indexer shoot + shooter aim (right trigger). */
    TELE_SHOOT,
    CUSTOM,
    PASS_FOCUS,
    PASS_SHOOT,
    AUTO_SHOOT_PASS,
  }

  private RobotStateCommands() {}

  /** Entry point for {@link SetRobotStateCommand} and tests. */
  public static Command commandFor(RobotState state) {
    return switch (state) {
      case DEFAULT -> defaultState();
      case AUTODEFAULT -> autoDefaultState();
      case TRENCH -> trenchState();
      case INTAKE -> intakeState();
      case OUTTAKE -> outtakeState();
      case SPIT -> spitState();
      case AUTOSTARTSHOOT -> autoShootStateCommand();
      case AGITATE -> agitateState();
      case UN_AGITATE -> unAgitateState();
      case HUB_FOCUS -> hubFocusState();
      case HUB_SHOOT -> shootHubState();
      case TELE_SHOOT -> teleShootState();
      case CUSTOM -> customState();
      case PASS_FOCUS -> passFocusState();
      case PASS_SHOOT -> shootPassState();
      case AUTO_SHOOT_PASS -> autoShootPassState();
    };
  }

  public static Command defaultState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        Shooter.getSetStateCommand(SHOOTER_STATE.HOME, RobotContainer.shooter));
  }

  public static Command autoDefaultState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        Shooter.getSetStateCommand(SHOOTER_STATE.AUTOHOME, RobotContainer.shooter));
  }

  public static Command trenchState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        Shooter.getSetStateCommand(SHOOTER_STATE.HOME, RobotContainer.shooter));
  }

  public static Command hubFocusState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        Shooter.getSetStateCommand(SHOOTER_STATE.HUB, RobotContainer.shooter));
  }

  public static Command passFocusState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        Shooter.getSetStateCommand(SHOOTER_STATE.PASS, RobotContainer.shooter));
  }

  public static Command shootHubState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer),
        Shooter.getSetStateCommand(SHOOTER_STATE.HUB, RobotContainer.shooter));
  }

  public static Command autoShootStateCommand() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.AUTOSHOOT, RobotContainer.indexer),
        Shooter.getSetStateCommand(SHOOTER_STATE.HUB, RobotContainer.shooter));
  }

  public static Command shootPassState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer),
        Shooter.getSetStateCommand(SHOOTER_STATE.PASS, RobotContainer.shooter));
  }

  public static Command autoShootPassState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.AUTOSHOOT, RobotContainer.indexer),
        Shooter.getSetStateCommand(SHOOTER_STATE.PASS, RobotContainer.shooter));
  }

  public static Command teleShootState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer),
        Shooter.getSetStateCommand(SHOOTER_STATE.AIM, RobotContainer.shooter));
  }

  public static Command customState() {
    return new ParallelCommandGroup(
        Shooter.getSetStateCommand(SHOOTER_STATE.CUSTOM, RobotContainer.shooter),
        Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer));
  }

  public static Command intakeState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.INTAKE, RobotContainer.indexer));
  }

  /** Exit agitate: rollers back to normal intake behavior. */
  public static Command unAgitateState() {
    return Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake);
  }

  public static Command outtakeState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.OUTTAKE, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.OUTTAKE, RobotContainer.indexer));
  }

  public static Command spitState() {
    return Indexer.getSetStateCommand(INDEXER_STATE.SPIT, RobotContainer.indexer);
  }

  public static Command agitateState() {
    return Intake.getSetStateCommand(INTAKE_STATE.AGITATE, RobotContainer.intake);
  }
}
