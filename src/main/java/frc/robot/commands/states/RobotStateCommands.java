// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.INDEXER_STATE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.INTAKE_STATE;
/** Composes mechanism state changes into robot-wide modes. */
public final class RobotStateCommands {

  /** High-level modes selected by OI or autonomous. */
  public enum RobotState {
    DEFAULT,
    AUTODEFAULT,
    TRENCH,
    INTAKE,
    OUTTAKE,
    AGITATE,
    UN_AGITATE,
    AUTO_START_SHOOT,
    HUB_FOCUS,

    /** Teleop: intake + indexer shoot + shooter aim (right trigger). */
    TELE_SHOOT,
    CUSTOM,
    PASS_FOCUS,
    AUTO_SHOOT_PASS,
  }

  private RobotStateCommands() {}

  /** Entry point for OI, auto paths, and tests. */
  public static Command commandFor(RobotState state) {
    return switch (state) {
      case DEFAULT -> defaultState();
      case AUTODEFAULT -> autoDefaultState();
      case TRENCH -> trenchState();
      case INTAKE -> intakeState();
      case OUTTAKE -> outtakeState();
      case AUTO_START_SHOOT -> autoShootStateCommand();
      case AGITATE -> agitateState();
      case UN_AGITATE -> unAgitateState();
      case HUB_FOCUS -> hubFocusState();
      case TELE_SHOOT -> teleShootState();
      case CUSTOM -> customShootState();
      case PASS_FOCUS -> passFocusState();
      case AUTO_SHOOT_PASS -> autoShootPassState();
    };
  }

  public static Command defaultState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            frc.robot.RobotState.ShooterMode.IDLE, RobotContainer.flywheel));
  }

  public static Command autoDefaultState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            frc.robot.RobotState.ShooterMode.IDLE, RobotContainer.flywheel));
  }

  public static Command trenchState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            frc.robot.RobotState.ShooterMode.IDLE, RobotContainer.flywheel));
  }

  public static Command hubFocusState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            frc.robot.RobotState.ShooterMode.HUB, RobotContainer.flywheel));
  }

  public static Command passFocusState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            frc.robot.RobotState.ShooterMode.PASS, RobotContainer.flywheel));
  }

  public static Command autoShootStateCommand() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.AUTOSHOOT, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            frc.robot.RobotState.ShooterMode.HUB, RobotContainer.flywheel));
  }

  public static Command autoShootPassState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.AUTOSHOOT, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            frc.robot.RobotState.ShooterMode.PASS, RobotContainer.flywheel));
  }

  public static Command teleShootState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            frc.robot.RobotState.ShooterMode.HUB, RobotContainer.flywheel));
  }

  public static Command customShootState() {
    return new ParallelCommandGroup(
        ShooterCommands.getSetStateCommand(
            frc.robot.RobotState.ShooterMode.CUSTOM, RobotContainer.flywheel),
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

  public static Command agitateState() {
    return Intake.getSetStateCommand(INTAKE_STATE.AGITATE, RobotContainer.intake);
  }
}
