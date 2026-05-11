// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterState;
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

    /** Teleop shoot: hub vs pass from field pose; see {@code ShooterState.teleopAimModeForOwnFieldSide}. */
    TELE_SHOOT,
    CUSTOM,
    PASS_FOCUS,
    AUTO_START_PASS,

    STOP_INTAKE,
    STOP_SHOOT
  }

  private RobotStateCommands() {}

  /** Entry point for OI, auto paths, and tests. */
  public static Command commandFor(RobotState state) {
    return switch (state) {
      case DEFAULT -> defaultState();
      case STOP_INTAKE -> stopIntakeState();
      case STOP_SHOOT -> stopShootingState();

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
      case AUTO_START_PASS -> autoShootPassState();
    };
  }

  public static Command defaultState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            ShooterState.ShooterMode.IDLE, RobotContainer.flywheel));
  }

  public static Command stopShootingState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            ShooterState.ShooterMode.IDLE, RobotContainer.flywheel));
  }

  public static Command stopIntakeState() {
    return Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake);
  }

  public static Command autoDefaultState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            ShooterState.ShooterMode.IDLE, RobotContainer.flywheel));
  }

  public static Command trenchState() {
    return new ParallelCommandGroup(
        Intake.getSetStateCommand(INTAKE_STATE.DOWN, RobotContainer.intake),
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            ShooterState.ShooterMode.IDLE, RobotContainer.flywheel));
  }

  public static Command hubFocusState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            ShooterState.ShooterMode.HUB, RobotContainer.flywheel));
  }

  public static Command passFocusState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.ZERO, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            ShooterState.ShooterMode.PASS, RobotContainer.flywheel));
  }

  public static Command autoShootStateCommand() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            ShooterState.ShooterMode.HUB, RobotContainer.flywheel));
  }

  public static Command autoShootPassState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer),
        ShooterCommands.getSetStateCommand(
            ShooterState.ShooterMode.PASS, RobotContainer.flywheel));
  }

  /**
   * Teleop shoot (driver right trigger): continuous hub/pass mode from field pose and indexer
   * {@link INDEXER_STATE#SHOOT} (gated by {@link ShooterState#isShooterReadyToShoot()} in {@link
   * Indexer#periodic()}). Intake is not included here; use the driver left trigger {@link
   * RobotState#INTAKE} path in parallel when feeding. Bind with {@code whileTrue} so release cancels
   * this group and {@code STOP_SHOOT} can return the shooter to idle.
   */
  public static Command teleShootState() {
    return new ParallelCommandGroup(
        Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer),
        Commands.run(
            () ->
                ShooterState.getInstance()
                    .setShooterMode(
                        ShooterState.teleopAimModeForOwnFieldSide(
                            frc.robot.RobotState.getInstance().getEstimatedPose()))));
  }

  public static Command customShootState() {
    return new ParallelCommandGroup(
      ShooterCommands.getSetStateCommand(ShooterState.ShooterMode.CUSTOM, RobotContainer.flywheel),
      Indexer.getSetStateCommand(INDEXER_STATE.SHOOT, RobotContainer.indexer)
    );
  }

  public static Command intakeState() {
    return new ParallelCommandGroup(
      Indexer.getSetStateCommand(INDEXER_STATE.INTAKE, RobotContainer.indexer),
      Intake.getSetStateCommand(INTAKE_STATE.INTAKE, RobotContainer.intake)
    );
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
