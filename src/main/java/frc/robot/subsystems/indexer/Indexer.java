package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.indexer.agitator.Agitator;
import frc.robot.subsystems.indexer.agitator.Agitator.AgitatorGoal;
import frc.robot.subsystems.indexer.agitator.AgitatorIO;
import frc.robot.subsystems.indexer.conveyor.Conveyor;
import frc.robot.subsystems.indexer.conveyor.Conveyor.ConveyorGoal;
import frc.robot.subsystems.indexer.conveyor.ConveyorIO;
import frc.robot.subsystems.indexer.kicker.Kicker;
import frc.robot.subsystems.indexer.kicker.Kicker.KickerGoal;
import frc.robot.subsystems.indexer.kicker.KickerIO;
import frc.robot.subsystems.indexer.rollers.Rollers;
import frc.robot.subsystems.indexer.rollers.Rollers.RollersGoal;
import frc.robot.subsystems.indexer.rollers.RollersIO;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Coordinates agitator, kicker, conveyor, and rollers into high-level {@link INDEXER_STATE}
 * behaviors.
 */
public class Indexer extends SubsystemBase {
  /** Named indexer behaviors selected by OI and autonomous. */
  public enum INDEXER_STATE {
    INTAKE,
    OUTTAKE,
    SPIT,
    AUTOSHOOT,
    SHOOT,
    ZERO,
    CUSTOM
  }

  @AutoLogOutput(key = "Indexer/currentState")
  private INDEXER_STATE currentIndexerState = INDEXER_STATE.ZERO;

  private final Agitator agitator;
  private final Kicker kicker;
  private final Conveyor conveyor;
  private final Rollers rollers;

  /**
   * @param agitatorIO agitator hardware (or sim/replay stub)
   * @param kickerIO kicker hardware
   * @param conveyorIO conveyor hardware
   * @param rollersIO rollers hardware
   */
  public Indexer(
      AgitatorIO agitatorIO, KickerIO kickerIO, ConveyorIO conveyorIO, RollersIO rollersIO) {
    this.agitator = new Agitator(agitatorIO);
    this.kicker = new Kicker(kickerIO);
    this.conveyor = new Conveyor(conveyorIO);
    this.rollers = new Rollers(rollersIO);
  }

  /** Updates the state machine input; {@link #periodic()} applies it to mechanisms. */
  public void setIndexerState(INDEXER_STATE state) {
    currentIndexerState = state;
  }

  /**
   * Applies {@link #currentIndexerState} to mechanism goals, then runs each mechanism {@code
   * periodic()}.
   */
  @Override
  public void periodic() {
    boolean readyToShoot = RobotState.getInstance().isShooterReadyToShoot();
    switch (currentIndexerState) {
      case INTAKE:
        agitator.setGoalSetPoint(AgitatorGoal.ZERO);
        kicker.setGoalSetPoint(KickerGoal.ZERO);
        conveyor.setGoalSetPoint(ConveyorGoal.ZERO);
        rollers.setGoalSetPoint(RollersGoal.INTAKE);
        break;
      case OUTTAKE:
        agitator.setGoalSetPoint(AgitatorGoal.OUTTAKE);
        kicker.setGoalSetPoint(KickerGoal.OUTTAKE);
        conveyor.setGoalSetPoint(ConveyorGoal.OUTTAKE);
        rollers.setGoalSetPoint(RollersGoal.OUTTAKE);
        break;
      case SPIT:
        agitator.setGoalSetPoint(AgitatorGoal.SPIT);
        kicker.setGoalSetPoint(KickerGoal.SPIT);
        conveyor.setGoalSetPoint(ConveyorGoal.SPIT);
        rollers.setGoalSetPoint(RollersGoal.SPIT);
        break;
      case SHOOT:
        if (readyToShoot) {
          agitator.setGoalSetPoint(AgitatorGoal.SHOOT);
          kicker.setGoalSetPoint(KickerGoal.SHOOT);
          conveyor.setGoalSetPoint(ConveyorGoal.SHOOT);
          rollers.setGoalSetPoint(RollersGoal.SHOOT);
        } else {
          agitator.setGoalSetPoint(AgitatorGoal.ZERO);
          kicker.setGoalSetPoint(KickerGoal.ZERO);
          conveyor.setGoalSetPoint(ConveyorGoal.ZERO);
          rollers.setGoalSetPoint(RollersGoal.ZERO);
        }
        break;
      case AUTOSHOOT:
        agitator.setGoalSetPoint(AgitatorGoal.SHOOT);
        kicker.setGoalSetPoint(KickerGoal.SHOOT);
        conveyor.setGoalSetPoint(ConveyorGoal.SHOOT);
        rollers.setGoalSetPoint(RollersGoal.SHOOT);
        break;
      case ZERO:
        agitator.setGoalSetPoint(AgitatorGoal.ZERO);
        kicker.setGoalSetPoint(KickerGoal.ZERO);
        conveyor.setGoalSetPoint(ConveyorGoal.ZERO);
        rollers.setGoalSetPoint(RollersGoal.ZERO);
        break;
      case CUSTOM:
        agitator.setGoalSetPoint(AgitatorGoal.CUSTOM);
        kicker.setGoalSetPoint(KickerGoal.CUSTOM);
        conveyor.setGoalSetPoint(ConveyorGoal.CUSTOM);
        rollers.setGoalSetPoint(RollersGoal.CUSTOM);
        break;
    }

    agitator.periodic();
    kicker.periodic();
    conveyor.periodic();
    rollers.periodic();
  }

  /** Schedules a one-shot update of {@link #setIndexerState(INDEXER_STATE)}. */
  public static Command getSetStateCommand(INDEXER_STATE state, Indexer indexer) {
    return Commands.runOnce(() -> indexer.setIndexerState(state), indexer);
  }
}
