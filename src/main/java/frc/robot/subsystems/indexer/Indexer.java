// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.subsystems.indexer.indexerAgitator.IndexerAgitator;
import frc.robot.subsystems.indexer.indexerAgitator.IndexerAgitator.IndexerAgitatorGoal;
import frc.robot.subsystems.indexer.indexerAgitator.IndexerAgitatorIO;
import frc.robot.subsystems.indexer.indexerConveyor.IndexerConveyor;
import frc.robot.subsystems.indexer.indexerConveyor.IndexerConveyor.IndexerConveyorGoal;
import frc.robot.subsystems.indexer.indexerConveyor.IndexerConveyorIO;
import frc.robot.subsystems.indexer.indexerKicker.IndexerKicker;
import frc.robot.subsystems.indexer.indexerKicker.IndexerKicker.IndexerKickerGoal;
import frc.robot.subsystems.indexer.indexerKicker.IndexerKickerIO;
import frc.robot.subsystems.indexer.indexerRollers.IndexerRollers;
import frc.robot.subsystems.indexer.indexerRollers.IndexerRollers.IndexerRollersGoal;
import frc.robot.subsystems.indexer.indexerRollers.IndexerRollersIO;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Coordinates indexer agitator, kicker, conveyor, and rollers into high-level {@link INDEXER_STATE}
 * behaviors.
 */
public class Indexer extends SubsystemBase {
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

  private final IndexerAgitator indexerAgitator;
  private final IndexerKicker indexerKicker;
  private final IndexerConveyor indexerConveyor;
  private final IndexerRollers indexerRollers;

  public Indexer(
      IndexerAgitatorIO indexerAgitatorIO,
      IndexerKickerIO indexerKickerIO,
      IndexerConveyorIO indexerConveyorIO,
      IndexerRollersIO indexerRollersIO) {
    this.indexerAgitator = new IndexerAgitator(indexerAgitatorIO);
    this.indexerKicker = new IndexerKicker(indexerKickerIO);
    this.indexerConveyor = new IndexerConveyor(indexerConveyorIO);
    this.indexerRollers = new IndexerRollers(indexerRollersIO);
  }

  public void setIndexerState(INDEXER_STATE state) {
    currentIndexerState = state;
  }

  @Override
  public void periodic() {
    switch (currentIndexerState) {
      case INTAKE:
        indexerAgitator.setGoalSetPoint(IndexerAgitatorGoal.INTAKE);
        // indexerKicker.setGoalSetPoint(IndexerKickerGoal.ZERO);
        // indexerConveyor.setGoalSetPoint(IndexerConveyorGoal.ZERO);
        indexerRollers.setGoalSetPoint(IndexerRollersGoal.INTAKE);
        break;
      case OUTTAKE:
        indexerAgitator.setGoalSetPoint(IndexerAgitatorGoal.OUTTAKE);
        // indexerKicker.setGoalSetPoint(IndexerKickerGoal.OUTTAKE);
        // indexerConveyor.setGoalSetPoint(IndexerConveyorGoal.OUTTAKE);
        indexerRollers.setGoalSetPoint(IndexerRollersGoal.OUTTAKE);
        break;
      case SPIT:
        indexerAgitator.setGoalSetPoint(IndexerAgitatorGoal.SPIT);
        indexerKicker.setGoalSetPoint(IndexerKickerGoal.SPIT);
        indexerConveyor.setGoalSetPoint(IndexerConveyorGoal.SPIT);
        indexerRollers.setGoalSetPoint(IndexerRollersGoal.SPIT);
        break;
      case SHOOT:
        if (ShooterState.getInstance().isShooterReadyToShoot()) {
          indexerAgitator.setGoalSetPoint(IndexerAgitatorGoal.SHOOT);
          indexerKicker.setGoalSetPoint(IndexerKickerGoal.SHOOT);
          indexerConveyor.setGoalSetPoint(IndexerConveyorGoal.SHOOT);
          indexerRollers.setGoalSetPoint(IndexerRollersGoal.SHOOT);
        } else {
          indexerAgitator.setGoalSetPoint(IndexerAgitatorGoal.INTAKE);
          indexerKicker.setGoalSetPoint(IndexerKickerGoal.ZERO);
          indexerConveyor.setGoalSetPoint(IndexerConveyorGoal.INTAKE);
          indexerRollers.setGoalSetPoint(IndexerRollersGoal.INTAKE);
        }
        break;
      case AUTOSHOOT:
        indexerAgitator.setGoalSetPoint(IndexerAgitatorGoal.SHOOT);
        indexerKicker.setGoalSetPoint(IndexerKickerGoal.SHOOT);
        indexerConveyor.setGoalSetPoint(IndexerConveyorGoal.SHOOT);
        indexerRollers.setGoalSetPoint(IndexerRollersGoal.SHOOT);
        break;
      case ZERO:
        indexerAgitator.setGoalSetPoint(IndexerAgitatorGoal.ZERO);
        indexerKicker.setGoalSetPoint(IndexerKickerGoal.ZERO);
        indexerConveyor.setGoalSetPoint(IndexerConveyorGoal.ZERO);
        indexerRollers.setGoalSetPoint(IndexerRollersGoal.ZERO);
        break;
      case CUSTOM:
        indexerAgitator.setGoalSetPoint(IndexerAgitatorGoal.CUSTOM);
        indexerKicker.setGoalSetPoint(IndexerKickerGoal.CUSTOM);
        indexerConveyor.setGoalSetPoint(IndexerConveyorGoal.CUSTOM);
        indexerRollers.setGoalSetPoint(IndexerRollersGoal.CUSTOM);
        break;
    }

    indexerAgitator.periodic();
    indexerKicker.periodic();
    indexerConveyor.periodic();
    indexerRollers.periodic();
  }

  public static Command getSetStateCommand(INDEXER_STATE state, Indexer indexer) {
    return Commands.runOnce(() -> indexer.setIndexerState(state), indexer);
  }
}
