package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  @AutoLogOutput(key = "Indexer/currentState") private INDEXER_STATE currentIndexerState = INDEXER_STATE.ZERO;

  private Agitator agitator;
  private Kicker kicker;
  private Conveyor conveyor;
  private Rollers Rollers;
  private boolean readyToShoot = false;

  public Indexer(
      AgitatorIO agitatorIO, KickerIO kickerIO, ConveyorIO conveyorIO, RollersIO RollersIO) {
    this.agitator = new Agitator(agitatorIO);
    this.kicker = new Kicker(kickerIO);
    this.conveyor = new Conveyor(conveyorIO);
    this.Rollers = new Rollers(RollersIO);
  }

  public void setIndexerState(INDEXER_STATE state) {
    currentIndexerState = state;
  }

  public void setReadyToShoot(boolean value){
    readyToShoot = value;
  }



  @Override
  public void periodic() {
    switch (currentIndexerState) {
      case INTAKE:
        agitator.Setpoint(AgitatorGoal.ZERO);
        kicker.Setpoint(KickerGoal.ZERO);
        conveyor.Setpoint(ConveyorGoal.ZERO);
        Rollers.Setpoint(RollersGoal.INTAKE);
        break;
      case OUTTAKE:
        agitator.Setpoint(AgitatorGoal.OUTTAKE);
        kicker.Setpoint(KickerGoal.OUTTAKE);
        conveyor.Setpoint(ConveyorGoal.OUTTAKE);
        Rollers.Setpoint(RollersGoal.OUTTAKE);
        break;
      case SPIT:
        agitator.Setpoint(AgitatorGoal.SPIT);
        kicker.Setpoint(KickerGoal.SPIT);
        conveyor.Setpoint(ConveyorGoal.SPIT);
        Rollers.Setpoint(RollersGoal.SPIT);
        break;
      case SHOOT:
        if(!readyToShoot){
          agitator.Setpoint(AgitatorGoal.SHOOT);
          kicker.Setpoint(KickerGoal.ZERO);
          conveyor.Setpoint(ConveyorGoal.INTAKE);
          Rollers.Setpoint(RollersGoal.SHOOT);
        } else {
          agitator.Setpoint(AgitatorGoal.SHOOT);
          kicker.Setpoint(KickerGoal.SHOOT);
          conveyor.Setpoint(ConveyorGoal.SHOOT);
          Rollers.Setpoint(RollersGoal.SHOOT);
        }
        break;
      case AUTOSHOOT:
        agitator.Setpoint(AgitatorGoal.SHOOT);
        kicker.Setpoint(KickerGoal.SHOOT);
        conveyor.Setpoint(ConveyorGoal.SHOOT);
        Rollers.Setpoint(RollersGoal.SHOOT);
        break;
      case ZERO:
        agitator.Setpoint(AgitatorGoal.ZERO);
        kicker.Setpoint(KickerGoal.ZERO);
        conveyor.Setpoint(ConveyorGoal.ZERO);
        Rollers.Setpoint(RollersGoal.ZERO);
        break;
      case CUSTOM:
        agitator.Setpoint(AgitatorGoal.CUSTOM);
        kicker.Setpoint(KickerGoal.CUSTOM);
        conveyor.Setpoint(ConveyorGoal.CUSTOM);
        Rollers.Setpoint(RollersGoal.CUSTOM);
        break;
    }

    agitator.periodic();
    kicker.periodic();
    conveyor.periodic();
    Rollers.periodic();
  }

  public static Command getSetStateCommand(INDEXER_STATE state, Indexer indexer) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(indexer);
        indexer.setIndexerState(state);
      }

      @Override
      public boolean isFinished() {
        return indexer.currentIndexerState == state;
      }
    };
  }
}
