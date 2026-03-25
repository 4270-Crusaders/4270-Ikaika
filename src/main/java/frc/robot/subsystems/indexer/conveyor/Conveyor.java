package frc.robot.subsystems.indexer.conveyor;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Conveyor {
  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Indexer/Conveyor/Gains/kP", IndexerConstants.ConveyorConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Indexer/Conveyor/Gains/kI", IndexerConstants.ConveyorConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Indexer/Conveyor/Gains/kD", IndexerConstants.ConveyorConstants.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Indexer/Conveyor/Gains/kA", IndexerConstants.ConveyorConstants.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Indexer/Conveyor/Gains/kV", IndexerConstants.ConveyorConstants.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Indexer/Conveyor/Gains/kS", IndexerConstants.ConveyorConstants.kS);

  public enum ConveyorGoal {
    ZERO(new LoggedTunableNumber("Indexer/Conveyor/Goals/Zero", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/Conveyor/Goals/Intake", 100)),
    SHOOT(new LoggedTunableNumber("Indexer/Conveyor/Goals/Shoot", 5000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/Conveyor/Goals/Outtake", 0)),
    SPIT(new LoggedTunableNumber("Indexer/Conveyor/Goals/Spit", -5000)),
    CUSTOM(new LoggedTunableNumber("Indexer/Conveyor/Goals/Custom", 100));

    private final DoubleSupplier ConveyorSetpointSupplier;

    private ConveyorGoal(DoubleSupplier ConveyorSetpointSupplier) {
      this.ConveyorSetpointSupplier = ConveyorSetpointSupplier;
    }

    private double getRPM() {
      return ConveyorSetpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/Conveyor/GoalSetpoint") private ConveyorGoal goalSetpoint = ConveyorGoal.ZERO;

  boolean closedLoop = true;

  private double goalRPM = 0.0;

  private boolean nearGoal = false;

  public void setGoalSetPoint(ConveyorGoal goal) {
    closedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltage) {
    closedLoop = false;
    io.runSetVoltage(voltage);
  }

  public Conveyor(ConveyorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/Conveyor", inputs);

    // motion magic setpoint code
    if (closedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal = EqualsUtil.epsilonEquals(inputs.motorMeasuredVelocityRPM, goalRPM, 5);
    Logger.recordOutput("Indexer/Conveyor/nearGoal", nearGoal);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA);
  }

  public void getSetVoltage(double goal) {
    setManualVoltage(goal);
  }
  ;

  public void Setpoint(ConveyorGoal goalSetPoint) {
    setGoalSetPoint(goalSetPoint);
  }
  ;
}
