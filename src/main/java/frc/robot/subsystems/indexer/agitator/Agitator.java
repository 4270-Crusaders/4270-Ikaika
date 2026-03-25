package frc.robot.subsystems.indexer.agitator;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Agitator {
  private final AgitatorIO io;
  private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Indexer/Agitator/kP", IndexerConstants.AgitatorConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Indexer/Agitator/kI", IndexerConstants.AgitatorConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Indexer/Agitator/kD", IndexerConstants.AgitatorConstants.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Indexer/Agitator/kA", IndexerConstants.AgitatorConstants.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Indexer/Agitator/kV", IndexerConstants.AgitatorConstants.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Indexer/Agitator/kS", IndexerConstants.AgitatorConstants.kS);

  public enum AgitatorGoal {
    ZERO(new LoggedTunableNumber("Indexer/Agitator/Goals/ZERO", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/Agitator/Goals/AGITATE", 100)),
    SHOOT(new LoggedTunableNumber("Indexer/Agitator/Goals/SHOOT", 3000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/Agitator/Goals/OUTTAKE", 0)),
    SPIT(new LoggedTunableNumber("Indexer/Agitator/Goals/SPIT", 0)),
    CUSTOM(new LoggedTunableNumber("Indexer/Agitator/Goals/CUSTOM", 100));

    private final DoubleSupplier AgitatorSetpointSupplier;

    private AgitatorGoal(DoubleSupplier AgitatorSetpointSupplier) {
      this.AgitatorSetpointSupplier = AgitatorSetpointSupplier;
    }

    private double getRPM() {
      return AgitatorSetpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/Agitator/GoalSetpoint") private AgitatorGoal goalSetpoint = AgitatorGoal.ZERO;

  private boolean closedLoop = true;

  private double goalRPM = 0.0;

  private boolean nearGoal = false;

  public void setGoalSetPoint(AgitatorGoal goal) {
    closedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltage) {
    closedLoop = false;
    io.runSetVoltage(voltage);
  }

  public Agitator(AgitatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/Agitator", inputs);

    // motion magic setpoint code
    if (closedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal = EqualsUtil.epsilonEquals(inputs.motorMeasuredVelocityRPM, goalRPM, 5);
    Logger.recordOutput("Indexer/Agitator/nearGoal", nearGoal);

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

  public void Setpoint(AgitatorGoal goalSetPoint) {
    setGoalSetPoint(goalSetPoint);
  }
  ;
}
