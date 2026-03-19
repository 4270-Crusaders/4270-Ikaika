package frc.robot.subsystems.indexer.kicker;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Kicker {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Indexer/Kicker/kP", IndexerConstants.KickerConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Indexer/Kicker/kI", IndexerConstants.KickerConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Indexer/Kicker/kD", IndexerConstants.KickerConstants.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Indexer/Kicker/kA", IndexerConstants.KickerConstants.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Indexer/Kicker/kV", IndexerConstants.KickerConstants.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Indexer/Kicker/kS", IndexerConstants.KickerConstants.kS);

  public enum KickerGoal {
    ZERO(new LoggedTunableNumber("Indexer/Kicker/Goals/ZERO", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/Kicker/Goals/AGITATE", 100)),
    SHOOT(new LoggedTunableNumber("Indexer/Kicker/Goals/SHOOT", 4500)),
    OUTTAKE(new LoggedTunableNumber("Indexer/Kicker/Goals/OUTTAKE", -1500)),
    SPIT(new LoggedTunableNumber("Indexer/Kicker/Goals/SPIT", -4000)),
    CUSTOM(new LoggedTunableNumber("Indexer/Kicker/Goals/CUSTOM", 100));

    private final DoubleSupplier KickerSetpointSupplier;

    private KickerGoal(DoubleSupplier KickerSetpointSupplier) {
      this.KickerSetpointSupplier = KickerSetpointSupplier;
    }

    private double getRPM() {
      return KickerSetpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/Kicker/GoalSetpoint") private KickerGoal goalSetpoint = KickerGoal.ZERO;

  boolean closedLoop = true;

  private double goalRPM = 0.0;

  private boolean nearGoal = false;

  public void setGoalSetPoint(KickerGoal goal) {
    closedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltage) {
    closedLoop = false;
    io.runSetVoltage(voltage);
  }

  public Kicker(KickerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/Kicker", inputs);

    // motion magic setpoint code
    if (closedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal = EqualsUtil.epsilonEquals(inputs.motorMeasuredVelocityRPM, goalRPM, 5);
    Logger.recordOutput("Indexer/Kicker/nearGoal", nearGoal);

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

  public void Setpoint(KickerGoal goalSetPoint) {
    setGoalSetPoint(goalSetPoint);
  }
  ;
}
