package frc.robot.subsystems.indexer.indexerKicker;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IndexerKicker {
  private final IndexerKickerIO io;
  private final IndexerKickerIOInputsAutoLogged inputs = new IndexerKickerIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Indexer/IndexerKicker/Gains/kP", IndexerConstants.IndexerKicker.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Indexer/IndexerKicker/Gains/kI", IndexerConstants.IndexerKicker.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Indexer/IndexerKicker/Gains/kD", IndexerConstants.IndexerKicker.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Indexer/IndexerKicker/Gains/kA", IndexerConstants.IndexerKicker.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Indexer/IndexerKicker/Gains/kV", IndexerConstants.IndexerKicker.Gains.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Indexer/IndexerKicker/Gains/kS", IndexerConstants.IndexerKicker.Gains.kS);

  public enum IndexerKickerGoal {
    ZERO(new LoggedTunableNumber("Indexer/IndexerKicker/Goals/Zero", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/IndexerKicker/Goals/Intake", 100)),
    SHOOT(new LoggedTunableNumber("Indexer/IndexerKicker/Goals/Shoot", 4000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/IndexerKicker/Goals/Outtake", -3000)),
    SPIT(new LoggedTunableNumber("Indexer/IndexerKicker/Goals/Spit", -4000)),
    CUSTOM(new LoggedTunableNumber("Indexer/IndexerKicker/Goals/Custom", 100));

    private final DoubleSupplier setpointSupplier;

    IndexerKickerGoal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getRPM() {
      return setpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/IndexerKicker/GoalSetpoint")
  private IndexerKickerGoal goalSetpoint = IndexerKickerGoal.ZERO;

  private boolean velocityClosedLoop = true;
  private double goalRPM = 0.0;
  private boolean nearGoal = false;

  public IndexerKicker(IndexerKickerIO io) {
    this.io = io;
  }

  public void setGoalSetPoint(IndexerKickerGoal goal) {
    velocityClosedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltageVolts) {
    velocityClosedLoop = false;
    io.runSetVoltage(voltageVolts);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/IndexerKicker", inputs);

    if (velocityClosedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.motorMeasuredVelocityRPM, goalRPM, IndexerConstants.NEAR_GOAL_RPM_TOLERANCE);
    Logger.recordOutput("Indexer/IndexerKicker/nearGoal", nearGoal);

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
}
