package frc.robot.subsystems.indexer.indexerRollers;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IndexerRollers {
  private final IndexerRollersIO io;
  private final IndexerRollersIOInputsAutoLogged inputs = new IndexerRollersIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Indexer/IndexerRollers/Gains/kP", IndexerConstants.IndexerRollers.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Indexer/IndexerRollers/Gains/kI", IndexerConstants.IndexerRollers.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Indexer/IndexerRollers/Gains/kD", IndexerConstants.IndexerRollers.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Indexer/IndexerRollers/Gains/kA", IndexerConstants.IndexerRollers.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Indexer/IndexerRollers/Gains/kV", IndexerConstants.IndexerRollers.Gains.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Indexer/IndexerRollers/Gains/kS", IndexerConstants.IndexerRollers.Gains.kS);

  public enum IndexerRollersGoal {
    ZERO(new LoggedTunableNumber("Indexer/IndexerRollers/Goals/Zero", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/IndexerRollers/Goals/Intake", 1500)),
    SHOOT(new LoggedTunableNumber("Indexer/IndexerRollers/Goals/Shoot", 4000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/IndexerRollers/Goals/Outtake", -1500)),
    SPIT(new LoggedTunableNumber("Indexer/IndexerRollers/Goals/Spit", 2000)),
    CUSTOM(new LoggedTunableNumber("Indexer/IndexerRollers/Goals/Custom", 100));

    private final DoubleSupplier setpointSupplier;

    IndexerRollersGoal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getRPM() {
      return setpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/IndexerRollers/GoalSetpoint")
  private IndexerRollersGoal goalSetpoint = IndexerRollersGoal.ZERO;

  private boolean velocityClosedLoop = true;
  private double goalRPM = 0.0;
  private boolean nearGoal = false;

  public IndexerRollers(IndexerRollersIO io) {
    this.io = io;
  }

  public void setGoalSetPoint(IndexerRollersGoal goal) {
    velocityClosedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltageVolts) {
    velocityClosedLoop = false;
    io.runSetVoltage(voltageVolts);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/IndexerRollers", inputs);

    if (velocityClosedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.motorMeasuredVelocityRPM, goalRPM, IndexerConstants.NEAR_GOAL_RPM_TOLERANCE);
    Logger.recordOutput("Indexer/IndexerRollers/nearGoal", nearGoal);

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
