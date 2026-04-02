// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.indexer.indexerAgitator;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IndexerAgitator {
  private final IndexerAgitatorIO io;
  private final IndexerAgitatorIOInputsAutoLogged inputs = new IndexerAgitatorIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Indexer/IndexerAgitator/Gains/kP", IndexerConstants.IndexerAgitator.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Indexer/IndexerAgitator/Gains/kI", IndexerConstants.IndexerAgitator.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Indexer/IndexerAgitator/Gains/kD", IndexerConstants.IndexerAgitator.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Indexer/IndexerAgitator/Gains/kA", IndexerConstants.IndexerAgitator.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Indexer/IndexerAgitator/Gains/kV", IndexerConstants.IndexerAgitator.Gains.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Indexer/IndexerAgitator/Gains/kS", IndexerConstants.IndexerAgitator.Gains.kS);

  public enum IndexerAgitatorGoal {
    ZERO(new LoggedTunableNumber("Indexer/IndexerAgitator/Goals/Zero", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/IndexerAgitator/Goals/Intake", 600)),
    SHOOT(new LoggedTunableNumber("Indexer/IndexerAgitator/Goals/Shoot", 3000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/IndexerAgitator/Goals/Outtake", 0)),
    SPIT(new LoggedTunableNumber("Indexer/IndexerAgitator/Goals/Spit", 0)),
    CUSTOM(new LoggedTunableNumber("Indexer/IndexerAgitator/Goals/Custom", 100));

    private final DoubleSupplier setpointSupplier;

    IndexerAgitatorGoal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getRPM() {
      return setpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/IndexerAgitator/GoalSetpoint")
  private IndexerAgitatorGoal goalSetpoint = IndexerAgitatorGoal.ZERO;

  private boolean velocityClosedLoop = true;
  private double goalRPM = 0.0;
  private boolean nearGoal = false;

  public IndexerAgitator(IndexerAgitatorIO io) {
    this.io = io;
  }

  public void setGoalSetPoint(IndexerAgitatorGoal goal) {
    velocityClosedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltageVolts) {
    velocityClosedLoop = false;
    io.runSetVoltage(voltageVolts);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/IndexerAgitator", inputs);

    if (velocityClosedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.motorMeasuredVelocityRPM, goalRPM, IndexerConstants.NEAR_GOAL_RPM_TOLERANCE);
    Logger.recordOutput("Indexer/IndexerAgitator/nearGoal", nearGoal);

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
