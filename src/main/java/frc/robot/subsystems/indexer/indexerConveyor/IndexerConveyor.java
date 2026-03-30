// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.indexer.indexerConveyor;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IndexerConveyor {
  private final IndexerConveyorIO io;
  private final IndexerConveyorIOInputsAutoLogged inputs = new IndexerConveyorIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Indexer/IndexerConveyor/Gains/kP", IndexerConstants.IndexerConveyor.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Indexer/IndexerConveyor/Gains/kI", IndexerConstants.IndexerConveyor.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Indexer/IndexerConveyor/Gains/kD", IndexerConstants.IndexerConveyor.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Indexer/IndexerConveyor/Gains/kA", IndexerConstants.IndexerConveyor.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Indexer/IndexerConveyor/Gains/kV", IndexerConstants.IndexerConveyor.Gains.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Indexer/IndexerConveyor/Gains/kS", IndexerConstants.IndexerConveyor.Gains.kS);

  public enum IndexerConveyorGoal {
    ZERO(new LoggedTunableNumber("Indexer/IndexerConveyor/Goals/Zero", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/IndexerConveyor/Goals/Intake", 100)),
    SHOOT(new LoggedTunableNumber("Indexer/IndexerConveyor/Goals/Shoot", 5000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/IndexerConveyor/Goals/Outtake", 0)),
    SPIT(new LoggedTunableNumber("Indexer/IndexerConveyor/Goals/Spit", -5000)),
    CUSTOM(new LoggedTunableNumber("Indexer/IndexerConveyor/Goals/Custom", 100));

    private final DoubleSupplier setpointSupplier;

    IndexerConveyorGoal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getRPM() {
      return setpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/IndexerConveyor/GoalSetpoint")
  private IndexerConveyorGoal goalSetpoint = IndexerConveyorGoal.ZERO;

  private boolean velocityClosedLoop = true;
  private double goalRPM = 0.0;
  private boolean nearGoal = false;

  public IndexerConveyor(IndexerConveyorIO io) {
    this.io = io;
  }

  public void setGoalSetPoint(IndexerConveyorGoal goal) {
    velocityClosedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltageVolts) {
    velocityClosedLoop = false;
    io.runSetVoltage(voltageVolts);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/IndexerConveyor", inputs);

    if (velocityClosedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.motorMeasuredVelocityRPM, goalRPM, IndexerConstants.NEAR_GOAL_RPM_TOLERANCE);
    Logger.recordOutput("Indexer/IndexerConveyor/nearGoal", nearGoal);

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
