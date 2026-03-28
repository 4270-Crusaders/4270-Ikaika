package frc.robot.subsystems.indexer.conveyor;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Indexer conveyor motor (velocity or voltage). */
public class Conveyor {
  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Indexer/Conveyor/Gains/kP", IndexerConstants.ComponentsConstants.Conveyor.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Indexer/Conveyor/Gains/kI", IndexerConstants.ComponentsConstants.Conveyor.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Indexer/Conveyor/Gains/kD", IndexerConstants.ComponentsConstants.Conveyor.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Indexer/Conveyor/Gains/kA", IndexerConstants.ComponentsConstants.Conveyor.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Indexer/Conveyor/Gains/kV", IndexerConstants.ComponentsConstants.Conveyor.Gains.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Indexer/Conveyor/Gains/kS", IndexerConstants.ComponentsConstants.Conveyor.Gains.kS);

  public enum ConveyorGoal {
    ZERO(new LoggedTunableNumber("Indexer/Conveyor/Goals/Zero", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/Conveyor/Goals/Intake", 100)),
    SHOOT(new LoggedTunableNumber("Indexer/Conveyor/Goals/Shoot", 5000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/Conveyor/Goals/Outtake", 0)),
    SPIT(new LoggedTunableNumber("Indexer/Conveyor/Goals/Spit", -5000)),
    CUSTOM(new LoggedTunableNumber("Indexer/Conveyor/Goals/Custom", 100));

    private final DoubleSupplier setpointSupplier;

    ConveyorGoal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getRPM() {
      return setpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/Conveyor/GoalSetpoint")
  private ConveyorGoal goalSetpoint = ConveyorGoal.ZERO;

  private boolean velocityClosedLoop = true;
  private double goalRPM = 0.0;
  private boolean nearGoal = false;

  public Conveyor(ConveyorIO io) {
    this.io = io;
  }

  /** Velocity closed-loop to the RPM from {@code goal}. */
  public void setGoalSetPoint(ConveyorGoal goal) {
    velocityClosedLoop = true;
    this.goalSetpoint = goal;
  }

  /** Open-loop output; disables velocity command until {@link #setGoalSetPoint}. */
  public void setManualVoltage(double voltageVolts) {
    velocityClosedLoop = false;
    io.runSetVoltage(voltageVolts);
  }

  /** Updates logging, PID refresh, and velocity/voltage command. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/Conveyor", inputs);

    if (velocityClosedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.motorMeasuredVelocityRPM, goalRPM, IndexerConstants.NEAR_GOAL_RPM_TOLERANCE);
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
}
