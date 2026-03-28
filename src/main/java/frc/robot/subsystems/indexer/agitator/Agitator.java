package frc.robot.subsystems.indexer.agitator;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Indexer agitator motor (velocity torque-current FOC or voltage). */
public class Agitator {
  private final AgitatorIO io;
  private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Indexer/Agitator/Gains/kP", IndexerConstants.ComponentsConstants.Agitator.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Indexer/Agitator/Gains/kI", IndexerConstants.ComponentsConstants.Agitator.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Indexer/Agitator/Gains/kD", IndexerConstants.ComponentsConstants.Agitator.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Indexer/Agitator/Gains/kA", IndexerConstants.ComponentsConstants.Agitator.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Indexer/Agitator/Gains/kV", IndexerConstants.ComponentsConstants.Agitator.Gains.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Indexer/Agitator/Gains/kS", IndexerConstants.ComponentsConstants.Agitator.Gains.kS);

  public enum AgitatorGoal {
    ZERO(new LoggedTunableNumber("Indexer/Agitator/Goals/Zero", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/Agitator/Goals/Intake", 100)),
    SHOOT(new LoggedTunableNumber("Indexer/Agitator/Goals/Shoot", 3000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/Agitator/Goals/Outtake", 0)),
    SPIT(new LoggedTunableNumber("Indexer/Agitator/Goals/Spit", 0)),
    CUSTOM(new LoggedTunableNumber("Indexer/Agitator/Goals/Custom", 100));

    private final DoubleSupplier setpointSupplier;

    AgitatorGoal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getRPM() {
      return setpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/Agitator/GoalSetpoint")
  private AgitatorGoal goalSetpoint = AgitatorGoal.ZERO;

  private boolean velocityClosedLoop = true;
  private double goalRPM = 0.0;
  private boolean nearGoal = false;

  public Agitator(AgitatorIO io) {
    this.io = io;
  }

  /** Velocity closed-loop to the RPM from {@code goal}. */
  public void setGoalSetPoint(AgitatorGoal goal) {
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
    Logger.processInputs("Indexer/Agitator", inputs);

    if (velocityClosedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.motorMeasuredVelocityRPM, goalRPM, IndexerConstants.NEAR_GOAL_RPM_TOLERANCE);
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
}
