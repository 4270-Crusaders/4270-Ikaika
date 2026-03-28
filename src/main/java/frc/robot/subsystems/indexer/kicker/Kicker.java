package frc.robot.subsystems.indexer.kicker;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Indexer kicker motor (velocity or voltage). */
public class Kicker {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Indexer/Kicker/Gains/kP", IndexerConstants.ComponentsConstants.Kicker.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Indexer/Kicker/Gains/kI", IndexerConstants.ComponentsConstants.Kicker.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Indexer/Kicker/Gains/kD", IndexerConstants.ComponentsConstants.Kicker.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Indexer/Kicker/Gains/kA", IndexerConstants.ComponentsConstants.Kicker.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Indexer/Kicker/Gains/kV", IndexerConstants.ComponentsConstants.Kicker.Gains.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Indexer/Kicker/Gains/kS", IndexerConstants.ComponentsConstants.Kicker.Gains.kS);

  public enum KickerGoal {
    ZERO(new LoggedTunableNumber("Indexer/Kicker/Goals/Zero", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/Kicker/Goals/Intake", 100)),
    SHOOT(new LoggedTunableNumber("Indexer/Kicker/Goals/Shoot", 4000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/Kicker/Goals/Outtake", -3000)),
    SPIT(new LoggedTunableNumber("Indexer/Kicker/Goals/Spit", -4000)),
    CUSTOM(new LoggedTunableNumber("Indexer/Kicker/Goals/Custom", 100));

    private final DoubleSupplier setpointSupplier;

    KickerGoal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getRPM() {
      return setpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/Kicker/GoalSetpoint")
  private KickerGoal goalSetpoint = KickerGoal.ZERO;

  private boolean velocityClosedLoop = true;
  private double goalRPM = 0.0;
  private boolean nearGoal = false;

  public Kicker(KickerIO io) {
    this.io = io;
  }

  /** Velocity closed-loop to the RPM from {@code goal}. */
  public void setGoalSetPoint(KickerGoal goal) {
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
    Logger.processInputs("Indexer/Kicker", inputs);

    if (velocityClosedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.motorMeasuredVelocityRPM, goalRPM, IndexerConstants.NEAR_GOAL_RPM_TOLERANCE);
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
}
