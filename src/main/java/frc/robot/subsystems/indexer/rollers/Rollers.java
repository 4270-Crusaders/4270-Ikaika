package frc.robot.subsystems.indexer.rollers;

import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Indexer rollers: velocity closed-loop or open-loop voltage via {@link RollersIO}. */
public class Rollers {
  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Indexer/Rollers/Gains/kP", IndexerConstants.ComponentsConstants.Rollers.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Indexer/Rollers/Gains/kI", IndexerConstants.ComponentsConstants.Rollers.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Indexer/Rollers/Gains/kD", IndexerConstants.ComponentsConstants.Rollers.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Indexer/Rollers/Gains/kA", IndexerConstants.ComponentsConstants.Rollers.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Indexer/Rollers/Gains/kV", IndexerConstants.ComponentsConstants.Rollers.Gains.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Indexer/Rollers/Gains/kS", IndexerConstants.ComponentsConstants.Rollers.Gains.kS);

  public enum RollersGoal {
    ZERO(new LoggedTunableNumber("Indexer/Rollers/Goals/Zero", 0)),
    INTAKE(new LoggedTunableNumber("Indexer/Rollers/Goals/Intake", 1500)),
    SHOOT(new LoggedTunableNumber("Indexer/Rollers/Goals/Shoot", 4000)),
    OUTTAKE(new LoggedTunableNumber("Indexer/Rollers/Goals/Outtake", -1500)),
    SPIT(new LoggedTunableNumber("Indexer/Rollers/Goals/Spit", 2000)),
    CUSTOM(new LoggedTunableNumber("Indexer/Rollers/Goals/Custom", 100));

    private final DoubleSupplier setpointSupplier;

    RollersGoal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getRPM() {
      return setpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Indexer/Rollers/GoalSetpoint")
  private RollersGoal goalSetpoint = RollersGoal.ZERO;

  private boolean velocityClosedLoop = true;
  private double goalRPM = 0.0;
  private boolean nearGoal = false;

  public Rollers(RollersIO io) {
    this.io = io;
  }

  /** Velocity closed-loop to the RPM from {@code goal}. */
  public void setGoalSetPoint(RollersGoal goal) {
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
    Logger.processInputs("Indexer/Rollers", inputs);

    if (velocityClosedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.motorMeasuredVelocityRPM, goalRPM, IndexerConstants.NEAR_GOAL_RPM_TOLERANCE);
    Logger.recordOutput("Indexer/Rollers/nearGoal", nearGoal);

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
