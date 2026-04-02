// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.util.EqualsUtil;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SpeedUtil;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  // TODO Retune based on ratio change
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Shooter/Flywheel/Gains/kP", ShooterConstants.ComponentsConstants.Flywheel.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Shooter/Flywheel/Gains/kI", ShooterConstants.ComponentsConstants.Flywheel.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Shooter/Flywheel/Gains/kD", ShooterConstants.ComponentsConstants.Flywheel.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Shooter/Flywheel/Gains/kA", ShooterConstants.ComponentsConstants.Flywheel.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Shooter/Flywheel/Gains/kV", ShooterConstants.ComponentsConstants.Flywheel.Gains.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Shooter/Flywheel/Gains/kS", ShooterConstants.ComponentsConstants.Flywheel.Gains.kS);

  /** Base half-width of the RPM window around goal for {@link #nearGoal}. */
  private static final LoggedTunableNumber nearGoalRpmTolerance =
      new LoggedTunableNumber(
          "Shooter/Flywheel/Ready/NearGoalRpmTolerance",
          ShooterConstants.READY_TO_SHOOT_FLYWHEEL_RPM_TOLERANCE);

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public enum FlyWheelGoal {
    ZERO(new LoggedTunableNumber("Shooter/Flywheel/Goals/Zero", 0.0)),
    IDLE(
        new LoggedTunableNumber(
            "Shooter/Flywheel/Goals/Idle",
            ShooterConstants.ComponentsConstants.Flywheel.ShotConstants.FLYWHEEL_GOAL_IDLE_RPM)),
    CUSTOM(
        new LoggedTunableNumber(
            "Shooter/Flywheel/Goals/Custom",
            2000));

    private final DoubleSupplier SHOOTER_SET_POINT_SUPPLIER;

    private FlyWheelGoal(DoubleSupplier shooterSetpointSupplier) {
      this.SHOOTER_SET_POINT_SUPPLIER = shooterSetpointSupplier;
    }

    private double getRPM() {
      return SHOOTER_SET_POINT_SUPPLIER.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Shooter/Flywheel/GoalSetpoint")
  private FlyWheelGoal goalSetpoint = FlyWheelGoal.ZERO;

  private boolean setpointMode = true;

  private double goalRPM = 0.0;
  public boolean nearGoal = false;

  public void setGoalSetPoint(double goalRPM) {
    setpointMode = false;
    this.goalRPM =
        MathUtil.clamp(goalRPM, 0.0, ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
  }

  public void setGoalSetPoint(FlyWheelGoal setpoint) {
    setpointMode = true;
    this.goalSetpoint = setpoint;
  }

  /**
   * Call when setpoint enums change after {@link #periodic()} (e.g. from {@link
   * ShooterCalculator#coordinateAfterScheduler}) so {@link #periodicAfterScheduler()} uses the new
   * {@code goalRPM}.
   */
  public void applySetpointForOutput() {
    if (setpointMode) {
      goalRPM =
          MathUtil.clamp(
              goalSetpoint.getRPM(),
              0.0,
              ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA);

    if (setpointMode) {
      goalRPM =
          MathUtil.clamp(
              goalSetpoint.getRPM(),
              0.0,
              ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
    }

    double toleranceBase = nearGoalRpmTolerance.get();
    double measuredRpm =
        0.5 * (inputs.motorMeasuredVelocityRpm[0] + inputs.motorMeasuredVelocityRpm[1]);

    nearGoal =
        EqualsUtil.epsilonEquals(
            measuredRpm,
            goalRPM,
            toleranceBase);
    if (ShooterConstants.Logging.LOG_SHOOTER_MECHANISM_TELEM) {
      Logger.recordOutput("Shooter/Flywheel/GoalRPM", goalRPM, RPM);
      Logger.recordOutput("Shooter/Flywheel/Ready/NearGoalRpmTolerance", toleranceBase);
      Logger.recordOutput("Shooter/Flywheel/NearGoal", nearGoal);
      Logger.recordOutput("Shooter/Flywheel/MeasuredRpm", measuredRpm);
    }
    ShooterState state = ShooterState.getInstance();
    state.recordShooterFlywheelSurfaceSpeedMps(getAverageWheelSurfaceVelocityMetersPerSec());
    state.recordShooterFlywheelTopBottomSurfaceDeltaMps(
        getHoodWheelSurfaceVelocityMetersPerSec() - getMainWheelSurfaceVelocityMetersPerSec());
  }

  @Override
  public void periodicAfterScheduler() {
    io.runSetVelocity(goalRPM / 60.0);
  }

  /**
   * MA-style: each loop while the command runs, apply {@link ShooterCalculator#getParameters()} (after
   * {@link ShooterCalculator#coordinateAfterScheduler} refreshes the calculator).
   */
  public Command runTrackTargetCommand() {
    return runEnd(
        () -> {
          if (!ShooterState.getInstance().isShooterTracking()) {
            return;
          }
          ShooterCalculator.ShootingParameters p = ShooterCalculator.getInstance().getParameters();
          if (p.isValid()) {
            setGoalSetPoint(p.flywheelSpeed());
          }
        },
        () -> setGoalSetPoint(FlyWheelGoal.ZERO));
  }

  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    io.setPID(kP, kI, kD, kS, kV, kA);
  }

  public double getMotorVelocityRPM() {
    return 0.5 * (inputs.motorMeasuredVelocityRpm[0] + inputs.motorMeasuredVelocityRpm[1]);
  }

  public double getMainFlyWheelVelocityRPM() {
    return inputs.mainFlywheelRpm;
  }

  public double getHoodFlyWheelVelocityRPM() {
    return inputs.hoodFlywheelRpm;
  }

  /** Average top/bottom wheel RPM in wheel space (after reductions). */
  public double getAverageWheelVelocityRPM() {
    return 0.5 * (getMainFlyWheelVelocityRPM() + getHoodFlyWheelVelocityRPM());
  }

  /** Main (typically lower) wheel surface speed at the ball, meters per second. */
  public double getMainWheelSurfaceVelocityMetersPerSec() {
    return (getMainFlyWheelVelocityRPM() / 60.0)
        * Math.PI
        * ShooterConstants.ComponentsConstants.Flywheel.MAIN_WHEEL_DIAMETER_METERS;
  }

  /** Hood (typically upper) wheel surface speed at the ball, meters per second. */
  public double getHoodWheelSurfaceVelocityMetersPerSec() {
    return (getHoodFlyWheelVelocityRPM() / 60.0)
        * Math.PI
        * ShooterConstants.ComponentsConstants.Flywheel.HOOD_WHEEL_DIAMETER_METERS;
  }

  /** Average top/bottom wheel surface speed in meters per second. */
  public double getAverageWheelSurfaceVelocityMetersPerSec() {
    return 0.5 * (getMainWheelSurfaceVelocityMetersPerSec() + getHoodWheelSurfaceVelocityMetersPerSec());
  }

  /**
   * Returns measured velocity as motor-equivalent RPM using top/bottom wheel velocities.
   *
   * <p>This averages wheel-surface speed from both wheels, then converts to the same RPM domain
   * expected by {@link ShooterCalculator}.
   */
  public double getAverageWheelVelocityMotorEquivalentRPM() {
    return SpeedUtil.rpmFromMetersPerSecond(getAverageWheelSurfaceVelocityMetersPerSec(),ShooterConstants.ComponentsConstants.Flywheel.AVERAGE_WHEEL_RADIUS_METERS);
  }
}
