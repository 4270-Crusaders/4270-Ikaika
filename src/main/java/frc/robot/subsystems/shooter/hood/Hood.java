// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.util.EqualsUtil;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {
  private final HoodIO io;

  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Shooter/Hood/Gains/kP", ShooterConstants.ComponentsConstants.Hood.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Shooter/Hood/Gains/kI", ShooterConstants.ComponentsConstants.Hood.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Shooter/Hood/Gains/kD", ShooterConstants.ComponentsConstants.Hood.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Shooter/Hood/Gains/kA", ShooterConstants.ComponentsConstants.Hood.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Shooter/Hood/Gains/kV", ShooterConstants.ComponentsConstants.Hood.Gains.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Shooter/Hood/Gains/kS", ShooterConstants.ComponentsConstants.Hood.Gains.kS);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber(
          "Shooter/Hood/Gains/kG", ShooterConstants.ComponentsConstants.Hood.Gains.kG);

  private static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber(
          "Shooter/Hood/MotionMagic/Jerk",
          ShooterConstants.ComponentsConstants.Hood.Gains.MotionMagic.jerk);
  private static final LoggedTunableNumber MOTION_MAGIC_ACCELERATION =
      new LoggedTunableNumber(
          "Shooter/Hood/MotionMagic/Acceleration",
          ShooterConstants.ComponentsConstants.Hood.Gains.MotionMagic.acceleration);
  private static final LoggedTunableNumber MOTION_MAGIC_VELOCITY =
      new LoggedTunableNumber(
          "Shooter/Hood/MotionMagic/Velocity",
          ShooterConstants.ComponentsConstants.Hood.Gains.MotionMagic.velocity);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kV =
      new LoggedTunableNumber(
          "Shooter/Hood/MotionMagic/ExpoKv",
          ShooterConstants.ComponentsConstants.Hood.Gains.MotionMagic.expo_kV);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kA =
      new LoggedTunableNumber(
          "Shooter/Hood/MotionMagic/ExpoKa",
          ShooterConstants.ComponentsConstants.Hood.Gains.MotionMagic.expo_kA);

  public Hood(HoodIO io) {
    this.io = io;
  }

  public enum HoodGoal {
    ZERO(new LoggedTunableNumber("Shooter/Hood/Goals/Zero", ShooterConstants.SHOOTER_HOOD_SETPOINT_MIN_DEG)),
    TEST(new LoggedTunableNumber("Shooter/Hood/Goals/Test", 15)),
    CUSTOM(new LoggedTunableNumber("Shooter/Hood/Goals/Custom", 9.25));

    private final DoubleSupplier HOOD_SET_POINT_SUPPLIER;

    private HoodGoal(DoubleSupplier hoodSetpointSupplier) {
      this.HOOD_SET_POINT_SUPPLIER = hoodSetpointSupplier;
    }

    public double getDegrees() {
      return this.HOOD_SET_POINT_SUPPLIER.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Shooter/Hood/GoalSetpoint")
  private HoodGoal goalSetpoint = HoodGoal.ZERO;

  private boolean setpointMode = true;

  private double goalDeg = 0.0;

  public boolean nearGoal = false;
  public boolean settled = false;
  private double lastMeasuredDeg = Double.NaN;

  public void setGoalSetPoint(double goal) {
    setpointMode = false;
    this.goalDeg =
        MathUtil.clamp(
            goal, ShooterConstants.SHOOTER_HOOD_SETPOINT_MIN_DEG, ShooterConstants.SHOOTER_HOOD_SETPOINT_MAX_DEG);
  }

  public void setGoalSetPoint(HoodGoal hoodGoal) {
    double clamped =
        MathUtil.clamp(
            hoodGoal.getDegrees(),
            ShooterConstants.SHOOTER_HOOD_SETPOINT_MIN_DEG,
            ShooterConstants.SHOOTER_HOOD_SETPOINT_MAX_DEG);
    if (clamped == hoodGoal.getDegrees()) {
      setpointMode = true;
      this.goalSetpoint = hoodGoal;
    } else {
      setpointMode = false;
      this.goalDeg = clamped;
    }
  }

  /**
   * Call when hood goal enums change after {@link #periodic()} so {@link #periodicAfterScheduler()}
   * uses the new {@code goalDeg}.
   */
  public void applySetpointForOutput() {
    if (setpointMode) {
      goalDeg = goalSetpoint.getDegrees();
    }
    goalDeg =
        MathUtil.clamp(
            goalDeg,
            ShooterConstants.SHOOTER_HOOD_SETPOINT_MIN_DEG,
            ShooterConstants.SHOOTER_HOOD_SETPOINT_MAX_DEG);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA,
        kG);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.setMotionMagicConstraints(
                MOTION_MAGIC_JERK.get(),
                MOTION_MAGIC_ACCELERATION.get(),
                MOTION_MAGIC_VELOCITY.get(),
                MOTION_MAGIC_EXPO_kA.get(),
                MOTION_MAGIC_EXPO_kV.get()),
        MOTION_MAGIC_JERK,
        MOTION_MAGIC_ACCELERATION,
        MOTION_MAGIC_VELOCITY,
        MOTION_MAGIC_EXPO_kA,
        MOTION_MAGIC_EXPO_kV);

    if (setpointMode) {
      goalDeg = goalSetpoint.getDegrees();
    }
    goalDeg =
        MathUtil.clamp(
            goalDeg,
            ShooterConstants.SHOOTER_HOOD_SETPOINT_MIN_DEG,
            ShooterConstants.SHOOTER_HOOD_SETPOINT_MAX_DEG);

    Logger.recordOutput("Shooter/Hood/GoalDegrees", goalDeg, Degrees);

    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.measuredPostionDeg,
            goalDeg,
            ShooterConstants.READY_TO_SHOOT_HOOD_DEG_TOLERANCE);
    double hoodVelDegPerSec = 0.0;
    if (Double.isFinite(lastMeasuredDeg)) {
      hoodVelDegPerSec =
          Math.abs((inputs.measuredPostionDeg - lastMeasuredDeg) / frc.robot.Constants.loopPeriodSecs);
    }
    settled = hoodVelDegPerSec < ShooterConstants.READY_TO_SHOOT_HOOD_MAX_DEG_PER_SEC;
    lastMeasuredDeg = inputs.measuredPostionDeg;
    Logger.recordOutput("Shooter/Hood/VelocityDegPerSec", hoodVelDegPerSec);
    Logger.recordOutput("Shooter/Hood/nearGoal", nearGoal);
    Logger.recordOutput("Shooter/Hood/settled", settled);
    ShooterState.getInstance()
        .recordShooterHoodMeasuredAngleRad(Units.degreesToRadians(inputs.measuredPostionDeg));
  }

  @Override
  public void periodicAfterScheduler() {
    io.runSetpointDegree(goalDeg);
  }

  public Command runTrackTargetCommand() {
    return runEnd(
        () -> {
          if (!ShooterState.getInstance().isShooterTracking()) {
            return;
          }
          ShooterCalculator.ShootingParameters p = ShooterCalculator.getInstance().getParameters();
          if (!p.isValid()) {
            return;
          }
          if (ShooterState.getInstance().isShooterTrenchProtectionActive()) {
            setGoalSetPoint(ShooterConstants.SHOOTER_HOOD_SETPOINT_MIN_DEG);
          } else {
            setGoalSetPoint(Units.radiansToDegrees(p.hoodAngle()));
          }
        },
        () -> setGoalSetPoint(HoodGoal.ZERO));
  }

  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    io.setPID(kP, kI, kD, kS, kV, kA, kG);
  }

  public double getPositionDeg() {
    return inputs.setPostionDeg;
  }

  /** Returns the latest measured hood angle in degrees. */
  public double getMeasuredAngleDeg() {
    return inputs.measuredPostionDeg;
  }
}
