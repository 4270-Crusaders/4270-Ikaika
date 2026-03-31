// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterCalculator;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterState;
import frc.robot.util.EqualsUtil;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Shooter/Turret/Gains/kP", ShooterConstants.ComponentsConstants.Turret.Gains.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Shooter/Turret/Gains/kI", ShooterConstants.ComponentsConstants.Turret.Gains.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Shooter/Turret/Gains/kD", ShooterConstants.ComponentsConstants.Turret.Gains.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Shooter/Turret/Gains/kA", ShooterConstants.ComponentsConstants.Turret.Gains.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Shooter/Turret/Gains/kV", ShooterConstants.ComponentsConstants.Turret.Gains.kV);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Turret/Gains/kS", 0.0);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber(
          "Shooter/Turret/Gains/kG", ShooterConstants.ComponentsConstants.Turret.Gains.kG);

  private static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber(
          "Shooter/Turret/MotionMagic/Jerk",
          ShooterConstants.ComponentsConstants.Turret.Gains.MotionMagic.jerk);
  private static final LoggedTunableNumber MOTION_MAGIC_ACCELERATION =
      new LoggedTunableNumber(
          "Shooter/Turret/MotionMagic/Acceleration",
          ShooterConstants.ComponentsConstants.Turret.Gains.MotionMagic.acceleration);
  private static final LoggedTunableNumber MOTION_MAGIC_VELOCITY =
      new LoggedTunableNumber(
          "Shooter/Turret/MotionMagic/Velocity",
          ShooterConstants.ComponentsConstants.Turret.Gains.MotionMagic.velocity);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kV =
      new LoggedTunableNumber(
          "Shooter/Turret/MotionMagic/ExpoKv",
          ShooterConstants.ComponentsConstants.Turret.Gains.MotionMagic.expo_kV);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kA =
      new LoggedTunableNumber(
          "Shooter/Turret/MotionMagic/ExpoKa",
          ShooterConstants.ComponentsConstants.Turret.Gains.MotionMagic.expo_kA);

  public Turret(TurretIO io) {
    this.io = io;
  }

  public enum TurretGoal {
    ZERO(new LoggedTunableNumber("Shooter/Turret/Goals/Zero", 0.0)),
    CUSTOM(new LoggedTunableNumber("Shooter/Turret/Goals/Custom", 0.0));

    private final DoubleSupplier TURRET_DOUBLE_SUPPLIER;

    private TurretGoal(DoubleSupplier turretSetpointSupplier) {
      this.TURRET_DOUBLE_SUPPLIER = turretSetpointSupplier;
    }

    private double getDegrees() {
      return this.TURRET_DOUBLE_SUPPLIER.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Shooter/Turret/GoalSetpoint")
  private TurretGoal goalSetpoint = TurretGoal.ZERO;

  private boolean setpointMode = true;

  private double goalDeg = 0.0;
  /** Commanded position after soft limits (sent to hardware in {@link #periodicAfterScheduler}). */
  private double commandedDeg = 0.0;

  public boolean nearGoal = false;

  /** True when turret angular speed is low (not slewing / wrapping). */
  public boolean settled = false;

  private static double wrapDeg(double originalDeg) {
    double minDeg = ShooterConstants.ComponentsConstants.Turret.TURRET_MIN_DEGREE;
    double maxDeg = ShooterConstants.ComponentsConstants.Turret.TURRET_MAX_DEGREE;
    double overlapPoint = ((360.0 + minDeg) + maxDeg) / 2.0;
    if (originalDeg > overlapPoint) {
      return 180.0 - originalDeg;
    } else {
      return originalDeg;
    }
  }

  private static double getLimitedDeg(double goalDeg) {
    double minDeg = ShooterConstants.ComponentsConstants.Turret.TURRET_MIN_DEGREE;
    double maxDeg = ShooterConstants.ComponentsConstants.Turret.TURRET_MAX_DEGREE;
    if (goalDeg >= maxDeg) {
      return maxDeg;
    } else if (goalDeg < minDeg) {
      return minDeg;
    } else {
      return goalDeg;
    }
  }

  public void setGoalSetPoint(double goal) {
    setpointMode = false;
    this.goalDeg = goal;
  }

  public void setGoalSetPoint(TurretGoal goal) {
    setpointMode = true;
    this.goalSetpoint = goal;
  }

  /**
   * Call when turret goal enums change after {@link #periodic()} so {@link #periodicAfterScheduler()}
   * uses the new {@code commandedDeg}.
   */
  public void applySetpointForOutput() {
    if (setpointMode) {
      goalDeg = goalSetpoint.getDegrees();
    }
    commandedDeg = getLimitedDeg(wrapDeg(goalDeg));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret", inputs);

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

    commandedDeg = getLimitedDeg(wrapDeg(goalDeg));

    Logger.recordOutput("Shooter/Turret/GoalDegreesRaw", goalDeg, Degrees);
    Logger.recordOutput("Shooter/Turret/CommandedDegrees", commandedDeg, Degrees);
    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.measuredPostionDeg,
            commandedDeg,
            ShooterConstants.READY_TO_SHOOT_TURRET_DEG_TOLERANCE);

    double velDegPerSec = Math.abs(Units.radiansToDegrees(inputs.velocityRadPerSec));
    settled = velDegPerSec < ShooterConstants.READY_TO_SHOOT_TURRET_MAX_DEG_PER_SEC;

    Logger.recordOutput("Shooter/Turret/Velocity", velDegPerSec, DegreesPerSecond);
    Logger.recordOutput("Shooter/Turret/NearGoal", nearGoal);
    Logger.recordOutput("Shooter/Turret/Settled", settled);

    if (DriverStation.isEnabled()) {
      RobotState.getInstance()
          .addTurretObservation(
              new RobotState.TurretObservation(
                  Timer.getFPGATimestamp(), Rotation2d.fromDegrees(inputs.measuredPostionDeg)));
    }
  }

  @Override
  public void periodicAfterScheduler() {
    io.runSetpointDegree(commandedDeg);
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
            setGoalSetPoint(0.0);
          } else {
            Rotation2d robotAngle = RobotState.getInstance().getEstimatedPose().getRotation();
            setGoalSetPoint(robotAngle.minus(p.turretAngle()).getDegrees());
          }
        },
        () -> setGoalSetPoint(TurretGoal.ZERO));
  }

  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    io.setPID(kP, kI, kD, kS, kV, kA, kG);
  }

  public double getPositionDeg() {
    return inputs.setPostionDeg;
  }
}
