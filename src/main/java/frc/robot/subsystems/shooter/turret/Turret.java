package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Turret subsystem
 *
 * <p>Mirrors the structure of the Hood subsystem: provides tunable PID and motion-magic parameters
 * (via LoggedTunableNumber), maintains a desired goal angle (goalDeg), updates hardware inputs each
 * loop, logs values, and exposes simple setpoint commands that complete when the turret is near the
 * requested position.
 */
public class Turret {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  // Tunable PID/feedforward values (visible in the robot log/dashboard)
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/Turret/Gains/kP", ShooterConstants.TurretConstants.TurretkP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/Turret/Gains/kI", ShooterConstants.TurretConstants.TurretkI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/Turret/Gains/kD", ShooterConstants.TurretConstants.TurretkD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Shooter/Turret/Gains/kA", ShooterConstants.TurretConstants.TurretkA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Shooter/Turret/Gains/kV", ShooterConstants.TurretConstants.TurretMotionMagickV);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Turret/Gains/kS", 0.0);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Shooter/Turret/Gains/kG", ShooterConstants.TurretConstants.TurretkG);

  // Motion-magic tunables
  private static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber(
          "Shooter/Turret/MotionMagic/Jerk", ShooterConstants.TurretConstants.TurretMotionMagicJerk);
  private static final LoggedTunableNumber MOTION_MAGIC_ACCELERATION =
      new LoggedTunableNumber(
          "Shooter/Turret/MotionMagic/Acceleration",
          ShooterConstants.TurretConstants.TurretMotionMagicAcceleration);
  private static final LoggedTunableNumber MOTION_MAGIC_VELOCITY =
      new LoggedTunableNumber(
          "Shooter/Turret/MotionMagic/Velocity", ShooterConstants.TurretConstants.TurretMotionMagicVelocity);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kV =
      new LoggedTunableNumber(
          "Shooter/Turret/MotionMagic/ExpoKv", ShooterConstants.TurretConstants.TurretMotionMagickV);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kA =
      new LoggedTunableNumber(
          "Shooter/Turret/MotionMagic/ExpoKa", ShooterConstants.TurretConstants.TurretMotionMagickA);

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

  @AutoLogOutput(key="Shooter/Turret/GoalSetpoint") private TurretGoal goalSetpoint = TurretGoal.ZERO;

  private boolean setpointMode = true;

  private double goalDeg = 0.0;

  public boolean nearGoal = false;

  /** True when turret angular speed is low (not slewing / wrapping). */
  public boolean settled = false;

  private double wrapDeg(double orginalDeg) {
    double overlapPoint =
        ((360 + ShooterConstants.TurretConstants.TURRET_MIN_DEGREE)
                + ShooterConstants.TurretConstants.TURRET_MAX_DEGREE)
            / 2;

    if (orginalDeg > overlapPoint) {
      return 180 - orginalDeg;
    } else {
      return orginalDeg;
    }
  }

  private double getLimitedDeg(double goal) {
    if (goal
        >= ShooterConstants.TurretConstants.TURRET_MAX_DEGREE) {
      return ShooterConstants.TurretConstants.TURRET_MAX_DEGREE;
    } else if (goal
        < ShooterConstants.TurretConstants.TURRET_MIN_DEGREE) {
      return ShooterConstants.TurretConstants.TURRET_MIN_DEGREE;
    } else {
      return goal;
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

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret", inputs);

    // If any tunable changes, push updated PID/gains to the hardware io
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

    // Update motion-magic constraints when tunables change
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

    double commandedDeg = getLimitedDeg(wrapDeg(goalDeg));
    io.runSetpointDegree(commandedDeg);

    // Diagnostics
    Logger.recordOutput("Shooter/Turret/GoalDegrees", goalDeg, Degrees);
    Logger.recordOutput("Shooter/Turret/CommandedDegrees", commandedDeg, Degrees);
    nearGoal =
        EqualsUtil.epsilonEquals(
            inputs.measuredPostionDeg,
            commandedDeg,
            ShooterConstants.READY_TO_SHOOT_TURRET_DEG_TOLERANCE);

    double velDegPerSec = Math.abs(Units.radiansToDegrees(inputs.velocityRadPerSec));
    settled = velDegPerSec < ShooterConstants.READY_TO_SHOOT_TURRET_MAX_DEG_PER_SEC;
    Logger.recordOutput("Shooter/Turret/VelocityDegPerSec", velDegPerSec);
    Logger.recordOutput("Shooter/Turret/nearGoal", nearGoal);
    Logger.recordOutput("Shooter/Turret/settled", settled);
  }

  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    io.setPID(kP, kI, kD, kS, kV, kA, kG);
  }

  public double getPositionDeg() {
    // Return the most recent closed-loop set position reported by the motor controller
    return inputs.setPostionDeg;
  }
}
