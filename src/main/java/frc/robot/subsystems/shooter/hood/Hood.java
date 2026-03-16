package frc.robot.subsystems.shooter.hood;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood {
  /**
   * Hood subsystem
   *
   * <p>This class wraps a hardware-specific io implementation (HoodIO) and provides higher-level
   * behaviors for commanding the hood angle. It: - Holds tunable PID and motion magic parameters
   * (LoggedTunableNumber) - Tracks a desired goal angle (goalDeg) and whether the mechanism is near
   * that goal (nearGoal) - Updates hardware each periodic loop and logs inputs/outputs via
   * Littleton Robotics' Logger utilities.
   *
   * <p>The subsystem exposes commands (getSetpointCommand) which set the desired goal and finish
   * when the hood reaches the target.
   */
  private final HoodIO io;

  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/Hood/kP", ShooterConstants.HoodConstants.HoodkP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/Hood/kI", ShooterConstants.HoodConstants.HoodkI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/Hood/kD", ShooterConstants.HoodConstants.HoodkD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Shooter/Hood/kA", ShooterConstants.HoodConstants.HoodkA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/Hood/kV", ShooterConstants.HoodConstants.HoodkV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/Hood/kS", ShooterConstants.HoodConstants.HoodkS);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Shooter/Hood/kG", ShooterConstants.HoodConstants.HoodkG);

  private static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber(
          "Shooter/Hood/MMJerk", ShooterConstants.HoodConstants.HoodMotionMagicJerk);
  private static final LoggedTunableNumber MOTION_MAGIC_ACCELERATION =
      new LoggedTunableNumber(
          "Shooter/Hood/MMAcceleration",
          ShooterConstants.HoodConstants.HoodMotionMagicAcceleration);
  private static final LoggedTunableNumber MOTION_MAGIC_VELOCITY =
      new LoggedTunableNumber(
          "Shooter/Hood/MMVelocity", ShooterConstants.HoodConstants.HoodMotionMagicVelocity);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kV =
      new LoggedTunableNumber(
          "Shooter/Hood/MMExpokV", ShooterConstants.HoodConstants.HoodMotionMagickV);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kA =
      new LoggedTunableNumber(
          "Shooter/Hood/MMExpokA", ShooterConstants.HoodConstants.HoodMotionMagickA);

  public Hood(HoodIO io) {
    this.io = io;
  }

  public enum HoodGoal {
    ZERO(new LoggedTunableNumber("Shooter/Hood/Goals/Zero", 0)),
    TEST(new LoggedTunableNumber("Shooter/Hood/Goals/TEST", 15)),
    CUSTOM(new LoggedTunableNumber("Shooter/Hood/Goals/CUSTOM", 25));

    private final DoubleSupplier HOOD_SET_POINT_SUPPLIER;

    private HoodGoal(DoubleSupplier hoodSetpointSupplier) {
      this.HOOD_SET_POINT_SUPPLIER = hoodSetpointSupplier;
    }

    private double getDegrees() {
      return this.HOOD_SET_POINT_SUPPLIER.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Shooter/Hood/GoalSetpoint") private HoodGoal goalSetpoint = HoodGoal.ZERO;

  private boolean setpointMode = true;

  private double goalDeg = 0.0;

  public boolean nearGoal = false;

  public void setGoalSetPoint(double goal) {
    if (goal < 25) {
      setpointMode = false;
      this.goalDeg = goal;
    } else {
      setpointMode = false;
      this.goalDeg = 25;
    }
  }

  public void setGoalSetPoint(HoodGoal goal) {
    if (goal.getDegrees() < 25) {
      setpointMode = true;
      this.goalSetpoint = goal;
    } else {
      setpointMode = true;
      this.goalDeg = 25;
    }
  }

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

    io.runSetpointDegree(goalDeg);

    // Record the active goal for diagnostics and dashboards
    Logger.recordOutput("Shooter/Hood/GoalDegrees", goalDeg, Degrees);

    // Determine if the hood is within a small tolerance of the goal.
    // EqualsUtil.epsilonEquals compares two doubles within the given
    // epsilon (1 degree here). This flag is used by setpoint commands
    // to indicate completion.
    nearGoal = EqualsUtil.epsilonEquals(inputs.measuredPostionDeg, goalDeg, 1);
    Logger.recordOutput("Shooter/Hood/nearGoal", nearGoal);
  }

  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    io.setPID(kP, kI, kD, kS, kV, kA, kG);
  }

  public double getPositionDeg() {
    // Expose the most recently-read set position (from motor closed-loop)
    // Note: this returns the value stored in inputs, which is refreshed in
    // periodic() by calling io.updateInputs(...).
    return inputs.setPostionDeg;
  }
}
