package frc.robot.subsystems.shooter.flywheel;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/FlyWheel/kP", ShooterConstants.FlywheelConstants.FlyWheelkP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/FlyWheel/kI", ShooterConstants.FlywheelConstants.FlyWheelkI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/FlyWheel/kD", ShooterConstants.FlywheelConstants.FlyWheelkD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Shooter/FlyWheel/kA", ShooterConstants.FlywheelConstants.FlyWheelkA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/FlyWheel/kV", ShooterConstants.FlywheelConstants.FlyWheelkV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/FlyWheel/kS", ShooterConstants.FlywheelConstants.FlyWheelkS);

  private static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber(
          "Shooter/FlyWheel/MMJerk", ShooterConstants.FlywheelConstants.FlyWheelMotionMagicJerk);
  private static final LoggedTunableNumber MOTION_MAGIC_ACCELERATION =
      new LoggedTunableNumber(
          "Shooter/FlyWheel/MMAcceleration",
          ShooterConstants.FlywheelConstants.FlyWheelMotionMagicAcceleration);
  private static final LoggedTunableNumber MOTION_MAGIC_VELOCITY =
      new LoggedTunableNumber(
          "Shooter/FlyWheel/MMVelocity",
          ShooterConstants.FlywheelConstants.FlyWheelMotionMagicVelocity);

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public enum FlyWheelGoal {
    ZERO(new LoggedTunableNumber("Shooter/FlyWheel/Goals/Zero", 0.0)),
    AUTOIDLE(new LoggedTunableNumber("Shooter/FlyWheel/Goals/Idle", 3000)),
    TELEIDLE(new LoggedTunableNumber("Shooter/FlyWheel/Goals/Idle", 2500)),
    SHOOTING(
        new LoggedTunableNumber("Shooter/FlyWheel/Goals/Shooting", 3000)), // TODO: Tune this value
    PASSNEUTRAL(
        new LoggedTunableNumber(
            "Shooter/FlyWheel/Goals/PassNeutral", 4000)), // TODO: Tune this value
    PASSOPPONENT(
        new LoggedTunableNumber("Shooter/FlyWheel/Goals/pass", 5000)), // TODO: Tune this value
    CUSTOM(new LoggedTunableNumber("Shooter/FlyWheel/Goals/custom", 80)); // TODO: Tune this value

    private final DoubleSupplier SHOOTER_SET_POINT_SUPPLIER;

    private FlyWheelGoal(DoubleSupplier shooterSetpointSupplier) {
      this.SHOOTER_SET_POINT_SUPPLIER = shooterSetpointSupplier;
    }

    private double getRPM() {
      return SHOOTER_SET_POINT_SUPPLIER.getAsDouble();
    }
  }

  @AutoLogOutput private FlyWheelGoal goalSetpoint = FlyWheelGoal.ZERO;

  private boolean setpointMode = true;

  private double goalRPM = 0.0;

  public boolean nearGoal = false;

  public void setGoalSetPoint(double goalRPM) {
    setpointMode = false;
    this.goalRPM = goalRPM;
  }

  public void setGoalSetPoint(FlyWheelGoal setpoint) {
    setpointMode = true;
    this.goalSetpoint = setpoint;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/FlyWheel", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.setMotionMagicConstraints(
                MOTION_MAGIC_JERK.get(),
                MOTION_MAGIC_ACCELERATION.get(),
                MOTION_MAGIC_VELOCITY.get()),
        MOTION_MAGIC_JERK,
        MOTION_MAGIC_ACCELERATION,
        MOTION_MAGIC_VELOCITY);

    if (setpointMode) {
      goalRPM = goalSetpoint.getRPM();
    }

    io.runSetVelocity(goalRPM / 60);

    Logger.recordOutput("Shooter/FlyWheel/GoalRPM", goalRPM, RadiansPerSecond);
    nearGoal =
        EqualsUtil.epsilonEquals(
            (inputs.motorMeasuredVelocityRPM[0] + inputs.motorMeasuredVelocityRPM[1]) / 2,
            goalRPM,
            150);
    Logger.recordOutput("Shooter/FlyWheel/nearGoal", nearGoal);
  }

  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    io.setPID(kP, kI, kD, kS, kV, kA);
  }

  public double getMotorVelocityRPM() {
    return (inputs.motorMeasuredVelocityRPM[0] + inputs.motorMeasuredVelocityRPM[1]) / 2;
  }

  public double getMainFlyWheelVelocityRPM() {
    return inputs.MainFlyWheelRPM;
  }

  public double getHoodFlyWheelVelocityRPM() {
    return inputs.HoodFlyWheelRPM;
  }
}
