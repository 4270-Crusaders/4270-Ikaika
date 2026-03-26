package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel {
  //TODO Retune based on ratio change 
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/Flywheel/Gains/kP", ShooterConstants.FlywheelConstants.FlyWheelkP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/Flywheel/Gains/kI", ShooterConstants.FlywheelConstants.FlyWheelkI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/Flywheel/Gains/kD", ShooterConstants.FlywheelConstants.FlyWheelkD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Shooter/Flywheel/Gains/kA", ShooterConstants.FlywheelConstants.FlyWheelkA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/Flywheel/Gains/kV", ShooterConstants.FlywheelConstants.FlyWheelkV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/Flywheel/Gains/kS", ShooterConstants.FlywheelConstants.FlyWheelkS);

  private static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber(
          "Shooter/Flywheel/MotionMagic/Jerk", ShooterConstants.FlywheelConstants.FlyWheelMotionMagicJerk);
  private static final LoggedTunableNumber MOTION_MAGIC_ACCELERATION =
      new LoggedTunableNumber(
          "Shooter/Flywheel/MotionMagic/Acceleration",
          ShooterConstants.FlywheelConstants.FlyWheelMotionMagicAcceleration);
  private static final LoggedTunableNumber MOTION_MAGIC_VELOCITY =
      new LoggedTunableNumber(
          "Shooter/Flywheel/MotionMagic/Velocity",
          ShooterConstants.FlywheelConstants.FlyWheelMotionMagicVelocity);

  /**
   * Half-width of the RPM window around goal for {@link #nearGoal}. Effective tolerance is {@code
   * NearGoalRpmTolerance × PhysicsLaunchEfficiencyScale}.
   */
  private static final LoggedTunableNumber nearGoalRpmTolerance =
      new LoggedTunableNumber(
          "Shooter/Flywheel/Ready/NearGoalRpmTolerance",
          ShooterConstants.READY_TO_SHOOT_FLYWHEEL_RPM_TOLERANCE);

  /**
   * Physics launch RPM assumes no load; when a ball passes through, wheels sag. This multiplier
   * raises the commanded RPM so effective speed through the shot matches the ballistic model.
   */
  private static final LoggedTunableNumber ballThroughRpmMultiplier =
      new LoggedTunableNumber("Shooter/Flywheel/BallThrough/RpmMultiplier", 1.02);

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public enum FlyWheelGoal {
    ZERO(new LoggedTunableNumber("Shooter/Flywheel/Goals/Zero", 0.0)),
    AUTOIDLE(new LoggedTunableNumber("Shooter/Flywheel/Goals/AutoIdle", 3000)),
    TELEIDLE(new LoggedTunableNumber("Shooter/Flywheel/Goals/TeleIdle", 2500)),
    CUSTOM(new LoggedTunableNumber("Shooter/Flywheel/Goals/Custom", 2300)); // TODO: Tune this value

    private final DoubleSupplier SHOOTER_SET_POINT_SUPPLIER;

    private FlyWheelGoal(DoubleSupplier shooterSetpointSupplier) {
      this.SHOOTER_SET_POINT_SUPPLIER = shooterSetpointSupplier;
    }

    private double getRPM() {
      return SHOOTER_SET_POINT_SUPPLIER.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Shooter/Flywheel/GoalSetpoint") private FlyWheelGoal goalSetpoint = FlyWheelGoal.ZERO;

  private boolean setpointMode = true;

  private double goalRPM = 0.0;

  public boolean nearGoal = false;

  /**
   * Extra multiplier on RPM tolerance from launch physics (ideal min / empirical map). Set from
   * {@link frc.robot.subsystems.shooter.LaunchCalculator} while aiming; default {@code 1.0}.
   */
  private double physicsLaunchEfficiencyScale = 1.0;

  public void setPhysicsLaunchEfficiencyScale(double scale) {
    this.physicsLaunchEfficiencyScale = MathUtil.clamp(scale, 0.5, 2.5);
  }

  public void setGoalSetPoint(double goalRPM) {
    setpointMode = false;
    this.goalRPM = goalRPM;
  }

  public void setGoalSetPoint(FlyWheelGoal setpoint) {
    setpointMode = true;
    this.goalSetpoint = setpoint;
  }

  /** Tunable gain to offset RPM sag when a ball passes through the flywheels. */
  public double getBallThroughRpmMultiplier() {
    return ballThroughRpmMultiplier.get();
  }

  /** {@code launchRpm × ballThroughRpmMultiplier}, clamped to max flywheel RPM. */
  public double compensateLaunchRpmForBallThrough(double launchRpmFromPhysics) {
    return MathUtil.clamp(
        launchRpmFromPhysics * ballThroughRpmMultiplier.get(),
        0,
        ShooterConstants.FlywheelConstants.FLYWHEEL_MAX_RPM);
  }

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
      goalRPM = goalSetpoint.getRPM()>ShooterConstants.FlywheelConstants.FLYWHEEL_MAX_RPM?ShooterConstants.FlywheelConstants.FLYWHEEL_MAX_RPM:goalSetpoint.getRPM();
    }

    io.runSetVelocity(goalRPM / 60);

    Logger.recordOutput("Shooter/Flywheel/GoalRPM", goalRPM, RadiansPerSecond);
    double toleranceBase = nearGoalRpmTolerance.get();
    double effectiveRpmTolerance = toleranceBase * physicsLaunchEfficiencyScale;
    nearGoal =
        EqualsUtil.epsilonEquals(
            (inputs.motorMeasuredVelocityRPM[0] + inputs.motorMeasuredVelocityRPM[1]) / 2,
            goalRPM,
            effectiveRpmTolerance);
    Logger.recordOutput("Shooter/Flywheel/BallThrough/RpmMultiplier", ballThroughRpmMultiplier.get());
    Logger.recordOutput("Shooter/Flywheel/Ready/PhysicsLaunchEfficiencyScale", physicsLaunchEfficiencyScale);
    Logger.recordOutput("Shooter/Flywheel/Ready/NearGoalRpmTolerance", toleranceBase);
    Logger.recordOutput("Shooter/Flywheel/Ready/NearGoalRpmToleranceEffective", effectiveRpmTolerance);
    Logger.recordOutput("Shooter/Flywheel/NearGoal", nearGoal);
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
