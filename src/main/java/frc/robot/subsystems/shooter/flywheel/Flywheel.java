package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;

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

  /**
   * Half-width of the RPM window around goal for {@link #nearGoal}. Effective tolerance is {@code
   * NearGoalRpmTolerance × PhysicsLaunchEfficiencyScale}.
   */
  private static final LoggedTunableNumber nearGoalRpmTolerance =
      new LoggedTunableNumber(
          "Shooter/Flywheel/Ready/NearGoalRpmTolerance",
          ShooterConstants.READY_TO_SHOOT_FLYWHEEL_RPM_TOLERANCE);

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public enum FlyWheelGoal {
    ZERO(new LoggedTunableNumber("Shooter/Flywheel/Goals/Zero", 0.0)),
    AUTOIDLE(
        new LoggedTunableNumber(
            "Shooter/Flywheel/Goals/AutoIdle",
            ShooterConstants.FlywheelShotConstants.FLYWHEEL_GOAL_AUTOIDLE_RPM)),
    TELEIDLE(
        new LoggedTunableNumber(
            "Shooter/Flywheel/Goals/TeleIdle",
            ShooterConstants.FlywheelShotConstants.FLYWHEEL_GOAL_TELEIDLE_RPM)),
    CUSTOM(
        new LoggedTunableNumber(
            "Shooter/Flywheel/Goals/Custom",
            ShooterConstants.FlywheelShotConstants.FLYWHEEL_GOAL_CUSTOM_RPM));

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

  /**
   * Extra multiplier on RPM tolerance from launch physics (ideal min / empirical map). Set from
   * {@link frc.robot.subsystems.shooter.LaunchCalculator} while aiming; default {@code 1.0}.
   */
  private double physicsLaunchEfficiencyScale = 1.0;

  public void setPhysicsLaunchEfficiencyScale(double scale) {
    this.physicsLaunchEfficiencyScale =
        MathUtil.clamp(
            scale,
            ShooterConstants.FlywheelShotConstants.PHYSICS_LAUNCH_EFFICIENCY_INPUT_MIN,
            ShooterConstants.FlywheelShotConstants.PHYSICS_LAUNCH_EFFICIENCY_INPUT_MAX);
  }

  public void setGoalSetPoint(double goalRPM) {
    setpointMode = false;
    this.goalRPM =
        MathUtil.clamp(goalRPM, 0.0, ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
  }

  public void setGoalSetPoint(FlyWheelGoal setpoint) {
    setpointMode = true;
    this.goalSetpoint = setpoint;
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

    Logger.recordOutput("Shooter/Flywheel/GoalRPM", goalRPM, RPM);
    double toleranceBase = nearGoalRpmTolerance.get();
    double effectiveRpmTolerance = toleranceBase * physicsLaunchEfficiencyScale;
    double measuredRpm =
        ShooterConstants.LaunchCalculatorConstants.DUAL_WHEEL_SURFACE_BLEND
            * (inputs.motorMeasuredVelocityRpm[0] + inputs.motorMeasuredVelocityRpm[1]);

    nearGoal =
        EqualsUtil.epsilonEquals(
            measuredRpm,
            goalRPM,
            effectiveRpmTolerance);
    Logger.recordOutput("Shooter/Flywheel/Ready/PhysicsLaunchEfficiencyScale", physicsLaunchEfficiencyScale);
    Logger.recordOutput("Shooter/Flywheel/Ready/NearGoalRpmTolerance", toleranceBase);
    Logger.recordOutput("Shooter/Flywheel/Ready/NearGoalRpmToleranceEffective", effectiveRpmTolerance);
    Logger.recordOutput("Shooter/Flywheel/NearGoal", nearGoal);
    Logger.recordOutput("Shooter/Flywheel/MeasuredRpm", measuredRpm);
    RobotState.getInstance()
        .recordLauncherFlywheelSurfaceSpeedMps(getAverageWheelSurfaceVelocityMetersPerSec());
  }

  @Override
  public void periodicAfterScheduler() {
    io.runSetVelocity(goalRPM / ShooterConstants.LaunchCalculatorConstants.SECONDS_PER_MINUTE);
  }

  /**
   * MA-style: each loop while the command runs, apply {@link LaunchCalculator#getParameters()} (after
   * {@link frc.robot.commands.launcher.LaunchCoordinatorSubsystem} refreshes the calculator).
   */
  public Command runTrackTargetCommand() {
    return runEnd(
        () -> {
          if (!RobotState.getInstance().isLauncherTracking()) {
            return;
          }
          LaunchCalculator.LaunchingParameters p = LaunchCalculator.getInstance().getParameters();
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
    return ShooterConstants.LaunchCalculatorConstants.DUAL_WHEEL_SURFACE_BLEND
        * (inputs.motorMeasuredVelocityRpm[0] + inputs.motorMeasuredVelocityRpm[1]);
  }

  public double getMainFlyWheelVelocityRPM() {
    return inputs.mainFlywheelRpm;
  }

  public double getHoodFlyWheelVelocityRPM() {
    return inputs.hoodFlywheelRpm;
  }

  /** Average top/bottom wheel RPM in wheel space (after reductions). */
  public double getAverageWheelVelocityRPM() {
    return ShooterConstants.LaunchCalculatorConstants.DUAL_WHEEL_SURFACE_BLEND
        * (getMainFlyWheelVelocityRPM() + getHoodFlyWheelVelocityRPM());
  }

  /** Average top/bottom wheel surface speed in meters per second. */
  public double getAverageWheelSurfaceVelocityMetersPerSec() {
    double mainWheelSurfaceMps =
        (getMainFlyWheelVelocityRPM() / ShooterConstants.LaunchCalculatorConstants.SECONDS_PER_MINUTE)
            * Math.PI
            * ShooterConstants.ComponentsConstants.Flywheel.MAIN_WHEEL_DIAMETER_METERS;
    double hoodWheelSurfaceMps =
        (getHoodFlyWheelVelocityRPM() / ShooterConstants.LaunchCalculatorConstants.SECONDS_PER_MINUTE)
            * Math.PI
            * ShooterConstants.ComponentsConstants.Flywheel.HOOD_WHEEL_DIAMETER_METERS;
    return ShooterConstants.LaunchCalculatorConstants.DUAL_WHEEL_SURFACE_BLEND
        * (mainWheelSurfaceMps + hoodWheelSurfaceMps);
  }

  /**
   * Returns measured velocity as motor-equivalent RPM using top/bottom wheel velocities.
   *
   * <p>This averages wheel-surface speed from both wheels, then converts to the same RPM domain
   * expected by {@link LaunchCalculator}.
   */
  public double getAverageWheelVelocityMotorEquivalentRPM() {
    return LaunchCalculator.rpmFromSurfaceVelocity(getAverageWheelSurfaceVelocityMetersPerSec());
  }
}
