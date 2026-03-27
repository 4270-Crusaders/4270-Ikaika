package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class LaunchCalculator {
  private static LaunchCalculator instance;

  /** Cached wheel kinematics (rad/solve); avoids repeated composite math on the hot path. */
  private static final double EFFECTIVE_METERS_PER_MOTOR_REV;

  private static final double SURFACE_MPS_TO_RPM;
  private static final double RPM_TO_SURFACE_MPS;

  private static final double HOOD_THETA_CLAMP_MIN_RAD;
  private static final double HOOD_THETA_CLAMP_MAX_RAD;

  static {
    double eps = ShooterConstants.LaunchCalculatorConstants.EPSILON_DENOMINATOR;
    double mainMetersPerMotorRev =
        (Math.PI * ShooterConstants.ComponentsConstants.Flywheel.MAIN_WHEEL_DIAMETER_METERS)
            / Math.max(
                ShooterConstants.ComponentsConstants.Flywheel.TurretMotorToMainFlyWheelReduction,
                eps);
    double hoodMetersPerMotorRev =
        (Math.PI * ShooterConstants.ComponentsConstants.Flywheel.HOOD_WHEEL_DIAMETER_METERS)
            / Math.max(
                ShooterConstants.ComponentsConstants.Flywheel.TurretMotorToHoodFlyWheelReduction,
                eps);
    double avgMetersPerMotorRev =
        ShooterConstants.LaunchCalculatorConstants.DUAL_WHEEL_SURFACE_BLEND
            * (mainMetersPerMotorRev + hoodMetersPerMotorRev);
    EFFECTIVE_METERS_PER_MOTOR_REV =
        avgMetersPerMotorRev * ShooterConstants.ComponentsConstants.Flywheel.BALL_EXIT_TRANSFER_EFFICIENCY;
    double spm = ShooterConstants.LaunchCalculatorConstants.SECONDS_PER_MINUTE;
    SURFACE_MPS_TO_RPM = spm / Math.max(EFFECTIVE_METERS_PER_MOTOR_REV, eps);
    RPM_TO_SURFACE_MPS = EFFECTIVE_METERS_PER_MOTOR_REV / spm;

    double thetaMinDeg =
        ShooterConstants.LaunchCalculatorConstants.MECHANICAL_RIGHT_ANGLE_DEG
            + ShooterConstants.ComponentsConstants.Hood.MECHANICAL_ANGLE_OFFSET_DEG
            - ShooterConstants.ComponentsConstants.Hood.MAX_DEGREE;
    double thetaMaxDeg =
        ShooterConstants.LaunchCalculatorConstants.MECHANICAL_RIGHT_ANGLE_DEG
            + ShooterConstants.ComponentsConstants.Hood.MECHANICAL_ANGLE_OFFSET_DEG;
    double thetaMin = Units.degreesToRadians(thetaMinDeg);
    double thetaMax = Units.degreesToRadians(thetaMaxDeg);
    if (!Double.isFinite(thetaMin) || !Double.isFinite(thetaMax) || thetaMax <= thetaMin) {
      thetaMin =
          Units.degreesToRadians(ShooterConstants.LaunchCalculatorConstants.FALLBACK_THETA_MIN_DEG);
      thetaMax =
          Units.degreesToRadians(ShooterConstants.LaunchCalculatorConstants.FALLBACK_THETA_MAX_DEG);
    }
    HOOD_THETA_CLAMP_MIN_RAD = thetaMin;
    HOOD_THETA_CLAMP_MAX_RAD = thetaMax;
  }

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage(
          (int)
              (ShooterConstants.LaunchCalculatorConstants.ANGLE_VELOCITY_FILTER_WINDOW_SEC
                  / Constants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage(
          (int)
              (ShooterConstants.LaunchCalculatorConstants.ANGLE_VELOCITY_FILTER_WINDOW_SEC
                  / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private double lastPhysicsMinToEmpiricalRpmRatio =
      ShooterConstants.LaunchCalculatorConstants.NEUTRAL_UNIT_RATIO;
  private double measuredToCommandRpmRatio =
      ShooterConstants.LaunchCalculatorConstants.NEUTRAL_UNIT_RATIO;
  private double filteredMeasuredFlywheelSurfaceSpeedMps = Double.NaN;

  /** Latest ballistic solve written by {@link #solveBallisticsInPlace}; no heap allocation. */
  private double scratchLaunchSpeedMps;

  private double scratchThetaRad;
  private double scratchTofSec;
  private double scratchRpmWheel;

  public enum ArcSelection {
    HIGH,
    LOW
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double timeOfFlightSec) {}

  private LaunchingParameters latestParameters = null;

  private static final LaunchingParameters EMPTY_PARAMETERS =
      new LaunchingParameters(false, Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public static double rpmFromSurfaceVelocity(double surfaceVelocityMetersPerSec) {
    if (EFFECTIVE_METERS_PER_MOTOR_REV < ShooterConstants.LaunchCalculatorConstants.EPSILON_DENOMINATOR) {
      return 0.0;
    }
    return surfaceVelocityMetersPerSec * SURFACE_MPS_TO_RPM;
  }

  public static double surfaceVelocityFromRpm(double rpm) {
    return rpm * RPM_TO_SURFACE_MPS;
  }

  public static double minimumExitVelocity(double horizontalDistanceM, double heightDeltaM) {
    double g = ShooterConstants.GRAVITY;
    double d = Math.max(horizontalDistanceM, 0.0);
    double h = heightDeltaM;
    return Math.sqrt(Math.max(0.0, g * (h + Math.hypot(d, h))));
  }

  private static double solveThetaForSpeed(
      double launchSpeedMps, double horizontalDistanceM, double heightDeltaM, ArcSelection arc) {
    double g = ShooterConstants.GRAVITY;
    double d = horizontalDistanceM;
    double h = heightDeltaM;
    double v2 = launchSpeedMps * launchSpeedMps;
    double epsD = ShooterConstants.LaunchCalculatorConstants.EPSILON_METERS;
    double epsV = ShooterConstants.LaunchCalculatorConstants.EPSILON_SURFACE_SPEED_MPS;
    if (d < epsD || launchSpeedMps < epsV) return Double.NaN;

    double disc = v2 * v2 - g * (g * d * d + 2.0 * h * v2);
    if (disc < 0.0) return Double.NaN;
    double sqrtDisc = Math.sqrt(disc);
    double denom = g * d;
    if (Math.abs(denom) < ShooterConstants.LaunchCalculatorConstants.EPSILON_DENOMINATOR)
      return Double.NaN;

    double tanLow = (v2 - sqrtDisc) / denom;
    double tanHigh = (v2 + sqrtDisc) / denom;
    double thetaLow = Math.atan(tanLow);
    double thetaHigh = Math.atan(tanHigh);
    return arc == ArcSelection.HIGH ? Math.max(thetaLow, thetaHigh) : Math.min(thetaLow, thetaHigh);
  }

  private static double clampThetaToHoodLimits(double thetaAboveHorizontalRad) {
    return MathUtil.clamp(thetaAboveHorizontalRad, HOOD_THETA_CLAMP_MIN_RAD, HOOD_THETA_CLAMP_MAX_RAD);
  }

  public static double mechanicalHoodAngleRadFromPhysicsTheta(double thetaAboveHorizontalRad) {
    double thetaDeg = Units.radiansToDegrees(thetaAboveHorizontalRad);
    double hoodDeg =
        ShooterConstants.LaunchCalculatorConstants.MECHANICAL_RIGHT_ANGLE_DEG
            - thetaDeg
            + ShooterConstants.ComponentsConstants.Hood.MECHANICAL_ANGLE_OFFSET_DEG;
    hoodDeg =
        MathUtil.clamp(hoodDeg, 0.0, ShooterConstants.ComponentsConstants.Hood.MAX_DEGREE);
    return Units.degreesToRadians(hoodDeg);
  }

  private static double timeOfFlightFromPhysics(
      double horizontalDistanceM, double launchSpeedMps, double thetaAboveHorizontalRad) {
    double vx = launchSpeedMps * Math.cos(thetaAboveHorizontalRad);
    if (vx < ShooterConstants.LaunchCalculatorConstants.EPSILON_SURFACE_SPEED_MPS)
      return Double.NaN;
    return horizontalDistanceM / vx;
  }

  private void solveBallisticsInPlace(double horizontalDistanceM, double heightDeltaM, ArcSelection arc) {
    ArcSelection effectiveArc = heightDeltaM < 0.0 ? ArcSelection.LOW : arc;
    double vMin = minimumExitVelocity(horizontalDistanceM, heightDeltaM);
    double launchEfficiency =
        Math.max(
            ShooterConstants.LaunchCalculatorConstants.INITIAL_SPEED_EFFICIENCY,
            ShooterConstants.LaunchCalculatorConstants.EPSILON_DENOMINATOR);
    double compensatedMinSpeed =
        (vMin * ShooterConstants.LaunchCalculatorConstants.MIN_SPEED_MARGIN) / launchEfficiency;
    double rpmWheelCmd = Math.max(0.0, rpmFromSurfaceVelocity(compensatedMinSpeed));
    double vCmd = surfaceVelocityFromRpm(rpmWheelCmd);

    double theta = solveThetaForSpeed(vCmd, horizontalDistanceM, heightDeltaM, effectiveArc);
    if (!Double.isFinite(theta)) {
      for (int i = 0;
          i < ShooterConstants.LaunchCalculatorConstants.BALLISTIC_RPM_BOOST_MAX_ITERATIONS
              && !Double.isFinite(theta);
          i++) {
        rpmWheelCmd *= ShooterConstants.LaunchCalculatorConstants.BALLISTIC_RPM_BOOST_FACTOR;
        vCmd = surfaceVelocityFromRpm(rpmWheelCmd);
        theta = solveThetaForSpeed(vCmd, horizontalDistanceM, heightDeltaM, effectiveArc);
      }
    }
    if (!Double.isFinite(theta)) {
      scratchLaunchSpeedMps = vCmd;
      scratchThetaRad =
          Units.degreesToRadians(ShooterConstants.LaunchCalculatorConstants.DEFAULT_SOLVE_THETA_DEG);
      scratchTofSec = Double.NaN;
      scratchRpmWheel = rpmWheelCmd;
      return;
    }

    theta = clampThetaToHoodLimits(theta);
    scratchLaunchSpeedMps = vCmd;
    scratchThetaRad = theta;
    scratchTofSec = timeOfFlightFromPhysics(horizontalDistanceM, vCmd, theta);
    scratchRpmWheel = rpmWheelCmd;
  }

  /**
   * Latest solve from {@link #refreshCachedParameters}; safe to call from mechanism commands after the
   * launcher has refreshed the cache for this cycle.
   */
  public LaunchingParameters getParameters() {
    return latestParameters != null ? latestParameters : EMPTY_PARAMETERS;
  }

  /**
   * Recomputes and stores {@link #getParameters()} from {@link RobotState} solve inputs and measured
   * flywheel surface speed. {@link frc.robot.commands.launcher.LaunchCoordinatorSubsystem} must set
   * solve inputs via {@link RobotState#setLauncherSolveInputs} each loop while tracking.
   */
  public void refreshCachedParameters() {
    RobotState rs = RobotState.getInstance();
    if (!rs.isLauncherSolveInputsValid()) {
      return;
    }
    Pose3d shooterLaunchPose3d = rs.getLauncherSolveShooterLaunchPose3d();
    Translation3d shooterVelocity3d = rs.getLauncherSolveShooterVelocity3d();
    Translation3d targetTranslation3d = rs.getLauncherSolveTarget3d();
    ArcSelection arcSelection = rs.isLauncherSolveUseLowArc() ? ArcSelection.LOW : ArcSelection.HIGH;
    double measuredFlywheelSurfaceSpeedMps = rs.getLauncherFlywheelSurfaceSpeedMps();

    Translation3d shooterWorld = shooterLaunchPose3d.getTranslation();
    double sx = shooterWorld.getX();
    double sy = shooterWorld.getY();
    double sz = shooterWorld.getZ();
    double tx = targetTranslation3d.getX();
    double ty = targetTranslation3d.getY();
    double tz = targetTranslation3d.getZ();
    double vx = shooterVelocity3d.getX();
    double vy = shooterVelocity3d.getY();
    double vz = shooterVelocity3d.getZ();

    double lx = sx;
    double ly = sy;
    double lz = sz;
    for (int i = 0;
        i < ShooterConstants.LaunchCalculatorConstants.MOVING_TARGET_LEAD_ITERATIONS;
        i++) {
      double rdx = tx - lx;
      double rdy = ty - ly;
      double rdz = tz - lz;
      double d = Math.hypot(rdx, rdy);
      solveBallisticsInPlace(d, rdz, arcSelection);

      double tof = Double.isFinite(scratchTofSec) ? scratchTofSec : 0.0;
      lx = sx + vx * tof;
      ly = sy + vy * tof;
      lz = sz + vz * tof;
    }

    double relFx = tx - lx;
    double relFy = ty - ly;
    double relFz = tz - lz;
    double dFinal = Math.hypot(relFx, relFy);
    Rotation2d turretAngle = new Rotation2d(Math.atan2(relFy, relFx));

    // Hood follows actual wheel speed with filtered correction only (simpler, less oscillation).
    ArcSelection effectiveArc = relFz < 0.0 ? ArcSelection.LOW : arcSelection;
    double tau = ShooterConstants.HoodCompensationConstants.RPM_FILTER_TIME_CONSTANT_SEC;
    // TODO(PHYSICS_TUNE): review alpha behavior if loop rate changes from 20ms.
    double alpha =
        Constants.loopPeriodSecs
            / Math.max(
                Constants.loopPeriodSecs + tau,
                ShooterConstants.LaunchCalculatorConstants.EPSILON_TIME_AND_RATIO);
    if (!Double.isFinite(filteredMeasuredFlywheelSurfaceSpeedMps)) {
      filteredMeasuredFlywheelSurfaceSpeedMps = measuredFlywheelSurfaceSpeedMps;
    }
    filteredMeasuredFlywheelSurfaceSpeedMps +=
        alpha * (measuredFlywheelSurfaceSpeedMps - filteredMeasuredFlywheelSurfaceSpeedMps);

    double measuredSpeed = filteredMeasuredFlywheelSurfaceSpeedMps;
    double thetaNominal = scratchThetaRad;
    double thetaFromMeasured = solveThetaForSpeed(measuredSpeed, dFinal, relFz, effectiveArc);
    double thetaError = 0.0;
    if (Double.isFinite(thetaFromMeasured)
        && measuredSpeed > ShooterConstants.LaunchCalculatorConstants.MIN_VALID_SHOT_SPEED_MPS) {
      thetaError = clampThetaToHoodLimits(thetaFromMeasured) - thetaNominal;
    }
    double gain = ShooterConstants.HoodCompensationConstants.NORMAL_GAIN;
    double maxCorrRad =
        Units.degreesToRadians(ShooterConstants.HoodCompensationConstants.MAX_CORRECTION_DEG);
    double thetaCorrected = thetaNominal + MathUtil.clamp(thetaError * gain, -maxCorrRad, maxCorrRad);
    double thetaUsedRad = clampThetaToHoodLimits(thetaCorrected);

    double hoodAngle = mechanicalHoodAngleRadFromPhysicsTheta(thetaUsedRad);
    if (!Double.isNaN(lastHoodAngle)) {
      double maxDelta =
          Units.degreesToRadians(
                  ShooterConstants.HoodCompensationConstants.MAX_CORRECTION_RATE_DEG_PER_SEC)
              * Constants.loopPeriodSecs;
      hoodAngle = MathUtil.clamp(hoodAngle, lastHoodAngle - maxDelta, lastHoodAngle + maxDelta);
    }
    double tofUsed = timeOfFlightFromPhysics(dFinal, scratchLaunchSpeedMps, scratchThetaRad);

    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;

    double vMinIdeal = minimumExitVelocity(dFinal, relFz);
    double minRpmIdeal = rpmFromSurfaceVelocity(vMinIdeal);
    double ratioEps = ShooterConstants.LaunchCalculatorConstants.EPSILON_TIME_AND_RATIO;
    lastPhysicsMinToEmpiricalRpmRatio =
        scratchRpmWheel > ratioEps
            ? minRpmIdeal / scratchRpmWheel
            : ShooterConstants.LaunchCalculatorConstants.NEUTRAL_UNIT_RATIO;
    // TODO(PHYSICS_TUNE): revisit clamp limits if drivetrain brownout causes deeper RPM dips.
    measuredToCommandRpmRatio =
        scratchRpmWheel > ratioEps
            ? MathUtil.clamp(
                rpmFromSurfaceVelocity(measuredSpeed) / scratchRpmWheel,
                0.0,
                ShooterConstants.LaunchCalculatorConstants.MEASURED_TO_COMMAND_RPM_RATIO_MAX)
            : ShooterConstants.LaunchCalculatorConstants.NEUTRAL_UNIT_RATIO;

    latestParameters =
        new LaunchingParameters(
            true,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            scratchRpmWheel,
            tofUsed);
    if (ShooterConstants.Logging.LOG_LAUNCH_CALC_HOOD_COMP) {
      Logger.recordOutput(
          "Shooter/LaunchCalculator/HoodComp/MeasuredSurfaceSpeedMpsRaw",
          measuredFlywheelSurfaceSpeedMps);
      Logger.recordOutput(
          "Shooter/LaunchCalculator/HoodComp/MeasuredSurfaceSpeedMpsFiltered",
          filteredMeasuredFlywheelSurfaceSpeedMps);
      Logger.recordOutput(
          "Shooter/LaunchCalculator/HoodComp/ThetaNominalDeg", Units.radiansToDegrees(thetaNominal));
      Logger.recordOutput("Shooter/LaunchCalculator/HoodComp/ThetaUsedDeg", Units.radiansToDegrees(thetaUsedRad));
      Logger.recordOutput(
          "Shooter/LaunchCalculator/HoodComp/ThetaErrorDeg",
          Units.radiansToDegrees(thetaUsedRad - thetaNominal));
    }
  }

  public double getPhysicsLaunchEfficiencyScale() {
    // Use measured-vs-command ratio so tolerance adapts to real wheel sag, not only model ideality.
    double measuredRatio = measuredToCommandRpmRatio;
    if (!Double.isFinite(measuredRatio))
      return ShooterConstants.FlywheelShotConstants.PHYSICS_LAUNCH_EFFICIENCY_SCALE_NEUTRAL;
    measuredRatio =
        MathUtil.clamp(
            measuredRatio,
            0.0,
            ShooterConstants.LaunchCalculatorConstants.MEASURED_TO_COMMAND_RPM_RATIO_MAX);
    double unity = ShooterConstants.FlywheelShotConstants.PHYSICS_LAUNCH_EFFICIENCY_SCALE_NEUTRAL;
    return MathUtil.clamp(
        unity + (unity - Math.min(measuredRatio, unity)),
        ShooterConstants.LaunchCalculatorConstants.PHYSICS_LAUNCH_EFFICIENCY_SCALE_OUT_MIN,
        ShooterConstants.LaunchCalculatorConstants.PHYSICS_LAUNCH_EFFICIENCY_SCALE_OUT_MAX);
  }

  public double getPhysicsMinToEmpiricalRpmRatio() {
    return lastPhysicsMinToEmpiricalRpmRatio;
  }

  public double getMeasuredToCommandRpmRatio() {
    return measuredToCommandRpmRatio;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
    filteredMeasuredFlywheelSurfaceSpeedMps = Double.NaN;
    RobotState.getInstance().clearLauncherSolveInputs();
  }
}
