// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlyWheelGoal;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.Hood.HoodGoal;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.Turret.TurretGoal;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class ShooterCalculator {
  private static ShooterCalculator instance;

  /** Cached wheel kinematics (rad/solve); avoids repeated composite math on the hot path. */
  private static final double EFFECTIVE_METERS_PER_MOTOR_REV;

  private static final double SURFACE_MPS_TO_RPM;
  private static final double RPM_TO_SURFACE_MPS;

  private static final double HOOD_THETA_CLAMP_MIN_RAD;
  private static final double HOOD_THETA_CLAMP_MAX_RAD;

  static {
    double eps = ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR;
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
        ShooterConstants.ShooterCalculatorConstants.DUAL_WHEEL_SURFACE_BLEND
            * (mainMetersPerMotorRev + hoodMetersPerMotorRev);
    EFFECTIVE_METERS_PER_MOTOR_REV =
        avgMetersPerMotorRev * ShooterConstants.ComponentsConstants.Flywheel.BALL_EXIT_TRANSFER_EFFICIENCY;
    // Talon velocity is rotations per second; WPILib has no RPM<->RPS helper, so use 60 s/min.
    SURFACE_MPS_TO_RPM = 60.0 / Math.max(EFFECTIVE_METERS_PER_MOTOR_REV, eps);
    RPM_TO_SURFACE_MPS = EFFECTIVE_METERS_PER_MOTOR_REV / 60.0;

    // Physics shot-angle limits = inverse of mechanical travel, same formula as
    // physicsThetaRadFromMechanicalHoodDeg (offset k and MECHANICAL_RIGHT_ANGLE_DEG stay in sync).
    double thetaMin =
        physicsThetaRadFromMechanicalHoodDeg(ShooterConstants.SHOOTER_HOOD_SETPOINT_MAX_DEG);
    double thetaMax =
        physicsThetaRadFromMechanicalHoodDeg(ShooterConstants.SHOOTER_HOOD_SETPOINT_MIN_DEG);
    if (!Double.isFinite(thetaMin) || !Double.isFinite(thetaMax) || thetaMax <= thetaMin) {
      thetaMin =
          Units.degreesToRadians(ShooterConstants.ShooterCalculatorConstants.FALLBACK_THETA_MIN_DEG);
      thetaMax =
          Units.degreesToRadians(ShooterConstants.ShooterCalculatorConstants.FALLBACK_THETA_MAX_DEG);
    }
    HOOD_THETA_CLAMP_MIN_RAD = thetaMin;
    HOOD_THETA_CLAMP_MAX_RAD = thetaMax;
  }

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage(
          (int)
              (ShooterConstants.ShooterCalculatorConstants.ANGLE_VELOCITY_FILTER_WINDOW_SEC
                  / Constants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage(
          (int)
              (ShooterConstants.ShooterCalculatorConstants.ANGLE_VELOCITY_FILTER_WINDOW_SEC
                  / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;

  /** Low-pass state for field-native shooter-point horizontal acceleration (m/s^2). */
  private Translation2d filteredShooterFieldAccelNative = null;
  private double lastPhysicsMinToEmpiricalRpmRatio = 1.0;
  private double measuredToCommandRpmRatio = 1.0;
  private double filteredMeasuredFlywheelSurfaceSpeedMps = Double.NaN;

  /** Latest ballistic solve written by {@link #solveBallisticsInPlace}; no heap allocation. */
  private double scratchExitSpeedMps;

  private double scratchThetaRad;
  private double scratchTofSec;
  private double scratchRpmWheel;

  public enum ArcSelection {
    HIGH,
    LOW
  }

  /**
   * @param hoodAngle mechanical hood setpoint (rad, Talon frame), typically from {@link
   *     #mechanicalHoodAngleRadFromPhysicsTheta}. Ballistics use exit elevation above horizontal
   *     internally ({@link #clampThetaToHoodLimits}, offset {@link
   *     ShooterConstants.ComponentsConstants.Hood#MECHANICAL_ANGLE_OFFSET_DEG}).
   */
  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double timeOfFlightSec) {}

  private ShootingParameters latestParameters = null;

  private static final ShootingParameters EMPTY_PARAMETERS =
      new ShootingParameters(false, Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static ShooterCalculator getInstance() {
    if (instance == null) instance = new ShooterCalculator();
    return instance;
  }

  public static double rpmFromSurfaceVelocity(double surfaceVelocityMetersPerSec) {
    if (EFFECTIVE_METERS_PER_MOTOR_REV < ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR) {
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
      double exitSpeedMps, double horizontalDistanceM, double heightDeltaM, ArcSelection arc) {
    double g = ShooterConstants.GRAVITY;
    double d = horizontalDistanceM;
    double h = heightDeltaM;
    double v2 = exitSpeedMps * exitSpeedMps;
    double epsD = ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS;
    double epsV = ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS;
    if (d < epsD || exitSpeedMps < epsV) return Double.NaN;

    double disc = v2 * v2 - g * (g * d * d + 2.0 * h * v2);
    if (disc < 0.0) return Double.NaN;
    double sqrtDisc = Math.sqrt(disc);
    double denom = g * d;
    if (Math.abs(denom) < ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR)
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

  /**
   * Maps physics exit angle (above horizontal) to Talon hood setpoint deg: {@code m = 90 - theta - k}
   * ({@link ShooterConstants.ComponentsConstants.Hood#MECHANICAL_ANGLE_OFFSET_DEG} is {@code k}).
   */
  public static double mechanicalHoodAngleRadFromPhysicsTheta(double thetaAboveHorizontalRad) {
    double thetaDeg = Units.radiansToDegrees(thetaAboveHorizontalRad);
    double hoodDeg =
        ShooterConstants.ShooterCalculatorConstants.MECHANICAL_RIGHT_ANGLE_DEG
            - thetaDeg
            - ShooterConstants.ComponentsConstants.Hood.MECHANICAL_ANGLE_OFFSET_DEG;
    hoodDeg =
        MathUtil.clamp(
            hoodDeg,
            ShooterConstants.SHOOTER_HOOD_SETPOINT_MIN_DEG,
            ShooterConstants.SHOOTER_HOOD_SETPOINT_MAX_DEG);
    return Units.degreesToRadians(hoodDeg);
  }

  /**
   * Inverse of {@link #mechanicalHoodAngleRadFromPhysicsTheta} before hood clamp: {@code theta_deg = 90 -
   * m - k}.
   */
  public static double physicsThetaRadFromMechanicalHoodDeg(double hoodMechanicalDeg) {
    double thetaDeg =
        ShooterConstants.ShooterCalculatorConstants.MECHANICAL_RIGHT_ANGLE_DEG
            - hoodMechanicalDeg
            - ShooterConstants.ComponentsConstants.Hood.MECHANICAL_ANGLE_OFFSET_DEG;
    return Units.degreesToRadians(thetaDeg);
  }

  /**
   * Exit speed (m/s) so trajectory through {@code horizontalDistanceM} hits height {@code heightDeltaM}
   * at exit angle {@code thetaAboveHorizontalRad} (vacuum).
   */
  private static double exitSpeedForFixedTheta(
      double horizontalDistanceM, double heightDeltaM, double thetaAboveHorizontalRad) {
    double g = ShooterConstants.GRAVITY;
    double d = Math.max(horizontalDistanceM, ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS);
    double h = heightDeltaM;
    double cosT = Math.cos(thetaAboveHorizontalRad);
    if (cosT < ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS) {
      return Double.NaN;
    }
    double tanT = Math.tan(thetaAboveHorizontalRad);
    double denom = d * tanT - h;
    if (denom <= ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS) {
      return Double.NaN;
    }
    double v2 = g * d * d / (2.0 * cosT * cosT * denom);
    if (v2 <= 0.0 || !Double.isFinite(v2)) {
      return Double.NaN;
    }
    return Math.sqrt(v2);
  }

  private void solveBallisticsFixedThetaInPlace(
      double horizontalDistanceM, double heightDeltaM, double thetaAboveHorizontalRad) {
    double theta = clampThetaToHoodLimits(thetaAboveHorizontalRad);
    double vIdeal = exitSpeedForFixedTheta(horizontalDistanceM, heightDeltaM, theta);
    if (!Double.isFinite(vIdeal) || vIdeal <= 0.0) {
      scratchExitSpeedMps = 0.0;
      scratchThetaRad = theta;
      scratchTofSec = Double.NaN;
      scratchRpmWheel = 0.0;
      return;
    }
    double shotEfficiency =
        Math.max(
            ShooterConstants.ShooterCalculatorConstants.INITIAL_SPEED_EFFICIENCY,
            ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR);
    double compensatedSpeed =
        (vIdeal * ShooterConstants.ShooterCalculatorConstants.MIN_SPEED_MARGIN) / shotEfficiency;
    double rpmWheelCmd = Math.max(0.0, rpmFromSurfaceVelocity(compensatedSpeed));
    double vCmd = surfaceVelocityFromRpm(rpmWheelCmd);
    scratchThetaRad = theta;
    scratchExitSpeedMps = vCmd;
    scratchTofSec = timeOfFlightFromPhysics(horizontalDistanceM, vCmd, theta);
    scratchRpmWheel = rpmWheelCmd;
    for (int i = 0;
        i < ShooterConstants.ShooterCalculatorConstants.BALLISTIC_RPM_BOOST_MAX_ITERATIONS
            && (!Double.isFinite(scratchTofSec) || scratchTofSec <= 0.0);
        i++) {
      rpmWheelCmd *= ShooterConstants.ShooterCalculatorConstants.BALLISTIC_RPM_BOOST_FACTOR;
      vCmd = surfaceVelocityFromRpm(rpmWheelCmd);
      scratchExitSpeedMps = vCmd;
      scratchTofSec = timeOfFlightFromPhysics(horizontalDistanceM, vCmd, theta);
      scratchRpmWheel = rpmWheelCmd;
    }
  }

  private static double timeOfFlightFromPhysics(
      double horizontalDistanceM, double exitSpeedMps, double thetaAboveHorizontalRad) {
    double vx = exitSpeedMps * Math.cos(thetaAboveHorizontalRad);
    if (vx < ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS)
      return Double.NaN;
    return horizontalDistanceM / vx;
  }

  private void solveBallisticsInPlace(double horizontalDistanceM, double heightDeltaM, ArcSelection arc) {
    ArcSelection effectiveArc = heightDeltaM < 0.0 ? ArcSelection.LOW : arc;
    double vMin = minimumExitVelocity(horizontalDistanceM, heightDeltaM);
    double shotEfficiency =
        Math.max(
            ShooterConstants.ShooterCalculatorConstants.INITIAL_SPEED_EFFICIENCY,
            ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR);
    double compensatedMinSpeed =
        (vMin * ShooterConstants.ShooterCalculatorConstants.MIN_SPEED_MARGIN) / shotEfficiency;
    double rpmWheelCmd = Math.max(0.0, rpmFromSurfaceVelocity(compensatedMinSpeed));
    double vCmd = surfaceVelocityFromRpm(rpmWheelCmd);

    double theta = solveThetaForSpeed(vCmd, horizontalDistanceM, heightDeltaM, effectiveArc);
    if (!Double.isFinite(theta)) {
      for (int i = 0;
          i < ShooterConstants.ShooterCalculatorConstants.BALLISTIC_RPM_BOOST_MAX_ITERATIONS
              && !Double.isFinite(theta);
          i++) {
        rpmWheelCmd *= ShooterConstants.ShooterCalculatorConstants.BALLISTIC_RPM_BOOST_FACTOR;
        vCmd = surfaceVelocityFromRpm(rpmWheelCmd);
        theta = solveThetaForSpeed(vCmd, horizontalDistanceM, heightDeltaM, effectiveArc);
      }
    }
    if (!Double.isFinite(theta)) {
      scratchExitSpeedMps = vCmd;
      scratchThetaRad =
          Units.degreesToRadians(ShooterConstants.ShooterCalculatorConstants.DEFAULT_SOLVE_THETA_DEG);
      scratchTofSec = Double.NaN;
      scratchRpmWheel = rpmWheelCmd;
      return;
    }

    theta = clampThetaToHoodLimits(theta);
    scratchExitSpeedMps = vCmd;
    scratchThetaRad = theta;
    scratchTofSec = timeOfFlightFromPhysics(horizontalDistanceM, vCmd, theta);
    scratchRpmWheel = rpmWheelCmd;
  }

  /**
   * Latest solve from {@link #refreshCachedParameters}; safe to call from mechanism commands after the
   * shooter has refreshed the cache for this cycle.
   */
  public ShootingParameters getParameters() {
    return latestParameters != null ? latestParameters : EMPTY_PARAMETERS;
  }

  /**
   * Recomputes and stores {@link #getParameters()} from {@link RobotState} solve inputs and measured
   * flywheel surface speed. {@link #coordinateAfterScheduler} must set solve inputs via {@link
   * RobotState#setShooterSolveInputs} each loop while tracking.
   */
  public void refreshCachedParameters() {
    RobotState rs = RobotState.getInstance();
    if (!rs.isShooterSolveInputsValid()) {
      return;
    }
    Pose3d shooterPose3d = rs.getShooterSolvePose3d();
    Translation3d shooterVelocity3d = rs.getShooterSolveVelocity3d();
    Translation3d shooterAcceleration3d = rs.getShooterSolveAcceleration3d();
    Translation3d targetTranslation3d = rs.getShooterSolveTarget3d();
    ArcSelection arcSelection = rs.isShooterSolveUseLowArc() ? ArcSelection.LOW : ArcSelection.HIGH;
    double measuredFlywheelSurfaceSpeedMps = rs.getShooterFlywheelSurfaceSpeedMps();

    boolean passUseFixedMaxHood = rs.getShooterMode() == RobotState.ShooterMode.PASS;
    double passHoodMechanicalDeg = ShooterConstants.SHOOTER_HOOD_SETPOINT_MAX_DEG;
    double passThetaFixed =
        clampThetaToHoodLimits(physicsThetaRadFromMechanicalHoodDeg(passHoodMechanicalDeg));

    Translation3d shooterWorld = shooterPose3d.getTranslation();
    double sx = shooterWorld.getX();
    double sy = shooterWorld.getY();
    double sz = shooterWorld.getZ();
    double tx = targetTranslation3d.getX();
    double ty = targetTranslation3d.getY();
    double tz = targetTranslation3d.getZ();
    double vx = shooterVelocity3d.getX();
    double vy = shooterVelocity3d.getY();
    double vz = shooterVelocity3d.getZ();
    double ax = shooterAcceleration3d.getX();
    double ay = shooterAcceleration3d.getY();
    double az = shooterAcceleration3d.getZ();

    double lx = sx;
    double ly = sy;
    double lz = sz;
    for (int i = 0;
        i < ShooterConstants.ShooterCalculatorConstants.MOVING_TARGET_LEAD_ITERATIONS;
        i++) {
      double rdx = tx - lx;
      double rdy = ty - ly;
      double rdz = tz - lz;
      double d = Math.hypot(rdx, rdy);
      if (passUseFixedMaxHood) {
        solveBallisticsFixedThetaInPlace(d, rdz, passThetaFixed);
      } else {
        solveBallisticsInPlace(d, rdz, arcSelection);
      }

      double tof = Double.isFinite(scratchTofSec) ? scratchTofSec : 0.0;
      double tof2 = tof * tof;
      lx = sx + vx * tof + 0.5 * ax * tof2;
      ly = sy + vy * tof + 0.5 * ay * tof2;
      lz = sz + vz * tof + 0.5 * az * tof2;
    }

    double relFx = tx - lx;
    double relFy = ty - ly;
    double relFz = tz - lz;
    double dFinal = Math.hypot(relFx, relFy);
    Rotation2d turretAngle = new Rotation2d(Math.atan2(relFy, relFx));

    if (passUseFixedMaxHood) {
      solveBallisticsFixedThetaInPlace(dFinal, relFz, passThetaFixed);
    } else {
      solveBallisticsInPlace(dFinal, relFz, arcSelection);
      enforceHubFunnelClearanceIfApplicable(
          shooterPose3d, targetTranslation3d, dFinal, relFz, arcSelection);
    }

    // Hood follows actual wheel speed with filtered correction only (simpler, less oscillation).
    ArcSelection effectiveArc = relFz < 0.0 ? ArcSelection.LOW : arcSelection;
    double tau = ShooterConstants.HoodCompensationConstants.RPM_FILTER_TIME_CONSTANT_SEC;
    // TODO(PHYSICS_TUNE): review alpha behavior if loop rate changes from 20ms.
    double alpha =
        Constants.loopPeriodSecs
            / Math.max(
                Constants.loopPeriodSecs + tau,
                ShooterConstants.ShooterCalculatorConstants.EPSILON_TIME_AND_RATIO);
    if (!Double.isFinite(filteredMeasuredFlywheelSurfaceSpeedMps)) {
      filteredMeasuredFlywheelSurfaceSpeedMps = measuredFlywheelSurfaceSpeedMps;
    }
    filteredMeasuredFlywheelSurfaceSpeedMps +=
        alpha * (measuredFlywheelSurfaceSpeedMps - filteredMeasuredFlywheelSurfaceSpeedMps);

    double measuredSpeed = filteredMeasuredFlywheelSurfaceSpeedMps;
    double thetaNominal = scratchThetaRad;
    double thetaUsedRad;
    double hoodAngle;

    if (passUseFixedMaxHood) {
      thetaUsedRad = passThetaFixed;
      hoodAngle = Units.degreesToRadians(passHoodMechanicalDeg);
    } else {
      double thetaFromMeasured = solveThetaForSpeed(measuredSpeed, dFinal, relFz, effectiveArc);
      double thetaError = 0.0;
      if (Double.isFinite(thetaFromMeasured)
          && measuredSpeed > ShooterConstants.ShooterCalculatorConstants.MIN_VALID_SHOT_SPEED_MPS) {
        thetaError = clampThetaToHoodLimits(thetaFromMeasured) - thetaNominal;
      }
      double gain = ShooterConstants.HoodCompensationConstants.NORMAL_GAIN;
      double maxCorrRad =
          Units.degreesToRadians(ShooterConstants.HoodCompensationConstants.MAX_CORRECTION_DEG);
      double thetaCorrected =
          thetaNominal + MathUtil.clamp(thetaError * gain, -maxCorrRad, maxCorrRad);
      thetaUsedRad = clampThetaToHoodLimits(thetaCorrected);

      hoodAngle = mechanicalHoodAngleRadFromPhysicsTheta(thetaUsedRad);
      if (!Double.isNaN(lastHoodAngle)) {
        double maxDelta =
            Units.degreesToRadians(
                    ShooterConstants.HoodCompensationConstants.MAX_CORRECTION_RATE_DEG_PER_SEC)
                * Constants.loopPeriodSecs;
        hoodAngle = MathUtil.clamp(hoodAngle, lastHoodAngle - maxDelta, lastHoodAngle + maxDelta);
      }
    }
    double tofUsed = timeOfFlightFromPhysics(dFinal, scratchExitSpeedMps, scratchThetaRad);

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
    double ratioEps = ShooterConstants.ShooterCalculatorConstants.EPSILON_TIME_AND_RATIO;
    lastPhysicsMinToEmpiricalRpmRatio =
        scratchRpmWheel > ratioEps
            ? minRpmIdeal / scratchRpmWheel
            : 1.0;
    // TODO(PHYSICS_TUNE): revisit clamp limits if drivetrain brownout causes deeper RPM dips.
    measuredToCommandRpmRatio =
        scratchRpmWheel > ratioEps
            ? MathUtil.clamp(
                rpmFromSurfaceVelocity(measuredSpeed) / scratchRpmWheel,
                0.0,
                ShooterConstants.ShooterCalculatorConstants.MEASURED_TO_COMMAND_RPM_RATIO_MAX)
            : 1.0;

    latestParameters =
        new ShootingParameters(
            true,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            scratchRpmWheel,
            tofUsed);
    if (ShooterConstants.Logging.LOG_SHOOTER_CALC_HOOD_COMP) {
      Logger.recordOutput(
          "Shooter/Calculator/HoodComp/MeasuredSurfaceSpeedMpsRaw",
          measuredFlywheelSurfaceSpeedMps);
      Logger.recordOutput(
          "Shooter/Calculator/HoodComp/MeasuredSurfaceSpeedMpsFiltered",
          filteredMeasuredFlywheelSurfaceSpeedMps);
      Logger.recordOutput(
          "Shooter/Calculator/HoodComp/ThetaNominalDeg", Units.radiansToDegrees(thetaNominal));
      Logger.recordOutput("Shooter/Calculator/HoodComp/ThetaUsedDeg", Units.radiansToDegrees(thetaUsedRad));
      Logger.recordOutput(
          "Shooter/Calculator/HoodComp/ThetaErrorDeg",
          Units.radiansToDegrees(thetaUsedRad - thetaNominal));
    }
  }

  public double getPhysicsShotEfficiencyScale() {
    // Use measured-vs-command ratio so tolerance adapts to real wheel sag, not only model ideality.
    double measuredRatio = measuredToCommandRpmRatio;
    if (!Double.isFinite(measuredRatio))
      return ShooterConstants.FlywheelShotConstants.PHYSICS_SHOT_EFFICIENCY_SCALE_NEUTRAL;
    measuredRatio =
        MathUtil.clamp(
            measuredRatio,
            0.0,
            ShooterConstants.ShooterCalculatorConstants.MEASURED_TO_COMMAND_RPM_RATIO_MAX);
    double unity = ShooterConstants.FlywheelShotConstants.PHYSICS_SHOT_EFFICIENCY_SCALE_NEUTRAL;
    return MathUtil.clamp(
        unity + (unity - Math.min(measuredRatio, unity)),
        ShooterConstants.ShooterCalculatorConstants.PHYSICS_SHOT_EFFICIENCY_SCALE_OUT_MIN,
        ShooterConstants.ShooterCalculatorConstants.PHYSICS_SHOT_EFFICIENCY_SCALE_OUT_MAX);
  }

  public double getPhysicsMinToEmpiricalRpmRatio() {
    return lastPhysicsMinToEmpiricalRpmRatio;
  }

  public double getMeasuredToCommandRpmRatio() {
    return measuredToCommandRpmRatio;
  }

  public void clearShootingParameters() {
    latestParameters = null;
    filteredMeasuredFlywheelSurfaceSpeedMps = Double.NaN;
    filteredShooterFieldAccelNative = null;
    RobotState.getInstance().clearShooterSolveInputs();
  }

  /**
   * Run once per loop after {@code CommandScheduler.getInstance().run()} and before {@link
   * frc.robot.util.FullSubsystem#runAllPeriodicAfterScheduler()}: trench checks, solve refresh,
   * idle/custom goals on mechanisms, readiness in {@link RobotState}. Tracking goals for
   * flywheel/hood/turret remain driven by default commands reading {@link #getParameters()}.
   */
  public void coordinateAfterScheduler(Flywheel flywheel, Hood hood, Turret turret) {
    RobotState rs = RobotState.getInstance();
    RobotState.ShooterMode mode = rs.getShooterMode();

    Pose2d robotEstimatedPose = rs.getEstimatedPose();
    ChassisSpeeds robotChassisSpeeds = rs.getRobotVelocity();
    Translation2d shooterFieldPoseNative = getShooterFieldPoseNative(robotEstimatedPose);
    ChassisSpeeds shooterFieldRelativeSpeedsNative =
        getShooterFieldRelativeSpeedsNative(robotEstimatedPose, robotChassisSpeeds);
    Translation2d rawShooterAccelNative =
        getShooterFieldRelativeAccelNative(robotEstimatedPose, rs.getRobotAcceleration());
    double accelAlpha =
        ShooterConstants.ShooterCalculatorConstants.SHOOTER_LEAD_ACCEL_LOWPASS_ALPHA;
    if (filteredShooterFieldAccelNative == null) {
      filteredShooterFieldAccelNative = rawShooterAccelNative;
    } else {
      filteredShooterFieldAccelNative =
          rawShooterAccelNative
              .times(accelAlpha)
              .plus(filteredShooterFieldAccelNative.times(1.0 - accelAlpha));
    }
    Translation2d predictedFieldPoseNative =
        getPredictedFieldPoseNative(
            shooterFieldPoseNative,
            shooterFieldRelativeSpeedsNative,
            filteredShooterFieldAccelNative);
    Translation2d shooterFieldPose = AllianceFlipUtil.apply(shooterFieldPoseNative);
    Translation2d predictedFieldPose = AllianceFlipUtil.apply(predictedFieldPoseNative);
    boolean currentlyUnderTrench = isUnderTrenchOverhang(shooterFieldPose);
    boolean predictedUnderTrench = isUnderTrenchOverhang(predictedFieldPose);
    boolean predictedOrCurrentUnderTrench = currentlyUnderTrench || predictedUnderTrench;
    boolean hoodFoldedForTrench =
        hood.getMeasuredAngleDeg() <= ShooterConstants.ShooterAimConstants.Trench.SAFE_HOOD_ANGLE_DEG;
    boolean trenchTeleNear = predictedOrCurrentUnderTrench && !hoodFoldedForTrench;
    if (ShooterConstants.Logging.LOG_SHOOTER_COORD_EVERY_CYCLE) {
      Logger.recordOutput(
          "Shooter/ShootMode",
          shooterFieldPose.getX() > ShooterConstants.ShooterAimConstants.passPoint ? "PASS" : "HUB");
    }

    boolean needShooterBallisticsPose =
        mode == RobotState.ShooterMode.IDLE
            || mode == RobotState.ShooterMode.HUB
            || mode == RobotState.ShooterMode.PASS
            || mode == RobotState.ShooterMode.POINT_3D;

    Translation3d hubTarget3d = null;
    if (mode == RobotState.ShooterMode.IDLE
        || mode == RobotState.ShooterMode.CUSTOM
        || mode == RobotState.ShooterMode.HUB) {
      hubTarget3d = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
    }

    Translation3d pass3dTarget = new Translation3d();
    if (mode == RobotState.ShooterMode.PASS) {
      pass3dTarget = passTarget3dFromRobotPose(robotEstimatedPose);
    }

    Pose3d shooterPose3d = null;
    Translation3d shooterVelocity3d = null;
    Translation3d shooterAcceleration3d = null;
    if (needShooterBallisticsPose) {
      shooterPose3d = new Pose3d(robotEstimatedPose);
      shooterPose3d =
          shooterPose3d.transformBy(ShooterConstants.ShooterAimConstants.TurretOffset.ROBOT_TO_TURRET);
      shooterVelocity3d =
          new Translation3d(
              shooterFieldRelativeSpeedsNative.vxMetersPerSecond,
              shooterFieldRelativeSpeedsNative.vyMetersPerSecond,
              0.0);
      shooterAcceleration3d =
          new Translation3d(
              filteredShooterFieldAccelNative.getX(),
              filteredShooterFieldAccelNative.getY(),
              0.0);
    }

    Translation3d solveTarget3d = new Translation3d();
    ShootingParameters shootParam = EMPTY_PARAMETERS;
    Translation3d[] solveTargetOut = new Translation3d[1];

    switch (mode) {
      case IDLE:
        Translation3d idleTurretTarget3d = hubTarget3d;
        if (shooterFieldPose.getX() > ShooterConstants.ShooterAimConstants.passPoint) {
          idleTurretTarget3d = passTarget3dFromRobotPose(robotEstimatedPose);
        }
        if (shooterPose3d != null && idleTurretTarget3d != null) {
          shootParam =
              runTrackingForTarget(
                  shooterPose3d,
                  shooterVelocity3d,
                  shooterAcceleration3d,
                  idleTurretTarget3d,
                  solveTargetOut);
        } else {
          clearShootingParameters();
        }
        solveTarget3d = idleTurretTarget3d != null ? idleTurretTarget3d : solveTarget3d;
        flywheel.setGoalSetPoint(
            DriverStation.isAutonomous() ? FlyWheelGoal.AUTOIDLE : FlyWheelGoal.TELEIDLE);
        hood.setGoalSetPoint(HoodGoal.ZERO);
        shootParam = getParameters();
        break;
      case CUSTOM:
        clearShootingParameters();
        solveTarget3d = hubTarget3d != null ? hubTarget3d : solveTarget3d;
        flywheel.setGoalSetPoint(FlyWheelGoal.CUSTOM);
        turret.setGoalSetPoint(TurretGoal.CUSTOM);
        hood.setGoalSetPoint(HoodGoal.CUSTOM);
        shootParam = getParameters();
        break;
      case HUB:
        shootParam =
            runTrackingForTarget(
                shooterPose3d,
                shooterVelocity3d,
                shooterAcceleration3d,
                hubTarget3d,
                solveTargetOut);
        break;
      case PASS:
        shootParam =
            runTrackingForTarget(
                shooterPose3d,
                shooterVelocity3d,
                shooterAcceleration3d,
                pass3dTarget,
                solveTargetOut);
        hood.setGoalSetPoint(ShooterConstants.SHOOTER_HOOD_SETPOINT_MAX_DEG);
        break;
      case POINT_3D:
        shootParam =
            runTrackingForTarget(
                shooterPose3d,
                shooterVelocity3d,
                shooterAcceleration3d,
                rs.getShooterPoint3dTarget(),
                solveTargetOut);
        break;
    }
    if (solveTargetOut[0] != null) {
      solveTarget3d = solveTargetOut[0];
    }

    boolean shooterTrackingTarget =
        mode == RobotState.ShooterMode.HUB
            || mode == RobotState.ShooterMode.PASS
            || mode == RobotState.ShooterMode.POINT_3D;
    flywheel.setPhysicsShotEfficiencyScale(
        shooterTrackingTarget
            ? getPhysicsShotEfficiencyScale()
            : ShooterConstants.FlywheelShotConstants.PHYSICS_SHOT_EFFICIENCY_SCALE_NEUTRAL);

    if (ShooterConstants.Logging.LOG_SHOOTER_COORD_EVERY_CYCLE) {
      Logger.recordOutput("Shooter/TrenchProtection/Active", trenchTeleNear);
      Logger.recordOutput("Shooter/ShooterMode", mode.toString());
    }
    if (ShooterConstants.Logging.SHOOTER_VERBOSE_TRENCH) {
      Logger.recordOutput("Shooter/TrenchProtection/CurrentFieldPose", shooterFieldPose);
      Logger.recordOutput("Shooter/TrenchProtection/PredictedFieldPose", predictedFieldPose);
      Logger.recordOutput("Shooter/TrenchProtection/CurrentlyUnderTrench", currentlyUnderTrench);
      Logger.recordOutput("Shooter/TrenchProtection/PredictedUnderTrench", predictedUnderTrench);
      Logger.recordOutput(
          "Shooter/TrenchProtection/PredictedOrCurrentUnderTrench", predictedOrCurrentUnderTrench);
      Logger.recordOutput("Shooter/TrenchProtection/HoodFoldedForTrench", hoodFoldedForTrench);
    }
    if (shooterTrackingTarget && ShooterConstants.Logging.SHOOTER_VERBOSE_AIMING) {
      Logger.recordOutput("Shooter/Calculator/FlywheelSpeed", shootParam.flywheelSpeed());
      Logger.recordOutput("Shooter/Calculator/TimeOfFlightSec", shootParam.timeOfFlightSec());
      Logger.recordOutput(
          "Shooter/Calculator/MeasuredToCommandRpmRatio", getMeasuredToCommandRpmRatio());
      Logger.recordOutput(
          "Shooter/Calculator/HoodAngleDeg", Units.radiansToDegrees(shootParam.hoodAngle()));
      Logger.recordOutput(
          "Shooter/Calculator/TurretAngleFieldCentric", shootParam.turretAngle().getDegrees());
      Logger.recordOutput(
          "Shooter/Calculator/TurretAngleTurretCentric",
          robotEstimatedPose.getRotation().minus(shootParam.turretAngle()).getDegrees());
      Logger.recordOutput(
          "Shooter/Target", new Pose2d(solveTarget3d.toTranslation2d(), Rotation2d.kZero));
    }

    rs.recordShooterMechanismProcess(
        flywheel.nearGoal,
        hood.nearGoal,
        turret.nearGoal,
        turret.constrainedBySoftLimit,
        trenchTeleNear);
    rs.setShooterReadyToShoot(readyToShoot(flywheel, hood, turret, trenchTeleNear));

    flywheel.applySetpointForOutput();
    hood.applySetpointForOutput();
    turret.applySetpointForOutput();
  }

  private static boolean readyToShoot(
      Flywheel flywheel, Hood hood, Turret turret, boolean trenchTeleNear) {
    return flywheel.nearGoal
        && hood.nearGoal
        && turret.nearGoal
        && !turret.constrainedBySoftLimit
        && !trenchTeleNear;
  }

  private ShootingParameters runTrackingForTarget(
      Pose3d shooterPose3d,
      Translation3d shooterVelocity3d,
      Translation3d shooterAcceleration3d,
      Translation3d target3d,
      Translation3d[] solveTargetOut) {
    if (target3d == null || shooterPose3d == null || shooterVelocity3d == null) {
      return getParameters();
    }
    if (shooterAcceleration3d == null) {
      shooterAcceleration3d = Translation3d.kZero;
    }
    Translation3d solveTarget3d = target3d;
    if (solveTargetOut != null && solveTargetOut.length > 0) {
      solveTargetOut[0] = solveTarget3d;
    }
    ArcSelection arcSelection =
        solveTarget3d.getZ() - shooterPose3d.getZ()
                < -ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS
            ? ArcSelection.LOW
            : ArcSelection.HIGH;
    RobotState.getInstance()
        .setShooterSolveInputs(
            shooterPose3d,
            shooterVelocity3d,
            shooterAcceleration3d,
            solveTarget3d,
            arcSelection == ArcSelection.LOW);
    refreshCachedParameters();
    return getParameters();
  }

  private static boolean isHubClassTarget(Translation3d target) {
    if (!ShooterConstants.HubFunnelClearance.ENABLE) {
      return false;
    }
    Translation3d hub = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
    return target.getDistance(hub)
        < ShooterConstants.HubFunnelClearance.HUB_CLASSIFY_MAX_DIST_METERS;
  }

  /**
   * Along a straight-line approach to the hole, horizontal offset from hole axis is {@code D − s}.
   * Require that offset to stay inside both the axial frustum and vertical aperture (see {@link
   * FieldConstants.Hub}).
   */
  private static boolean hubShotClearsFunnel(
      double sz,
      double D,
      double v,
      double theta,
      double radialMargin,
      double funnelCheckDepth,
      double funnelAxialDepth,
      int numSamples) {
    double g = ShooterConstants.GRAVITY;
    double cosT = Math.cos(theta);
    if (cosT < ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS) {
      return true;
    }
    double tanT = Math.tan(theta);
    double cos2 = cosT * cosT;

    double sStart = Math.max(0.0, D - funnelCheckDepth);
    for (int i = 1; i <= numSamples; i++) {
      double s = sStart + (D - sStart) * (i / (double) numSamples);
      double a = D - s;
      if (a > funnelAxialDepth) {
        continue;
      }
      double z = sz + s * tanT - g * s * s / (2.0 * v * v * cos2);
      double rA = FieldConstants.Hub.funnelInnerRadiusAtApproach(a, funnelAxialDepth);
      double rZ = FieldConstants.Hub.funnelInnerRadiusAtZ(z);
      double maxR = Math.min(rA, rZ) - radialMargin;
      if (maxR < 0.0 || a > maxR) {
        return false;
      }
    }
    return true;
  }

  private void enforceHubFunnelClearanceIfApplicable(
      Pose3d shooterPose3d,
      Translation3d targetTranslation3d,
      double dFinal,
      double relFz,
      ArcSelection arcSelection) {
    if (RobotState.getInstance().getShooterMode() == RobotState.ShooterMode.IDLE) {
      return;
    }
    if (!isHubClassTarget(targetTranslation3d)) {
      return;
    }
    Translation3d sTr = shooterPose3d.getTranslation();
    double margin = ShooterConstants.HubFunnelClearance.RADIAL_MARGIN_METERS;
    double checkDepth = ShooterConstants.HubFunnelClearance.FUNNEL_CHECK_DEPTH_METERS;
    double axialDepth = ShooterConstants.HubFunnelClearance.FUNNEL_AXIAL_DEPTH_METERS;
    int samples = ShooterConstants.HubFunnelClearance.TRAJECTORY_SAMPLES;
    int maxIter = ShooterConstants.HubFunnelClearance.MAX_SPEED_BOOST_ITERATIONS;
    double boost = ShooterConstants.HubFunnelClearance.SPEED_BOOST_FACTOR;

    ArcSelection effectiveArc = relFz < 0.0 ? ArcSelection.LOW : arcSelection;
    for (int iter = 0; iter < maxIter; iter++) {
      if (hubShotClearsFunnel(
          sTr.getZ(),
          dFinal,
          scratchExitSpeedMps,
          scratchThetaRad,
          margin,
          checkDepth,
          axialDepth,
          samples)) {
        if (iter > 0) {
          Logger.recordOutput("Shooter/HubFunnel/SpeedBoostIterations", iter);
        }
        return;
      }
      scratchRpmWheel =
          Math.min(
              scratchRpmWheel * boost,
              ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
      scratchExitSpeedMps = surfaceVelocityFromRpm(scratchRpmWheel);
      double theta = solveThetaForSpeed(scratchExitSpeedMps, dFinal, relFz, effectiveArc);
      if (!Double.isFinite(theta)) {
        break;
      }
      scratchThetaRad = clampThetaToHoodLimits(theta);
      scratchTofSec = timeOfFlightFromPhysics(dFinal, scratchExitSpeedMps, scratchThetaRad);
    }
    Logger.recordOutput("Shooter/HubFunnel/SpeedBoostIterations", maxIter);
  }

  /** Same LEFT/RIGHT pass 3D selection as {@link RobotState.ShooterMode#PASS}. */
  private static Translation3d passTarget3dFromRobotPose(Pose2d robotEstimatedPose) {
    boolean flipAlliance = AllianceFlipUtil.shouldFlip();
    double halfWidth = FieldConstants.fieldWidth * 0.5;
    double robotY = robotEstimatedPose.getY();
    if (!flipAlliance) {
      return robotY > halfWidth
          ? AllianceFlipUtil.apply(ShooterConstants.ShooterAimConstants.PassTargets.LEFT)
          : AllianceFlipUtil.apply(ShooterConstants.ShooterAimConstants.PassTargets.RIGHT);
    }
    return robotY < halfWidth
        ? AllianceFlipUtil.apply(ShooterConstants.ShooterAimConstants.PassTargets.LEFT)
        : AllianceFlipUtil.apply(ShooterConstants.ShooterAimConstants.PassTargets.RIGHT);
  }

  private static Translation2d getShooterFieldPoseNative(Pose2d robotEstimatedPose) {
    return robotEstimatedPose
        .getTranslation()
        .plus(
            ShooterConstants.ShooterAimConstants.TurretOffset.TURRET_OFFSET_ROBOT_2D.rotateBy(
                robotEstimatedPose.getRotation()));
  }

  private static ChassisSpeeds getShooterFieldRelativeSpeedsNative(
      Pose2d robotEstimatedPose, ChassisSpeeds robotChassisSpeeds) {
    double offsetX = ShooterConstants.ShooterAimConstants.TurretOffset.TURRET_OFFSET_ROBOT_2D.getX();
    double offsetY = ShooterConstants.ShooterAimConstants.TurretOffset.TURRET_OFFSET_ROBOT_2D.getY();
    double shooterRobotVx =
        robotChassisSpeeds.vxMetersPerSecond - robotChassisSpeeds.omegaRadiansPerSecond * offsetY;
    double shooterRobotVy =
        robotChassisSpeeds.vyMetersPerSecond + robotChassisSpeeds.omegaRadiansPerSecond * offsetX;
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        shooterRobotVx,
        shooterRobotVy,
        robotChassisSpeeds.omegaRadiansPerSecond,
        robotEstimatedPose.getRotation());
  }

  private static ChassisSpeeds clampRobotAccelerationRobotFrame(ChassisSpeeds robotAccel) {
    double lLim = ShooterConstants.ShooterCalculatorConstants.ROBOT_ACCEL_CLAMP_LINEAR_MPS2;
    double oLim = ShooterConstants.ShooterCalculatorConstants.ROBOT_ACCEL_CLAMP_ANGULAR_RAD_PER_SEC2;
    return new ChassisSpeeds(
        MathUtil.clamp(robotAccel.vxMetersPerSecond, -lLim, lLim),
        MathUtil.clamp(robotAccel.vyMetersPerSecond, -lLim, lLim),
        MathUtil.clamp(robotAccel.omegaRadiansPerSecond, -oLim, oLim));
  }

  /**
   * Field-native horizontal linear acceleration (m/s^2) of the shooter mount. {@code
   * robotAccelRobotFrame.omega} is angular acceleration (rad/s^2).
   */
  private static Translation2d getShooterFieldRelativeAccelNative(
      Pose2d robotEstimatedPose, ChassisSpeeds robotAccelRobotFrame) {
    ChassisSpeeds a = clampRobotAccelerationRobotFrame(robotAccelRobotFrame);
    double offsetX = ShooterConstants.ShooterAimConstants.TurretOffset.TURRET_OFFSET_ROBOT_2D.getX();
    double offsetY = ShooterConstants.ShooterAimConstants.TurretOffset.TURRET_OFFSET_ROBOT_2D.getY();
    double axS = a.vxMetersPerSecond - a.omegaRadiansPerSecond * offsetY;
    double ayS = a.vyMetersPerSecond + a.omegaRadiansPerSecond * offsetX;
    return new Translation2d(axS, ayS).rotateBy(robotEstimatedPose.getRotation());
  }

  /**
   * Constant-acceleration horizontal lookahead in field-native frame: {@code r + v t + 0.5 a t^2}.
   */
  private static Translation2d getPredictedFieldPoseNative(
      Translation2d shooterFieldPoseNative,
      ChassisSpeeds shooterFieldRelativeSpeedsNative,
      Translation2d shooterFieldAccelNative) {
    double t = ShooterConstants.ShooterAimConstants.Trench.LOOKAHEAD_TIME_SEC;
    Translation2d v =
        new Translation2d(
            shooterFieldRelativeSpeedsNative.vxMetersPerSecond,
            shooterFieldRelativeSpeedsNative.vyMetersPerSecond);
    Translation2d aTerm = shooterFieldAccelNative.times(0.5 * t * t);
    return shooterFieldPoseNative.plus(v.times(t)).plus(aTerm);
  }

  private static boolean isUnderTrenchOverhang(Translation2d fieldPose) {
    double x = fieldPose.getX();
    double y = fieldPose.getY();
    double margin = ShooterConstants.ShooterAimConstants.Trench.PROTECTION_MARGIN_METERS;
    boolean underAllianceTrenchX =
        x > ShooterConstants.ShooterAimConstants.Trench.START_X_METERS + margin
            && x < ShooterConstants.ShooterAimConstants.Trench.END_X_METERS - margin;
    boolean underOpponentTrenchX =
        x > ShooterConstants.ShooterAimConstants.Trench.OPP_START_X_METERS + margin
            && x < ShooterConstants.ShooterAimConstants.Trench.OPP_END_X_METERS - margin;
    boolean underAnyTrenchX = underAllianceTrenchX || underOpponentTrenchX;
    boolean underLeftTrench = y > ShooterConstants.ShooterAimConstants.Trench.LEFT_MIN_Y_METERS;
    boolean underRightTrench = y < ShooterConstants.ShooterAimConstants.Trench.RIGHT_MAX_Y_METERS;
    return underAnyTrenchX && (underLeftTrench || underRightTrench);
  }
}
