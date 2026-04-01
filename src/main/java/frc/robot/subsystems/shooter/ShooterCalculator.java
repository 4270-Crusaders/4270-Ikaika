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
import frc.robot.util.SpeedUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class ShooterCalculator {
  private static ShooterCalculator instance;

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

  private Rotation2d lastTurretAngle = null;
  private double lastHoodAngleRad = Double.NaN;

  private double scratchExitSpeedMps = 0.0;
  private double scratchThetaRad = 0.0;
  /** Ballistic physics theta above horizontal before {@link #clampThetaToHoodLimits(double)}. */
  private double scratchThetaUnclampedRad = Double.NaN;
  private double scratchRpmWheel = 0.0;
  private double scratchTofSec = 0.0;

  /** Last valid solve outputs for {@link #recordSanityBundle}; NaN when solve invalid. */
  private double lastSolveLaunchThetaDeg = Double.NaN;
  private double lastSolveBallExitMps = Double.NaN;
  private double lastSolveTofSec = Double.NaN;

  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  private ShootingParameters latestParameters = null;
  private static final ShootingParameters EMPTY_PARAMETERS =
      new ShootingParameters(false, Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0);

  public static ShooterCalculator getInstance() {
    if (instance == null) instance = new ShooterCalculator();
    return instance;
  }

  private static double clampThetaToHoodLimits(double thetaAboveHorizontalRad) {
    double thetaMin =
        physicsThetaRadFromMechanicalHoodDeg(ShooterConstants.ComponentsConstants.Hood.MAX_DEGREE);
    double thetaMax =
        physicsThetaRadFromMechanicalHoodDeg(ShooterConstants.ComponentsConstants.Hood.MIN_DEGREE);
    return MathUtil.clamp(thetaAboveHorizontalRad, thetaMin, thetaMax);
  }

  private static double averageWheelRadiusMeters() {
    return ShooterConstants.ComponentsConstants.Flywheel.AVERAGE_WHEEL_RADIUS_METERS;
  }

  public static double rpmFromSurfaceVelocity(double surfaceVelocityMetersPerSec) {
    return SpeedUtil.rpmFromMetersPerSecond(surfaceVelocityMetersPerSec, averageWheelRadiusMeters());
  }

  public static double surfaceVelocityFromRpm(double rpm) {
    return SpeedUtil.metersPerSecondFromRpm(rpm, averageWheelRadiusMeters());
  }

  private record TrajectoryAtDistance(boolean isValid, double heightMeters, double timeSeconds) {}

  private static double estimateBallSpinRadPerSec(double launchSpeedMetersPerSec) {
    double radiusM = ShooterConstants.ProjectileConstants.FUEL_RADIUS_METERS;
    if (radiusM < ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR) {
      return 0.0;
    }

    // Geometry path: more compression generally couples stronger spin into the game piece.
    double freeDiameterM = ShooterConstants.ProjectileConstants.FUEL_DIAMETER_METERS;
    double compressionM = ShooterConstants.ComponentsConstants.Flywheel.BALL_COMPRESSION_METERS;
    double compressedDiameterM =
        Math.max(
            freeDiameterM - 2.0 * compressionM,
            ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS);
    double geometryScale =
        MathUtil.clamp(
            compressedDiameterM / Math.max(freeDiameterM, ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS),
            ShooterConstants.ProjectileConstants.SPIN_GEOMETRY_SCALE_MIN,
            ShooterConstants.ProjectileConstants.SPIN_GEOMETRY_SCALE_MAX);
    double omegaGeometry = (launchSpeedMetersPerSec / radiusM) * geometryScale;

    // Fallback path remains valid even if compression parameters are modified unexpectedly.
    double omegaFallback =
        (launchSpeedMetersPerSec / radiusM) * ShooterConstants.ProjectileConstants.SPIN_SLIP_RATIO;
    double omegaBase = Double.isFinite(omegaGeometry) ? omegaGeometry : omegaFallback;

    // Optional signed wheel mismatch term for future dual-wheel asymmetry tuning.
    double wheelDelta = 0.0;
    return Math.max(
        0.0,
        omegaBase + wheelDelta * ShooterConstants.ProjectileConstants.WHEEL_DELTA_SPIN_GAIN);
  }

  private static TrajectoryAtDistance simulateTrajectoryToDistance(
      double launchSpeedMetersPerSec, double thetaAboveHorizontalRad, double horizontalDistanceM) {
    if (launchSpeedMetersPerSec <= ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS
        || horizontalDistanceM < 0.0) {
      return new TrajectoryAtDistance(false, Double.NaN, Double.NaN);
    }

    double dt = ShooterConstants.ShooterCalculatorConstants.TRAJECTORY_SIM_DT_SEC;
    double maxTime = ShooterConstants.ShooterCalculatorConstants.TRAJECTORY_SIM_MAX_TIME_SEC;
    double g = ShooterConstants.GRAVITY;
    double dragFactor = ShooterConstants.ProjectileConstants.DRAG_ACCEL_FACTOR_PER_M;
    double magnusFactor = ShooterConstants.ProjectileConstants.MAGNUS_ACCEL_FACTOR_PER_M;
    double omegaBallRadPerSec = estimateBallSpinRadPerSec(launchSpeedMetersPerSec);

    double x = 0.0;
    double z = 0.0;
    double vx = launchSpeedMetersPerSec * Math.cos(thetaAboveHorizontalRad);
    double vz = launchSpeedMetersPerSec * Math.sin(thetaAboveHorizontalRad);
    double t = 0.0;

    double prevX = x;
    double prevZ = z;
    double prevT = t;
    while (t < maxTime) {
      if (x >= horizontalDistanceM) {
        break;
      }
      if (vx <= ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS && x < horizontalDistanceM) {
        return new TrajectoryAtDistance(false, Double.NaN, Double.NaN);
      }
      if (z < -1.5) {
        return new TrajectoryAtDistance(false, Double.NaN, Double.NaN);
      }

      prevX = x;
      prevZ = z;
      prevT = t;

      double speed = Math.hypot(vx, vz);
      double dragAx = -dragFactor * speed * vx;
      double dragAz = -dragFactor * speed * vz;
      double magnusAz = magnusFactor * speed * omegaBallRadPerSec;

      double ax = dragAx;
      double az = -g + dragAz + magnusAz;

      vx += ax * dt;
      vz += az * dt;
      x += vx * dt;
      z += vz * dt;
      t += dt;
    }

    if (x < horizontalDistanceM) {
      return new TrajectoryAtDistance(false, Double.NaN, Double.NaN);
    }

    double segmentDx = x - prevX;
    if (Math.abs(segmentDx) < ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR) {
      return new TrajectoryAtDistance(true, z, t);
    }
    double lerp = (horizontalDistanceM - prevX) / segmentDx;
    lerp = MathUtil.clamp(lerp, 0.0, 1.0);
    double zAtDistance = prevZ + (z - prevZ) * lerp;
    double tAtDistance = prevT + (t - prevT) * lerp;
    return new TrajectoryAtDistance(true, zAtDistance, tAtDistance);
  }

  private static double getExitTransferEfficiency() {
    return Math.max(
        ShooterConstants.ShooterCalculatorConstants.EPSILON_TIME_AND_RATIO,
        ShooterConstants.ComponentsConstants.Flywheel.ShotConstants.BALL_EXIT_TRANSFER_EFFICIENCY);
  }

  /** Ball exit speed predicted from wheel surface speed and slip/transfer efficiency. */
  private static double predictedBallSpeedFromWheelSurfaceSpeed(double wheelSurfaceSpeedMps) {
    return wheelSurfaceSpeedMps * getExitTransferEfficiency();
  }

  /** Wheel surface speed required for a desired ball exit speed. */
  private static double requiredWheelSurfaceSpeedForBallSpeed(double desiredBallSpeedMps) {
    return desiredBallSpeedMps / getExitTransferEfficiency();
  }

  /** Vacuum minimum-speed estimate used as baseline command speed. */
  public static double minimumExitVelocity(double horizontalDistanceM, double heightDeltaM) {
    double d = Math.max(horizontalDistanceM, ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS);
    double h = heightDeltaM;
    double g = ShooterConstants.GRAVITY;
    double root = g * (h + Math.hypot(d, h));
    if (!Double.isFinite(root) || root <= 0.0) return 0.0;
    return Math.sqrt(root);
  }

  /**
   * Launch angle <b>above horizontal</b> in radians (0 = along ground, π/2 = straight up). Not angle
   * from vertical; not slope-from-hood convention.
   */
  private static double ballisticThetaAboveHorizontalRad(
      double surfaceVelocityMetersPerSec,
      double horizontalDistanceM,
      double heightDeltaM,
      ShooterState.ShootingArc shootingArc) {
    double v =
        Math.max(
            surfaceVelocityMetersPerSec, ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS);
    double d = Math.max(horizontalDistanceM, ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS);
    double targetHeightM = heightDeltaM;

    double thetaMin =
        physicsThetaRadFromMechanicalHoodDeg(ShooterConstants.ComponentsConstants.Hood.MAX_DEGREE);
    double thetaMax =
        physicsThetaRadFromMechanicalHoodDeg(ShooterConstants.ComponentsConstants.Hood.MIN_DEGREE);
    if (!Double.isFinite(thetaMin) || !Double.isFinite(thetaMax) || thetaMax <= thetaMin) {
      thetaMin = Units.degreesToRadians(ShooterConstants.ShooterCalculatorConstants.FALLBACK_THETA_MIN_DEG);
      thetaMax = Units.degreesToRadians(ShooterConstants.ShooterCalculatorConstants.FALLBACK_THETA_MAX_DEG);
    }

    int samples = ShooterConstants.ShooterCalculatorConstants.TRAJECTORY_THETA_SCAN_SAMPLES;
    int bisectMax = ShooterConstants.ShooterCalculatorConstants.TRAJECTORY_BISECTION_MAX_ITERS;
    double step = (thetaMax - thetaMin) / Math.max(1, samples - 1);
    double lowRoot = Double.NaN;
    double highRoot = Double.NaN;

    double prevTheta = thetaMin;
    TrajectoryAtDistance prevShot = simulateTrajectoryToDistance(v, prevTheta, d);
    double prevErr = prevShot.isValid() ? prevShot.heightMeters() - targetHeightM : Double.NaN;

    for (int i = 1; i < samples; i++) {
      double theta = thetaMin + step * i;
      TrajectoryAtDistance shot = simulateTrajectoryToDistance(v, theta, d);
      double err = shot.isValid() ? shot.heightMeters() - targetHeightM : Double.NaN;
      if (Double.isFinite(prevErr) && Double.isFinite(err) && (prevErr == 0.0 || prevErr * err <= 0.0)) {
        double lo = prevTheta;
        double hi = theta;
        double loErr = prevErr;
        for (int iter = 0; iter < bisectMax; iter++) {
          double mid = 0.5 * (lo + hi);
          TrajectoryAtDistance midShot = simulateTrajectoryToDistance(v, mid, d);
          double midErr = midShot.isValid() ? midShot.heightMeters() - targetHeightM : Double.NaN;
          if (!Double.isFinite(midErr)) {
            break;
          }
          if (Math.abs(midErr) < 1e-4) {
            lo = mid;
            hi = mid;
            break;
          }
          if (loErr * midErr <= 0.0) {
            hi = mid;
          } else {
            lo = mid;
            loErr = midErr;
          }
        }
        double root = 0.5 * (lo + hi);
        if (!Double.isFinite(lowRoot) || root < lowRoot) lowRoot = root;
        if (!Double.isFinite(highRoot) || root > highRoot) highRoot = root;
      }
      prevTheta = theta;
      prevErr = err;
    }

    if (shootingArc == ShooterState.ShootingArc.LOW) {
      return lowRoot;
    }
    return highRoot;
  }

  public static double mechanicalHoodAngleRadFromPhysicsTheta(double thetaAboveHorizontalRad) {
    double thetaDeg = Units.radiansToDegrees(thetaAboveHorizontalRad);
    double hoodDeg =
        ShooterConstants.ShooterCalculatorConstants.MECHANICAL_RIGHT_ANGLE_DEG
            - thetaDeg
            - ShooterConstants.ComponentsConstants.Hood.MECHANICAL_ANGLE_OFFSET_DEG;
    return Units.degreesToRadians(hoodDeg);
  }

  public static double physicsThetaRadFromMechanicalHoodDeg(double hoodMechanicalDeg) {
    double thetaDeg =
        ShooterConstants.ShooterCalculatorConstants.MECHANICAL_RIGHT_ANGLE_DEG
            - hoodMechanicalDeg
            - ShooterConstants.ComponentsConstants.Hood.MECHANICAL_ANGLE_OFFSET_DEG;
    return Units.degreesToRadians(thetaDeg);
  }

  private static double exitSpeedForFixedTheta(
      double horizontalDistanceM, double heightDeltaM, double thetaAboveHorizontalRad) {
    double d = Math.max(horizontalDistanceM, ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS);
    double targetHeightM = heightDeltaM;

    double vMin = ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS;
    double vMax = ShooterConstants.ShooterCalculatorConstants.TRAJECTORY_MAX_LAUNCH_SPEED_MPS;
    int samples = ShooterConstants.ShooterCalculatorConstants.TRAJECTORY_THETA_SCAN_SAMPLES;
    int bisectMax = ShooterConstants.ShooterCalculatorConstants.TRAJECTORY_BISECTION_MAX_ITERS;

    double prevV = vMin;
    TrajectoryAtDistance prevShot = simulateTrajectoryToDistance(prevV, thetaAboveHorizontalRad, d);
    double prevErr = prevShot.isValid() ? prevShot.heightMeters() - targetHeightM : Double.NaN;
    for (int i = 1; i < samples; i++) {
      double ratio = (double) i / (double) (samples - 1);
      double v = vMin + (vMax - vMin) * ratio;
      TrajectoryAtDistance shot = simulateTrajectoryToDistance(v, thetaAboveHorizontalRad, d);
      double err = shot.isValid() ? shot.heightMeters() - targetHeightM : Double.NaN;

      if (Double.isFinite(prevErr) && Double.isFinite(err) && (prevErr == 0.0 || prevErr * err <= 0.0)) {
        double loV = prevV;
        double hiV = v;
        double loErr = prevErr;
        for (int iter = 0; iter < bisectMax; iter++) {
          double midV = 0.5 * (loV + hiV);
          TrajectoryAtDistance midShot = simulateTrajectoryToDistance(midV, thetaAboveHorizontalRad, d);
          double midErr = midShot.isValid() ? midShot.heightMeters() - targetHeightM : Double.NaN;
          if (!Double.isFinite(midErr)) {
            break;
          }
          if (Math.abs(midErr) < 1e-4) {
            return midV;
          }
          if (loErr * midErr <= 0.0) {
            hiV = midV;
          } else {
            loV = midV;
            loErr = midErr;
          }
        }
        return 0.5 * (loV + hiV);
      }
      prevV = v;
      prevErr = err;
    }
    return Double.NaN;
  }

  private static double timeToReachDistanceSeconds(
      double surfaceVelocityMetersPerSec, double thetaAboveHorizontalRad, double horizontalDistanceM) {
    double d = Math.max(horizontalDistanceM, 0.0);
    TrajectoryAtDistance shot =
        simulateTrajectoryToDistance(surfaceVelocityMetersPerSec, thetaAboveHorizontalRad, d);
    return shot.isValid() ? shot.timeSeconds() : Double.NaN;
  }

  private void computeCommandRpmShoot(
      double horizontalDistanceM, double heightDeltaM, ShooterState.ShootingArc shootingArc) {
    double vMinBall = minimumExitVelocity(horizontalDistanceM, heightDeltaM);
    double vCmdBall =
        vMinBall * (1.0 + ShooterConstants.ShooterCalculatorConstants.MIN_EXIT_VELOCITY_HEADROOM_RATIO);
    double theta =
        ballisticThetaAboveHorizontalRad(vCmdBall, horizontalDistanceM, heightDeltaM, shootingArc);
    if (!Double.isFinite(theta)) {
      scratchThetaUnclampedRad = Double.NaN;
      scratchThetaRad = Double.NaN;
      scratchRpmWheel = 0.0;
      scratchExitSpeedMps = 0.0;
      scratchTofSec = Double.NaN;
      return;
    }
    scratchThetaUnclampedRad = theta;
    theta = clampThetaToHoodLimits(theta);
    scratchThetaRad = theta;
    double wheelSurfaceCmdMps = requiredWheelSurfaceSpeedForBallSpeed(vCmdBall);
    scratchRpmWheel =
        MathUtil.clamp(
            rpmFromSurfaceVelocity(wheelSurfaceCmdMps),
            0.0,
            ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
    scratchExitSpeedMps =
        predictedBallSpeedFromWheelSurfaceSpeed(surfaceVelocityFromRpm(scratchRpmWheel));
    scratchTofSec = timeToReachDistanceSeconds(scratchExitSpeedMps, scratchThetaRad, horizontalDistanceM);
  }

  private void computeCommandRpmPass(
      double horizontalDistanceM, double heightDeltaM, double passThetaFixedRad) {
    scratchThetaUnclampedRad = passThetaFixedRad;
    double theta = clampThetaToHoodLimits(passThetaFixedRad);
    double vCmdBall = exitSpeedForFixedTheta(horizontalDistanceM, heightDeltaM, theta);
    if (!Double.isFinite(vCmdBall) || vCmdBall <= 0.0) {
      scratchThetaRad = theta;
      scratchRpmWheel = 0.0;
      scratchExitSpeedMps = 0.0;
      scratchTofSec = Double.NaN;
      return;
    }
    scratchThetaRad = theta;
    double wheelSurfaceCmdMps = requiredWheelSurfaceSpeedForBallSpeed(vCmdBall);
    scratchRpmWheel =
        MathUtil.clamp(
            rpmFromSurfaceVelocity(wheelSurfaceCmdMps),
            0.0,
            ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
    scratchExitSpeedMps =
        predictedBallSpeedFromWheelSurfaceSpeed(surfaceVelocityFromRpm(scratchRpmWheel));
    scratchTofSec = timeToReachDistanceSeconds(scratchExitSpeedMps, scratchThetaRad, horizontalDistanceM);
  }

  public ShootingParameters getParameters() {
    return latestParameters != null ? latestParameters : EMPTY_PARAMETERS;
  }

  public void refreshCachedParameters() {
    ShooterState ss = ShooterState.getInstance();
    if (!ss.isShooterSolveInputsValid()) {
      return;
    }

    Pose3d shooterPose3d = ss.getShooterSolvePose3d();
    Translation3d shooterVelocity3d = ss.getShooterSolveVelocity3d();
    Translation3d targetTranslation3d = ss.getShooterSolveTarget3d();
    boolean passMode = ss.getShooterMode() == ShooterState.ShooterMode.PASS;

    double passMechanicalAngleDeg =
        ShooterConstants.ComponentsConstants.Hood.shootingConstants.PASS_ANGLE_DEG;
    double passThetaFixedRad = physicsThetaRadFromMechanicalHoodDeg(passMechanicalAngleDeg);

    double sx = shooterPose3d.getX();
    double sy = shooterPose3d.getY();
    double sz = shooterPose3d.getZ();
    double tx = targetTranslation3d.getX();
    double ty = targetTranslation3d.getY();
    double tz = targetTranslation3d.getZ();

    // Constant-velocity lead only (no acceleration).
    double lx = sx;
    double ly = sy;
    double lz = sz;
    for (int i = 0; i < ShooterConstants.ShooterCalculatorConstants.MOVING_TARGET_LEAD_ITERATIONS; i++) {
      double dIter = Math.hypot(tx - lx, ty - ly);
      double hIter = tz - lz;
      if (passMode) {
        computeCommandRpmPass(dIter, hIter, passThetaFixedRad);
      } else {
        computeCommandRpmShoot(dIter, hIter, ss.getShootingArc());
      }
      double tof = Double.isFinite(scratchTofSec) ? scratchTofSec : 0.0;
      lx = sx + shooterVelocity3d.getX() * tof;
      ly = sy + shooterVelocity3d.getY() * tof;
      lz = sz + shooterVelocity3d.getZ() * tof;
    }

    double relFx = tx - lx;
    double relFy = ty - ly;
    double relFz = tz - lz;
    double dFinal = Math.hypot(relFx, relFy);
    Rotation2d solvedTurretAngle = new Rotation2d(Math.atan2(relFy, relFx));

    if (passMode) {
      computeCommandRpmPass(dFinal, relFz, passThetaFixedRad);
    } else {
      computeCommandRpmShoot(dFinal, relFz, ss.getShootingArc());
    }

    if (!Double.isFinite(scratchThetaRad) || scratchRpmWheel <= 0.0) {
      lastSolveLaunchThetaDeg = Double.NaN;
      lastSolveBallExitMps = Double.NaN;
      lastSolveTofSec = Double.NaN;
      latestParameters = EMPTY_PARAMETERS;
      return;
    }

    double hoodAngleRad =
        passMode
            ? Units.degreesToRadians(passMechanicalAngleDeg)
            : mechanicalHoodAngleRadFromPhysicsTheta(scratchThetaRad);

    if (lastTurretAngle == null) {
      lastTurretAngle = solvedTurretAngle;
    }
    if (Double.isNaN(lastHoodAngleRad)) {
      lastHoodAngleRad = hoodAngleRad;
    }
    double turretVelocityRadPerSec =
        turretAngleFilter.calculate(
            solvedTurretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    double hoodVelocityRadPerSec =
        hoodAngleFilter.calculate((hoodAngleRad - lastHoodAngleRad) / Constants.loopPeriodSecs);
    lastTurretAngle = solvedTurretAngle;
    lastHoodAngleRad = hoodAngleRad;

    latestParameters =
        new ShootingParameters(
            true,
            solvedTurretAngle,
            turretVelocityRadPerSec,
            hoodAngleRad,
            hoodVelocityRadPerSec,
            scratchRpmWheel);

    lastSolveLaunchThetaDeg = Units.radiansToDegrees(scratchThetaRad);
    lastSolveBallExitMps = scratchExitSpeedMps;
    lastSolveTofSec = scratchTofSec;

    if (ShooterConstants.Logging.LOG_SHOOTER_CALC_HOOD_COMP
        || ShooterConstants.Logging.SHOOTER_VERBOSE_AIMING) {
      double mechHoodFromTrajectoryThetaRad =
          mechanicalHoodAngleRadFromPhysicsTheta(scratchThetaRad);
      double mechHoodFromUnclampedThetaRad =
          Double.isFinite(scratchThetaUnclampedRad)
              ? mechanicalHoodAngleRadFromPhysicsTheta(scratchThetaUnclampedRad)
              : Double.NaN;
      Logger.recordOutput(
          "Shooter/Calculator/Diagnostics/TrajectoryPhysicsThetaDeg",
          Units.radiansToDegrees(scratchThetaRad));
      Logger.recordOutput(
          "Shooter/Calculator/Diagnostics/TrajectoryPhysicsThetaUnclampedDeg",
          Units.radiansToDegrees(scratchThetaUnclampedRad));
      Logger.recordOutput(
          "Shooter/Calculator/Diagnostics/MechanicalHoodFromPhysicsThetaRad", hoodAngleRad);
      Logger.recordOutput(
          "Shooter/Calculator/Diagnostics/MechanicalHoodFromPhysicsThetaDeg",
          Units.radiansToDegrees(hoodAngleRad));
      Logger.recordOutput(
          "Shooter/Calculator/Diagnostics/MechanicalHoodFromTrajectoryThetaRad",
          mechHoodFromTrajectoryThetaRad);
      Logger.recordOutput(
          "Shooter/Calculator/Diagnostics/MechanicalHoodFromUnclampedPhysicsThetaDeg",
          Units.radiansToDegrees(mechHoodFromUnclampedThetaRad));
      Logger.recordOutput("Shooter/Calculator/Diagnostics/TrajectoryExitSpeedMps", scratchExitSpeedMps);
      Logger.recordOutput("Shooter/Calculator/Diagnostics/TrajectoryTimeOfFlightSec", scratchTofSec);
      Logger.recordOutput("Shooter/Calculator/Diagnostics/ShootingArc", ss.getShootingArc().name());
      Logger.recordOutput("Shooter/Calculator/Solve/LaunchThetaDeg", Units.radiansToDegrees(scratchThetaRad));
      Logger.recordOutput(
          "Shooter/Calculator/Solve/LaunchThetaUnclampedDeg",
          Units.radiansToDegrees(scratchThetaUnclampedRad));
      Logger.recordOutput("Shooter/Calculator/Solve/BallExitSpeedMps", scratchExitSpeedMps);
      Logger.recordOutput("Shooter/Calculator/Solve/TimeOfFlightSec", scratchTofSec);
      Logger.recordOutput("Shooter/Calculator/Solve/ArcSelection", ss.getShootingArc().name());
    }
  }

  public void clearShootingParameters() {
    latestParameters = null;
    lastSolveLaunchThetaDeg = Double.NaN;
    lastSolveBallExitMps = Double.NaN;
    lastSolveTofSec = Double.NaN;
    ShooterState.getInstance().clearShooterSolveInputs();
  }

  /**
   * Compact channels for one glance in AdvantageScope. Order matches common debug flow: geometry → aim →
   * ballistics → mechanisms.
   */
  private static void recordSanityBundle(
      ShooterState.ShooterMode mode,
      boolean solveValid,
      boolean trenchProtectionActive,
      String arcName,
      double shooterToTargetM,
      double lookAheadToTargetM,
      double heightDeltaM,
      double launchThetaDeg,
      double hoodCmdDeg,
      double turretFieldDeg,
      double flywheelRpm,
      double ballExitMps,
      double timeOfFlightSec) {
    Logger.recordOutput("Shooter/Calculator/Sanity/Mode", mode.name());
    Logger.recordOutput("Shooter/Calculator/Sanity/SolveValid", solveValid);
    Logger.recordOutput("Shooter/Calculator/Sanity/TrenchProtection", trenchProtectionActive);
    Logger.recordOutput("Shooter/Calculator/Sanity/Arc", arcName);
    Logger.recordOutput("Shooter/Calculator/Sanity/ShooterToTargetM", shooterToTargetM);
    Logger.recordOutput("Shooter/Calculator/Sanity/LookAheadToTargetM", lookAheadToTargetM);
    Logger.recordOutput("Shooter/Calculator/Sanity/HeightDeltaM", heightDeltaM);
    Logger.recordOutput("Shooter/Calculator/Sanity/LaunchThetaDeg", launchThetaDeg);
    Logger.recordOutput("Shooter/Calculator/Sanity/HoodCmdDeg", hoodCmdDeg);
    Logger.recordOutput("Shooter/Calculator/Sanity/TurretFieldDeg", turretFieldDeg);
    Logger.recordOutput("Shooter/Calculator/Sanity/FlywheelRpm", flywheelRpm);
    Logger.recordOutput("Shooter/Calculator/Sanity/BallExitMps", ballExitMps);
    Logger.recordOutput("Shooter/Calculator/Sanity/TimeOfFlightSec", timeOfFlightSec);
  }

  public void coordinateAfterScheduler(Flywheel flywheel, Hood hood, Turret turret) {
    RobotState rs = RobotState.getInstance();
    ShooterState ss = ShooterState.getInstance();
    ShooterState.ShooterMode mode = ss.getShooterMode();

    Pose2d robotEstimatedPose = rs.getEstimatedPose();
    ChassisSpeeds robotChassisSpeeds = rs.getRobotVelocity();
    Translation2d shooterFieldPoseNative = getShooterFieldPoseNative(robotEstimatedPose);
    ChassisSpeeds shooterFieldRelativeSpeedsNative =
        getShooterFieldRelativeSpeedsNative(robotEstimatedPose, robotChassisSpeeds);
    Translation2d predictedFieldPoseNative =
        getPredictedLookAheadFieldPoseNative(shooterFieldPoseNative, shooterFieldRelativeSpeedsNative);
    Translation2d shooterFieldPose = AllianceFlipUtil.apply(shooterFieldPoseNative);
    Translation2d predictedFieldPose = AllianceFlipUtil.apply(predictedFieldPoseNative);
    boolean trenchTeleNear =
        (isUnderTrenchOverhang(shooterFieldPose) || isUnderTrenchOverhang(predictedFieldPose))
            && hood.getMeasuredAngleDeg() > ShooterConstants.ShooterAimConstants.Trench.SAFE_HOOD_ANGLE_DEG;

    Translation3d hubTarget3d = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.minus(new Translation3d(0,0,Units.inchesToMeters(5))));
    Translation3d pass3dTarget = getPassTarget3dFromRobotLocation(robotEstimatedPose);
    Pose3d shooterPose3d =
        new Pose3d(robotEstimatedPose)
            .transformBy(ShooterConstants.ShooterAimConstants.TurretOffset.ROBOT_TO_TURRET);
    Translation3d shooterVelocity3d =
        new Translation3d(
            shooterFieldRelativeSpeedsNative.vxMetersPerSecond,
            shooterFieldRelativeSpeedsNative.vyMetersPerSecond,
            0.0);

    ShootingParameters shootParam = EMPTY_PARAMETERS;
    Translation3d solveTarget3d = Translation3d.kZero;
    switch (mode) {
      case IDLE:
        clearShootingParameters();
        flywheel.setGoalSetPoint(
            DriverStation.isAutonomous() ? FlyWheelGoal.IDLE : FlyWheelGoal.IDLE);
        turret.setGoalSetPoint(TurretGoal.ZERO);
        hood.setGoalSetPoint(HoodGoal.ZERO);
        solveTarget3d = hubTarget3d;
        break;
      case CUSTOM:
        clearShootingParameters();
        flywheel.setGoalSetPoint(FlyWheelGoal.CUSTOM);
        turret.setGoalSetPoint(TurretGoal.CUSTOM);
        hood.setGoalSetPoint(HoodGoal.CUSTOM);
        solveTarget3d = hubTarget3d;
        break;
      case HUB:
        solveTarget3d = hubTarget3d;
        shootParam = runTrackingForTarget(shooterPose3d, shooterVelocity3d, hubTarget3d);
        break;
      case PASS:
        solveTarget3d = pass3dTarget;
        shootParam = runTrackingForTarget(shooterPose3d, shooterVelocity3d, pass3dTarget);
        hood.setGoalSetPoint(ShooterConstants.ComponentsConstants.Hood.shootingConstants.PASS_ANGLE_DEG);
        break;
      case POINT_3D:
        solveTarget3d = ss.getShooterPoint3dTarget();
        shootParam = runTrackingForTarget(shooterPose3d, shooterVelocity3d, solveTarget3d);
        break;
    }

    if (ShooterConstants.Logging.SHOOTER_VERBOSE_AIMING && shootParam.isValid()) {
      Logger.recordOutput("Shooter/Calculator/FlywheelSpeed", shootParam.flywheelSpeed());
      Logger.recordOutput("Shooter/Calculator/HoodAngleDeg", Units.radiansToDegrees(shootParam.hoodAngle()));
      Logger.recordOutput("Shooter/Calculator/TurretAngleFieldCentric", shootParam.turretAngle().getDegrees());
      Logger.recordOutput("Shooter/Calculator/Target", new Pose2d(solveTarget3d.toTranslation2d(), Rotation2d.kZero));
    }

    boolean logInputs =
        ShooterConstants.Logging.LOG_SHOOTER_CALC_HOOD_COMP
            || ShooterConstants.Logging.SHOOTER_VERBOSE_AIMING;
    boolean logSanity =
        ShooterConstants.Logging.LOG_SHOOTER_SANITY_BUNDLE
            || ShooterConstants.Logging.SHOOTER_VERBOSE_AIMING;

    if (logInputs || logSanity) {
      Translation2d target2d = solveTarget3d.toTranslation2d();
      Translation2d shooterToTarget = target2d.minus(shooterFieldPose);
      Translation2d lookAheadToTarget = target2d.minus(predictedFieldPose);
      double shooterToTargetDistanceM = shooterToTarget.getNorm();
      double lookAheadToTargetDistanceM = lookAheadToTarget.getNorm();
      double targetHeightDeltaFromShooterM = solveTarget3d.getZ() - shooterPose3d.getZ();

      if (logInputs) {
        Logger.recordOutput("Shooter/Calculator/Inputs/Mode", mode.name());
        Logger.recordOutput("Shooter/Calculator/Inputs/ShooterFieldPose", shooterFieldPose);
        Logger.recordOutput("Shooter/Calculator/Inputs/LookAheadFieldPose", predictedFieldPose);
        Logger.recordOutput("Shooter/Calculator/Inputs/TargetFieldPose2d", target2d);
        Logger.recordOutput("Shooter/Calculator/Inputs/TargetFieldPose3d", solveTarget3d);
        Logger.recordOutput("Shooter/Calculator/Inputs/ShooterVelocityFieldX", shooterVelocity3d.getX());
        Logger.recordOutput("Shooter/Calculator/Inputs/ShooterVelocityFieldY", shooterVelocity3d.getY());
        Logger.recordOutput("Shooter/Calculator/Inputs/ShooterVelocityFieldZ", shooterVelocity3d.getZ());
        Logger.recordOutput(
            "Shooter/Calculator/Inputs/ShooterToTargetDistanceM", shooterToTargetDistanceM);
        Logger.recordOutput(
            "Shooter/Calculator/Inputs/LookAheadToTargetDistanceM", lookAheadToTargetDistanceM);
        Logger.recordOutput(
            "Shooter/Calculator/Inputs/ShooterToTargetDeltaXM", shooterToTarget.getX());
        Logger.recordOutput(
            "Shooter/Calculator/Inputs/ShooterToTargetDeltaYM", shooterToTarget.getY());
        Logger.recordOutput(
            "Shooter/Calculator/Inputs/LookAheadToTargetDeltaXM", lookAheadToTarget.getX());
        Logger.recordOutput(
            "Shooter/Calculator/Inputs/LookAheadToTargetDeltaYM", lookAheadToTarget.getY());
        Logger.recordOutput(
            "Shooter/Calculator/Inputs/TargetHeightDeltaFromShooterM", targetHeightDeltaFromShooterM);
        Logger.recordOutput("Shooter/Calculator/Inputs/TrenchProtectionActive", trenchTeleNear);
      }

      if (logSanity) {
        boolean valid = shootParam.isValid();
        recordSanityBundle(
            mode,
            valid,
            trenchTeleNear,
            ss.getShootingArc().name(),
            shooterToTargetDistanceM,
            lookAheadToTargetDistanceM,
            targetHeightDeltaFromShooterM,
            valid ? lastSolveLaunchThetaDeg : Double.NaN,
            valid ? Units.radiansToDegrees(shootParam.hoodAngle()) : Double.NaN,
            valid ? shootParam.turretAngle().getDegrees() : Double.NaN,
            valid ? shootParam.flywheelSpeed() : Double.NaN,
            valid ? lastSolveBallExitMps : Double.NaN,
            valid ? lastSolveTofSec : Double.NaN);
      }
    }

    ss.recordShooterMechanismProcess(flywheel.nearGoal, hood.nearGoal, turret.nearGoal, trenchTeleNear);
    ss.setShooterReadyToShoot(flywheel.nearGoal && hood.nearGoal && turret.nearGoal && !trenchTeleNear);

    flywheel.applySetpointForOutput();
    hood.applySetpointForOutput();
    turret.applySetpointForOutput();
  }

  private ShootingParameters runTrackingForTarget(
      Pose3d shooterPose3d, Translation3d shooterVelocity3d, Translation3d target3d) {
    if (target3d == null || shooterPose3d == null || shooterVelocity3d == null) return getParameters();
    /*
     * Vacuum trajectory: two launch angles share the same speed and horizontal range (high = lob/steeper
     * {@link ShooterState.ShootingArc#HIGH}, low = direct/flatter {@link ShooterState.ShootingArc#LOW}).
     * Physics θ everywhere here is elevation <b>above horizontal</b> (0° along ground, 90° straight up).
     *
     * <p>Arc choice: if the aim point is strictly above the shooter origin, use the high arc; otherwise
     * use the low arc (level or downward targets).
     */
    ShooterState.ShootingArc shootingArc =
        target3d.getZ() > shooterPose3d.getZ()
            ? ShooterState.ShootingArc.HIGH
            : ShooterState.ShootingArc.LOW;
    ShooterState.getInstance().setShooterSolveInputs(shooterPose3d, shooterVelocity3d, target3d, shootingArc);
    refreshCachedParameters();
    return getParameters();
  }

  private static Translation3d getPassTarget3dFromRobotLocation(Pose2d robotEstimatedPose) {
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

  /** Constant velocity horizontal lookahead in field-native frame: {@code r + v t}. */
  private static Translation2d getPredictedLookAheadFieldPoseNative(
      Translation2d shooterFieldPoseNative, ChassisSpeeds shooterFieldRelativeSpeedsNative) {
    double t = ShooterConstants.ShooterAimConstants.Trench.LOOKAHEAD_TIME_SEC;
    Translation2d v =
        new Translation2d(
            shooterFieldRelativeSpeedsNative.vxMetersPerSecond,
            shooterFieldRelativeSpeedsNative.vyMetersPerSecond);
    return shooterFieldPoseNative.plus(v.times(t));
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
