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

  /** Latest ballistic solve written by {@link #solveBallisticsInPlace}; no heap allocation. */
  private double scratchExitSpeedMps;

  private double scratchThetaRad;
  private double scratchTofSec;
  private double scratchRpmWheel;

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

  /**
   * Closed-form vacuum minimum exit speed with {@link
   * ShooterConstants.BallisticDragConstants#AIR_DRAG_EXIT_VELOCITY_MULTIPLIER} (physics-way model).
   */
  public static double minimumExitVelocity(double horizontalDistanceM, double heightDeltaM) {
    double g = ShooterConstants.GRAVITY;
    double d = horizontalDistanceM;
    double h = heightDeltaM;
    if (d < ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS) {
      return 0.0;
    }
    return Math.sqrt(g * (h + Math.hypot(d, h)))
        * ShooterConstants.BallisticDragConstants.AIR_DRAG_EXIT_VELOCITY_MULTIPLIER;
  }

  private static double yErrorAtDistanceWithAirDragAndBackspin(
      double surfaceVelocityMetersPerSec,
      double launchAngleRad,
      double horizontalDistanceM,
      double hTarget) {
    double v = surfaceVelocityMetersPerSec;
    double d = horizontalDistanceM;
    if (d < ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS) {
      return -hTarget;
    }

    double k = ShooterConstants.BallisticDragConstants.AIR_DRAG_LINEAR_COEFF_1_PER_S;
    double vx0 = v * Math.cos(launchAngleRad);
    double vy0 = v * Math.sin(launchAngleRad);
    if (vx0 <= ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS) {
      return Double.NaN;
    }

    if (k <= ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR) {
      double t = d / vx0;
      double y = vy0 * t - 0.5 * ShooterConstants.GRAVITY * t * t;
      return y - hTarget;
    }

    double rem = 1.0 - (k * d) / vx0;
    if (rem <= 0.0) {
      return Double.NaN;
    }
    double t = -Math.log(rem) / k;

    double omegaBall =
        ShooterConstants.BallisticDragConstants.BACKSPIN_SPIN_RATE_RATIO
            * (v / ShooterConstants.BallisticDragConstants.BALL_RADIUS_METERS);
    double aLift =
        ShooterConstants.BallisticDragConstants.BACKSPIN_MAGNUS_LIFT_COEFF * omegaBall * v;
    double gEff = ShooterConstants.GRAVITY - aLift;
    gEff = Math.max(0.0, gEff);

    double expTerm = rem;
    double oneMinus = 1.0 - expTerm;
    double y = (vy0 + gEff / k) * (oneMinus / k) - (gEff * t / k);
    return y - hTarget;
  }

  private static double timeToReachDistanceSecondsWithAirDragAndBackspin(
      double surfaceVelocityMetersPerSec, double launchAngleRad, double horizontalDistanceM) {
    double v = surfaceVelocityMetersPerSec;
    double d = horizontalDistanceM;
    if (d < ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS
        || v < ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS) {
      return 0.0;
    }
    double k = ShooterConstants.BallisticDragConstants.AIR_DRAG_LINEAR_COEFF_1_PER_S;
    if (k <= ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR) {
      double denom = v * Math.cos(launchAngleRad);
      return denom < ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS
          ? 0.0
          : d / denom;
    }
    double vx0 = v * Math.cos(launchAngleRad);
    if (vx0 <= ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS) {
      return Double.NaN;
    }
    double rem = 1.0 - (k * d) / vx0;
    if (rem <= 0.0) {
      return Double.NaN;
    }
    return -Math.log(rem) / k;
  }

  /**
   * High vs low arc when two elevation roots exist: {@link ShooterState} solve flag (geometric Δz)
   * combined with the legacy hood-relative height check.
   */
  private static boolean preferLowerArcCombined(
      boolean robotStateUseLowArc, double heightHoodArcMeters) {
    return robotStateUseLowArc
        || (heightHoodArcMeters
                - ShooterConstants.ComponentsConstants.Hood.BALLISTIC_EXTRA_HEIGHT_METERS
            < 0.0);
  }

  /**
   * Launch angle above horizontal (rad) with linear drag + backspin lift; picks high vs low arc like
   * physics-way {@code LaunchCalculator}.
   *
   * @param preferLowerArc if true, bias toward the lower of two valid brackets when multiple
   *     sign-change intervals exist.
   */
  private static double ballisticThetaAboveHorizontalRad(
      double surfaceVelocityMetersPerSec,
      double horizontalDistanceM,
      double heightHoodArcM,
      boolean preferLowerArc) {
    double v = surfaceVelocityMetersPerSec;
    double d = horizontalDistanceM;
    double h = heightHoodArcM;
    if (d < 1e-6 || v < 1e-6) {
      return Double.NaN;
    }

    double thetaMin = HOOD_THETA_CLAMP_MIN_RAD;
    double thetaMax = HOOD_THETA_CLAMP_MAX_RAD;
    if (!Double.isFinite(thetaMin) || !Double.isFinite(thetaMax) || thetaMax <= thetaMin) {
      thetaMin = Units.degreesToRadians(ShooterConstants.ShooterCalculatorConstants.FALLBACK_THETA_MIN_DEG);
      thetaMax = Units.degreesToRadians(ShooterConstants.ShooterCalculatorConstants.FALLBACK_THETA_MAX_DEG);
    }

    final int samples = 12;
    final int bisectionIters = 20;
    double thetaPrev = thetaMin;
    double errPrev = yErrorAtDistanceWithAirDragAndBackspin(v, thetaPrev, d, h);

    double bestTheta = thetaPrev;
    double bestAbsErr = Double.isFinite(errPrev) ? Math.abs(errPrev) : Double.POSITIVE_INFINITY;

    double chosenA = Double.NaN;
    double chosenB = Double.NaN;

    for (int i = 1; i <= samples; i++) {
      double t = i / (double) samples;
      double theta = thetaMin + (thetaMax - thetaMin) * t;
      double err = yErrorAtDistanceWithAirDragAndBackspin(v, theta, d, h);

      if (Double.isFinite(err)) {
        double absErr = Math.abs(err);
        if (absErr < bestAbsErr) {
          bestAbsErr = absErr;
          bestTheta = theta;
        }
      }

      if (Double.isFinite(errPrev) && Double.isFinite(err) && errPrev * err < 0.0) {
        if (Double.isNaN(chosenA)) {
          chosenA = thetaPrev;
          chosenB = theta;
        } else if (!preferLowerArc) {
          chosenA = thetaPrev;
          chosenB = theta;
        }
      }

      thetaPrev = theta;
      errPrev = err;
    }

    if (Double.isNaN(chosenA) || Double.isNaN(chosenB)) {
      return bestTheta;
    }

    double a = chosenA;
    double b = chosenB;
    double errA = yErrorAtDistanceWithAirDragAndBackspin(v, a, d, h);
    for (int iter = 0; iter < bisectionIters; iter++) {
      double mid = 0.5 * (a + b);
      double errMid = yErrorAtDistanceWithAirDragAndBackspin(v, mid, d, h);
      if (!Double.isFinite(errMid)) {
        b = mid;
        continue;
      }
      if (errA * errMid > 0.0) {
        a = mid;
        errA = errMid;
      } else {
        b = mid;
      }
    }

    return 0.5 * (a + b);
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

  /**
   * Flywheel + θ command: {@link #minimumExitVelocity} (drag multiplier), numeric θ with air drag +
   * Magnus, RPM from effective wheel kinematics.
   */
  private void computeCommandRpmWithDragModel(
      double horizontalDistanceM,
      double heightGeomM,
      boolean applyBallisticExtraHeight,
      boolean robotStateUseLowArc) {
    double hHoodArc =
        heightGeomM
            + (applyBallisticExtraHeight
                ? ShooterConstants.ComponentsConstants.Hood.BALLISTIC_EXTRA_HEIGHT_METERS
                : 0.0);
    double vCmd = minimumExitVelocity(horizontalDistanceM, heightGeomM);
    boolean preferLowerArc = preferLowerArcCombined(robotStateUseLowArc, hHoodArc);
    double theta =
        ballisticThetaAboveHorizontalRad(vCmd, horizontalDistanceM, hHoodArc, preferLowerArc);
    if (!Double.isFinite(theta)) {
      theta =
          Units.degreesToRadians(
              ShooterConstants.ShooterCalculatorConstants.DEFAULT_SOLVE_THETA_DEG);
    }
    theta = clampThetaToHoodLimits(theta);
    scratchThetaRad = theta;
    scratchRpmWheel =
        MathUtil.clamp(
            rpmFromSurfaceVelocity(vCmd),
            0.0,
            ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
    scratchExitSpeedMps = surfaceVelocityFromRpm(scratchRpmWheel);
    scratchTofSec =
        timeToReachDistanceSecondsWithAirDragAndBackspin(
            scratchExitSpeedMps, theta, horizontalDistanceM);
    if (!Double.isFinite(scratchTofSec) || scratchTofSec <= 0.0) {
      double denom = scratchExitSpeedMps * Math.cos(theta);
      scratchTofSec =
          denom < 0.15
              ? horizontalDistanceM / 0.15
              : horizontalDistanceM / denom;
    }
  }

  /** Fixed exit angle (e.g. PASS): vacuum required speed for geometry → RPM. */
  private void computeFixedThetaCommandRpm(
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
    double vCmd = vIdeal;
    scratchThetaRad = theta;
    scratchRpmWheel =
        MathUtil.clamp(
            rpmFromSurfaceVelocity(vCmd),
            0.0,
            ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
    scratchExitSpeedMps = surfaceVelocityFromRpm(scratchRpmWheel);
    scratchTofSec =
        timeToReachDistanceSecondsWithAirDragAndBackspin(
            scratchExitSpeedMps, theta, horizontalDistanceM);
    if (!Double.isFinite(scratchTofSec) || scratchTofSec <= 0.0) {
      double denom = scratchExitSpeedMps * Math.cos(theta);
      scratchTofSec =
          denom < 0.15
              ? horizontalDistanceM / 0.15
              : horizontalDistanceM / denom;
    }
  }

  /**
   * Latest solve from {@link #refreshCachedParameters}; safe to call from mechanism commands after the
   * shooter has refreshed the cache for this cycle.
   */
  public ShootingParameters getParameters() {
    return latestParameters != null ? latestParameters : EMPTY_PARAMETERS;
  }

  /**
   * Recomputes and stores {@link #getParameters()} from {@link RobotState} solve inputs. Hood and
   * flywheel share the same θ and exit speed from the final {@link #computeCommandRpmWithDragModel} /
   * {@link #computeFixedThetaCommandRpm} solve.
   */
  public void refreshCachedParameters() {
    ShooterState ss = ShooterState.getInstance();
    if (!ss.isShooterSolveInputsValid()) {
      return;
    }
    Pose3d shooterPose3d = ss.getShooterSolvePose3d();
    Translation3d shooterVelocity3d = ss.getShooterSolveVelocity3d();
    Translation3d shooterAcceleration3d = ss.getShooterSolveAcceleration3d();
    Translation3d targetTranslation3d = ss.getShooterSolveTarget3d();

    boolean passUseFixedMaxHood = ss.getShooterMode() == ShooterState.ShooterMode.PASS;
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
    boolean applyBallisticExtraHeight =
        tz
            > ShooterConstants.ShooterAimConstants.PASS_TARGET_Z_METERS
                + ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS;
    boolean robotStateUseLowArc = ss.isShooterSolveUseLowArc();
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
        computeFixedThetaCommandRpm(d, rdz, passThetaFixed);
      } else {
        computeCommandRpmWithDragModel(d, rdz, applyBallisticExtraHeight, robotStateUseLowArc);
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
      computeFixedThetaCommandRpm(dFinal, relFz, passThetaFixed);
    } else {
      computeCommandRpmWithDragModel(
          dFinal, relFz, applyBallisticExtraHeight, robotStateUseLowArc);
    }

    double thetaUsedRad;
    double hoodAngle;

    if (passUseFixedMaxHood) {
      thetaUsedRad = passThetaFixed;
      hoodAngle = Units.degreesToRadians(passHoodMechanicalDeg);
    } else {
      thetaUsedRad = scratchThetaRad;
      hoodAngle = mechanicalHoodAngleRadFromPhysicsTheta(scratchThetaRad);
    }
    double tofUsed =
        Double.isFinite(scratchTofSec) && scratchTofSec > 0.0
            ? scratchTofSec
            : timeToReachDistanceSecondsWithAirDragAndBackspin(
                scratchExitSpeedMps, scratchThetaRad, dFinal);
    if (!Double.isFinite(tofUsed) || tofUsed <= 0.0) {
      double denomS = scratchExitSpeedMps * Math.cos(scratchThetaRad);
      tofUsed = denomS < 0.15 ? dFinal / 0.15 : dFinal / denomS;
    }

    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;

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
          "Shooter/Calculator/HoodComp/CommandExitSpeedMps", scratchExitSpeedMps);
      Logger.recordOutput(
          "Shooter/Calculator/HoodComp/ThetaUsedDeg", Units.radiansToDegrees(thetaUsedRad));
    }
  }

  public void clearShootingParameters() {
    latestParameters = null;
    filteredShooterFieldAccelNative = null;
    ShooterState.getInstance().clearShooterSolveInputs();
  }

  /**
   * Run once per loop after {@code CommandScheduler.getInstance().run()} and before {@link
   * frc.robot.util.FullSubsystem#runAllPeriodicAfterScheduler()}: trench checks, solve refresh,
   * idle/custom goals on mechanisms, readiness in {@link ShooterState}. Tracking goals for
   * flywheel/hood/turret remain driven by default commands reading {@link #getParameters()}.
   */
  public void coordinateAfterScheduler(Flywheel flywheel, Hood hood, Turret turret) {
    RobotState rs = RobotState.getInstance();
    ShooterState ss = ShooterState.getInstance();
    ShooterState.ShooterMode mode = ss.getShooterMode();

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
        mode == ShooterState.ShooterMode.IDLE
            || mode == ShooterState.ShooterMode.HUB
            || mode == ShooterState.ShooterMode.PASS
            || mode == ShooterState.ShooterMode.POINT_3D;

    Translation3d hubTarget3d = null;
    if (mode == ShooterState.ShooterMode.IDLE
        || mode == ShooterState.ShooterMode.CUSTOM
        || mode == ShooterState.ShooterMode.HUB) {
      hubTarget3d = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
    }

    Translation3d pass3dTarget = new Translation3d();
    if (mode == ShooterState.ShooterMode.PASS) {
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
        clearShootingParameters();
        solveTarget3d = hubTarget3d != null ? hubTarget3d : solveTarget3d;
        flywheel.setGoalSetPoint(
            DriverStation.isAutonomous() ? FlyWheelGoal.AUTOIDLE : FlyWheelGoal.TELEIDLE);
        turret.setGoalSetPoint(TurretGoal.ZERO);
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
                ss.getShooterPoint3dTarget(),
                solveTargetOut);
        break;
    }
    if (solveTargetOut[0] != null) {
      solveTarget3d = solveTargetOut[0];
    }

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
    boolean shooterTrackingTarget =
        mode == ShooterState.ShooterMode.HUB
            || mode == ShooterState.ShooterMode.PASS
            || mode == ShooterState.ShooterMode.POINT_3D;
    if (shooterTrackingTarget && ShooterConstants.Logging.SHOOTER_VERBOSE_AIMING) {
      Logger.recordOutput("Shooter/Calculator/FlywheelSpeed", shootParam.flywheelSpeed());
      Logger.recordOutput("Shooter/Calculator/TimeOfFlightSec", shootParam.timeOfFlightSec());
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

    ss.recordShooterMechanismProcess(
        flywheel.nearGoal,
        hood.nearGoal,
        turret.nearGoal,
        turret.constrainedBySoftLimit,
        trenchTeleNear);
    ss.setShooterReadyToShoot(readyToShoot(flywheel, hood, turret, trenchTeleNear));

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
    // Target above shooter (positive delta Z) → high arc; below → low arc.
    double dz = solveTarget3d.getZ() - shooterPose3d.getZ();
    boolean useLowArc = dz <= ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS;
    ShooterState.getInstance()
        .setShooterSolveInputs(
            shooterPose3d,
            shooterVelocity3d,
            shooterAcceleration3d,
            solveTarget3d,
            useLowArc);
    refreshCachedParameters();
    return getParameters();
  }

  /** Same LEFT/RIGHT pass 3D selection as {@link ShooterState.ShooterMode#PASS}. */
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
