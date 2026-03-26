// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.LauncherConstants.BALLISTIC_WHEEL_RADIUS_METERS;
import static frc.robot.subsystems.shooter.LauncherConstants.robotToTurret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import lombok.experimental.ExtensionMethod;
import frc.robot.Constants;
import frc.robot.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class LaunchCalculator {
  private static LaunchCalculator instance;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  private LaunchingParameters latestParameters = null;

  private static final int PHYSICS_EFFICIENCY_GRAPH_SAMPLES = 51;

  /** Precomputed linear graph: physics minimum RPM / empirical map RPM vs distance (launch band). */
  private static final double[] PHYSICS_EFFICIENCY_GRAPH_DISTANCE_M =
      new double[PHYSICS_EFFICIENCY_GRAPH_SAMPLES];

  private static final double[] PHYSICS_EFFICIENCY_GRAPH_MIN_RPM_OVER_EMP_RPM =
      new double[PHYSICS_EFFICIENCY_GRAPH_SAMPLES];

  static {
    double d0 = LauncherConstants.LAUNCH_MIN_DISTANCE_M;
    double d1 = LauncherConstants.LAUNCH_MAX_DISTANCE_M;
    for (int i = 0; i < PHYSICS_EFFICIENCY_GRAPH_SAMPLES; i++) {
      double d = d0 + (d1 - d0) * i / (PHYSICS_EFFICIENCY_GRAPH_SAMPLES - 1);
      PHYSICS_EFFICIENCY_GRAPH_DISTANCE_M[i] = d;
      // Fully-physics solve: no empirical table usage (and no interpolation).
      // Keep this graph as a stable 1.0 baseline for dashboards.
      PHYSICS_EFFICIENCY_GRAPH_MIN_RPM_OVER_EMP_RPM[i] = 1.0;
    }
  }

  /**
   * Ideal-minimum RPM (vacuum ballistic to geometric hub height) / empirical map RPM at current
   * lookahead. {@code ~1} means empirical matches ideal scale; lower means extra wheel speed vs
   * ideal (drag, compression, etc.).
   */
  private double lastPhysicsMinToEmpiricalRpmRatio = 1.0;

  /** Static curve — log once (was every getParameters call, ~100 doubles/frame). */
  private static boolean loggedPhysicsEfficiencyGraph;

  /**
   * Minimum exit speed (m/s) needed to reach horizontal distance {@code d} with vertical rise {@code
   * h} (hub opening above shooter exit), ignoring drag. Used as the basis for RPM vs distance.
   */
  public static double minimumExitVelocity(double horizontalDistanceM, double heightDeltaM) {
    double g = ShooterConstants.GRAVITY;
    double d = horizontalDistanceM;
    double h = heightDeltaM;
    if (d < 1e-6) {
      return 0;
    }
    // Closed-form vacuum minimum-exit speed, scaled to better match losses.
    return Math.sqrt(g * (h + Math.hypot(d, h)))
        * ShooterConstants.AIR_DRAG_EXIT_VELOCITY_MULTIPLIER;
  }

  public static double rpmFromSurfaceVelocity(double surfaceVelocityMetersPerSec) {
    double r = BALLISTIC_WHEEL_RADIUS_METERS;
    if (r < 1e-9) {
      return 0;
    }
    return surfaceVelocityMetersPerSec * 60.0 / (2.0 * Math.PI * r);
  }

  public static double surfaceVelocityFromRpm(double rpm) {
    return (rpm / 60.0) * (2.0 * Math.PI * BALLISTIC_WHEEL_RADIUS_METERS);
  }

  /**
   * Launch angle θ above horizontal (radians) for ballistics. When two solutions exist, uses the
   * higher arc. {@code heightDeltaM} may include {@link
   * ShooterConstants.HoodConstants#BALLISTIC_EXTRA_HEIGHT_METERS} for a higher trajectory.
   */
  public static double ballisticThetaAboveHorizontalRad(
      double surfaceVelocityMetersPerSec, double horizontalDistanceM, double heightDeltaM) {
    double v = surfaceVelocityMetersPerSec;
    double d = horizontalDistanceM;
    double h = heightDeltaM;
    if (d < 1e-6 || v < 1e-6) {
      return 0.0;
    }

    // When targeting something below the shooter, prefer the lower arc.
    // heightDeltaM may include BALLISTIC_EXTRA_HEIGHT_METERS, so subtract it back out to decide.
    boolean preferLowerArc = h - ShooterConstants.HoodConstants.BALLISTIC_EXTRA_HEIGHT_METERS < 0.0;

    // Theta bounds derived from hood mechanical limits.
    double thetaMinDeg =
        90.0
            + ShooterConstants.HoodConstants.MECHANICAL_ANGLE_OFFSET_DEG
            - ShooterConstants.HoodConstants.MAX_DEGREE;
    double thetaMaxDeg = 90.0 + ShooterConstants.HoodConstants.MECHANICAL_ANGLE_OFFSET_DEG;
    double thetaMin = Units.degreesToRadians(thetaMinDeg);
    double thetaMax = Units.degreesToRadians(thetaMaxDeg);

    // Fallback bounds.
    if (!Double.isFinite(thetaMin) || !Double.isFinite(thetaMax) || thetaMax <= thetaMin) {
      thetaMin = Units.degreesToRadians(5.0);
      thetaMax = Units.degreesToRadians(85.0);
    }

    // Sample + bisection over y(d)-h error to find roots.
    final int samples = 12;
    final int bisectionIters = 20;
    double thetaPrev = thetaMin;
    double errPrev = yErrorAtDistanceWithAirDragAndBackspin(v, thetaPrev, d, h);

    double bestTheta = thetaPrev;
    double bestAbsErr =
        Double.isFinite(errPrev) ? Math.abs(errPrev) : Double.POSITIVE_INFINITY;

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
          // Update so we end up with the higher-arc root interval.
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

  private static double timeToReachDistanceSecondsWithAirDragAndBackspin(
      double surfaceVelocityMetersPerSec,
      double launchAngleRad,
      double horizontalDistanceM) {
    double v = surfaceVelocityMetersPerSec;
    double d = horizontalDistanceM;
    if (d < 1e-6 || v < 1e-6) {
      return 0.0;
    }
    double k = ShooterConstants.AIR_DRAG_LINEAR_COEFF_1_PER_S;
    if (k <= 1e-9) {
      // No drag: t = d / (v*cos(theta))
      double denom = v * Math.cos(launchAngleRad);
      return denom < 1e-6 ? 0.0 : d / denom;
    }
    double vx0 = v * Math.cos(launchAngleRad);
    if (vx0 <= 1e-6) {
      return Double.NaN;
    }
    // x(t) = vx0/k * (1 - exp(-k t))  =>  exp(-k t) = 1 - k x / vx0
    double rem = 1.0 - (k * d) / vx0;
    if (rem <= 0.0) {
      return Double.NaN;
    }
    return -Math.log(rem) / k;
  }

  private static double yErrorAtDistanceWithAirDragAndBackspin(
      double surfaceVelocityMetersPerSec,
      double launchAngleRad,
      double horizontalDistanceM,
      double hTarget) {
    double v = surfaceVelocityMetersPerSec;
    double d = horizontalDistanceM;
    if (d < 1e-6) {
      return -hTarget;
    }

    double k = ShooterConstants.AIR_DRAG_LINEAR_COEFF_1_PER_S;
    double vx0 = v * Math.cos(launchAngleRad);
    double vy0 = v * Math.sin(launchAngleRad);
    if (vx0 <= 1e-6) {
      return Double.NaN;
    }

    if (k <= 1e-9) {
      // Vacuum y(d) for comparison/fallback.
      double t = d / vx0;
      double y = vy0 * t - 0.5 * ShooterConstants.GRAVITY * t * t;
      return y - hTarget;
    }

    double rem = 1.0 - (k * d) / vx0; // exp(-k t)
    if (rem <= 0.0) {
      return Double.NaN;
    }
    double t = -Math.log(rem) / k;

    // Backspin lift (Magnus) as a simplified upward acceleration term.
    double omegaBall =
        ShooterConstants.BACKSPIN_SPIN_RATE_RATIO * (v / ShooterConstants.BALL_RADIUS_METERS);
    double aLift = ShooterConstants.BACKSPIN_MAGNUS_LIFT_COEFF * omegaBall * v;
    double gEff = ShooterConstants.GRAVITY - aLift;
    gEff = Math.max(0.0, gEff);

    // With linear drag and effective gravity: y(t) =
    // (vy0 + gEff/k) * (1 - exp(-k t))/k - gEff * t / k
    double expTerm = rem; // exp(-k t)
    double oneMinus = 1.0 - expTerm;
    double y = (vy0 + gEff / k) * (oneMinus / k) - (gEff * t / k);
    return y - hTarget;
  }

  /**
   * Mechanical hood angle (radians) from physics launch angle θ (above horizontal). Convention:
   * {@code 0°} = toward sky, {@code 90°} = forward horizontal; only {@code [0, MAX_DEGREE]} is
   * used. {@code hoodDeg = 90° − θ_deg + MECHANICAL_ANGLE_OFFSET_DEG}.
   */
  public static double mechanicalHoodAngleRadFromPhysicsTheta(double thetaAboveHorizontalRad) {
    double thetaDeg = Units.radiansToDegrees(thetaAboveHorizontalRad);
    double hoodDeg =
        90.0
            - thetaDeg
            + ShooterConstants.HoodConstants.MECHANICAL_ANGLE_OFFSET_DEG;
    hoodDeg =
        MathUtil.clamp(
            hoodDeg, 0.0, ShooterConstants.HoodConstants.MAX_DEGREE);
    return Units.degreesToRadians(hoodDeg);
  }

  public LaunchingParameters getParameters(
      Pose2d robotEstimatedPose2d,
      ChassisSpeeds robotRelativeVelocityChassisSpeed,
      Translation2d targeTranslation2d,
      double IncreaseValue,
      double multiplier) {

    Pose2d estimatedPose = robotEstimatedPose2d;
    ChassisSpeeds robotRelativeVelocity = robotRelativeVelocityChassisSpeed;

    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * LauncherConstants.LAUNCH_PHASE_DELAY_S,
                robotRelativeVelocity.vyMetersPerSecond * LauncherConstants.LAUNCH_PHASE_DELAY_S,
                robotRelativeVelocity.omegaRadiansPerSecond * LauncherConstants.LAUNCH_PHASE_DELAY_S));

    Translation2d target = targeTranslation2d;

    Pose2d turretPosition = estimatedPose.transformBy(robotToTurret.toTransform2d());
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeVelocityChassisSpeed, robotEstimatedPose2d.getRotation());
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    double h = ShooterConstants.DELTA_HEIGHT;
    double hHoodArc =
        h + ShooterConstants.HoodConstants.BALLISTIC_EXTRA_HEIGHT_METERS;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    double timeOfFlight = 0;
    double flywheelRpm = 0;
    double thetaPhysicsRad = 0;

    // Fixed-point solve for moving-shot lead. Kept small since theta solving is numeric.
    for (int i = 0; i < 10; i++) {
      double d = lookaheadTurretToTargetDistance;
      double vCmd = minimumExitVelocity(d, h);
      thetaPhysicsRad = ballisticThetaAboveHorizontalRad(vCmd, d, hHoodArc);

      hoodAngle = mechanicalHoodAngleRadFromPhysicsTheta(thetaPhysicsRad);
      flywheelRpm = (rpmFromSurfaceVelocity(vCmd) + IncreaseValue) * multiplier;
      flywheelRpm =
          MathUtil.clamp(
              flywheelRpm, 0, ShooterConstants.FlywheelConstants.FLYWHEEL_MAX_RPM);

      timeOfFlight =
          timeToReachDistanceSecondsWithAirDragAndBackspin(vCmd, thetaPhysicsRad, d);
      if (!Double.isFinite(timeOfFlight) || timeOfFlight <= 0.0) {
        // Fallback to vacuum lead time to avoid NaNs.
        double denom = vCmd * Math.cos(thetaPhysicsRad);
        timeOfFlight = denom < 0.15 ? d / 0.15 : d / denom;
      }

      lastPhysicsMinToEmpiricalRpmRatio = 1.0;
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= LauncherConstants.LAUNCH_MIN_DISTANCE_M
                && lookaheadTurretToTargetDistance <= LauncherConstants.LAUNCH_MAX_DISTANCE_M,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelRpm);

    // Avoid per-frame Logger.recordOutput here — this runs inside a 20-iter solve and was a major
    // contributor to CommandScheduler loop overrun on the RIO.
    if (!loggedPhysicsEfficiencyGraph) {
      loggedPhysicsEfficiencyGraph = true;
      Logger.recordOutput(
          "Shooter/LaunchCalculator/PhysicsEfficiencyGraphDistanceM", PHYSICS_EFFICIENCY_GRAPH_DISTANCE_M);
      Logger.recordOutput(
          "Shooter/LaunchCalculator/PhysicsEfficiencyGraphMinRpmOverEmpRpm",
          PHYSICS_EFFICIENCY_GRAPH_MIN_RPM_OVER_EMP_RPM);
    }

    return latestParameters;
  }

  /**
   * Same solve as {@link #getParameters(Pose2d, ChassisSpeeds, Translation2d, double, double)},
   * but uses the target Z to compute vertical rise/drop. This lets you aim at any point in 3D.
   *
   * <p>Target Z is in the field coordinate system where {@code 0.0} is the floor.
   */
  public LaunchingParameters getParameters(
      Pose2d robotEstimatedPose2d,
      ChassisSpeeds robotRelativeVelocityChassisSpeed,
      Translation3d targetField,
      double IncreaseValue,
      double multiplier) {

    Pose2d estimatedPose = robotEstimatedPose2d;
    ChassisSpeeds robotRelativeVelocity = robotRelativeVelocityChassisSpeed;

    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * LauncherConstants.LAUNCH_PHASE_DELAY_S,
                robotRelativeVelocity.vyMetersPerSecond * LauncherConstants.LAUNCH_PHASE_DELAY_S,
                robotRelativeVelocity.omegaRadiansPerSecond * LauncherConstants.LAUNCH_PHASE_DELAY_S));

    Translation2d target = targetField.toTranslation2d();

    Pose2d turretPosition = estimatedPose.transformBy(robotToTurret.toTransform2d());
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeVelocityChassisSpeed, robotEstimatedPose2d.getRotation());
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Vertical rise/drop from the shooter exit to the target. (Hub uses a precomputed constant.)
    double h = targetField.getZ() - ShooterConstants.TURRET_HEIGHT;
    // BALLISTIC_EXTRA_HEIGHT_METERS is tuned for hub shots. For passing-to-floor targets
    // (z ~= PASS_TARGET_Z_METERS), we do not add this term to avoid biasing the trajectory
    // toward a higher arc.
    boolean passingToFloor =
        targetField.getZ() <= LauncherConstants.PASS_TARGET_Z_METERS + 1e-6;
    double hHoodArc =
        h +
            (passingToFloor
                ? 0.0
                : ShooterConstants.HoodConstants.BALLISTIC_EXTRA_HEIGHT_METERS);

    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    double timeOfFlight = 0;
    double flywheelRpm = 0;
    double thetaPhysicsRad = 0;

    for (int i = 0; i < 10; i++) {
      double d = lookaheadTurretToTargetDistance;
      double vCmd = minimumExitVelocity(d, h);
      thetaPhysicsRad = ballisticThetaAboveHorizontalRad(vCmd, d, hHoodArc);
      hoodAngle = mechanicalHoodAngleRadFromPhysicsTheta(thetaPhysicsRad);

      flywheelRpm = (rpmFromSurfaceVelocity(vCmd) + IncreaseValue) * multiplier;
      flywheelRpm =
          MathUtil.clamp(
              flywheelRpm, 0, ShooterConstants.FlywheelConstants.FLYWHEEL_MAX_RPM);

      timeOfFlight =
          timeToReachDistanceSecondsWithAirDragAndBackspin(vCmd, thetaPhysicsRad, d);
      if (!Double.isFinite(timeOfFlight) || timeOfFlight <= 0.0) {
        // Fallback to vacuum lead time to avoid NaNs.
        double denom = vCmd * Math.cos(thetaPhysicsRad);
        timeOfFlight = denom < 0.15 ? d / 0.15 : d / denom;
      }

      lastPhysicsMinToEmpiricalRpmRatio = 1.0;

      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;

      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;

    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= LauncherConstants.LAUNCH_MIN_DISTANCE_M
                && lookaheadTurretToTargetDistance <= LauncherConstants.LAUNCH_MAX_DISTANCE_M,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelRpm);

    // Avoid per-frame Logger.recordOutput here — this runs inside a 20-iter solve and was a major
    // contributor to CommandScheduler loop overrun on the RIO.
    if (!loggedPhysicsEfficiencyGraph) {
      loggedPhysicsEfficiencyGraph = true;
      Logger.recordOutput(
          "Shooter/LaunchCalculator/PhysicsEfficiencyGraphDistanceM",
          PHYSICS_EFFICIENCY_GRAPH_DISTANCE_M);
      Logger.recordOutput(
          "Shooter/LaunchCalculator/PhysicsEfficiencyGraphMinRpmOverEmpRpm",
          PHYSICS_EFFICIENCY_GRAPH_MIN_RPM_OVER_EMP_RPM);
    }

    return latestParameters;
  }

  /**
   * Scale for widening flywheel ready band when empirical RPM is well above ideal vacuum minimum
   * (drag / losses). Applied in {@link frc.robot.subsystems.shooter.flywheel.Flywheel} with {@code
   * NearGoalRpmTolerance}.
   */
  public double getPhysicsLaunchEfficiencyScale() {
    double r = lastPhysicsMinToEmpiricalRpmRatio;
    if (!Double.isFinite(r)) {
      return 1.0;
    }
    r = MathUtil.clamp(r, 0.0, 2.0);
    return MathUtil.clamp(1.0 + (1.0 - Math.min(r, 1.0)), 0.8, 1.6);
  }

  public double getPhysicsMinToEmpiricalRpmRatio() {
    return lastPhysicsMinToEmpiricalRpmRatio;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}
