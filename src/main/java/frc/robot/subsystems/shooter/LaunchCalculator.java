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

  /** Smooths field velocity along the shot so moving-RPM correction does not step (stream overshoot). */
  private final LinearFilter shotAlongVelocityFilter =
      LinearFilter.movingAverage((int) (0.12 / Constants.loopPeriodSecs));

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
      double rpmE = LauncherConstants.interpolateFlywheelRpm(d);
      double rpmP =
          rpmFromSurfaceVelocity(minimumExitVelocity(d, ShooterConstants.DELTA_HEIGHT));
      PHYSICS_EFFICIENCY_GRAPH_MIN_RPM_OVER_EMP_RPM[i] = rpmP / Math.max(rpmE, 1e-6);
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
    return Math.sqrt(g * (h + Math.hypot(d, h)));
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
   * Flywheel RPM vs horizontal distance (meters) from piecewise-linear empirical data in {@link
   * LauncherConstants}. {@code heightDeltaM} is unused but kept for call-site compatibility.
   */
  public static double flywheelRpmFromDistance(double horizontalDistanceM, double heightDeltaM) {
    double rpm = LauncherConstants.interpolateFlywheelRpm(horizontalDistanceM);
    if (!Double.isFinite(rpm)) {
      return 0;
    }
    return MathUtil.clamp(
        rpm, 0, ShooterConstants.FlywheelConstants.FLYWHEEL_MAX_RPM);
  }

  /**
   * Launch angle θ above horizontal (radians) for ballistics. When two solutions exist, uses the
   * higher arc. {@code heightDeltaM} may include {@link
   * ShooterConstants.HoodConstants#BALLISTIC_EXTRA_HEIGHT_METERS} for a higher trajectory.
   */
  public static double ballisticThetaAboveHorizontalRad(
      double surfaceVelocityMetersPerSec, double horizontalDistanceM, double heightDeltaM) {
    double g = ShooterConstants.GRAVITY;
    double d = horizontalDistanceM;
    double h = heightDeltaM;
    double v = surfaceVelocityMetersPerSec;
    if (d < 1e-6 || v < 1e-6) {
      return 0;
    }
    double A = g * d * d / (2 * v * v);
    double disc = d * d - 4 * A * (A + h);
    if (disc < 0) {
      double vMin = minimumExitVelocity(d, h);
      if (v < vMin) {
        return ballisticThetaAboveHorizontalRad(
            Math.max(vMin * 1.001, vMin + 1e-3), d, h);
      }
      return Math.atan2(h, d);
    }
    double sqrtD = Math.sqrt(disc);
    double u1 = (d + sqrtD) / (2 * A);
    double u2 = (d - sqrtD) / (2 * A);
    // When targeting something below the shooter, prefer the lower arc.
    // heightDeltaM may include BALLISTIC_EXTRA_HEIGHT_METERS, so subtract it back out to decide.
    boolean preferLowerArc =
        heightDeltaM - ShooterConstants.HoodConstants.BALLISTIC_EXTRA_HEIGHT_METERS < 0.0;
    double u = preferLowerArc ? Math.min(u1, u2) : Math.max(u1, u2);
    return Math.atan(u);
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
    double timeOfFlightPhysics = 0;

    for (int i = 0; i < 20; i++) {
      double d = lookaheadTurretToTargetDistance;
      double rpmBase = flywheelRpmFromDistance(d, h);
      flywheelRpm = (rpmBase + IncreaseValue) * multiplier;
      flywheelRpm =
          MathUtil.clamp(
              flywheelRpm, 0, ShooterConstants.FlywheelConstants.FLYWHEEL_MAX_RPM);
      double vCmd = surfaceVelocityFromRpm(flywheelRpm);
      thetaPhysicsRad = ballisticThetaAboveHorizontalRad(vCmd, d, hHoodArc);
      hoodAngle = Units.degreesToRadians(LauncherConstants.interpolateHoodMechanicalDeg(d));
      double denom = vCmd * Math.cos(thetaPhysicsRad);
      if (denom < 0.15) {
        timeOfFlightPhysics = d / 0.15;
      } else {
        timeOfFlightPhysics = d / denom;
      }
      timeOfFlight = LauncherConstants.interpolateTimeOfFlight(d);
      double rpmPhysicsMin =
          rpmFromSurfaceVelocity(minimumExitVelocity(d, ShooterConstants.DELTA_HEIGHT));
      lastPhysicsMinToEmpiricalRpmRatio = rpmPhysicsMin / Math.max(rpmBase, 1e-6);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Empirical RPM vs distance assumes v_robot ≈ 0. Field velocity along the horizontal shot adds
    // to ball range; subtract it from the wheel surface speed before converting back to RPM.
    Translation2d toTarget = target.minus(turretPosition.getTranslation());
    double distToTarget = toTarget.getNorm();
    if (distToTarget > 1e-6) {
      double vAlongRaw =
          (turretVelocityX * toTarget.getX() + turretVelocityY * toTarget.getY()) / distToTarget;
      double vAlong = shotAlongVelocityFilter.calculate(vAlongRaw);
      double vSurface = surfaceVelocityFromRpm(flywheelRpm);
      double vAdjusted =
          Math.max(
              vSurface - LauncherConstants.MOVING_SHOT_ALONG_V_CORRECTION_GAIN * vAlong, 0.0);
      flywheelRpm = rpmFromSurfaceVelocity(vAdjusted);
      flywheelRpm =
          MathUtil.clamp(
              flywheelRpm, 0, ShooterConstants.FlywheelConstants.FLYWHEEL_MAX_RPM);
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

    // If the target is below the shooter, avoid empirical/interpolated models that are tuned
    // primarily for upward (hub) shots.
    boolean usePhysicsForDownwardTarget = h < 0.0;

    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    double timeOfFlight = 0;
    double flywheelRpm = 0;
    double thetaPhysicsRad = 0;
    double timeOfFlightPhysics = 0;

    for (int i = 0; i < 20; i++) {
      double d = lookaheadTurretToTargetDistance;
      double rpmPhysicsMin = rpmFromSurfaceVelocity(minimumExitVelocity(d, h));
      double rpmBase =
          usePhysicsForDownwardTarget ? rpmPhysicsMin : flywheelRpmFromDistance(d, h);
      flywheelRpm = (rpmBase + IncreaseValue) * multiplier;
      flywheelRpm =
          MathUtil.clamp(
              flywheelRpm, 0, ShooterConstants.FlywheelConstants.FLYWHEEL_MAX_RPM);

      double vCmd = surfaceVelocityFromRpm(flywheelRpm);
      thetaPhysicsRad = ballisticThetaAboveHorizontalRad(vCmd, d, hHoodArc);

      // If aiming downward (e.g., passing to ground), derive hood angle from physics instead
      // of using the hub-tuned empirical hood curve.
      if (h < 0) {
        hoodAngle = mechanicalHoodAngleRadFromPhysicsTheta(thetaPhysicsRad);
      } else {
        hoodAngle = Units.degreesToRadians(LauncherConstants.interpolateHoodMechanicalDeg(d));
      }

      double denom = vCmd * Math.cos(thetaPhysicsRad);
      if (denom < 0.15) {
        timeOfFlightPhysics = d / 0.15;
      } else {
        timeOfFlightPhysics = d / denom;
      }

      timeOfFlight =
          usePhysicsForDownwardTarget
              ? timeOfFlightPhysics
              : LauncherConstants.interpolateTimeOfFlight(d);

      lastPhysicsMinToEmpiricalRpmRatio =
          usePhysicsForDownwardTarget ? 1.0 : rpmPhysicsMin / Math.max(rpmBase, 1e-6);

      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;

      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Empirical RPM vs distance assumes v_robot ≈ 0. Field velocity along the horizontal shot adds
    // to ball range; subtract it from the wheel surface speed before converting back to RPM.
    Translation2d toTarget = target.minus(turretPosition.getTranslation());
    double distToTarget = toTarget.getNorm();
    if (distToTarget > 1e-6) {
      double vAlongRaw =
          (turretVelocityX * toTarget.getX() + turretVelocityY * toTarget.getY()) / distToTarget;
      double vAlong = shotAlongVelocityFilter.calculate(vAlongRaw);
      double vSurface = surfaceVelocityFromRpm(flywheelRpm);
      double vAdjusted =
          Math.max(
              vSurface - LauncherConstants.MOVING_SHOT_ALONG_V_CORRECTION_GAIN * vAlong, 0.0);
      flywheelRpm = rpmFromSurfaceVelocity(vAdjusted);
      flywheelRpm =
          MathUtil.clamp(
              flywheelRpm, 0, ShooterConstants.FlywheelConstants.FLYWHEEL_MAX_RPM);
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
