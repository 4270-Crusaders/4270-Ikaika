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
  private double scratchRpmWheel = 0.0;
  private double scratchTofSec = 0.0;

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

  /** Vacuum minimum-speed estimate used as baseline command speed. */
  public static double minimumExitVelocity(double horizontalDistanceM, double heightDeltaM) {
    double d = Math.max(horizontalDistanceM, ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS);
    double h = heightDeltaM;
    double g = ShooterConstants.GRAVITY;
    double root = g * (h + Math.hypot(d, h));
    if (!Double.isFinite(root) || root <= 0.0) return 0.0;
    return Math.sqrt(root);
  }

  /** Returns launch angle above horizontal (rad), selecting high/low branch from arc preference. */
  private static double ballisticThetaAboveHorizontalRad(
      double surfaceVelocityMetersPerSec,
      double horizontalDistanceM,
      double heightDeltaM,
      ShooterState.ShootingArc shootingArc) {
    double v = Math.max(surfaceVelocityMetersPerSec, ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS);
    double d = Math.max(horizontalDistanceM, ShooterConstants.ShooterCalculatorConstants.EPSILON_METERS);
    double h = heightDeltaM;
    double g = ShooterConstants.GRAVITY;

    double v2 = v * v;
    double disc = (v2 * v2) - g * (g * d * d + 2.0 * h * v2);
    if (!Double.isFinite(disc) || disc < 0.0) {
      return Double.NaN;
    }

    double sqrtDisc = Math.sqrt(disc);
    double denom = g * d;
    if (Math.abs(denom) < ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR) {
      return Double.NaN;
    }

    double low = Math.atan((v2 - sqrtDisc) / denom);
    double high = Math.atan((v2 + sqrtDisc) / denom);
    return shootingArc == ShooterState.ShootingArc.LOW ? low : high;
  }

  public static double mechanicalHoodAngleRadFromPhysicsTheta(double thetaAboveHorizontalRad) {
    double thetaDeg = Units.radiansToDegrees(thetaAboveHorizontalRad);
    double hoodDeg =
        ShooterConstants.ShooterCalculatorConstants.MECHANICAL_RIGHT_ANGLE_DEG
            - thetaDeg
            - ShooterConstants.ComponentsConstants.Hood.MECHANICAL_ANGLE_OFFSET_DEG;
    hoodDeg =
        MathUtil.clamp(
            hoodDeg,
            ShooterConstants.ComponentsConstants.Hood.MIN_DEGREE,
            ShooterConstants.ComponentsConstants.Hood.MAX_DEGREE);
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
    double h = heightDeltaM;
    double g = ShooterConstants.GRAVITY;
    double cosT = Math.cos(thetaAboveHorizontalRad);
    if (Math.abs(cosT) < ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR) {
      return Double.NaN;
    }
    double denom = 2.0 * cosT * cosT * (d * Math.tan(thetaAboveHorizontalRad) - h);
    if (denom <= ShooterConstants.ShooterCalculatorConstants.EPSILON_DENOMINATOR) {
      return Double.NaN;
    }
    return Math.sqrt((g * d * d) / denom);
  }

  private static double timeToReachDistanceSeconds(
      double surfaceVelocityMetersPerSec, double thetaAboveHorizontalRad, double horizontalDistanceM) {
    double vx = surfaceVelocityMetersPerSec * Math.cos(thetaAboveHorizontalRad);
    if (vx < ShooterConstants.ShooterCalculatorConstants.EPSILON_SURFACE_SPEED_MPS) {
      return Double.NaN;
    }
    return horizontalDistanceM / vx;
  }

  private void computeCommandRpmShoot(
      double horizontalDistanceM, double heightDeltaM, ShooterState.ShootingArc shootingArc) {
    double vMin = minimumExitVelocity(horizontalDistanceM, heightDeltaM);
    double vCmd =
        vMin * (1.0 + ShooterConstants.ShooterCalculatorConstants.MIN_EXIT_VELOCITY_HEADROOM_RATIO);
    double theta = ballisticThetaAboveHorizontalRad(vCmd, horizontalDistanceM, heightDeltaM, shootingArc);
    if (!Double.isFinite(theta)) {
      scratchThetaRad = Double.NaN;
      scratchRpmWheel = 0.0;
      scratchExitSpeedMps = 0.0;
      scratchTofSec = Double.NaN;
      return;
    }
    theta = clampThetaToHoodLimits(theta);
    scratchThetaRad = theta;
    scratchRpmWheel =
        MathUtil.clamp(
            rpmFromSurfaceVelocity(vCmd), 0.0, ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
    scratchExitSpeedMps = surfaceVelocityFromRpm(scratchRpmWheel);
    scratchTofSec = timeToReachDistanceSeconds(scratchExitSpeedMps, scratchThetaRad, horizontalDistanceM);
  }

  private void computeCommandRpmPass(
      double horizontalDistanceM, double heightDeltaM, double passThetaFixedRad) {
    double theta = clampThetaToHoodLimits(passThetaFixedRad);
    double vCmd = exitSpeedForFixedTheta(horizontalDistanceM, heightDeltaM, theta);
    if (!Double.isFinite(vCmd) || vCmd <= 0.0) {
      scratchThetaRad = theta;
      scratchRpmWheel = 0.0;
      scratchExitSpeedMps = 0.0;
      scratchTofSec = Double.NaN;
      return;
    }
    scratchThetaRad = theta;
    scratchRpmWheel =
        MathUtil.clamp(
            rpmFromSurfaceVelocity(vCmd), 0.0, ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_MAX_RPM);
    scratchExitSpeedMps = surfaceVelocityFromRpm(scratchRpmWheel);
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
  }

  public void clearShootingParameters() {
    latestParameters = null;
    ShooterState.getInstance().clearShooterSolveInputs();
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

    Translation3d hubTarget3d = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
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
      Logger.recordOutput("Shooter/Target", new Pose2d(solveTarget3d.toTranslation2d(), Rotation2d.kZero));
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
    ShooterState.ShootingArc shootingArc =
        target3d.getZ() <= shooterPose3d.getZ() ? ShooterState.ShootingArc.LOW : ShooterState.ShootingArc.HIGH;
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
