package frc.robot.commands.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.LaunchCalculator.ArcSelection;
import frc.robot.subsystems.shooter.LaunchCalculator.LaunchingParameters;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlyWheelGoal;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.Hood.HoodGoal;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.Turret.TurretGoal;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Runs after flywheel, turret, and hood {@code periodic()} each loop: trench evaluation, {@link
 * LaunchCalculator#refreshCachedParameters()} inputs, mechanism-ready flags. Does not set mechanism
 * goals while tracking — that is handled by each {@link Flywheel#runTrackTargetCommand()} etc.
 */
public class LaunchCoordinatorSubsystem extends SubsystemBase {
  private boolean trenchTeleNear = false;

  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;

  private final LaunchCalculator calculator = LaunchCalculator.getInstance();
  private Translation3d pass3dTarget = new Translation3d();
  private Translation3d solveTarget3d = new Translation3d();
  private LaunchingParameters launchParam =
      new LaunchingParameters(false, new Rotation2d(), 0, 0, 0, 0, 0);

  public LaunchCoordinatorSubsystem(Flywheel flywheel, Turret turret, Hood hood) {
    this.flywheel = flywheel;
    this.turret = turret;
    this.hood = hood;
  }

  public boolean isTrenchTeleNear() {
    return trenchTeleNear;
  }

  @Override
  public void periodic() {
    RobotState rs = RobotState.getInstance();
    RobotState.LauncherMode mode = rs.getLauncherMode();

    Pose2d robotEstimatedPose = rs.getEstimatedPose();
    ChassisSpeeds robotChassisSpeeds = rs.getRobotVelocity();
    Translation2d shooterFieldPoseNative = getShooterFieldPoseNative(robotEstimatedPose);
    ChassisSpeeds shooterFieldRelativeSpeedsNative =
        getShooterFieldRelativeSpeedsNative(robotEstimatedPose, robotChassisSpeeds);
    Translation2d predictedFieldPoseNative =
        getPredictedFieldPoseNative(shooterFieldPoseNative, shooterFieldRelativeSpeedsNative);
    Translation2d shooterFieldPose = AllianceFlipUtil.apply(shooterFieldPoseNative);
    Translation2d predictedFieldPose = AllianceFlipUtil.apply(predictedFieldPoseNative);
    boolean currentlyUnderTrench = isUnderTrenchOverhang(shooterFieldPose);
    boolean predictedUnderTrench = isUnderTrenchOverhang(predictedFieldPose);
    boolean predictedOrCurrentUnderTrench = currentlyUnderTrench || predictedUnderTrench;
    boolean hoodFoldedForTrench =
        hood.getMeasuredAngleDeg() <= ShooterConstants.LaunchConstants.Trench.SAFE_HOOD_ANGLE_DEG;
    trenchTeleNear = predictedOrCurrentUnderTrench && !hoodFoldedForTrench;
    if (ShooterConstants.Logging.LOG_LAUNCH_COORD_EVERY_CYCLE) {
      Logger.recordOutput(
          "Launcher/ShootMode",
          shooterFieldPose.getX() > ShooterConstants.LaunchConstants.passPoint ? "PASS" : "HUB");
    }

    boolean tracking =
        mode == RobotState.LauncherMode.HUB
            || mode == RobotState.LauncherMode.PASS
            || mode == RobotState.LauncherMode.POINT_3D;

    Translation3d hubTarget3d = null;
    if (mode == RobotState.LauncherMode.IDLE
        || mode == RobotState.LauncherMode.CUSTOM
        || mode == RobotState.LauncherMode.HUB) {
      hubTarget3d = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);
    }

    if (mode == RobotState.LauncherMode.PASS) {
      boolean flipAlliance = AllianceFlipUtil.shouldFlip();
      double halfWidth = FieldConstants.fieldWidth * 0.5;
      double robotY = robotEstimatedPose.getY();
      if (!flipAlliance) {
        pass3dTarget =
            robotY > halfWidth
                ? AllianceFlipUtil.apply(ShooterConstants.LaunchConstants.PassTargets.LEFT)
                : AllianceFlipUtil.apply(ShooterConstants.LaunchConstants.PassTargets.RIGHT);
      } else {
        pass3dTarget =
            robotY < halfWidth
                ? AllianceFlipUtil.apply(ShooterConstants.LaunchConstants.PassTargets.LEFT)
                : AllianceFlipUtil.apply(ShooterConstants.LaunchConstants.PassTargets.RIGHT);
      }
    }

    Pose3d shooterLaunchPose3d = null;
    Translation3d shooterVelocity3d = null;
    if (tracking) {
      Pose3d shooterPose3d = new Pose3d(robotEstimatedPose);
      shooterLaunchPose3d =
          shooterPose3d.transformBy(ShooterConstants.LaunchConstants.TurretOffset.ROBOT_TO_TURRET);
      shooterVelocity3d =
          new Translation3d(
              shooterFieldRelativeSpeedsNative.vxMetersPerSecond,
              shooterFieldRelativeSpeedsNative.vyMetersPerSecond,
              0.0);
    }

    switch (mode) {
      case IDLE:
        calculator.clearLaunchingParameters();
        solveTarget3d = hubTarget3d;
        flywheel.setGoalSetPoint(
            DriverStation.isAutonomous() ? FlyWheelGoal.AUTOIDLE : FlyWheelGoal.TELEIDLE);
        turret.setGoalSetPoint(TurretGoal.ZERO);
        hood.setGoalSetPoint(HoodGoal.ZERO);
        launchParam = calculator.getParameters();
        break;
      case CUSTOM:
        calculator.clearLaunchingParameters();
        solveTarget3d = hubTarget3d;
        runManualGoals();
        launchParam = calculator.getParameters();
        break;
      case HUB:
        runTrackingForTarget(shooterLaunchPose3d, shooterVelocity3d, hubTarget3d);
        break;
      case PASS:
        runTrackingForTarget(shooterLaunchPose3d, shooterVelocity3d, pass3dTarget);
        break;
      case POINT_3D:
        runTrackingForTarget(shooterLaunchPose3d, shooterVelocity3d, rs.getLauncherPoint3dTarget());
        break;
    }

    boolean aimingLaunch =
        mode == RobotState.LauncherMode.HUB
            || mode == RobotState.LauncherMode.PASS
            || mode == RobotState.LauncherMode.POINT_3D;
    flywheel.setPhysicsLaunchEfficiencyScale(
        aimingLaunch
            ? calculator.getPhysicsLaunchEfficiencyScale()
            : ShooterConstants.FlywheelShotConstants.PHYSICS_LAUNCH_EFFICIENCY_SCALE_NEUTRAL);

    if (ShooterConstants.Logging.LOG_LAUNCH_COORD_EVERY_CYCLE) {
      Logger.recordOutput("Launcher/TrenchProtection/Active", trenchTeleNear);
      Logger.recordOutput("Launcher/LauncherState", mode.toString());
    }
    if (ShooterConstants.Logging.SHOOTER_VERBOSE_TRENCH) {
      Logger.recordOutput("Launcher/TrenchProtection/CurrentFieldPose", shooterFieldPose);
      Logger.recordOutput("Launcher/TrenchProtection/PredictedFieldPose", predictedFieldPose);
      Logger.recordOutput("Launcher/TrenchProtection/CurrentlyUnderTrench", currentlyUnderTrench);
      Logger.recordOutput("Launcher/TrenchProtection/PredictedUnderTrench", predictedUnderTrench);
      Logger.recordOutput(
          "Launcher/TrenchProtection/PredictedOrCurrentUnderTrench", predictedOrCurrentUnderTrench);
      Logger.recordOutput("Launcher/TrenchProtection/HoodFoldedForTrench", hoodFoldedForTrench);
    }
    if (aimingLaunch && ShooterConstants.Logging.SHOOTER_VERBOSE_AIMING) {
      Logger.recordOutput("Launcher/LaunchCalculator/FlywheelSpeed", launchParam.flywheelSpeed());
      Logger.recordOutput("Launcher/LaunchCalculator/TimeOfFlightSec", launchParam.timeOfFlightSec());
      Logger.recordOutput(
          "Launcher/LaunchCalculator/MeasuredToCommandRpmRatio", calculator.getMeasuredToCommandRpmRatio());
      Logger.recordOutput(
          "Launcher/LaunchCalculator/HoodAngleDeg", Units.radiansToDegrees(launchParam.hoodAngle()));
      Logger.recordOutput(
          "Launcher/LaunchCalculator/TurretAngleFieldCentric", launchParam.turretAngle().getDegrees());
      Logger.recordOutput(
          "Launcher/LaunchCalculator/TurretAngleTurretCentric",
          robotEstimatedPose.getRotation().minus(launchParam.turretAngle()).getDegrees());
      Logger.recordOutput(
          "Launcher/Target", new Pose2d(solveTarget3d.toTranslation2d(), Rotation2d.kZero));
    }

    rs.recordLauncherMechanismProcess(
        flywheel.nearGoal,
        hood.nearGoal,
        turret.nearGoal,
        turret.constrainedBySoftLimit,
        trenchTeleNear);
    rs.setShooterReadyToShoot(readyToShoot());
  }

  private boolean readyToShoot() {
    return flywheel.nearGoal
        && hood.nearGoal
        && turret.nearGoal
        && !turret.constrainedBySoftLimit
        && !trenchTeleNear;
  }

  private void runManualGoals() {
    flywheel.setGoalSetPoint(FlyWheelGoal.CUSTOM);
    turret.setGoalSetPoint(TurretGoal.CUSTOM);
    hood.setGoalSetPoint(HoodGoal.CUSTOM);
  }

  private void runTrackingForTarget(
      Pose3d shooterLaunchPose3d,
      Translation3d shooterVelocity3d,
      Translation3d target3d) {
    solveTarget3d = target3d;
    ArcSelection arcSelection =
        solveTarget3d.getZ() - shooterLaunchPose3d.getZ()
                < -ShooterConstants.LaunchCalculatorConstants.EPSILON_METERS
            ? ArcSelection.LOW
            : ArcSelection.HIGH;
    RobotState.getInstance()
        .setLauncherSolveInputs(
            shooterLaunchPose3d,
            shooterVelocity3d,
            solveTarget3d,
            arcSelection == ArcSelection.LOW);
    calculator.refreshCachedParameters();
    launchParam = calculator.getParameters();
  }

  private Translation2d getShooterFieldPoseNative(Pose2d robotEstimatedPose) {
    return robotEstimatedPose
        .getTranslation()
        .plus(
            ShooterConstants.LaunchConstants.TurretOffset.TURRET_OFFSET_ROBOT_2D.rotateBy(
                robotEstimatedPose.getRotation()));
  }

  private ChassisSpeeds getShooterFieldRelativeSpeedsNative(
      Pose2d robotEstimatedPose, ChassisSpeeds robotChassisSpeeds) {
    double offsetX = ShooterConstants.LaunchConstants.TurretOffset.TURRET_OFFSET_ROBOT_2D.getX();
    double offsetY = ShooterConstants.LaunchConstants.TurretOffset.TURRET_OFFSET_ROBOT_2D.getY();
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

  private Translation2d getPredictedFieldPoseNative(
      Translation2d shooterFieldPoseNative, ChassisSpeeds shooterFieldRelativeSpeedsNative) {
    Translation2d lookaheadDelta =
        new Translation2d(
                shooterFieldRelativeSpeedsNative.vxMetersPerSecond,
                shooterFieldRelativeSpeedsNative.vyMetersPerSecond)
            .times(ShooterConstants.LaunchConstants.Trench.LOOKAHEAD_TIME_SEC);
    return shooterFieldPoseNative.plus(lookaheadDelta);
  }

  private boolean isUnderTrenchOverhang(Translation2d fieldPose) {
    double x = fieldPose.getX();
    double y = fieldPose.getY();
    double margin = ShooterConstants.LaunchConstants.Trench.PROTECTION_MARGIN_METERS;
    boolean underAllianceTrenchX =
        x > ShooterConstants.LaunchConstants.Trench.START_X_METERS + margin
            && x < ShooterConstants.LaunchConstants.Trench.END_X_METERS - margin;
    boolean underOpponentTrenchX =
        x > ShooterConstants.LaunchConstants.Trench.OPP_START_X_METERS + margin
            && x < ShooterConstants.LaunchConstants.Trench.OPP_END_X_METERS - margin;
    boolean underAnyTrenchX = underAllianceTrenchX || underOpponentTrenchX;
    boolean underLeftTrench = y > ShooterConstants.LaunchConstants.Trench.LEFT_MIN_Y_METERS;
    boolean underRightTrench = y < ShooterConstants.LaunchConstants.Trench.RIGHT_MAX_Y_METERS;
    return underAnyTrenchX && (underLeftTrench || underRightTrench);
  }
}
