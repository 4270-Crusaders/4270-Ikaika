package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.LaunchCalculator.LaunchingParameters;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlyWheelGoal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.Hood.HoodGoal;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.Turret.TurretGoal;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.geometry.AllianceFlipUtil;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public enum SHOOTER_STATE {
    AUTOHOME,
    HOME,
    AIM,

    AUTOAIMHUB,
    AUTOAIMPASS,

    /** Hub target — same launch solve as {@link #AUTOAIMHUB}. */
    HUB,
    /** Pass target — same launch solve as {@link #AUTOAIMPASS}. */
    PASS,

    ZERO,

    CUSTOM,

    /** Shoot/aim at an arbitrary field point (x,y,z) in meters (z=0 is the floor). */
    POINT_3D,
  }

  private boolean trenchTeleNear = false;

  public enum SHOOT_MODE {
    HUB,
    PASS
  }

  private static final LoggedTunableNumber RPMIncreaseValue =
      new LoggedTunableNumber("Shooter/Launch/RpmOffset", LauncherConstants.INCREASE);
  private static final LoggedTunableNumber RPMMultiplierValue =
      new LoggedTunableNumber("Shooter/Launch/RpmMultiplier", 1);

  @AutoLogOutput(key = "Shooter/ShooterState") private SHOOTER_STATE currentShooterState = SHOOTER_STATE.ZERO;
  @AutoLogOutput(key = "Shooter/ShootMode") private SHOOT_MODE currentShooterMode = SHOOT_MODE.HUB;

  private Flywheel flywheel;
  private Turret turret;
  private Hood hood;
  private double currentRPMIncrease = RPMIncreaseValue.get();
  private double currentRPMMultiplier = RPMMultiplierValue.get();

  private Pose2d RobotEstimatedPose = new Pose2d();
  private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();

  private Translation3d point3dTarget = FieldConstants.Hub.topCenterPoint;

  public Shooter(FlywheelIO flywheelIO, TurretIO turretIO, HoodIO hoodIO) {
    this.flywheel = new Flywheel(flywheelIO);
    this.turret = new Turret(turretIO);
    this.hood = new Hood(hoodIO);
  }

  public void setRobotEstimatedPose(Pose2d pose) {
    RobotEstimatedPose = pose;
  }

  public void setRobotSpeed(ChassisSpeeds speed){
    robotChassisSpeeds = speed;
  }

  public void setShooterState(SHOOTER_STATE shooterState) {
    this.currentShooterState = shooterState;
  }

  /**
   * Sets a 3D target for {@link SHOOTER_STATE#POINT_3D}.
   *
   * <p>Assumes the target is specified in the blue-field coordinate system; AllianceFlipUtil is
   * applied internally so it lines up with {@code RobotEstimatedPose}.
   */
  public void setPoint3dTarget(Translation3d targetFieldBluePerspective) {
    this.point3dTarget = AllianceFlipUtil.apply(targetFieldBluePerspective);
    this.currentShooterState = SHOOTER_STATE.POINT_3D;
  }

  public boolean readyToShoot() {
    return flywheel.nearGoal && turret.nearGoal && !trenchTeleNear;
  }

  LaunchingParameters launchParam = new LaunchingParameters(false, new Rotation2d(), 0, 0, 0, 0); //empty param
  LaunchCalculator calculator = LaunchCalculator.getInstance();
  Translation3d pass3dTarget = new Translation3d();
  Translation3d solveTarget3d = new Translation3d();

  @Override
  public void periodic() {

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          currentRPMIncrease = RPMIncreaseValue.get();
          currentRPMMultiplier = RPMMultiplierValue.get();
        },
        RPMIncreaseValue,
        RPMMultiplierValue);

    Translation2d poseOnField = AllianceFlipUtil.apply(RobotEstimatedPose.getTranslation());
    double fieldX = poseOnField.getX();
    trenchTeleNear =
        fieldX > FieldConstants.LinesVertical.startTrench
            && fieldX < FieldConstants.LinesVertical.endTrench;
    currentShooterMode =
        fieldX > LauncherConstants.passPoint ? SHOOT_MODE.PASS : SHOOT_MODE.HUB;

    boolean flipAlliance = AllianceFlipUtil.shouldFlip();
    double halfWidth = FieldConstants.fieldWidth * 0.5;
    double robotY = RobotEstimatedPose.getY();
    if (!flipAlliance) {
      pass3dTarget =
          robotY > halfWidth
              ? AllianceFlipUtil.apply(LauncherConstants.pass3dTargetLeft)
              : AllianceFlipUtil.apply(LauncherConstants.pass3dTargetRight);
    } else {
      pass3dTarget =
          robotY < halfWidth
              ? AllianceFlipUtil.apply(LauncherConstants.pass3dTargetLeft)
              : AllianceFlipUtil.apply(LauncherConstants.pass3dTargetRight);
    }

    Translation3d hubTarget3d = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);
    // Select what we're solving for this tick.
    solveTarget3d =
        switch (currentShooterState) {
          case POINT_3D -> point3dTarget; // user-specified 3D target
          case AUTOAIMHUB, HUB -> hubTarget3d;
          case AUTOAIMPASS, PASS -> pass3dTarget; // zone pass target at z=0
          case AIM -> currentShooterMode == SHOOT_MODE.PASS ? pass3dTarget : hubTarget3d;
          default -> hubTarget3d;
        };

    switch (currentShooterState) {
      case ZERO:
        turret.setGoalSetPoint(0);
        hood.setGoalSetPoint(0);
        flywheel.setGoalSetPoint(0);
        break;
      case HOME:
        flywheel.setGoalSetPoint(FlyWheelGoal.TELEIDLE);
        turret.setGoalSetPoint(0);
        hood.setGoalSetPoint(0);
        break;
      case AUTOHOME:
        flywheel.setGoalSetPoint(FlyWheelGoal.AUTOIDLE);
        turret.setGoalSetPoint(0);
        hood.setGoalSetPoint(0);
        break;
      case CUSTOM:
        flywheel.setGoalSetPoint(FlyWheelGoal.CUSTOM);
        turret.setGoalSetPoint(TurretGoal.CUSTOM);
        hood.setGoalSetPoint(HoodGoal.CUSTOM);
        break;

      case AIM:
      case AUTOAIMHUB:
      case HUB:
      case AUTOAIMPASS:
      case PASS:
      case POINT_3D:
        launchParam =
            calculator.getParameters(
                RobotEstimatedPose,
                robotChassisSpeeds,
                solveTarget3d,
                currentRPMIncrease,
                currentRPMMultiplier);

        double turretGoalDeg =
            trenchTeleNear ? 0 : RobotEstimatedPose.getRotation().minus(launchParam.turretAngle()).getDegrees();
        double hoodGoalDeg = trenchTeleNear ? 0 : Units.radiansToDegrees(launchParam.hoodAngle());

        turret.setGoalSetPoint(turretGoalDeg);
        hood.setGoalSetPoint(hoodGoalDeg);
        flywheel.setGoalSetPoint(
            flywheel.compensateLaunchRpmForBallThrough(launchParam.flywheelSpeed()));
        break;
      default:
        break;
    }

    boolean aimingLaunch =
        currentShooterState == SHOOTER_STATE.AIM
            || currentShooterState == SHOOTER_STATE.AUTOAIMHUB
            || currentShooterState == SHOOTER_STATE.AUTOAIMPASS
            || currentShooterState == SHOOTER_STATE.HUB
            || currentShooterState == SHOOTER_STATE.PASS
            || currentShooterState == SHOOTER_STATE.POINT_3D;
    flywheel.setPhysicsLaunchEfficiencyScale(
        aimingLaunch ? calculator.getPhysicsLaunchEfficiencyScale() : 1.0);

    flywheel.periodic();
    turret.periodic();
    hood.periodic();

    if (aimingLaunch) {
      double launchRpmAfterBallThrough =
          flywheel.compensateLaunchRpmForBallThrough(launchParam.flywheelSpeed());
      Logger.recordOutput("Shooter/LaunchCalculator/FlywheelSpeed", launchParam.flywheelSpeed(), RPM);
      Logger.recordOutput(
          "Shooter/Flywheel/LaunchRpmAfterBallThroughCompensation", launchRpmAfterBallThrough, RPM);
      Logger.recordOutput("Shooter/LaunchCalculator/HoodAngleDeg", Units.radiansToDegrees(launchParam.hoodAngle()), Degrees);
      Logger.recordOutput("Shooter/LaunchCalculator/TurretAngleFieldCentric", launchParam.turretAngle().getDegrees(), Degrees);
      Logger.recordOutput("Shooter/LaunchCalculator/TurretAngleTurretCentric", RobotEstimatedPose.getRotation().minus(launchParam.turretAngle()).getDegrees(), Degrees);
    }
    Logger.recordOutput("Shooter/RobotEstimatedPose", RobotEstimatedPose);
    Logger.recordOutput("Shooter/RobotVelocity", robotChassisSpeeds);
    if (aimingLaunch) {
      Translation2d logTarget2d = solveTarget3d.toTranslation2d();
      Logger.recordOutput(
          "Shooter/Target",
          new Pose2d(logTarget2d.getX(), logTarget2d.getY(), Rotation2d.kZero));
    }
  }

  public static Command getSetStateCommand(SHOOTER_STATE state, Shooter shooter) {
    return Commands.runOnce(() -> shooter.setShooterState(state), shooter);
  }

  /** Convenience wrapper for aiming at an arbitrary 3D field point. */
  public static Command getAimAtTranslation3dCommand(
      Translation3d targetFieldBluePerspective, Shooter shooter) {
    return Commands.runOnce(() -> shooter.setPoint3dTarget(targetFieldBluePerspective), shooter);
  }
}
