package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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

    ZERO,

    CUSTOM,
  }

  private boolean trenchTeleNear = false;

  public enum SHOOT_MODE {
    HUB,
    PASS
  }

  @AutoLogOutput(key = "Shooter/ShooterState") private SHOOTER_STATE currentShooterState = SHOOTER_STATE.ZERO;
  @AutoLogOutput(key = "Shooter/ShootMode") private SHOOT_MODE currentShooterMode = SHOOT_MODE.HUB;

  private Flywheel flywheel;
  private Turret turret;
  private Hood hood;

  private Pose2d RobotEstimatedPose = new Pose2d();
  private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();

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

  public boolean readyToShoot(){
    return (flywheel.nearGoal && turret.nearGoal && !trenchTeleNear);
  }

  LaunchingParameters launchParam = new LaunchingParameters(false, new Rotation2d(), 0, 0, 0, 0); //empty param
  LaunchCalculator calculator = LaunchCalculator.getInstance();
  Translation2d passPoint = new Translation2d();
  Translation2d target = new Translation2d();

  @Override
  public void periodic() {
    flywheel.periodic();
    turret.periodic();
    hood.periodic();

    trenchTeleNear = 
    (AllianceFlipUtil.apply(RobotEstimatedPose.getTranslation()).getX()>FieldConstants.LinesVertical.startTrench&&
    AllianceFlipUtil.apply(RobotEstimatedPose.getTranslation()).getX()<FieldConstants.LinesVertical.endTrench) ?
    true:false;

    currentShooterMode = 
    (AllianceFlipUtil.apply(RobotEstimatedPose.getTranslation()).getX()>LauncherConstants.passPoint)?
    SHOOT_MODE.PASS:SHOOT_MODE.HUB;

    passPoint = !AllianceFlipUtil.shouldFlip()?
      (RobotEstimatedPose.getY() > FieldConstants.fieldWidth/2 ?
          AllianceFlipUtil.apply(LauncherConstants.passPointLeft) :
          AllianceFlipUtil.apply(LauncherConstants.passPointRight)):
      (RobotEstimatedPose.getY() < FieldConstants.fieldWidth/2 ?
          AllianceFlipUtil.apply(LauncherConstants.passPointLeft) :
          AllianceFlipUtil.apply(LauncherConstants.passPointRight));

    target = currentShooterMode == SHOOT_MODE.PASS ?
      passPoint : 
      AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    launchParam = calculator.getParameters(
      RobotEstimatedPose,
      robotChassisSpeeds,
      target
    );

    switch (currentShooterState) {
      case ZERO:
        turret.setGoalSetPoint(0);
        hood.setGoalSetPoint(0);
        flywheel.setGoalSetPoint(0);
        break;
      case AIM:
        turret.setGoalSetPoint(trenchTeleNear ? 0 : RobotEstimatedPose.getRotation().minus(launchParam.turretAngle()).getDegrees());
        flywheel.setGoalSetPoint(launchParam.flywheelSpeed());
        hood.setGoalSetPoint(trenchTeleNear ? 0 : Units.radiansToDegrees(launchParam.hoodAngle()));
        break;
      case AUTOAIMHUB:
        launchParam = calculator.getParameters(
          RobotEstimatedPose,
          robotChassisSpeeds,
          AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
        );
        turret.setGoalSetPoint(RobotEstimatedPose.getRotation().minus(launchParam.turretAngle()).getDegrees());
        hood.setGoalSetPoint(Units.radiansToDegrees(launchParam.hoodAngle()));
        flywheel.setGoalSetPoint(launchParam.flywheelSpeed());
        break; 
      case AUTOAIMPASS:
        launchParam = calculator.getParameters(
          RobotEstimatedPose,
          robotChassisSpeeds,
          passPoint
        );
        turret.setGoalSetPoint(RobotEstimatedPose.getRotation().minus(launchParam.turretAngle()).getDegrees());
        hood.setGoalSetPoint(Units.radiansToDegrees(launchParam.hoodAngle()));
        flywheel.setGoalSetPoint(launchParam.flywheelSpeed());
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
      default:
        break;
    }

    Logger.recordOutput("Shooter/LaunchCalculator/FlywheelSpeed", launchParam.flywheelSpeed(), RPM);
    Logger.recordOutput("Shooter/LaunchCalculator/HoodAngleDeg", Units.radiansToDegrees(launchParam.hoodAngle()), Degrees);
    Logger.recordOutput("Shooter/LaunchCalculator/TurretAngleFieldCentric", launchParam.turretAngle().getDegrees(), Degrees);
    Logger.recordOutput("Shooter/LaunchCalculator/TurretAngleTurretCentric", RobotEstimatedPose.getRotation().minus(launchParam.turretAngle()).getDegrees(), Degrees);
    Logger.recordOutput("Shooter/RobotEstimatedPose", RobotEstimatedPose);

    Logger.recordOutput("Shooter/RobotVelocity", robotChassisSpeeds);
    Logger.recordOutput("Shooter/Target", new Pose2d(target.getX(), target.getY(), new Rotation2d()));
  }

  public static Command getSetStateCommand(SHOOTER_STATE state, Shooter shooter) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(shooter);
        shooter.setShooterState(state);
      }

      @Override
      public boolean isFinished() {
        return shooter.currentShooterState == state;
      }
    };
  }
}
