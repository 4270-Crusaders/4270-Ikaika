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
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public enum SHOOTER_STATE {
    AUTOHOME,
    HOME,
    HUB,
    PASS,
    ZERO,

    CUSTOM,
  }

  @AutoLogOutput private SHOOTER_STATE currentShooterState = SHOOTER_STATE.ZERO;
  

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

  LaunchingParameters launchParam = new LaunchingParameters(false, new Rotation2d(), 0, 0, 0, 0); //empty param
  LaunchCalculator calculator = LaunchCalculator.getInstance();
  Translation2d passPoint = new Translation2d();
  Translation2d target = new Translation2d();

  @Override
  public void periodic() {
    flywheel.periodic();
    turret.periodic();
    hood.periodic();

    calculator = LaunchCalculator.getInstance();

    if(!AllianceFlipUtil.shouldFlip()){ // if blue
      passPoint = RobotEstimatedPose.getY() > FieldConstants.fieldWidth/2 ?
          AllianceFlipUtil.apply(LauncherConstants.passPointLeft) :
          AllianceFlipUtil.apply(LauncherConstants.passPointRight);
    } else {
      passPoint = RobotEstimatedPose.getY() < FieldConstants.fieldWidth/2 ?
          AllianceFlipUtil.apply(LauncherConstants.passPointLeft) :
          AllianceFlipUtil.apply(LauncherConstants.passPointRight);
    }

    target = currentShooterState == SHOOTER_STATE.PASS ?
              passPoint : 
              AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    launchParam = calculator.getParameters(
      RobotEstimatedPose,
      robotChassisSpeeds,
      AllianceFlipUtil.apply(target)
    );

    switch (currentShooterState) {
      case ZERO:
        turret.setGoalSetPoint(0);
        hood.setGoalSetPoint(0);
        flywheel.setGoalSetPoint(0);
        break;
      case HUB:
        turret.setGoalSetPoint(RobotEstimatedPose.getRotation().minus(launchParam.turretAngle()).getDegrees());
        hood.setGoalSetPoint(Units.radiansToDegrees(launchParam.hoodAngle()));
        flywheel.setGoalSetPoint(launchParam.flywheelSpeed());

        // flywheel.setGoalSetPoint(FlyWheelGoal.CUSTOM);
        // turret.setGoalSetPoint(TurretGoal.CUSTOM);
        // hood.setGoalSetPoint(HoodGoal.CUSTOM);
        break;
      case PASS:
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
      default:
        break;
      case CUSTOM:
        flywheel.setGoalSetPoint(FlyWheelGoal.CUSTOM);
        turret.setGoalSetPoint(TurretGoal.CUSTOM);
        hood.setGoalSetPoint(HoodGoal.CUSTOM);
        break;
    }

    Logger.recordOutput("Shooter/Calculation/FlywheelSpeed", launchParam.flywheelSpeed(), RPM);
    Logger.recordOutput("Shooter/Calculation/HoodAngle", launchParam.hoodAngle(), Radians);
    Logger.recordOutput("Shooter/Calculation/TurretAngleFieldCentric", launchParam.turretAngle().getDegrees(), Degrees);
    Logger.recordOutput("Shooter/Calculation/TurretAngleTurretCentric", RobotEstimatedPose.getRotation().minus(launchParam.turretAngle()).getDegrees(), Degrees);
    Logger.recordOutput("Shooter/RobotEstimatedPose", RobotEstimatedPose);
    Logger.recordOutput("Shooter/RobotVelocity", robotChassisSpeeds);
    Logger.recordOutput("Shooter/Target", target);
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
