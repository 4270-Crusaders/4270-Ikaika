package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
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

import org.littletonrobotics.junction.AutoLogOutput;

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

  LaunchingParameters launchParam = new LaunchingParameters(false, null, 0, 0, 0, 0); //empty param
  LaunchCalculator calculator = new LaunchCalculator();

  @Override
  public void periodic() {
    if(currentShooterState == SHOOTER_STATE.PASS){
      launchParam = calculator.getParameters(
        RobotEstimatedPose,
        robotChassisSpeeds,
        AllianceFlipUtil.apply(new Translation2d()) //TODO change place to pass
      ); //update
    } else if(currentShooterState == SHOOTER_STATE.HUB){
      launchParam = calculator.getParameters(
        RobotEstimatedPose,
        robotChassisSpeeds,
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
      ); //update
    }
    

    flywheel.periodic();
    turret.periodic();
    hood.periodic();

    switch (currentShooterState) {
      case ZERO:
        turret.setGoalSetPoint(0);
        hood.setGoalSetPoint(0);
        flywheel.setGoalSetPoint(0);
        break;
      case HUB:
        turret.setGoalSetPoint(launchParam.turretAngle().getDegrees());
        hood.setGoalSetPoint(Units.radiansToDegrees(launchParam.hoodAngle()));
        flywheel.setGoalSetPoint(launchParam.flywheelSpeed());
        break;
      case PASS:
        turret.setGoalSetPoint(launchParam.turretAngle().getDegrees());
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
