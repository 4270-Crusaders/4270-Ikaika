package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.Flywheel.FlyWheelGoal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.Hood.HoodGoal;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.Turret.TurretGoal;
import frc.robot.subsystems.shooter.turret.TurretIO;
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

  public Shooter(FlywheelIO flywheelIO, TurretIO turretIO, HoodIO hoodIO) {
    this.flywheel = new Flywheel(flywheelIO);
    this.turret = new Turret(turretIO);
    this.hood = new Hood(hoodIO);
  }

  public void setShooterState(SHOOTER_STATE shooterState) {
    this.currentShooterState = shooterState;
  }

  @Override
  public void periodic() {
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
        break;
      case PASS:
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
