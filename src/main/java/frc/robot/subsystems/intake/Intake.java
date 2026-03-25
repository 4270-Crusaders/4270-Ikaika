package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers.IntakeRollersGoal;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollersIO;
import frc.robot.subsystems.intake.intakeWrist.IntakeWrist;
import frc.robot.subsystems.intake.intakeWrist.IntakeWrist.IntakeWristGoal;
import frc.robot.subsystems.intake.intakeWrist.IntakeWristIO;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends SubsystemBase {
  public enum INTAKE_STATE {
    UP,
    DOWN,
    INTAKE,
    OUTTAKE,
    SHOOT,
    AGITATE,
    CUSTOM
  }

  @AutoLogOutput(key = "Intake/CurretState") private INTAKE_STATE currentIntakeState = INTAKE_STATE.UP;


  private IntakeRollers intakeRollers;
  private IntakeWrist intakeWrist;

  public Intake(IntakeRollersIO intakeRollersIO, IntakeWristIO intakeWristIO) {
    this.intakeRollers = new IntakeRollers(intakeRollersIO);
    this.intakeWrist = new IntakeWrist(intakeWristIO);
  }

  public void setIntakeState(INTAKE_STATE state) {
    currentIntakeState = state;
  }


  @Override
  public void periodic() {
      switch (currentIntakeState) {
        case UP:
          intakeRollers.Setpoint(IntakeRollersGoal.ZERO);
          intakeWrist.Setpoint(IntakeWristGoal.UP);
          break;
        case DOWN:
          intakeRollers.Setpoint(IntakeRollersGoal.ZERO);
          intakeWrist.Setpoint(IntakeWristGoal.DOWN);
          break;
        case INTAKE:
            intakeRollers.Setpoint(IntakeRollersGoal.INTAKE);
            intakeWrist.Setpoint(IntakeWristGoal.DOWN);
          break;
        case OUTTAKE:
          intakeRollers.Setpoint(IntakeRollersGoal.OUTTAKE);
          intakeWrist.Setpoint(IntakeWristGoal.DOWN);
          break;
        case AGITATE:
          intakeRollers.Setpoint(IntakeRollersGoal.AGITATE);
          intakeWrist.Setpoint(IntakeWristGoal.AGITATE);
          break;
        case SHOOT:
          intakeRollers.Setpoint(IntakeRollersGoal.INTAKE);
          break;
        case CUSTOM:
          intakeRollers.Setpoint(IntakeRollersGoal.CUSTOM);
          intakeWrist.Setpoint(IntakeWristGoal.CUSTOM);
          break;
    }

    intakeRollers.periodic();
    intakeWrist.periodic();
  }

  public static Command getSetStateCommand(INTAKE_STATE state, Intake intake) {
    return Commands.runOnce(() -> intake.setIntakeState(state), intake);
  }
}
