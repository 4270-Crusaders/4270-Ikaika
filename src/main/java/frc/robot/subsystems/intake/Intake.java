package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intake.intakeRoller.IntakeRoller.IntakeRollerGoal;
import frc.robot.subsystems.intake.intakeRoller.IntakeRollerIO;
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

  @AutoLogOutput(key = "Intake/currentState")
  private INTAKE_STATE currentIntakeState = INTAKE_STATE.UP;

  private final IntakeRoller intakeRoller;
  private final IntakeWrist intakeWrist;

  public Intake(IntakeRollerIO intakeRollerIO, IntakeWristIO intakeWristIO) {
    this.intakeRoller = new IntakeRoller(intakeRollerIO);
    this.intakeWrist = new IntakeWrist(intakeWristIO);
  }

  public void setIntakeState(INTAKE_STATE state) {
    currentIntakeState = state;
  }

  @Override
  public void periodic() {
    switch (currentIntakeState) {
      case UP:
        intakeRoller.setGoalSetPoint(IntakeRollerGoal.ZERO);
        intakeWrist.setGoalSetPoint(IntakeWristGoal.UP);
        break;
      case DOWN:
        intakeRoller.setGoalSetPoint(IntakeRollerGoal.ZERO);
        intakeWrist.setGoalSetPoint(IntakeWristGoal.DOWN);
        break;
      case INTAKE:
        intakeRoller.setGoalSetPoint(IntakeRollerGoal.INTAKE);
        intakeWrist.setGoalSetPoint(IntakeWristGoal.DOWN);
        break;
      case OUTTAKE:
        intakeRoller.setGoalSetPoint(IntakeRollerGoal.OUTTAKE);
        intakeWrist.setGoalSetPoint(IntakeWristGoal.DOWN);
        break;
      case AGITATE:
        intakeRoller.setGoalSetPoint(IntakeRollerGoal.AGITATE);
        intakeWrist.setGoalSetPoint(IntakeWristGoal.AGITATE);
        break;
      case SHOOT:
        intakeRoller.setGoalSetPoint(IntakeRollerGoal.INTAKE);
        break;
      case CUSTOM:
        intakeRoller.setGoalSetPoint(IntakeRollerGoal.CUSTOM);
        intakeWrist.setGoalSetPoint(IntakeWristGoal.CUSTOM);
        break;
    }

    intakeRoller.periodic();
    intakeWrist.periodic();
  }

  public static Command getSetStateCommand(INTAKE_STATE state, Intake intake) {
    return Commands.runOnce(() -> intake.setIntakeState(state), intake);
  }
}
