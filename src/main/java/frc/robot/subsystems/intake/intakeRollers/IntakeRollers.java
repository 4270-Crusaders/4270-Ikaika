package frc.robot.subsystems.intake.intakeRollers;

import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/Rollers/kP", IntakeConstants.IntakeRollerConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Intake/Rollers/kI", IntakeConstants.IntakeRollerConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Intake/Rollers/kD", IntakeConstants.IntakeRollerConstants.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Intake/Rollers/kA", IntakeConstants.IntakeRollerConstants.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Intake/Rollers/kV", IntakeConstants.IntakeRollerConstants.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Intake/Rollers/kS", IntakeConstants.IntakeRollerConstants.kS);

  public enum IntakeRollersGoal {
    ZERO(new LoggedTunableNumber("Intake/Roller/Goals/ZERO", 0.0)),
    INTAKE(new LoggedTunableNumber("Intake/Roller/Goals/INTAKE", 4000)),
    OUTTAKE(new LoggedTunableNumber("Intake/Roller/Goals/OUTTAKE", -2000)),
    AGITATE(new LoggedTunableNumber("Intake/Roller/Goals/AGITATE", 200)),
    CUSTOM(new LoggedTunableNumber("Intake/Roller/Goals/CUSTOM", 0));

    private final DoubleSupplier intakeRollerSetpointSupplier;

    private IntakeRollersGoal(DoubleSupplier intakeRollerSetpointSupplier) {
      this.intakeRollerSetpointSupplier = intakeRollerSetpointSupplier;
    }

    private double getRPM() {
      return intakeRollerSetpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput private IntakeRollersGoal goalSetpoint = IntakeRollersGoal.ZERO;

  private boolean closedLoop = false;

  private double goalRPM = 0.0;

  private boolean nearGoal = false;

  public void setGoalSetPoint(IntakeRollersGoal goal) {
    closedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setGoalSetPoint(double goal) {
    closedLoop = true;
    this.goalRPM = goal;
  }

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Roller", inputs);

    if (closedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runSetVelocity(goalRPM);
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA);

    nearGoal = EqualsUtil.epsilonEquals(inputs.appliedVolts, goalRPM, 10);
    Logger.recordOutput("Intake/Roller/nearGoal", nearGoal);
  }

  public void Setpoint(IntakeRollersGoal goalSetPoint) {
    setGoalSetPoint(goalSetPoint);
  }

  public void Setpoint(double goalSetPoint) {
    setGoalSetPoint(goalSetPoint);
  }
}
