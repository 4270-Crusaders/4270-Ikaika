// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.intake.intakeRoller;

import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Intake rollers: dual motor velocity closed-loop. */
public class IntakeRoller {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Intake/IntakeRoller/Gains/kP", IntakeConstants.IntakeRollerConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Intake/IntakeRoller/Gains/kI", IntakeConstants.IntakeRollerConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Intake/IntakeRoller/Gains/kD", IntakeConstants.IntakeRollerConstants.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Intake/IntakeRoller/Gains/kA", IntakeConstants.IntakeRollerConstants.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Intake/IntakeRoller/Gains/kV", IntakeConstants.IntakeRollerConstants.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Intake/IntakeRoller/Gains/kS", IntakeConstants.IntakeRollerConstants.kS);

  public enum IntakeRollerGoal {
    ZERO(new LoggedTunableNumber("Intake/IntakeRoller/Goals/Zero", 0)),
    INTAKE(new LoggedTunableNumber("Intake/IntakeRoller/Goals/Intake", 3000)),
    OUTTAKE(new LoggedTunableNumber("Intake/IntakeRoller/Goals/Outtake", -3000)),
    AGITATE(new LoggedTunableNumber("Intake/IntakeRoller/Goals/Agitate", 500)),
    CUSTOM(new LoggedTunableNumber("Intake/IntakeRoller/Goals/Custom", 0));

    private final DoubleSupplier setpointSupplier;

    IntakeRollerGoal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getRPM() {
      return setpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Intake/IntakeRoller/GoalSetpoint")
  private IntakeRollerGoal goalSetpoint = IntakeRollerGoal.ZERO;

  private boolean velocityClosedLoop = true;
  private double goalRPM = 0.0;

  @AutoLogOutput public boolean nearGoal = false;

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  public void setGoalSetPoint(IntakeRollerGoal goal) {
    velocityClosedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltageVolts) {
    velocityClosedLoop = false;
    io.runSetVoltage(voltageVolts);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/IntakeRoller", inputs);

    if (velocityClosedLoop) {
      goalRPM = goalSetpoint.getRPM();
      io.runVelocityRPM(goalRPM);
    }

    double avgMeasuredRpm =
        0.5 * (inputs.motorMeasuredVelocityRpm[0] + inputs.motorMeasuredVelocityRpm[1]);
    nearGoal =
        EqualsUtil.epsilonEquals(
            avgMeasuredRpm, goalRPM, IntakeConstants.IntakeRollerConstants.NEAR_GOAL_RPM_TOLERANCE);
    Logger.recordOutput("Intake/IntakeRoller/nearGoal", nearGoal);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA);
  }
}
