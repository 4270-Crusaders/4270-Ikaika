package frc.robot.subsystems.intake.intakeWrist;

import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeWrist {
  private final IntakeWristIO io;
  private final IntakeWristIOInputsAutoLogged inputs = new IntakeWristIOInputsAutoLogged();

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Intake/Wrist/kP", IntakeConstants.IntakeWristConstants.IntakeWristkP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Intake/Wrist/kI", IntakeConstants.IntakeWristConstants.IntakeWristkI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Intake/Wrist/kD", IntakeConstants.IntakeWristConstants.IntakeWristkD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Intake/Wrist/kA", IntakeConstants.IntakeWristConstants.IntakeWristkA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Intake/Wrist/kV", IntakeConstants.IntakeWristConstants.IntakeWristkV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Intake/Wrist/kS", IntakeConstants.IntakeWristConstants.IntakeWristkS);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber(
          "Intake/Wrist/kG", IntakeConstants.IntakeWristConstants.IntakeWristkG);

  private static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber(
          "Intake/Wrist/MMJerk", IntakeConstants.IntakeWristConstants.IntakeWristMotionMagicJerk);
  private static final LoggedTunableNumber MOTION_MAGIC_ACCELERATION =
      new LoggedTunableNumber(
          "Intake/Wrist/MMAcceleration",
          IntakeConstants.IntakeWristConstants.IntakeWristMotionMagicAcceleration);
  private static final LoggedTunableNumber MOTION_MAGIC_VELOCITY =
      new LoggedTunableNumber(
          "Intake/Wrist/MMVelocity",
          IntakeConstants.IntakeWristConstants.IntakeWristMotionMagicVelocity);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kV =
      new LoggedTunableNumber(
          "Intake/Wrist/MMExpokV", IntakeConstants.IntakeWristConstants.IntakeWristMotionMagickV);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kA =
      new LoggedTunableNumber(
          "Intake/Wrist/MMExpokA", IntakeConstants.IntakeWristConstants.IntakeWristMotionMagickA);

  public enum IntakeWristGoal {
    UP(new LoggedTunableNumber("Intake/Wrist/Goals/UP(", 5)),
    DOWN(new LoggedTunableNumber("Intake/Wrist/Goals/DOWN", 88)),
    AGITATE(new LoggedTunableNumber("Intake/Wrist/Goals/AGITATE", 40)),
    CUSTOM(new LoggedTunableNumber("Intake/Wrist/Goals/CUSTOM", 0));

    private final DoubleSupplier intakeWristSetpointSupplier;

    private IntakeWristGoal(DoubleSupplier intakeWristSetpointSupplier) {
      this.intakeWristSetpointSupplier = intakeWristSetpointSupplier;
    }

    private double getDegrees() {
      return intakeWristSetpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput private IntakeWristGoal goalSetpoint = IntakeWristGoal.UP;

  private boolean closedLoop = true;

  private double goalDeg = 0.0;

  private boolean nearGoal = false;

  public void setGoalSetPoint(IntakeWristGoal goal) {
    closedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltage) {
    closedLoop = false;
    io.runSetVoltage(voltage);
  }

  public IntakeWrist(IntakeWristIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Wrist", inputs);

    // motion magic setpoint code
    if (closedLoop) {
      goalDeg = goalSetpoint.getDegrees();
      io.runSetpointDegree(goalDeg);
    }

    nearGoal = EqualsUtil.epsilonEquals(inputs.measuredPostionDeg, goalDeg, 5);
    Logger.recordOutput("Intake/Wrist/nearGoal", nearGoal);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA,
        kG);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.setMotionMagicConstraints(
                MOTION_MAGIC_JERK.get(),
                MOTION_MAGIC_ACCELERATION.get(),
                MOTION_MAGIC_VELOCITY.get(),
                MOTION_MAGIC_EXPO_kA.get(),
                MOTION_MAGIC_EXPO_kV.get()),
        MOTION_MAGIC_JERK,
        MOTION_MAGIC_ACCELERATION,
        MOTION_MAGIC_VELOCITY,
        MOTION_MAGIC_EXPO_kA,
        MOTION_MAGIC_EXPO_kV);
  }

  public void getSetVoltage(double goal) {
    setManualVoltage(goal);
  }
  ;

  public void Setpoint(IntakeWristGoal goalSetPoint) {
    setGoalSetPoint(goalSetPoint);
  }
  ;
}
