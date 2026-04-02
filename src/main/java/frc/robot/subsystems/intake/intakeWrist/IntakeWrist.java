// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

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
          "Intake/IntakeWrist/Gains/kP", IntakeConstants.IntakeWristConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/Gains/kI", IntakeConstants.IntakeWristConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/Gains/kD", IntakeConstants.IntakeWristConstants.kD);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/Gains/kA", IntakeConstants.IntakeWristConstants.kA);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/Gains/kV", IntakeConstants.IntakeWristConstants.kV);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/Gains/kS", IntakeConstants.IntakeWristConstants.kS);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/Gains/kG", IntakeConstants.IntakeWristConstants.kG);

  private static final LoggedTunableNumber MOTION_MAGIC_JERK =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/MotionMagic/Jerk",
          IntakeConstants.IntakeWristConstants.motionMagicJerk);
  private static final LoggedTunableNumber MOTION_MAGIC_ACCELERATION =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/MotionMagic/Acceleration",
          IntakeConstants.IntakeWristConstants.motionMagicAcceleration);
  private static final LoggedTunableNumber MOTION_MAGIC_VELOCITY =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/MotionMagic/Velocity",
          IntakeConstants.IntakeWristConstants.motionMagicVelocity);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kV =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/MotionMagic/ExpoKv",
          IntakeConstants.IntakeWristConstants.motionMagicExpoKV);
  private static final LoggedTunableNumber MOTION_MAGIC_EXPO_kA =
      new LoggedTunableNumber(
          "Intake/IntakeWrist/MotionMagic/ExpoKa",
          IntakeConstants.IntakeWristConstants.motionMagicExpoKA);

  public enum IntakeWristGoal {
    UP(new LoggedTunableNumber("Intake/IntakeWrist/Goals/Up", 0.25)),
    DOWN(new LoggedTunableNumber("Intake/IntakeWrist/Goals/Down", 88)),
    AGITATE(new LoggedTunableNumber("Intake/IntakeWrist/Goals/Agitate", 40)),
    CUSTOM(new LoggedTunableNumber("Intake/IntakeWrist/Goals/Custom", 0));

    private final DoubleSupplier setpointSupplier;

    IntakeWristGoal(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getDegrees() {
      return setpointSupplier.getAsDouble();
    }
  }

  @AutoLogOutput private IntakeWristGoal goalSetpoint = IntakeWristGoal.UP;

  private boolean closedLoop = true;
  private double goalDeg = 0.0;
  private boolean nearGoal = false;

  public IntakeWrist(IntakeWristIO io) {
    this.io = io;
  }

  public void setGoalSetPoint(IntakeWristGoal goal) {
    closedLoop = true;
    this.goalSetpoint = goal;
  }

  public void setManualVoltage(double voltage) {
    closedLoop = false;
    io.runSetVoltage(voltage);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/IntakeWrist", inputs);

    if (closedLoop) {
      goalDeg = goalSetpoint.getDegrees();
      io.runSetpointDegree(goalDeg);
    }

    nearGoal = EqualsUtil.epsilonEquals(inputs.measuredPostionDeg, goalDeg, 5);
    Logger.recordOutput("Intake/IntakeWrist/nearGoal", nearGoal);

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
}
