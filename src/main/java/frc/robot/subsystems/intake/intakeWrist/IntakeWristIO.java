// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.intake.intakeWrist;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWristIO {
  @AutoLog
  class IntakeWristIOInputs {
    public double measuredPostionDeg = 0.0;
    public double setpointPostionDeg = 0.0;
    public double appliedVolts = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double deviceTemperature = 0.0;
  }

  default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  default void setMotionMagicConstraints(
      double jerk, double acceleration, double velocity, double expokA, double expokV) {}

  default void updateInputs(IntakeWristIOInputs inputs) {}

  default void runSetpointDegree(double setpointDeg) {}

  default void runSetVoltage(double voltage) {}
}
