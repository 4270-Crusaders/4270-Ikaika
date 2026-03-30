package frc.robot.subsystems.intake.intakeRoller;

import org.littletonrobotics.junction.AutoLog;

/** IO for intake rollers (two motors, velocity or voltage). */
public interface IntakeRollerIO {
  @AutoLog
  class IntakeRollerIOInputs {
    public double[] motorMeasuredVelocityRps = new double[] {0.0, 0.0};
    public double[] motorSetpointVelocityRps = new double[] {0.0, 0.0};
    public double[] motorMeasuredVelocityRpm = new double[] {0.0, 0.0};
    public double[] appliedVolts = new double[] {0.0, 0.0};
    public double[] supplyCurrentAmps = new double[] {0.0, 0.0};
    public double[] torqueCurrentAmps = new double[] {0.0, 0.0};
    public double[] deviceTemperature = new double[] {0.0, 0.0};
    public double[] positionRad = new double[] {0.0, 0.0};
  }

  default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  default void updateInputs(IntakeRollerIOInputs inputs) {}

  default void runSetVoltage(double voltageVolts) {}

  default void runVelocityRPM(double rpm) {}
}
