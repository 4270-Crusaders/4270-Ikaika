package frc.robot.subsystems.intake.intakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  public static class IntakeRollersIOInputs {
    public double motorMeasuredVelocityRPS = 0.0;
    public double motorSetpointVelocityRPS = 0.0;
    public double motorMeasuredVelocityRPM = 0.0;
    public double deviceTemperature = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double positionRad = 0.0;
  }

  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  public default void runSetVelocity(double velocity) {}
}
