package frc.robot.subsystems.intake.intakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  public static class IntakeRollersIOInputs {
    public double[] motorMeasuredVelocityRPS = new double[] {};
    public double[] motorSetpointVelocityRPS = new double[] {};
    public double[] motorMeasuredVelocityRPM = new double[] {};
    public double[] deviceTemperature = new double[] {};
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] positionRad = new double[] {};
  }

  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  /** Update the set of loggable inputs. */
  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  public default void runSetVelocity(double velocity) {}
}
