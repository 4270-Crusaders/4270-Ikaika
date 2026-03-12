package frc.robot.subsystems.intake.intakeWrist;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWristIO {
  @AutoLog
  public static class IntakeWristIOInputs {
    public double measuredPostionDeg = 0.0;
    public double setpointPostionDeg = 0.0;
    public double appliedVolts = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double deviceTemperature = 0.0;
  }

  /**
   * Sets PID values for subsystem
   *
   * @param kP P val
   * @param kI I valw
   * @param kD D val
   * @param kS S val
   * @param kV V val
   * @param kA A val
   * @param kG G val
   */
  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void setMotionMagicConstraints(
      double jerk, double acceleration, double velocity, double expokA, double expokV) {}

  public default void updateInputs(IntakeWristIOInputs inputs) {}
  ;

  /** Run open loop at the specified velocity. */
  public default void runSetpointDegree(double setpointDeg) {}

  public default void runSetVoltage(double voltage) {}
}
