package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    // Fields collected from hardware and automatically logged by
    // Littleton's AutoLog annotation. These are filled by the
    // concrete HoodIO implementation in updateInputs(...).

    // Current measured hood angle in degrees (from motor/controller)
    public double measuredPostionDeg = 0.0;
    // The setpoint (closed-loop) position in degrees as reported by
    // the motor controller; useful for verifying controller state
    public double setPostionDeg = 0.0;
    // Motor applied voltage
    public double appliedVolts = 0.0;
    // Motor torque-producing current (torque measurement)
    public double torqueCurrentAmps = 0.0;
    // Total supply current drawn by the motor
    public double supplyCurrentAmps = 0.0;
    // Motor/device temperature (Celsius)
    public double deviceTemperature = 0.0;
    // Raw absolute encoder reading (rotations) from the CANcoder
    public double measuredEncoderPositionRot = 0.0;
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

  public default void updateInputs(HoodIOInputs inputs) {}
  ;

  /** Run open loop at the specified velocity. */
  public default void runSetpointDegree(double setpointDeg) {}
}
