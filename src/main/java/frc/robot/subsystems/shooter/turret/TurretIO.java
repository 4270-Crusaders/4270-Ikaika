package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware I/O contract for the turret. Designed to mirror typical "IO" interfaces (like HoodIO)
 * used to decouple platform-specific hardware from higher-level subsystem logic.
 */
public interface TurretIO {
  /**
   * Container for inputs read from hardware each robot loop. Fields are public for simple,
   * allocation-free use.
   */
  @AutoLog
  class TurretIOInputs {
    public double measuredPostionDeg = 0.0;
    public double setPostionDeg = 0.0;
    public double velocityRadPerSec = 0.0; // angular velocity
    public double appliedVolts = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double deviceTemperature = 0.0;
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

  public default void updateInputs(TurretIOInputs inputs) {}

  /** Run open loop at the specified velocity. */
  public default void runSetpointDegree(double setpointDeg) {}
}
