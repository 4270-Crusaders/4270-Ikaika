package frc.robot.subsystems.indexer.rollers;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for indexer rollers (velocity or voltage). */
public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    /** Mechanism position in radians (sensor). */
    public double positionRad = 0.0;

    public double appliedVolts = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    /** Closed-loop velocity reference, rotations per second. */
    public double motorSetpointVelocityRPS = 0.0;

    public double motorMeasuredVelocityRPS = 0.0;
    /** Measured rotor speed in revolutions per minute. */
    public double motorMeasuredVelocityRPM = 0.0;

    public double deviceTemperature = 0.0;
  }

  /**
   * Applies PID and feedforward gains to Talon slot 0.
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kS static friction feedforward
   * @param kV velocity feedforward
   * @param kA acceleration feedforward
   */
  default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  default void updateInputs(RollersIOInputs inputs) {}

  /** Open-loop voltage (volts). */
  default void runSetVoltage(double voltageVolts) {}

  /** Closed-loop velocity target in revolutions per minute. */
  default void runVelocityRPM(double rpm) {}
}
