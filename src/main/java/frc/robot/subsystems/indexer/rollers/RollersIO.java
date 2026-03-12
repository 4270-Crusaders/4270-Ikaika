package frc.robot.subsystems.indexer.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public double positionRad = 0.0;
    public double appliedVolts = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double motorSetpointVelocityRPS = 0.0;
    public double motorMeasuredVelocityRPS = 0.0;
    public double motorMeasuredVelocityRPM = 0.0;
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
   */
  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public default void updateInputs(RollersIOInputs inputs) {}
  ;

  /** Run open loop at the specified voltage. */
  public default void runSetVoltage(double voltage) {}

  public default void runVelocityRPM(double RPM) {}
}
