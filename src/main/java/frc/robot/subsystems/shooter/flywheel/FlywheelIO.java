package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double MainFlyWheelRPM = 0.0;
    public double HoodFlyWheelRPM = 0.0;
    public double positionRad[] = new double[] {};
    public double appliedVolts[] = new double[] {};
    public double torqueCurrentAmps[] = new double[] {};
    public double supplyCurrentAmps[] = new double[] {};
    public double motorSetpointVelocityRPS[] = new double[] {};
    public double motorMeasuredVelocityRPS[] = new double[] {};
    public double motorSetpointVelocityRPM[] = new double[] {};
    public double motorMeasuredVelocityRPM[] = new double[] {};
    public double deviceTemperature[] = new double[] {};
  }

  /**
   * Sets PID values for subsystem
   *
   * @param kP P val
   * @param kI I val
   * @param kD D val
   * @param kS S val
   * @param kV V val
   * @param kA A val
   */
  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public default void setMotionMagicConstraints(
      double jerk, double acceleration, double velocity) {}

  public default void updateInputs(FlywheelIOInputs inputs) {}
  ;

  /** Run open loop at the specified velocity. */
  public default void runSetVelocity(double setpointVelocityRotPerSec) {}
}
