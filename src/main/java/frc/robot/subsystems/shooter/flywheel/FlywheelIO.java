package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    /** Estimated main flywheel wheel speed (RPM), derived from motor speed and gearing. */
    public double mainFlywheelRpm = 0.0;
    /** Estimated hood flywheel wheel speed (RPM), derived from motor speed and gearing. */
    public double hoodFlywheelRpm = 0.0;
    public double motorPositionRadians[] = new double[] {};
    public double appliedVolts[] = new double[] {};
    public double torqueCurrentAmps[] = new double[] {};
    public double supplyCurrentAmps[] = new double[] {};
    public double motorSetpointVelocityRps[] = new double[] {};
    public double motorMeasuredVelocityRps[] = new double[] {};
    public double motorSetpointVelocityRpm[] = new double[] {};
    public double motorMeasuredVelocityRpm[] = new double[] {};
    public double deviceTemperatureCelsius[] = new double[] {};
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

  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run open loop at the specified velocity. */
  public default void runSetVelocity(double setpointVelocityRotPerSec) {}
}
