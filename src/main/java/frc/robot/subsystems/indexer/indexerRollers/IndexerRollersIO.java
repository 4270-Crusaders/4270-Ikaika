// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.indexer.indexerRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerRollersIO {
  @AutoLog
  class IndexerRollersIOInputs {
    public double positionRad = 0.0;
    public double appliedVolts = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double motorSetpointVelocityRPS = 0.0;
    public double motorMeasuredVelocityRPS = 0.0;
    public double motorMeasuredVelocityRPM = 0.0;
    public double deviceTemperature = 0.0;
  }

  default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  default void updateInputs(IndexerRollersIOInputs inputs) {}

  default void runSetVoltage(double voltageVolts) {}

  default void runVelocityRPM(double rpm) {}
}
