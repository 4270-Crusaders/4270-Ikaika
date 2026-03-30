// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.indexer.indexerKicker;

import static frc.robot.Constants.TalonFxIo.CONFIG_APPLY_TIMEOUT_SEC;
import static frc.robot.Constants.TalonFxIo.CONFIG_RETRY_COUNT;
import static frc.robot.Constants.TalonFxIo.STATUS_SIGNAL_UPDATE_HZ;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.indexer.IndexerConstants;

public class IndexerKickerIOTalonFX implements IndexerKickerIO {
  private final TalonFX motor = new TalonFX(IndexerConstants.IndexerKicker.CAN_ID);

  private final StatusSignal<AngularVelocity> measuredVeloRPS = motor.getVelocity();
  private final StatusSignal<Double> setVeloRPS = motor.getClosedLoopReference();
  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<Voltage> appliedVoltage = motor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
  private final StatusSignal<Current> torqueCurrentAmps = motor.getTorqueCurrent();
  private final StatusSignal<Temperature> deviceTemperature = motor.getDeviceTemp();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public IndexerKickerIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = IndexerConstants.IndexerKicker.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable =
        IndexerConstants.IndexerKicker.CURRENT_LIMIT_ENABLE;
    config.MotorOutput.NeutralMode = IndexerConstants.IndexerKicker.NEUTRAL_MODE;
    config.MotorOutput.Inverted = IndexerConstants.IndexerKicker.INVERTED;

    config.Slot0.kI = IndexerConstants.IndexerKicker.Gains.kI;
    config.Slot0.kP = IndexerConstants.IndexerKicker.Gains.kP;
    config.Slot0.kD = IndexerConstants.IndexerKicker.Gains.kD;
    config.Slot0.kA = IndexerConstants.IndexerKicker.Gains.kA;
    config.Slot0.kV = IndexerConstants.IndexerKicker.Gains.kV;
    config.Slot0.kS = IndexerConstants.IndexerKicker.Gains.kS;

    tryUntilOk(CONFIG_RETRY_COUNT, () -> motor.getConfigurator().apply(config, CONFIG_APPLY_TIMEOUT_SEC));

    BaseStatusSignal.setUpdateFrequencyForAll(
        STATUS_SIGNAL_UPDATE_HZ,
        setVeloRPS,
        measuredVeloRPS,
        position,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature);
  }

  @Override
  public void updateInputs(IndexerKickerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        setVeloRPS,
        measuredVeloRPS,
        position,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature);

    inputs.motorMeasuredVelocityRPS = measuredVeloRPS.getValueAsDouble();
    inputs.motorSetpointVelocityRPS = setVeloRPS.getValueAsDouble();
    inputs.motorMeasuredVelocityRPM = inputs.motorMeasuredVelocityRPS * 60;
    inputs.deviceTemperature = deviceTemperature.getValueAsDouble();
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.positionRad = position.getValueAsDouble();
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    tryUntilOk(CONFIG_RETRY_COUNT, () -> motor.getConfigurator().apply(config));
  }

  @Override
  public void runSetVoltage(double voltage) {
    motor.setControl(voltageRequest.withEnableFOC(true).withOutput(voltage));
  }

  @Override
  public void runVelocityRPM(double rpm) {
    motor.setControl(velocityRequest.withVelocity(rpm / 60.0).withEnableFOC(true));
  }
}
