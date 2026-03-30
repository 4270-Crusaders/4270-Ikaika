package frc.robot.subsystems.indexer.indexerConveyor;

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

public class IndexerConveyorIOTalonFX implements IndexerConveyorIO {
  private final TalonFX motor = new TalonFX(IndexerConstants.IndexerConveyor.CAN_ID);

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

  public IndexerConveyorIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = IndexerConstants.IndexerConveyor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable =
        IndexerConstants.IndexerConveyor.CURRENT_LIMIT_ENABLE;
    config.MotorOutput.NeutralMode = IndexerConstants.IndexerConveyor.NEUTRAL_MODE;
    config.MotorOutput.Inverted = IndexerConstants.IndexerConveyor.INVERTED;

    config.Slot0.kI = IndexerConstants.IndexerConveyor.Gains.kI;
    config.Slot0.kP = IndexerConstants.IndexerConveyor.Gains.kP;
    config.Slot0.kD = IndexerConstants.IndexerConveyor.Gains.kD;
    config.Slot0.kA = IndexerConstants.IndexerConveyor.Gains.kA;
    config.Slot0.kV = IndexerConstants.IndexerConveyor.Gains.kV;
    config.Slot0.kS = IndexerConstants.IndexerConveyor.Gains.kS;

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
  public void updateInputs(IndexerConveyorIOInputs inputs) {
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
