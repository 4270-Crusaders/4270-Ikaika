package frc.robot.subsystems.indexer.conveyor;

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

/** Real-hardware {@link ConveyorIO} using a Talon FX. */
public class ConveyorIOTalonFX implements ConveyorIO {
  private final TalonFX leadMotor =
      new TalonFX(IndexerConstants.ComponentsConstants.Conveyor.CAN_ID);

  private final StatusSignal<AngularVelocity> measuredVeloRPS = leadMotor.getVelocity();
  private final StatusSignal<Double> setVeloRPS = leadMotor.getClosedLoopReference();
  private final StatusSignal<Angle> position = leadMotor.getPosition();
  private final StatusSignal<Voltage> appliedVoltage = leadMotor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrentAmps = leadMotor.getSupplyCurrent();
  private final StatusSignal<Current> torqueCurrentAmps = leadMotor.getTorqueCurrent();
  private final StatusSignal<Temperature> deviceTemperature = leadMotor.getDeviceTemp();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public ConveyorIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit =
        IndexerConstants.ComponentsConstants.Conveyor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable =
        IndexerConstants.ComponentsConstants.Conveyor.CURRENT_LIMIT_ENABLE;
    config.MotorOutput.NeutralMode = IndexerConstants.ComponentsConstants.Conveyor.NEUTRAL_MODE;
    config.MotorOutput.Inverted = IndexerConstants.ComponentsConstants.Conveyor.INVERTED;

    config.Slot0.kI = IndexerConstants.ComponentsConstants.Conveyor.Gains.kI;
    config.Slot0.kP = IndexerConstants.ComponentsConstants.Conveyor.Gains.kP;
    config.Slot0.kD = IndexerConstants.ComponentsConstants.Conveyor.Gains.kD;
    config.Slot0.kA = IndexerConstants.ComponentsConstants.Conveyor.Gains.kA;
    config.Slot0.kV = IndexerConstants.ComponentsConstants.Conveyor.Gains.kV;
    config.Slot0.kS = IndexerConstants.ComponentsConstants.Conveyor.Gains.kS;

    tryUntilOk(5, () -> leadMotor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        setVeloRPS,
        measuredVeloRPS,
        position,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature);
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
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
    tryUntilOk(5, () -> leadMotor.getConfigurator().apply(config));
  }

  @Override
  public void runSetVoltage(double voltage) {
    leadMotor.setControl(voltageRequest.withEnableFOC(true).withOutput(voltage));
  }

  @Override
  public void runVelocityRPM(double rpm) {
    leadMotor.setControl(velocityRequest.withVelocity(rpm / 60.0).withEnableFOC(true));
  }
}
