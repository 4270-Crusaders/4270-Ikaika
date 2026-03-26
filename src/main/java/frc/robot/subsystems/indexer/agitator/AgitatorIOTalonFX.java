package frc.robot.subsystems.indexer.agitator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.ImperialMarchChime;

public class AgitatorIOTalonFX implements AgitatorIO {
  private final TalonFX LeadMotor = new TalonFX(IndexerConstants.AgitatorConstants.AGITATOR_CAN_ID);

  private final StatusSignal<AngularVelocity> measuredVeloRPS = LeadMotor.getVelocity();
  private final StatusSignal<Double> setVeloRPS = LeadMotor.getClosedLoopReference();
  private final StatusSignal<Angle> position = LeadMotor.getPosition();
  private final StatusSignal<Voltage> appliedVoltage = LeadMotor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrentAmps = LeadMotor.getSupplyCurrent();
  private final StatusSignal<Current> torqueCurrentAmps = LeadMotor.getTorqueCurrent();
  private final StatusSignal<Temperature> deviceTemperature = LeadMotor.getDeviceTemp();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0.0);

  private TalonFXConfiguration config = new TalonFXConfiguration();

  public AgitatorIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit =
        IndexerConstants.AgitatorConstants.AGITATOR_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable =
        IndexerConstants.AgitatorConstants.AGITATOR_CURRENT_LIMIT_ENABLE;
    config.MotorOutput.NeutralMode = IndexerConstants.AgitatorConstants.AGITATOR_NEUTRAL_MODE_VALUE;
    config.MotorOutput.Inverted = IndexerConstants.AgitatorConstants.AGITI_INVERTED_VALUE;

    config.Slot0.kI = IndexerConstants.AgitatorConstants.kI;
    config.Slot0.kP = IndexerConstants.AgitatorConstants.kP;
    config.Slot0.kD = IndexerConstants.AgitatorConstants.kD;
    config.Slot0.kA = IndexerConstants.AgitatorConstants.kA;
    config.Slot0.kV = IndexerConstants.AgitatorConstants.kV;
    config.Slot0.kS = IndexerConstants.AgitatorConstants.kS;

    tryUntilOk(5, () -> LeadMotor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        setVeloRPS,
        measuredVeloRPS,
        position,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature);

    ImperialMarchChime.registerChimeMotor(LeadMotor);
  }

  @Override
  public void updateInputs(AgitatorIOInputs inputs) {
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
    tryUntilOk(5, () -> LeadMotor.getConfigurator().apply(config));
  }

  @Override
  public void runSetVoltage(double voltage) {
    if (ImperialMarchChime.isPlaying()) {
      return;
    }
    LeadMotor.setControl(voltageRequest.withEnableFOC(true).withOutput(voltage));
  }

  @Override
  public void runVelocityRPM(double RPM) {
    if (ImperialMarchChime.isPlaying()) {
      return;
    }
    LeadMotor.setControl(velocityRequest.withVelocity(RPM / 60));
  }
}
