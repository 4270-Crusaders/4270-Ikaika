// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.intake.intakeRoller;

import static frc.robot.Constants.TalonFxIo.CONFIG_APPLY_TIMEOUT_SEC;
import static frc.robot.Constants.TalonFxIo.CONFIG_RETRY_COUNT;
import static frc.robot.Constants.TalonFxIo.STATUS_SIGNAL_UPDATE_HZ;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.List;

/** Intake rollers: lead commanded; follow mirrors via hardware follower. */
public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX leadMotor = new TalonFX(IntakeConstants.IntakeRollerConstants.LEAD_CAN_ID);
  private final TalonFX followMotor = new TalonFX(IntakeConstants.IntakeRollerConstants.FOLLOW_CAN_ID);
  private final Follower followController =
      new Follower(
          IntakeConstants.IntakeRollerConstants.LEAD_CAN_ID,
          MotorAlignmentValue.Opposed);

  private final List<StatusSignal<AngularVelocity>> measuredVeloRps =
      List.of(leadMotor.getVelocity(), followMotor.getVelocity());
  private final List<StatusSignal<Double>> setVeloRps =
      List.of(leadMotor.getClosedLoopReference(), followMotor.getClosedLoopReference());
  private final List<StatusSignal<Angle>> positions =
      List.of(leadMotor.getPosition(), followMotor.getPosition());
  private final List<StatusSignal<Voltage>> appliedVoltage =
      List.of(leadMotor.getMotorVoltage(), followMotor.getMotorVoltage());
  private final List<StatusSignal<Current>> supplyCurrentAmps =
      List.of(leadMotor.getSupplyCurrent(), followMotor.getSupplyCurrent());
  private final List<StatusSignal<Current>> torqueCurrentAmps =
      List.of(leadMotor.getTorqueCurrent(), followMotor.getTorqueCurrent());
  private final List<StatusSignal<Temperature>> deviceTemperature =
      List.of(leadMotor.getDeviceTemp(), followMotor.getDeviceTemp());

  private final VelocityVoltage velocityLead = new VelocityVoltage(0.0);
  private final VoltageOut voltageLead = new VoltageOut(0.0);

  private final TalonFXConfiguration configLead = new TalonFXConfiguration();
  private final TalonFXConfiguration configFollow = new TalonFXConfiguration();

  public IntakeRollerIOTalonFX() {
    applyCommonMotorConfig(configLead, IntakeConstants.IntakeRollerConstants.LEAD_INVERTED);
    applyCommonMotorConfig(configFollow, IntakeConstants.IntakeRollerConstants.FOLLOW_INVERTED);

    tryUntilOk(
        CONFIG_RETRY_COUNT, () -> leadMotor.getConfigurator().apply(configLead, CONFIG_APPLY_TIMEOUT_SEC));
    tryUntilOk(
        CONFIG_RETRY_COUNT,
        () -> followMotor.getConfigurator().apply(configFollow, CONFIG_APPLY_TIMEOUT_SEC));

    BaseStatusSignal.setUpdateFrequencyForAll(
        STATUS_SIGNAL_UPDATE_HZ,
        setVeloRps.get(0),
        setVeloRps.get(1),
        measuredVeloRps.get(0),
        measuredVeloRps.get(1),
        positions.get(0),
        positions.get(1),
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrentAmps.get(0),
        supplyCurrentAmps.get(1),
        torqueCurrentAmps.get(0),
        torqueCurrentAmps.get(1),
        deviceTemperature.get(0),
        deviceTemperature.get(1));

    // Only command the lead; the follow controller mirrors it in hardware.
    followMotor.setControl(followController);
  }

  private static void applyCommonMotorConfig(TalonFXConfiguration config, InvertedValue inverted) {
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.IntakeRollerConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable =
        IntakeConstants.IntakeRollerConstants.CURRENT_LIMIT_ENABLE;
    config.MotorOutput.NeutralMode = IntakeConstants.IntakeRollerConstants.NEUTRAL_MODE;
    config.MotorOutput.Inverted = inverted;

    config.Slot0.kI = IntakeConstants.IntakeRollerConstants.kI;
    config.Slot0.kP = IntakeConstants.IntakeRollerConstants.kP;
    config.Slot0.kD = IntakeConstants.IntakeRollerConstants.kD;
    config.Slot0.kA = IntakeConstants.IntakeRollerConstants.kA;
    config.Slot0.kV = IntakeConstants.IntakeRollerConstants.kV;
    config.Slot0.kS = IntakeConstants.IntakeRollerConstants.kS;
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        setVeloRps.get(0),
        setVeloRps.get(1),
        measuredVeloRps.get(0),
        measuredVeloRps.get(1),
        positions.get(0),
        positions.get(1),
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrentAmps.get(0),
        supplyCurrentAmps.get(1),
        torqueCurrentAmps.get(0),
        torqueCurrentAmps.get(1),
        deviceTemperature.get(0),
        deviceTemperature.get(1));

    for (int i = 0; i < 2; i++) {
      inputs.motorMeasuredVelocityRps[i] = measuredVeloRps.get(i).getValueAsDouble();
      inputs.motorSetpointVelocityRps[i] = setVeloRps.get(i).getValueAsDouble();
      inputs.motorMeasuredVelocityRpm[i] = inputs.motorMeasuredVelocityRps[i] * 60.0;
      inputs.appliedVolts[i] = appliedVoltage.get(i).getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentAmps.get(i).getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentAmps.get(i).getValueAsDouble();
      inputs.deviceTemperature[i] = deviceTemperature.get(i).getValueAsDouble();
      inputs.positionRad[i] = positions.get(i).getValueAsDouble();
    }
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    for (TalonFXConfiguration c : List.of(configLead, configFollow)) {
      c.Slot0.kP = kP;
      c.Slot0.kI = kI;
      c.Slot0.kD = kD;
      c.Slot0.kS = kS;
      c.Slot0.kV = kV;
      c.Slot0.kA = kA;
    }
    tryUntilOk(CONFIG_RETRY_COUNT, () -> leadMotor.getConfigurator().apply(configLead));
    tryUntilOk(CONFIG_RETRY_COUNT, () -> followMotor.getConfigurator().apply(configFollow));
  }

  @Override
  public void runVelocityRPM(double rpm) {
    double rps = rpm / 60.0;
    leadMotor.setControl(velocityLead.withVelocity(rps).withEnableFOC(true));
  }

  @Override
  public void runSetVoltage(double voltage) {
    leadMotor.setControl(voltageLead.withOutput(voltage).withEnableFOC(true));
  }
}
