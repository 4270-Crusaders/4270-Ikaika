package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.PhoenixUtil;
import java.util.List;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final int CONFIG_RETRY_COUNT = 5;
  private static final double CONFIG_TIMEOUT_SEC = 0.25;
  private static final double STATUS_UPDATE_HZ = 50.0;
  private final TalonFX leadMotor =
      new TalonFX(ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_LEAD_CAN_ID);
  private final TalonFX followMotor =
      new TalonFX(ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_FOLLOW_CAN_ID);

  private final List<StatusSignal<AngularVelocity>> measuredVelocityRpsSignals =
      List.of(leadMotor.getVelocity(), followMotor.getVelocity());
  private final List<StatusSignal<Double>> setpointVelocityRpsSignals =
      List.of(leadMotor.getClosedLoopReference(), followMotor.getClosedLoopReference());
  private final List<StatusSignal<Angle>> position =
      List.of(leadMotor.getPosition(), followMotor.getPosition());

  private final List<StatusSignal<Voltage>> appliedVoltage =
      List.of(leadMotor.getMotorVoltage(), followMotor.getMotorVoltage());
  private final List<StatusSignal<Current>> supplyCurrentAmps =
      List.of(leadMotor.getSupplyCurrent(), followMotor.getSupplyCurrent());
  private final List<StatusSignal<Current>> torqueCurrentAmps =
      List.of(leadMotor.getTorqueCurrent(), followMotor.getTorqueCurrent());
  private final List<StatusSignal<Temperature>> deviceTemperature =
      List.of(leadMotor.getDeviceTemp(), followMotor.getDeviceTemp());

  private final Follower followController =
      new Follower(
          ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_LEAD_CAN_ID,
          MotorAlignmentValue.Opposed);

  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0.0);

  private TalonFXConfiguration config = new TalonFXConfiguration();

  public FlywheelIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable =
        ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_CURRENT_LIMIT_ENABLE;
    config.MotorOutput.NeutralMode = ShooterConstants.ComponentsConstants.Flywheel.FLYWHEEL_NEUTRAL_MODE;
    config.MotorOutput.Inverted =
        ShooterConstants.ComponentsConstants.Flywheel.MAIN_FLYWHEEL_INVERTED_VALUE;
    config.Slot0.kP = ShooterConstants.ComponentsConstants.Flywheel.Gains.kP;
    config.Slot0.kI = ShooterConstants.ComponentsConstants.Flywheel.Gains.kI;
    config.Slot0.kD = ShooterConstants.ComponentsConstants.Flywheel.Gains.kD;
    config.Slot0.kA = ShooterConstants.ComponentsConstants.Flywheel.Gains.kA;
    config.Slot0.kV = ShooterConstants.ComponentsConstants.Flywheel.Gains.kV;
    config.Slot0.kS = ShooterConstants.ComponentsConstants.Flywheel.Gains.kS;

    config.Audio.AllowMusicDurDisable = true;
    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = false;

    tryUntilOk(CONFIG_RETRY_COUNT, () -> leadMotor.getConfigurator().apply(config, CONFIG_TIMEOUT_SEC));
    tryUntilOk(CONFIG_RETRY_COUNT, () -> followMotor.getConfigurator().apply(config, CONFIG_TIMEOUT_SEC));
    followMotor.setControl(followController);

    BaseStatusSignal.setUpdateFrequencyForAll(
        STATUS_UPDATE_HZ,
        setpointVelocityRpsSignals.get(0),
        setpointVelocityRpsSignals.get(1),
        measuredVelocityRpsSignals.get(0),
        measuredVelocityRpsSignals.get(1),
        position.get(0),
        position.get(1),
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrentAmps.get(0),
        supplyCurrentAmps.get(1),
        torqueCurrentAmps.get(0),
        torqueCurrentAmps.get(1),
        deviceTemperature.get(0),
        deviceTemperature.get(1));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        setpointVelocityRpsSignals.get(0),
        setpointVelocityRpsSignals.get(1),
        measuredVelocityRpsSignals.get(0),
        measuredVelocityRpsSignals.get(1),
        position.get(0),
        position.get(1),
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrentAmps.get(0),
        supplyCurrentAmps.get(1),
        torqueCurrentAmps.get(0),
        torqueCurrentAmps.get(1),
        deviceTemperature.get(0),
        deviceTemperature.get(1));

    inputs.motorMeasuredVelocityRps =
        measuredVelocityRpsSignals.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.motorSetpointVelocityRps =
        setpointVelocityRpsSignals.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.motorMeasuredVelocityRpm =
        new double[] {
          (inputs.motorMeasuredVelocityRps[0] * 60), (inputs.motorMeasuredVelocityRps[1] * 60)
        };
    inputs.motorSetpointVelocityRpm =
        new double[] {
          (inputs.motorSetpointVelocityRps[0] * 60), (inputs.motorSetpointVelocityRps[1] * 60)
        };
    inputs.mainFlywheelRpm =
        ((inputs.motorMeasuredVelocityRpm[0] + inputs.motorMeasuredVelocityRpm[1]) / 2)
            / ShooterConstants.ComponentsConstants.Flywheel.TurretMotorToMainFlyWheelReduction;

    inputs.hoodFlywheelRpm =
        ((inputs.motorMeasuredVelocityRpm[0] + inputs.motorMeasuredVelocityRpm[1]) / 2)
            / ShooterConstants.ComponentsConstants.Flywheel.TurretMotorToHoodFlyWheelReduction;

    inputs.deviceTemperatureCelsius =
        deviceTemperature.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.appliedVolts =
        appliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.supplyCurrentAmps =
        supplyCurrentAmps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.torqueCurrentAmps =
        torqueCurrentAmps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.motorPositionRadians =
        position.stream()
            .mapToDouble(signal -> Units.rotationsToRadians(signal.getValueAsDouble()))
            .toArray();
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    PhoenixUtil.tryUntilOk(CONFIG_RETRY_COUNT, () -> leadMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(CONFIG_RETRY_COUNT, () -> followMotor.getConfigurator().apply(config));
  }

  @Override
  public void runSetVelocity(double setpointVelocityRotPerSec) {
    velocityRequest.Velocity = setpointVelocityRotPerSec;
    velocityRequest.Acceleration =
        ShooterConstants.ComponentsConstants.Flywheel.Gains.VELOCITY_ACCELERATION_RPS_PER_SEC;
    leadMotor.setControl(velocityRequest.withEnableFOC(true));
  }
}
