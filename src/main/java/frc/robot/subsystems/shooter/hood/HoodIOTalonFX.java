// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.TalonFxIo.CONFIG_APPLY_TIMEOUT_SEC;
import static frc.robot.Constants.TalonFxIo.CONFIG_RETRY_COUNT;
import static frc.robot.Constants.TalonFxIo.STATUS_SIGNAL_UPDATE_HZ;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX motor = new TalonFX(ShooterConstants.ComponentsConstants.Hood.HOOD_CAN_ID);
  private final CANcoder encoder =
      new CANcoder(ShooterConstants.ComponentsConstants.Hood.HOOD_ENCODER_CAN_ID);

  private final StatusSignal<AngularVelocity> velocityRps = motor.getVelocity();
  private final StatusSignal<Double> setpointPositionRotations = motor.getClosedLoopReference();
  private final StatusSignal<Angle> measuredPositionRotations = motor.getPosition();
  private final StatusSignal<Angle> measuredEncoderPositionRotations = encoder.getAbsolutePosition();
  private final StatusSignal<Voltage> appliedVoltage = motor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
  private final StatusSignal<Current> torqueCurrentAmps = motor.getTorqueCurrent();
  private final StatusSignal<Temperature> deviceTemperature = motor.getDeviceTemp();

  private final MotionMagicExpoTorqueCurrentFOC positionRequest =
      new MotionMagicExpoTorqueCurrentFOC(0.0).withOverrideCoastDurNeutral(false);

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  public HoodIOTalonFX() {
    // Constructor configures CANcoder (absolute encoder) and TalonFX
    // motor controller with default values taken from ShooterConstants.
    // tryUntilOk is a helper that retries configuration calls if the
    // initial attempt fails (robustness for CAN bus startup timing).
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        ShooterConstants.ComponentsConstants.Hood.HoodEncoderAbsoluteSensorDiscontinuityPoint;
    encoderConfig.MagnetSensor.MagnetOffset =
        ShooterConstants.ComponentsConstants.Hood.HoodEncoderMagnetOffset;
    encoderConfig.MagnetSensor.SensorDirection =
        ShooterConstants.ComponentsConstants.Hood.hoodEncoderDirection;

    tryUntilOk(
        CONFIG_RETRY_COUNT, () -> encoder.getConfigurator().apply(encoderConfig, CONFIG_APPLY_TIMEOUT_SEC));

    motorConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.ComponentsConstants.Hood.HoodCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        ShooterConstants.ComponentsConstants.Hood.HoodSupplyCurrentLimitEnable;
    motorConfig.MotorOutput.NeutralMode = ShooterConstants.ComponentsConstants.Hood.HoodNeutralModeValue;
    motorConfig.MotorOutput.Inverted = ShooterConstants.ComponentsConstants.Hood.HoodInvertedValue;
    motorConfig.Slot0.kP = ShooterConstants.ComponentsConstants.Hood.Gains.kP;
    motorConfig.Slot0.kI = ShooterConstants.ComponentsConstants.Hood.Gains.kI;
    motorConfig.Slot0.kD = ShooterConstants.ComponentsConstants.Hood.Gains.kD;
    motorConfig.Slot0.kA = ShooterConstants.ComponentsConstants.Hood.Gains.kA;
    motorConfig.Slot0.kV = ShooterConstants.ComponentsConstants.Hood.Gains.kV;
    motorConfig.Slot0.kS = ShooterConstants.ComponentsConstants.Hood.Gains.kS;
    motorConfig.Slot0.kG = ShooterConstants.ComponentsConstants.Hood.Gains.kG;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.ComponentsConstants.Hood.Gains.MotionMagic.velocity; // Target cruise velocity in rps
    motorConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.ComponentsConstants.Hood.Gains.MotionMagic.acceleration; // Target acceleration in rps/s (0.5 seconds)
    motorConfig.MotionMagic.MotionMagicJerk =
        ShooterConstants.ComponentsConstants.Hood.Gains.MotionMagic.jerk; // Target jerk in rps/s/s (0.1 seconds)
    motorConfig.MotionMagic.MotionMagicExpo_kA =
        ShooterConstants.ComponentsConstants.Hood.Gains.MotionMagic.expo_kA;
    motorConfig.MotionMagic.MotionMagicExpo_kV =
        ShooterConstants.ComponentsConstants.Hood.Gains.MotionMagic.expo_kV;

    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.SensorToMechanismRatio =
        ShooterConstants.ComponentsConstants.Hood.sensorToMechanismRatio;
    motorConfig.Feedback.RotorToSensorRatio =
        ShooterConstants.ComponentsConstants.Hood.rotorToSensorRatio;
    tryUntilOk(
        CONFIG_RETRY_COUNT, () -> motor.getConfigurator().apply(motorConfig, CONFIG_APPLY_TIMEOUT_SEC));

    BaseStatusSignal.setUpdateFrequencyForAll(
        STATUS_SIGNAL_UPDATE_HZ,
        velocityRps,
        setpointPositionRotations,
        measuredPositionRotations,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature,
        measuredEncoderPositionRotations);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityRps,
        setpointPositionRotations,
        measuredPositionRotations,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature,
        measuredEncoderPositionRotations);

    // Read values from signals and populate the input struct. The
    // StatusSignal getters convert hardware units to the specified
    // Java types (including unit types used by WPILib Units).
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.deviceTemperature = deviceTemperature.getValueAsDouble();
    // Motor reports rotational position as an Angle; convert to degrees
    inputs.measuredPostionDeg = Units.rotationsToDegrees(measuredPositionRotations.getValueAsDouble());
    // Closed-loop setpoint as reported by the motor controller
    inputs.setPostionDeg = Units.rotationsToDegrees(setpointPositionRotations.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    // Raw absolute encoder rotations (useful for diagnostics/offsets)
    inputs.measuredEncoderPositionRot = measuredEncoderPositionRotations.getValueAsDouble();
    inputs.velocityRadPerSec = velocityRps.getValue().in(RadiansPerSecond);
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kV = kV;
    motorConfig.Slot0.kA = kA;
    motorConfig.Slot0.kG = kG;
    // Apply the new PID/gains to the motor controller. tryUntilOk will
    // retry a few times to tolerate temporary CAN bus hiccups.
    tryUntilOk(CONFIG_RETRY_COUNT, () -> motor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setMotionMagicConstraints(
      double jerk, double acceleration, double velocity, double expokA, double expokV) {
    motorConfig.MotionMagic.MotionMagicJerk = jerk;
    motorConfig.MotionMagic.MotionMagicAcceleration = acceleration;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = velocity;
    motorConfig.MotionMagic.MotionMagicExpo_kA = expokA;
    motorConfig.MotionMagic.MotionMagicExpo_kV = expokV;
    tryUntilOk(CONFIG_RETRY_COUNT, () -> motor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void runSetpointDegree(double setpointDeg) {
    Logger.recordOutput("Shooter/Hood/SetpostionDegree", setpointDeg);
    motor.setControl(positionRequest.withPosition(Units.degreesToRotations(setpointDeg)));
  }
}
