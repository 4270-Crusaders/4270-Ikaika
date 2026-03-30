package frc.robot.subsystems.shooter.turret;

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
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * TalonFX-based Turret hardware implementation.
 *
 * <p>Mirrors the HoodIOTalonFX approach: configure a TalonFX and a CANcoder (absolute encoder),
 * expose status signals, provide methods to update inputs, apply PID/motion constraints, and
 * command a closed-loop position setpoint using MotionMagicExpoTorqueCurrentFOC.
 */
public class TurretIOTalonFX implements TurretIO {
  // Motor and absolute encoder on the turret
  private final TalonFX motor = new TalonFX(ShooterConstants.ComponentsConstants.Turret.TURRET_CAN_ID);
  private final CANcoder encoder =
      new CANcoder(ShooterConstants.ComponentsConstants.Turret.TURRET_ENCODER_CAN_ID);

  // Status signals for commonly-used telemetry values
  private final StatusSignal<AngularVelocity> velocityRps = motor.getVelocity();
  private final StatusSignal<Double> setpointPositionRotations = motor.getClosedLoopReference();
  private final StatusSignal<Angle> measuredPositionRotations = motor.getPosition();
  private final StatusSignal<Angle> measuredEncoderPositionRotations = encoder.getAbsolutePosition();
  private final StatusSignal<Voltage> appliedVoltage = motor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
  private final StatusSignal<Current> torqueCurrentAmps = motor.getTorqueCurrent();
  private final StatusSignal<Temperature> deviceTemperature = motor.getDeviceTemp();

  // MotionMagic/torque FOC request object (used for position commands)
  private final MotionMagicExpoTorqueCurrentFOC positionRequest =
      new MotionMagicExpoTorqueCurrentFOC(0.0);

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  public TurretIOTalonFX() {
    // Configure encoder and motor controller using Turret-specific constants
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        ShooterConstants.ComponentsConstants.Turret.TurretEncoderAbsoluteSensorDiscontinuityPoint;
    encoderConfig.MagnetSensor.SensorDirection =
        ShooterConstants.ComponentsConstants.Turret.turretEncoderDirection;
    encoderConfig.MagnetSensor.MagnetOffset =
        ShooterConstants.ComponentsConstants.Turret.TurretEncoderMagnetOffset;

    // Retry applying CANcoder config to tolerate CAN startup races
    tryUntilOk(
        CONFIG_RETRY_COUNT, () -> encoder.getConfigurator().apply(encoderConfig, CONFIG_APPLY_TIMEOUT_SEC));

    motorConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.ComponentsConstants.Turret.TurretCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        ShooterConstants.ComponentsConstants.Turret.TurretSupplyCurrentLimitEnable;
    motorConfig.MotorOutput.NeutralMode =
        ShooterConstants.ComponentsConstants.Turret.TurretNeutralModeValue;
    motorConfig.MotorOutput.Inverted = ShooterConstants.ComponentsConstants.Turret.TurretInvertedValue;

    // Default PID / feedforward values from constants
    motorConfig.Slot0.kP = ShooterConstants.ComponentsConstants.Turret.Gains.kP;
    motorConfig.Slot0.kI = ShooterConstants.ComponentsConstants.Turret.Gains.kI;
    motorConfig.Slot0.kD = ShooterConstants.ComponentsConstants.Turret.Gains.kD;
    motorConfig.Slot0.kA = ShooterConstants.ComponentsConstants.Turret.Gains.kA;
    motorConfig.Slot0.kV =
        ShooterConstants.ComponentsConstants.Turret.Gains.kV;
    motorConfig.Slot0.kS = ShooterConstants.ComponentsConstants.Turret.Gains.kS;
    motorConfig.Slot0.kG = ShooterConstants.ComponentsConstants.Turret.Gains.kG;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Motion magic defaults
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.ComponentsConstants.Turret.Gains.MotionMagic.velocity;
    motorConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.ComponentsConstants.Turret.Gains.MotionMagic.acceleration;
    motorConfig.MotionMagic.MotionMagicJerk =
        ShooterConstants.ComponentsConstants.Turret.Gains.MotionMagic.jerk;
    motorConfig.MotionMagic.MotionMagicExpo_kA =
        ShooterConstants.ComponentsConstants.Turret.Gains.MotionMagic.expo_kA;
    motorConfig.MotionMagic.MotionMagicExpo_kV =
        ShooterConstants.ComponentsConstants.Turret.Gains.MotionMagic.expo_kV;

    // Feedback sensor setup: use fused CANcoder similar to hood
    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.SensorToMechanismRatio =
        ShooterConstants.ComponentsConstants.Turret.sensorToMechanismRatio;
    motorConfig.Feedback.RotorToSensorRatio =
        ShooterConstants.ComponentsConstants.Turret.rotorToSensorRatio;

    tryUntilOk(
        CONFIG_RETRY_COUNT, () -> motor.getConfigurator().apply(motorConfig, CONFIG_APPLY_TIMEOUT_SEC));

    // Set the update frequency for the status signals we plan to read
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
  public void updateInputs(TurretIOInputs inputs) {
    // Refresh the cached status signals and populate the inputs struct.
    BaseStatusSignal.refreshAll(
        velocityRps,
        setpointPositionRotations,
        measuredPositionRotations,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature,
        measuredEncoderPositionRotations);

    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.deviceTemperature = deviceTemperature.getValueAsDouble();
    // Convert rotations to degrees for convenience in higher-level code
    inputs.measuredPostionDeg = Units.rotationsToDegrees(measuredPositionRotations.getValueAsDouble());
    inputs.setPostionDeg = Units.rotationsToDegrees(setpointPositionRotations.getValueAsDouble());
    inputs.velocityRadPerSec = velocityRps.getValue().in(RadiansPerSecond);
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.measuredEncoderPositionRot = measuredEncoderPositionRotations.getValueAsDouble();
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
    // Apply updated gains to the motor controller (retries for reliability)
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
    motor.setControl(positionRequest.withPosition(Units.degreesToRotations(setpointDeg)));
  }

  // public static double calculateTurretTarget(double currentAngle, double targetAngle){
  // }
}
