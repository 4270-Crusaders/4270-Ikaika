package frc.robot.subsystems.shooter.turret;

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
  private final TalonFX Motor = new TalonFX(ShooterConstants.TurretConstants.TURRET_CAN_ID);
  private final CANcoder Encoder =
      new CANcoder(ShooterConstants.TurretConstants.TURRET_ENCODER_CAN_ID);

  // Status signals for commonly-used telemetry values
  private final StatusSignal<AngularVelocity> veloRPS = Motor.getVelocity();
  private final StatusSignal<Double> setPosRot = Motor.getClosedLoopReference();
  private final StatusSignal<Angle> measuredPosRot = Motor.getPosition();
  private final StatusSignal<Angle> measuredEncoderPosRot = Encoder.getAbsolutePosition();
  private final StatusSignal<Voltage> appliedVoltage = Motor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrentAmps = Motor.getSupplyCurrent();
  private final StatusSignal<Current> torqueCurrentAmps = Motor.getTorqueCurrent();
  private final StatusSignal<Temperature> deviceTemperature = Motor.getDeviceTemp();

  // MotionMagic/torque FOC request object (used for position commands)
  private final MotionMagicExpoTorqueCurrentFOC positionRequest =
      new MotionMagicExpoTorqueCurrentFOC(0.0);

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  public TurretIOTalonFX() {
    // Configure encoder and motor controller using Turret-specific constants
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        ShooterConstants.TurretConstants.TurretEncoderAbsoluteSensorDiscontinuityPoint;
    encoderConfig.MagnetSensor.SensorDirection =
        ShooterConstants.TurretConstants.turretEncoderDirection;
    encoderConfig.MagnetSensor.MagnetOffset =
        ShooterConstants.TurretConstants.TurretEncoderMagnetOffset;

    // Retry applying CANcoder config to tolerate CAN startup races
    tryUntilOk(5, () -> Encoder.getConfigurator().apply(encoderConfig, 0.25));

    motorConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.TurretConstants.TurretCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        ShooterConstants.TurretConstants.TurretSupplyCurrentLimitEnable;
    motorConfig.MotorOutput.NeutralMode = ShooterConstants.TurretConstants.TurretNeutralModeValue;
    motorConfig.MotorOutput.Inverted = ShooterConstants.TurretConstants.TurretInvertedValue;

    // Default PID / feedforward values from constants
    motorConfig.Slot0.kP = ShooterConstants.TurretConstants.TurretkP;
    motorConfig.Slot0.kI = ShooterConstants.TurretConstants.TurretkI;
    motorConfig.Slot0.kD = ShooterConstants.TurretConstants.TurretkD;
    motorConfig.Slot0.kA = ShooterConstants.TurretConstants.TurretkA;
    motorConfig.Slot0.kV =
        ShooterConstants.TurretConstants
            .TurretMotionMagickV; // reuse motion magic kV if appropriate
    motorConfig.Slot0.kS = 0; // leave static gain at 0 unless provided in constants
    motorConfig.Slot0.kG = ShooterConstants.TurretConstants.TurretkG;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Motion magic defaults
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.TurretConstants.TurretMotionMagicVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.TurretConstants.TurretMotionMagicAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk =
        ShooterConstants.TurretConstants.TurretMotionMagicJerk;
    motorConfig.MotionMagic.MotionMagicExpo_kA =
        ShooterConstants.TurretConstants.TurretMotionMagickA;
    motorConfig.MotionMagic.MotionMagicExpo_kV =
        ShooterConstants.TurretConstants.TurretMotionMagickV;

    // Feedback sensor setup: use fused CANcoder similar to hood
    motorConfig.Feedback.FeedbackRemoteSensorID = Encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.SensorToMechanismRatio =
        ShooterConstants.TurretConstants.TurretSensorToMechanismRatio;
    motorConfig.Feedback.RotorToSensorRatio =
        ShooterConstants.TurretConstants.TurretRotorToSensorRatio;

    tryUntilOk(5, () -> Motor.getConfigurator().apply(motorConfig, 0.25));

    // Set the update frequency for the status signals we plan to read
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        veloRPS,
        setPosRot,
        measuredPosRot,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature,
        measuredEncoderPosRot);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Refresh the cached status signals and populate the inputs struct.
    BaseStatusSignal.refreshAll(
        veloRPS,
        setPosRot,
        measuredPosRot,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature,
        measuredEncoderPosRot);

    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.deviceTemperature = deviceTemperature.getValueAsDouble();
    // Convert rotations to degrees for convenience in higher-level code
    inputs.measuredPostionDeg = Units.rotationsToDegrees(measuredPosRot.getValueAsDouble());
    inputs.setPostionDeg = Units.rotationsToDegrees(setPosRot.getValueAsDouble());
    inputs.velocityRadPerSec = veloRPS.getValue().in(RadiansPerSecond);
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.measuredEncoderPositionRot = measuredEncoderPosRot.getValueAsDouble();
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
    tryUntilOk(5, () -> Motor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setMotionMagicConstraints(
      double jerk, double acceleration, double velocity, double expokA, double expokV) {
    motorConfig.MotionMagic.MotionMagicJerk = jerk;
    motorConfig.MotionMagic.MotionMagicAcceleration = acceleration;
    motorConfig.MotionMagic.MotionMagicExpo_kA = expokA;
    motorConfig.MotionMagic.MotionMagicExpo_kV = expokV;
    tryUntilOk(5, () -> Motor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void runSetpointDegree(double setpointDeg) {
    Motor.setControl(positionRequest.withPosition(Units.degreesToRotations(setpointDeg)));
  }

  // public static double calculateTurretTarget(double currentAngle, double targetAngle){
  // }
}
