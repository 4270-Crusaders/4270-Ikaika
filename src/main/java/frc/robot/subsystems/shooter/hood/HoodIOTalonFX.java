package frc.robot.subsystems.shooter.hood;

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
  private final TalonFX Motor = new TalonFX(ShooterConstants.HoodConstants.HOOD_CAN_ID);
  private final CANcoder Encoder = new CANcoder(ShooterConstants.HoodConstants.HOOD_ENCODER_CAN_ID);

  private final StatusSignal<AngularVelocity> veloRPS = Motor.getVelocity();
  private final StatusSignal<Double> setPosRot = Motor.getClosedLoopReference();
  private final StatusSignal<Angle> measuredPosRot = Motor.getPosition();
  private final StatusSignal<Angle> measuredEncoderPosRot = Encoder.getAbsolutePosition();
  private final StatusSignal<Voltage> appliedVoltage = Motor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrentAmps = Motor.getSupplyCurrent();
  private final StatusSignal<Current> torqueCurrentAmps = Motor.getTorqueCurrent();
  private final StatusSignal<Temperature> deviceTemperature = Motor.getDeviceTemp();

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
        ShooterConstants.HoodConstants.HoodEncoderAbsoluteSensorDiscontinuityPoint;
    encoderConfig.MagnetSensor.MagnetOffset =
        ShooterConstants.HoodConstants.HoodEncoderMagnetOffset;
    encoderConfig.MagnetSensor.SensorDirection =
        ShooterConstants.HoodConstants.hoodEncoderDirection;

    tryUntilOk(5, () -> Encoder.getConfigurator().apply(encoderConfig, 0.25));

    motorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.HoodConstants.HoodCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        ShooterConstants.HoodConstants.HoodSupplyCurrentLimitEnable;
    motorConfig.MotorOutput.NeutralMode = ShooterConstants.HoodConstants.HoodNeutralModeValue;
    motorConfig.MotorOutput.Inverted = ShooterConstants.HoodConstants.HoodInvertedValue;
    motorConfig.Slot0.kP = ShooterConstants.HoodConstants.HoodkP;
    motorConfig.Slot0.kI = ShooterConstants.HoodConstants.HoodkI;
    motorConfig.Slot0.kD = ShooterConstants.HoodConstants.HoodkD;
    motorConfig.Slot0.kA = ShooterConstants.HoodConstants.HoodkA;
    motorConfig.Slot0.kV = ShooterConstants.HoodConstants.HoodkV;
    motorConfig.Slot0.kS = ShooterConstants.HoodConstants.HoodkS;
    motorConfig.Slot0.kG = ShooterConstants.HoodConstants.HoodkG;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.HoodConstants.HoodMotionMagicVelocity; // Target cruise velocity in rps
    motorConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.HoodConstants
            .HoodMotionMagicAcceleration; // Target acceleration in rps/s (0.5 seconds)
    motorConfig.MotionMagic.MotionMagicJerk =
        ShooterConstants.HoodConstants.HoodMotionMagicJerk; // Target jerk in rps/s/s (0.1 seconds)
    motorConfig.MotionMagic.MotionMagicExpo_kA = ShooterConstants.HoodConstants.HoodMotionMagickA;
    motorConfig.MotionMagic.MotionMagicExpo_kV = ShooterConstants.HoodConstants.HoodMotionMagickV;

    motorConfig.Feedback.FeedbackRemoteSensorID = Encoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.SensorToMechanismRatio =
        ShooterConstants.HoodConstants.HoodSensorToMechanismRatio;
    motorConfig.Feedback.RotorToSensorRatio = ShooterConstants.HoodConstants.HoodRotorToSensorRatio;
    tryUntilOk(5, () -> Motor.getConfigurator().apply(motorConfig, 0.25));

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
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        veloRPS,
        setPosRot,
        measuredPosRot,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature,
        measuredEncoderPosRot);

    // Read values from signals and populate the input struct. The
    // StatusSignal getters convert hardware units to the specified
    // Java types (including unit types used by WPILib Units).
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.deviceTemperature = deviceTemperature.getValueAsDouble();
    // Motor reports rotational position as an Angle; convert to degrees
    inputs.measuredPostionDeg = Units.rotationsToDegrees(measuredPosRot.getValueAsDouble());
    // Closed-loop setpoint as reported by the motor controller
    inputs.setPostionDeg = Units.rotationsToDegrees(setPosRot.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    // Raw absolute encoder rotations (useful for diagnostics/offsets)
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
    // Apply the new PID/gains to the motor controller. tryUntilOk will
    // retry a few times to tolerate temporary CAN bus hiccups.
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
    Logger.recordOutput("Shooter/Hood/SetpostionDegree", setpointDeg);
    Motor.setControl(positionRequest.withPosition(Units.degreesToRotations(setpointDeg)));
  }
}
