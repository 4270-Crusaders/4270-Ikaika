package frc.robot.subsystems.intake.intakeWrist;

import static frc.robot.Constants.TalonFxIo.CONFIG_APPLY_TIMEOUT_SEC;
import static frc.robot.Constants.TalonFxIo.CONFIG_RETRY_COUNT;
import static frc.robot.Constants.TalonFxIo.STATUS_SIGNAL_UPDATE_HZ;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeWristIOTalonFX implements IntakeWristIO {
  private final TalonFX wristMotor = new TalonFX(IntakeConstants.IntakeWristConstants.CAN_ID);

  private final StatusSignal<AngularVelocity> veloRPS = wristMotor.getVelocity();
  private final StatusSignal<Double> setVeloRPS = wristMotor.getClosedLoopReference();
  private final StatusSignal<Angle> measuredPosRot = wristMotor.getPosition();
  private final StatusSignal<Voltage> appliedVoltage = wristMotor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrentAmps = wristMotor.getSupplyCurrent();
  private final StatusSignal<Current> torqueCurrentAmps = wristMotor.getTorqueCurrent();
  private final StatusSignal<Temperature> deviceTemperature = wristMotor.getDeviceTemp();

  private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  public IntakeWristIOTalonFX() {
    motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.IntakeWristConstants.CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        IntakeConstants.IntakeWristConstants.CURRENT_LIMIT_ENABLE;
    motorConfig.MotorOutput.NeutralMode = IntakeConstants.IntakeWristConstants.NEUTRAL_MODE;
    motorConfig.MotorOutput.Inverted = IntakeConstants.IntakeWristConstants.INVERTED;

    motorConfig.Slot0.kP = IntakeConstants.IntakeWristConstants.kP;
    motorConfig.Slot0.kI = IntakeConstants.IntakeWristConstants.kI;
    motorConfig.Slot0.kD = IntakeConstants.IntakeWristConstants.kD;
    motorConfig.Slot0.kA = IntakeConstants.IntakeWristConstants.kA;
    motorConfig.Slot0.kV = IntakeConstants.IntakeWristConstants.kV;
    motorConfig.Slot0.kS = IntakeConstants.IntakeWristConstants.kS;
    motorConfig.Slot0.kG = IntakeConstants.IntakeWristConstants.kG;
    motorConfig.Slot0.GravityType = IntakeConstants.IntakeWristConstants.gravityType;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.IntakeWristConstants.motionMagicVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.IntakeWristConstants.motionMagicAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = IntakeConstants.IntakeWristConstants.motionMagicJerk;
    motorConfig.MotionMagic.MotionMagicExpo_kA =
        IntakeConstants.IntakeWristConstants.motionMagicExpoKA;
    motorConfig.MotionMagic.MotionMagicExpo_kV =
        IntakeConstants.IntakeWristConstants.motionMagicExpoKV;

    motorConfig.Feedback.SensorToMechanismRatio =
        IntakeConstants.IntakeWristConstants.sensorToMechanismRatio;
    motorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    tryUntilOk(
        CONFIG_RETRY_COUNT, () -> wristMotor.getConfigurator().apply(motorConfig, CONFIG_APPLY_TIMEOUT_SEC));

    BaseStatusSignal.setUpdateFrequencyForAll(
        STATUS_SIGNAL_UPDATE_HZ,
        veloRPS,
        setVeloRPS,
        measuredPosRot,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature);
  }

  @Override
  public void updateInputs(IntakeWristIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        veloRPS,
        setVeloRPS,
        measuredPosRot,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature);

    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.deviceTemperature = deviceTemperature.getValueAsDouble();
    inputs.measuredPostionDeg = Units.rotationsToDegrees(measuredPosRot.getValueAsDouble());
    inputs.setpointPostionDeg = Units.rotationsToDegrees(setVeloRPS.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
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
    tryUntilOk(CONFIG_RETRY_COUNT, () -> wristMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setMotionMagicConstraints(
      double jerk, double acceleration, double velocity, double expokA, double expokV) {
    motorConfig.MotionMagic.MotionMagicJerk = jerk;
    motorConfig.MotionMagic.MotionMagicAcceleration = acceleration;
    motorConfig.MotionMagic.MotionMagicExpo_kA = expokA;
    motorConfig.MotionMagic.MotionMagicExpo_kV = expokV;
    tryUntilOk(CONFIG_RETRY_COUNT, () -> wristMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void runSetpointDegree(double setpointDeg) {
    Logger.recordOutput("Intake/IntakeWrist/SetpointDegree", setpointDeg);
    wristMotor.setControl(
        positionRequest.withPosition(Units.degreesToRotations(setpointDeg)).withEnableFOC(true));
  }

  @Override
  public void runSetVoltage(double voltage) {
    wristMotor.setControl(voltageRequest.withEnableFOC(true).withOutput(voltage));
  }
}
