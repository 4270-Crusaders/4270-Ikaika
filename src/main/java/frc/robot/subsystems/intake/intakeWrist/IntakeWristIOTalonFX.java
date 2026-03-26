package frc.robot.subsystems.intake.intakeWrist;

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
import frc.robot.util.ImperialMarchChime;
import org.littletonrobotics.junction.Logger;

public class IntakeWristIOTalonFX implements IntakeWristIO {
  private final TalonFX WristMotor =
      new TalonFX(IntakeConstants.IntakeWristConstants.INTAKE_WRIST_CAN_ID);

  private final StatusSignal<AngularVelocity> veloRPS = WristMotor.getVelocity();
  private final StatusSignal<Double> setVeloRPS = WristMotor.getClosedLoopReference();
  private final StatusSignal<Angle> measuredPosRot = WristMotor.getPosition();
  private final StatusSignal<Voltage> appliedVoltage = WristMotor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrentAmps = WristMotor.getSupplyCurrent();
  private final StatusSignal<Current> torqueCurrentAmps = WristMotor.getTorqueCurrent();
  private final StatusSignal<Temperature> deviceTemperature = WristMotor.getDeviceTemp();

  private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  public IntakeWristIOTalonFX() {
    motorConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConstants.IntakeWristConstants.INTAKE_WRIST_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        IntakeConstants.IntakeWristConstants.INTAKE_WRIST_CURRENT_LIMIT_ENABLE;
    motorConfig.MotorOutput.NeutralMode =
        IntakeConstants.IntakeWristConstants.INTAKE_WRIST_NEUTRAL_MODE_VALUE;
    motorConfig.MotorOutput.Inverted =
        IntakeConstants.IntakeWristConstants.INTAKE_WRIST_INVERTED_VALUE;

    motorConfig.Slot0.kP = IntakeConstants.IntakeWristConstants.IntakeWristkP;
    motorConfig.Slot0.kI = IntakeConstants.IntakeWristConstants.IntakeWristkI;
    motorConfig.Slot0.kD = IntakeConstants.IntakeWristConstants.IntakeWristkD;
    motorConfig.Slot0.kA = IntakeConstants.IntakeWristConstants.IntakeWristkA;
    motorConfig.Slot0.kV = IntakeConstants.IntakeWristConstants.IntakeWristkV;
    motorConfig.Slot0.kS = IntakeConstants.IntakeWristConstants.IntakeWristkS;
    motorConfig.Slot0.kG = IntakeConstants.IntakeWristConstants.IntakeWristkG;
    motorConfig.Slot0.GravityType = IntakeConstants.IntakeWristConstants.intakeWristGravityType;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.IntakeWristConstants
            .IntakeWristMotionMagicVelocity; // Target cruise velocity in rps
    motorConfig.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.IntakeWristConstants
            .IntakeWristMotionMagicAcceleration; // Target acceleration in rps/s (0.5 seconds)
    motorConfig.MotionMagic.MotionMagicJerk =
        IntakeConstants.IntakeWristConstants
            .IntakeWristMotionMagicJerk; // Target jerk in rps/s/s (0.1 seconds)
    motorConfig.MotionMagic.MotionMagicExpo_kA =
        IntakeConstants.IntakeWristConstants.IntakeWristMotionMagickA;
    motorConfig.MotionMagic.MotionMagicExpo_kV =
        IntakeConstants.IntakeWristConstants.IntakeWristMotionMagickV;

    motorConfig.Feedback.SensorToMechanismRatio =
        IntakeConstants.IntakeWristConstants.IntakeWristSensorToMechanismRatio;
    motorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    tryUntilOk(5, () -> WristMotor.getConfigurator().apply(motorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        veloRPS,
        setVeloRPS,
        measuredPosRot,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        deviceTemperature);

    ImperialMarchChime.registerChimeMotor(WristMotor);
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
    tryUntilOk(5, () -> WristMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setMotionMagicConstraints(
      double jerk, double acceleration, double velocity, double expokA, double expokV) {
    motorConfig.MotionMagic.MotionMagicJerk = jerk;
    motorConfig.MotionMagic.MotionMagicAcceleration = acceleration;
    motorConfig.MotionMagic.MotionMagicExpo_kA = expokA;
    motorConfig.MotionMagic.MotionMagicExpo_kV = expokV;
    tryUntilOk(5, () -> WristMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void runSetpointDegree(double setpointDeg) {
    if (ImperialMarchChime.isPlaying()) {
      return;
    }
    Logger.recordOutput("Intake/Wrist/SetpointDegree", setpointDeg);
    WristMotor.setControl(
        positionRequest.withPosition(Units.degreesToRotations(setpointDeg)).withEnableFOC(true));
  }

  @Override
  public void runSetVoltage(double voltage) {
    if (ImperialMarchChime.isPlaying()) {
      return;
    }
    WristMotor.setControl(voltageRequest.withEnableFOC(true).withOutput(voltage));
  }
}
