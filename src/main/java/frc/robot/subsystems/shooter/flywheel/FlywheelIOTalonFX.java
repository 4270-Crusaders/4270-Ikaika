package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.PhoenixUtil;
import java.util.List;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX LeadMotor =
      new TalonFX(ShooterConstants.FlywheelConstants.FLYWHEEL_LEAD_CAN_ID);
  private final TalonFX FollowMotor =
      new TalonFX(ShooterConstants.FlywheelConstants.FLYWHEEL_FOLLOW_CAN_ID);

  private final List<StatusSignal<AngularVelocity>> measuredVeloRPS =
      List.of(LeadMotor.getVelocity(), FollowMotor.getVelocity());
  private final List<StatusSignal<Double>> setVeloRPS =
      List.of(LeadMotor.getClosedLoopReference(), FollowMotor.getClosedLoopReference());
  private final List<StatusSignal<Angle>> position =
      List.of(LeadMotor.getPosition(), FollowMotor.getPosition());

  private final List<StatusSignal<Voltage>> appliedVoltage =
      List.of(LeadMotor.getMotorVoltage(), FollowMotor.getMotorVoltage());
  private final List<StatusSignal<Current>> supplyCurrentAmps =
      List.of(LeadMotor.getSupplyCurrent(), FollowMotor.getSupplyCurrent());
  private final List<StatusSignal<Current>> torqueCurrentAmps =
      List.of(LeadMotor.getTorqueCurrent(), FollowMotor.getTorqueCurrent());
  private final List<StatusSignal<Temperature>> deviceTemperature =
      List.of(LeadMotor.getDeviceTemp(), FollowMotor.getDeviceTemp());

  private final Follower followController =
      new Follower(
          ShooterConstants.FlywheelConstants.FLYWHEEL_LEAD_CAN_ID, MotorAlignmentValue.Opposed);
  private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  private TalonFXConfiguration config = new TalonFXConfiguration();

  public FlywheelIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.FlywheelConstants.FLYWHEEL_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable =
        ShooterConstants.FlywheelConstants.FLYWHEEL_CURRENT_LIMIT_ENABLE;
    config.MotorOutput.NeutralMode = ShooterConstants.FlywheelConstants.FLYWHEEL_NEUTRAL_MODE;
    config.MotorOutput.Inverted = ShooterConstants.FlywheelConstants.MAIN_FLYWHEEL_INVERTED_VALUE;
    config.Slot0.kP = ShooterConstants.FlywheelConstants.FlyWheelkP;
    config.Slot0.kI = ShooterConstants.FlywheelConstants.FlyWheelkI;
    config.Slot0.kD = ShooterConstants.FlywheelConstants.FlyWheelkD;
    config.Slot0.kA = ShooterConstants.FlywheelConstants.FlyWheelkA;
    config.Slot0.kV = ShooterConstants.FlywheelConstants.FlyWheelkV;
    config.Slot0.kS = ShooterConstants.FlywheelConstants.FlyWheelkS;
    config.MotionMagic.MotionMagicJerk = ShooterConstants.FlywheelConstants.FlyWheelMotionMagicJerk;
    config.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.FlywheelConstants.FlyWheelMotionMagicAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.FlywheelConstants.FlyWheelMotionMagicVelocity;

    tryUntilOk(5, () -> LeadMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> FollowMotor.getConfigurator().apply(config, 0.25));
    FollowMotor.setControl(followController);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        setVeloRPS.get(0),
        setVeloRPS.get(1),
        measuredVeloRPS.get(0),
        measuredVeloRPS.get(1),
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
        setVeloRPS.get(0),
        setVeloRPS.get(1),
        measuredVeloRPS.get(0),
        measuredVeloRPS.get(1),
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

    inputs.motorMeasuredVelocityRPS =
        measuredVeloRPS.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.motorSetpointVelocityRPS =
        setVeloRPS.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.motorMeasuredVelocityRPM =
        new double[] {
          (inputs.motorMeasuredVelocityRPS[0] * 60), (inputs.motorMeasuredVelocityRPS[1] * 60)
        };
    inputs.MainFlyWheelRPM =
        ((inputs.motorMeasuredVelocityRPM[0] + inputs.motorMeasuredVelocityRPM[1]) / 2)
            / ShooterConstants.FlywheelConstants.TurretMotorToMainFlyWheelReduction;

    inputs.HoodFlyWheelRPM =
        ((inputs.motorMeasuredVelocityRPM[0] + inputs.motorMeasuredVelocityRPM[1]) / 2)
            / ShooterConstants.FlywheelConstants.TurretMotorToHoodFlyWheelReduction;

    inputs.deviceTemperature =
        deviceTemperature.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.appliedVolts =
        appliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.supplyCurrentAmps =
        supplyCurrentAmps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.torqueCurrentAmps =
        torqueCurrentAmps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.positionRad = position.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    PhoenixUtil.tryUntilOk(5, () -> LeadMotor.getConfigurator().apply(config));
  }

  @Override
  public void setMotionMagicConstraints(double jerk, double acceleration, double velocity) {
    config.MotionMagic.MotionMagicJerk = jerk;
    config.MotionMagic.MotionMagicAcceleration = acceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = velocity;

    PhoenixUtil.tryUntilOk(5, () -> LeadMotor.getConfigurator().apply(config));

    velocityRequest.Acceleration = acceleration;
  }

  @Override
  public void runSetVelocity(double setpointVelocityRotPerSec) {
    LeadMotor.setControl(velocityRequest.withVelocity(setpointVelocityRotPerSec));
  }
}
