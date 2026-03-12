package frc.robot.subsystems.intake.intakeRollers;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.List;

public class IntakeRollerIOTalonFX implements IntakeRollersIO {
  private final TalonFX LeadMotor =
      new TalonFX(IntakeConstants.IntakeRollerConstants.MAIN_INTAKE_ROLLER_CAN_ID);
  private final TalonFX FollowMoter = new
  TalonFX(IntakeConstants.IntakeRollerConstants.FOLLOW_INTAKE_ROLLER_CAN_ID);

  private final List<StatusSignal<AngularVelocity>> measuredVeloRPS =
  List.of(LeadMotor.getVelocity(), FollowMoter.getVelocity());
  private final List<StatusSignal<Double>> setVeloRPS =
  List.of(LeadMotor.getClosedLoopReference(), FollowMoter.getClosedLoopReference());
  private final List<StatusSignal<Angle>> position = List.of(LeadMotor.getPosition(),
  FollowMoter.getPosition());

  private final List<StatusSignal<Voltage>> appliedVoltage = List.of(LeadMotor.getMotorVoltage(),
  FollowMoter.getMotorVoltage());
  private final List<StatusSignal<Current>> supplyCurrentAmps =
  List.of(LeadMotor.getSupplyCurrent(), FollowMoter.getSupplyCurrent());
  private final List<StatusSignal<Current>> torqueCurrentAmps =
  List.of(LeadMotor.getTorqueCurrent(), FollowMoter.getTorqueCurrent());
  private final List<StatusSignal<Temperature>> deviceTemperature =
  List.of(LeadMotor.getDeviceTemp(), FollowMoter.getDeviceTemp());


  private final Follower followController = new
  Follower(IntakeConstants.IntakeRollerConstants.MAIN_INTAKE_ROLLER_CAN_ID, MotorAlignmentValue.Opposed);

//   private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0.0);

  private TalonFXConfiguration config = new TalonFXConfiguration();

  public IntakeRollerIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit =
        IntakeConstants.IntakeRollerConstants.INTAKE_ROLLER_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable =
        IntakeConstants.IntakeRollerConstants.INTAKE_ROLLER_CURRENT_LIMIT_ENABLE;
    config.MotorOutput.NeutralMode =
        IntakeConstants.IntakeRollerConstants.INTAKE_ROLL_NEUTRAL_MODE_VALUE;
    config.MotorOutput.Inverted = IntakeConstants.IntakeRollerConstants.INTAKE_ROLL_INVERTED_VALUE;

    config.Slot0.kI = IntakeConstants.IntakeRollerConstants.kI;
    config.Slot0.kP = IntakeConstants.IntakeRollerConstants.kP;
    config.Slot0.kD = IntakeConstants.IntakeRollerConstants.kD;
    config.Slot0.kA = IntakeConstants.IntakeRollerConstants.kA;
    config.Slot0.kV = IntakeConstants.IntakeRollerConstants.kV;
    config.Slot0.kS = IntakeConstants.IntakeRollerConstants.kS;

    tryUntilOk(5, () -> LeadMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () ->
        FollowMoter.getConfigurator().apply(config, 0.25)
        );

    FollowMoter.setControl(followController);

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
  public void updateInputs(IntakeRollersIOInputs inputs) {
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
        new double[] {(inputs.motorMeasuredVelocityRPS[0] * 60)
          // (inputs.motorMeasuredVelocityRPS[1] * 60)
        };
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
    tryUntilOk(5, () -> LeadMotor.getConfigurator().apply(config));
  }

  @Override
  public void runSetVelocity(double velocity) {
    LeadMotor.setControl(velocityRequest.withVelocity(velocity / 60));
  }
}
