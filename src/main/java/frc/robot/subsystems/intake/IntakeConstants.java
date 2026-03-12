package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
  // Intake Wrist
  public static final class IntakeWristConstants {
    public static final int INTAKE_WRIST_CAN_ID = 30;
    public static final double INTAKE_WRIST_CURRENT_LIMIT = 60;
    public static final boolean INTAKE_WRIST_CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue INTAKE_WRIST_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    public static final InvertedValue INTAKE_WRIST_INVERTED_VALUE =
        InvertedValue.CounterClockwise_Positive;

    public static double IntakeWristkP = 150;
    public static double IntakeWristkI = 1;
    public static double IntakeWristkD = 5;
    public static double IntakeWristkA = 0;
    public static double IntakeWristkV = 0;
    public static double IntakeWristkS = 0.01;
    public static double IntakeWristkG = -0.2;
    public static GravityTypeValue intakeWristGravityType = GravityTypeValue.Arm_Cosine;
    public static double IntakeWristMotionMagicVelocity = 0.0; // Target cruise velocity in rps
    public static double IntakeWristMotionMagicAcceleration =
        0.0; // Target acceleration in rps/s (0.5 seconds)
    public static double IntakeWristMotionMagicJerk = 0.0; // Target jerk in rps/s/s (0.1 seconds)
    public static double IntakeWristMotionMagickA = 0.1;
    public static double IntakeWristMotionMagickV = 0.12;
    public static double IntakeWristSensorToMechanismRatio = 53.3333;
    public static double IntakeWristRotorToSensorRatio = 1;
  }

  // Intake Roller
  public static final class IntakeRollerConstants {
    public static final int MAIN_INTAKE_ROLLER_CAN_ID = 31;
    public static final int FOLLOW_INTAKE_ROLLER_CAN_ID = 32;
    public static final double INTAKE_ROLLER_CURRENT_LIMIT = 60;
    public static final boolean INTAKE_ROLLER_CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue INTAKE_ROLL_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    public static final InvertedValue INTAKE_ROLL_INVERTED_VALUE = InvertedValue.Clockwise_Positive;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kA = 0;
    public static final double kV = 0;
    public static final double kS = 0;
  }
}
