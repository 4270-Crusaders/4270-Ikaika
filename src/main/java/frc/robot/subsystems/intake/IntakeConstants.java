// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
  /** Intake wrist (single joint). */
  public static final class IntakeWristConstants {
    public static final int CAN_ID = 30;
    public static final double CURRENT_LIMIT = 30;
    public static final boolean CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static double kP = 150;
    public static double kI = 1;
    public static double kD = 5;
    public static double kA = 0;
    public static double kV = 0;
    public static double kS = 0.01;
    public static double kG = -0.2;
    public static GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;
    public static double motionMagicVelocity = 0.0;
    public static double motionMagicAcceleration = 0.0;
    public static double motionMagicJerk = 0.0;
    public static double motionMagicExpoKA = 0.1;
    public static double motionMagicExpoKV = 0.12;
    public static double sensorToMechanismRatio = 53.3333;
    public static double rotorToSensorRatio = 1;
  }

  /** Intake rollers (two independent Talon FX, velocity closed-loop). */
  public static final class IntakeRollerConstants {
    public static final int LEAD_CAN_ID = 31;
    public static final int FOLLOW_CAN_ID = 32;
    public static final double CURRENT_LIMIT = 50;
    public static final boolean CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final InvertedValue LEAD_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue FOLLOW_INVERTED = InvertedValue.Clockwise_Positive;

    public static final double kP = 0.025;
    public static final double kI = 0.00;
    public static final double kD = 0;
    public static final double kA = 0;
    public static final double kV = 0.0961953;
    public static final double kS = 0.528147;

    public static final double NEAR_GOAL_RPM_TOLERANCE = 300.0;
  }
}
