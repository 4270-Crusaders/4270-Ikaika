package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {

  public static final class RollersConstants {
    public static final int ROLLERS_CAN_ID = 40;
    public static final double ROLLERS_CURRENT_LIMIT = 15;
    public static final boolean ROLLERS_CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue ROLLERS_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    public static final InvertedValue TOP_ROLL_INVERTED_VALUE =
        InvertedValue.CounterClockwise_Positive;
    public static final double kP = 16;
    public static final double kI = 0.0;
    public static final double kD = 0.001;
    public static final double kA = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 2;
  }

  public static final class ConveyorConstants {
    public static final int CONVEYOR_CAN_ID = 41;
    public static final double CONVEYOR_CURRENT_LIMIT = 25;
    public static final boolean CONVEYOR_CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue CONVEY_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    public static final InvertedValue CONVEYOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    public static final double kP = 0.4;
    public static final double kI = 0.2;
    public static final double kD = 0.0;
    public static final double kA = 0.0;
    public static final double kV = 0.121666;
    public static final double kS = 0.436549;
  }

  public static final class AgitatorConstants {
    public static final int AGITATOR_CAN_ID = 42;
    public static final double AGITATOR_CURRENT_LIMIT = 60;
    public static final boolean AGITATOR_CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue AGITATOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    public static final InvertedValue AGITI_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    public static final double kP = 11;
    public static final double kI = 0.001;
    public static final double kD = 0.0;
    public static final double kA = 0.0;
    public static final double kV = 0.0;
    public static final double kS = 1;
  }

  public static final class KickerConstants {
    public static final int KICKER_CAN_ID = 43;
    public static final double KICKER_CURRENT_LIMIT = 75;
    public static final boolean KICKER_CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue KICKER_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    public static final InvertedValue KICKER_INVERTED_VALUE =
        InvertedValue.CounterClockwise_Positive;
    public static final double kP = 0.025;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kA = 0.0;
    public static final double kV = 0.11658;
    public static final double kS = 0.247818;
  }
}
