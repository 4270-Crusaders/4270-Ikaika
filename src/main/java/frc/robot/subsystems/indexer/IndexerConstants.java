package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Indexer hardware IDs, limits, and gains. Nested classes group constants per mechanism (see {@code
 * CodingStandard.md} §9).
 */
public class IndexerConstants {

  public static final double NEAR_GOAL_RPM_TOLERANCE = 5.0;

  public static final class IndexerRollers {
    public static final int CAN_ID = 40;
    public static final double CURRENT_LIMIT = 15;
    public static final boolean CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final class Gains {
      public static final double kP = 0.1;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kA = 0.0;
      public static final double kV = 0.0957467;
      public static final double kS = 0.739238;
    }
  }

  public static final class IndexerConveyor {
    public static final int CAN_ID = 41;
    public static final double CURRENT_LIMIT = 25;
    public static final boolean CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final class Gains {
      public static final double kP = 0.4;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kA = 0.0;
      public static final double kV = 0.121666;
      public static final double kS = 0.436549;
    }
  }

  public static final class IndexerAgitator {
    public static final int CAN_ID = 42;
    public static final double CURRENT_LIMIT = 60;
    public static final boolean CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final class Gains {
      public static final double kP = 11;
      public static final double kI = 0.001;
      public static final double kD = 0.0;
      public static final double kA = 0.0;
      public static final double kV = 0.0;
      public static final double kS = 1.0;
    }
  }

  public static final class IndexerKicker {
    public static final int CAN_ID = 43;
    public static final double CURRENT_LIMIT = 75;
    public static final boolean CURRENT_LIMIT_ENABLE = true;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final class Gains {
      public static final double kP = 0.025;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kA = 0.0;
      public static final double kV = 0.11658;
      public static final double kS = 0.247818;
    }
  }
}
