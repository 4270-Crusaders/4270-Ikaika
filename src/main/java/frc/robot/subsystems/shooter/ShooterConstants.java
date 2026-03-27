package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;

public class ShooterConstants {
  public static final class ComponentsConstants {
    // FlyWheel
    public static class Flywheel {
      public static final int FLYWHEEL_LEAD_CAN_ID = 20;
      public static final int FLYWHEEL_FOLLOW_CAN_ID = 21;

      public static final double FLYWHEEL_MAX_RPM = 6500;
      public static final double TOP_FLYWHEEL_RADIUS_METERS = 0.0254;
      public static final double BOTTOM_FLYWHEEL_RADIUS_METER = 0.0381;
      public static final boolean FLYWHEEL_CURRENT_LIMIT_ENABLE = true;
      public static final double FLYWHEEL_CURRENT_LIMIT = 75;
      public static final boolean FLYWHEEL_LIMIT_ENABLE = false;
      public static final double FLYWHEEL_MAIN_ROLLER_REDUCTION = 1;
      public static final double FLYWHEEL_HOOD_ROLLER_REDUCTION = 1;
      public static final InvertedValue MAIN_FLYWHEEL_INVERTED_VALUE =
          InvertedValue.Clockwise_Positive;
      public static final NeutralModeValue FLYWHEEL_NEUTRAL_MODE = NeutralModeValue.Coast;
      public static final class Gains {
        public static final double kP = 0.25;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kA = 0.0;
        public static final double kV = 0.0976759;
        public static final double kS = 0.849814;
        /**
         * Rotations per second² profile limit for {@link com.ctre.phoenix6.controls.VelocityVoltage}
         * (flywheel uses velocity closed-loop, not motion magic).
         */
        public static final double VELOCITY_ACCELERATION_RPS_PER_SEC = 250.0;
      }

      public static final double TurretMotorToMainFlyWheelReduction = 0.952941176471;
      public static final double TurretMotorToHoodFlyWheelReduction = 0.700692041522;

      /** Main shooter wheel diameter (meters). */
      public static final double MAIN_WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.0);
      /** Hood shooter wheel diameter (meters). */
      public static final double HOOD_WHEEL_DIAMETER_METERS = Units.inchesToMeters(2.0);
      /**
       * Ball geometry/properties:
       * Diameter: 5.91 in, material: high-density polyurethane foam.
       */
      public static final double BALL_DIAMETER_METERS = Units.inchesToMeters(5.91);
      public static final double BALL_MASS_MIN_KG = Units.lbsToKilograms(0.448);
      public static final double BALL_MASS_MAX_KG = Units.lbsToKilograms(0.5);
      public static final double BALL_MASS_NOMINAL_KG = 0.5 * (BALL_MASS_MIN_KG + BALL_MASS_MAX_KG);
      public static final double BALL_COMPRESSION_METERS = Units.inchesToMeters(0.625);
      /** Center distance between wheel shafts (diagnostic/geometry reference). */
      public static final double WHEEL_CENTER_DISTANCE_METERS = Units.inchesToMeters(8.0);
      /**
       * Exit transfer efficiency from wheel-surface speed to ball exit speed.
       * Derived from compression ratio as a conservative first-order estimate.
       */
      public static final double BALL_EXIT_TRANSFER_EFFICIENCY =
          MathUtil.clamp(
              1.0 - 0.7 * (BALL_COMPRESSION_METERS / BALL_DIAMETER_METERS),
              0.75,
              1.0);

    }

    // Hood
    public static class Hood {
      public static final int HOOD_CAN_ID = 22;

      public static final double MAX_DEGREE = 24;
      public static final double MIN_DEGREE = 0;

      /**
       * Mechanical hood: {@code 0°} = exit toward the sky, {@code 90°} = exit horizontal forward.
       * Commanded angle is {@code 90° − θ + offset} where θ is launch angle above horizontal (deg).
       */
      public static final double MECHANICAL_ANGLE_OFFSET_DEG = -14;
      public static final int HOOD_ENCODER_CAN_ID = 23;
      public static final double sensorToMechanismRatio = -21.1428571; // 296/14
      public static final double rotorToSensorRatio = -5.25000001; // 42/8
      public static final SensorDirectionValue hoodEncoderDirection =
          SensorDirectionValue.Clockwise_Positive;
      public static final double HoodEncoderAbsoluteSensorDiscontinuityPoint = 0.5;
      public static final double HoodEncoderMagnetOffset = -0.267333984375; // TUNE ALOT!!
      public static final double HoodCurrentLimit = 60.0;
      public static final InvertedValue HoodInvertedValue = InvertedValue.CounterClockwise_Positive;
      public static final boolean HoodSupplyCurrentLimitEnable = true;
      public static final NeutralModeValue HoodNeutralModeValue = NeutralModeValue.Brake;
      public static final class Gains {
        public static final double kP = 1500;
        public static final double kI = 100;
        public static final double kD = 75;
        public static final double kG = 0.05;
        public static final double kA = 0.5;
        public static final double kV = 0.5;
        public static final double kS = 0;
        public static final class MotionMagic {
          public static final double velocity = 0;
          public static final double acceleration = 0;
          public static final double jerk = 0;
          public static final double expo_kA = 0.1;
          public static final double expo_kV = 0.12;
        }
      }
    }

    // Turret
    public static class Turret {
      public static final int TURRET_CAN_ID = 24;
      public static final double TURRET_MAX_DEGREE = 170;
      public static final double TURRET_MIN_DEGREE = -146;

      public static final int TURRET_ENCODER_CAN_ID = 25;
      public static final double sensorToMechanismRatio = 1;
      public static final double rotorToSensorRatio = 62.5;
      public static final double TurretEncoderMagnetOffset = 0.378662109375;
      public static final SensorDirectionValue turretEncoderDirection =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final double TurretEncoderAbsoluteSensorDiscontinuityPoint = 0.5;

      public static final double TurretCurrentLimit = 65.0;
      public static final InvertedValue TurretInvertedValue = InvertedValue.Clockwise_Positive;
      public static final boolean TurretSupplyCurrentLimitEnable = true;
      public static final NeutralModeValue TurretNeutralModeValue = NeutralModeValue.Brake;
      public static final class Gains {
        public static final double kP = 450;
        public static final double kI = 20;
        public static final double kD = 40;
        public static final double kG = 0.61224;
        public static final double kA = 0.0;
        public static final double kV = 5;
        public static final double kS = 1.5;
        public static final class MotionMagic {
          public static final double velocity = 0;
          public static final double acceleration = 0;
          public static final double jerk = 0;
          public static final double expo_kA = 0.1;
          public static final double expo_kV = 0.12;
        }
      }
      /**
       * If |desiredRobotCentricDeg - commandedRobotCentricDeg| exceeds this, aim is constrained by
       * soft-limits and shooting should be suppressed.
       */
      public static final double SOFT_LIMIT_CONSTRAINT_TOLERANCE_DEG = 8.0;
    }
  }

  // General Constants
  public static final double GRAVITY = 9.81;
  /**
   * Default (initial) RPM band for flywheel {@code nearGoal} vs shooter-ready gating on {@link
   * frc.robot.RobotState#isShooterReadyToShoot()}.
   * Overridden at runtime by {@code NearGoalRpmTolerance} × {@code PhysicsLaunchEfficiencyScale}
   * (from ideal-min/empirical-map RPM while aiming).
   */
  public static final double READY_TO_SHOOT_FLYWHEEL_RPM_TOLERANCE = 450;
  public static final double READY_TO_SHOOT_HOOD_DEG_TOLERANCE = 1.0;
  /** Max |hood slew rate| (deg/s) to still count as settled. */
  public static final double READY_TO_SHOOT_HOOD_MAX_DEG_PER_SEC = 25.0;

  public static final double READY_TO_SHOOT_TURRET_DEG_TOLERANCE = 20;
  /** Max |turret slew rate| (deg/s) to still count as "settled" during slow aim tracking. */
  public static final double READY_TO_SHOOT_TURRET_MAX_DEG_PER_SEC = 35;

  /** Launch/targeting constants kept with shooter constants to avoid split files. */
  public static final class LaunchConstants {
    // TODO(PHYSICS_TUNE): tune pass/hub state switch distance for strategy + make rate.
    public static double passPoint = 4.425;

    public static final class TurretOffset {
      /** Turret pivot height above robot origin plane (meters). */
      public static final double TURRET_HEIGHT_METERS = Units.inchesToMeters(20.5);
      /** Shooter X offset in robot frame (meters, +X WPILib robot axis). */
      public static final double SHOOTER_X_OFFSET_METERS = Units.inchesToMeters(-7.75);
      /** Shooter Y offset in robot frame (meters, +Y WPILib robot axis). */
      public static final double SHOOTER_Y_OFFSET_METERS = Units.inchesToMeters(-5.75);
      public static final Translation3d TURRET_OFFSET_ROBOT =
          new Translation3d(
              SHOOTER_X_OFFSET_METERS,
              SHOOTER_Y_OFFSET_METERS,
              TURRET_HEIGHT_METERS);
      public static final Translation2d TURRET_OFFSET_ROBOT_2D =
          new Translation2d(TURRET_OFFSET_ROBOT.getX(), TURRET_OFFSET_ROBOT.getY());
      public static final Transform3d ROBOT_TO_TURRET =
          new Transform3d(TURRET_OFFSET_ROBOT, Rotation3d.kZero);
    }

    public static final class PassTargets {
      public static final Translation3d LEFT = new Translation3d(3.67, 6.0, 0.0);
      public static final Translation3d RIGHT = new Translation3d(3.67, 2.043, 0.0);
    }

    public static final class Trench {
      /** Shrinks trench fold-protection zone so shooting can occur closer to trench edges. */
      public static final double PROTECTION_MARGIN_METERS = 0.2;
      /**
       * Predictive horizon for trench protection. If the lookahead X position enters trench and hood
       * is not yet folded, shooter will suppress fire and fold.
       */
      public static final double LOOKAHEAD_TIME_SEC = 0.35;
      /** Hood angle threshold considered folded/safe for trench crossing. */
      public static final double SAFE_HOOD_ANGLE_DEG = 0;
      /** X start for alliance trench overhang protection (meters, blue-frame). */
      public static final double START_X_METERS =
          FieldConstants.LinesVertical.hubCenter
              - Math.max(FieldConstants.LeftTrench.depth, FieldConstants.RightTrench.depth);
      /** X end for alliance trench overhang protection (meters, blue-frame). */
      public static final double END_X_METERS = FieldConstants.LinesVertical.hubCenter;
      /** X start for opponent trench overhang protection (meters, blue-frame). */
      public static final double OPP_START_X_METERS =
          FieldConstants.LinesVertical.oppHubCenter
              - Math.max(FieldConstants.LeftTrench.depth, FieldConstants.RightTrench.depth);
      /** X end for opponent trench overhang protection (meters, blue-frame). */
      public static final double OPP_END_X_METERS = FieldConstants.LinesVertical.oppHubCenter;
      /** Left trench opening lower Y bound (meters, blue-frame). */
      public static final double LEFT_MIN_Y_METERS =
          FieldConstants.fieldWidth - FieldConstants.LeftTrench.openingWidth;
      /** Right trench opening upper Y bound (meters, blue-frame). */
      public static final double RIGHT_MAX_Y_METERS = FieldConstants.RightTrench.openingWidth;
    }
  }

  public static final class Logging {
    public static boolean SHOOTER_VERBOSE_AIMING = false;
    public static boolean SHOOTER_VERBOSE_TRENCH = false;
    /** When false, skips AdvantageKit outputs on the launch calculator hot path (hood-comp channels). */
    public static boolean LOG_LAUNCH_CALC_HOOD_COMP = false;
    /** When false, skips per-cycle Launcher/* dashboard fields (ShootMode, state string, trench active). */
    public static boolean LOG_LAUNCH_COORD_EVERY_CYCLE = false;
  }

  /** Hybrid hood compensation settings for measured flywheel RPM sag. */
  public static final class HoodCompensationConstants {
    /** Normal-mode blend on measured-theta correction (0..1). */
    // TODO(PHYSICS_TUNE): tune to minimize noise while preserving correction authority.
    public static final double NORMAL_GAIN = 0.25;
    /** Max absolute hood correction from RPM sag, in degrees. */
    // TODO(PHYSICS_TUNE): widen/narrow after observing hood saturation frequency.
    public static final double MAX_CORRECTION_DEG = 3.0;
    /** Max hood correction slew from compensation, in deg/s. */
    // TODO(PHYSICS_TUNE): adjust to avoid chatter yet still catch RPM sag quickly.
    public static final double MAX_CORRECTION_RATE_DEG_PER_SEC = 60.0;
    /** RPM low-pass time constant for normal mode smoothing. */
    // TODO(PHYSICS_TUNE): tune for stability vs responsiveness in normal mode.
    public static final double RPM_FILTER_TIME_CONSTANT_SEC = 0.12;
  }

  /**
   * Numerical thresholds, iteration limits, and solver defaults for {@link
   * frc.robot.subsystems.shooter.LaunchCalculator}.
   */
  public static final class LaunchCalculatorConstants {
    /** Moving-average window (s) for turret/hood angular velocity from filtered angle deltas. */
    public static final double ANGLE_VELOCITY_FILTER_WINDOW_SEC = 0.1;

    /**
     * Extra scale on vacuum minimum exit surface speed before RPM mapping (make-rate margin vs
     * model).
     */
    // TODO(PHYSICS_TUNE): tune launch margin to balance make-rate vs overspeed.
    public static final double MIN_SPEED_MARGIN = 1.08;
    /**
     * Fraction of no-load launch speed preserved at ball exit (0..1). Used to compensate initial
     * velocity drop by scaling required launch speed as {@code required / efficiency}.
     */
    public static final double INITIAL_SPEED_EFFICIENCY = 0.92;

    /**
     * Ignore measured-RPM hood correction below this wheel surface speed (m/s); avoids garbage
     * angles at very low RPM.
     */
    // TODO(PHYSICS_TUNE): set threshold where measured-RPM angle correction is trustworthy.
    public static final double MIN_VALID_SHOT_SPEED_MPS = 0.5;

    public static final double EPSILON_SURFACE_SPEED_MPS = 1e-6;
    /**
     * Near-zero threshold for horizontal range, height tolerances, and similar geometry checks (m).
     */
    public static final double EPSILON_METERS = 1e-6;
    public static final double EPSILON_DENOMINATOR = 1e-9;
    /** Used for timer / ratio comparisons where exact zero is ambiguous. */
    public static final double EPSILON_TIME_AND_RATIO = 1e-6;

    /** Fallback launch-angle limits (deg) if hood mechanical limits are non-finite or inverted. */
    public static final double FALLBACK_THETA_MIN_DEG = 5.0;
    public static final double FALLBACK_THETA_MAX_DEG = 85.0;

    /** Default launch angle above horizontal (deg) when the discriminant solve fails. */
    public static final double DEFAULT_SOLVE_THETA_DEG = 45.0;

    /**
     * Iterations for moving-target lead (successive ballistic refinement). Fewer iterations reduce
     * worst-case latency while aiming on the move; increase if lookahead snaps noticeably short on
     * fast crosses.
     */
    public static final int MOVING_TARGET_LEAD_ITERATIONS = 5;

    /** Attempts to raise wheel RPM when no real θ exists at the current speed command. */
    public static final int BALLISTIC_RPM_BOOST_MAX_ITERATIONS = 8;

    /**
     * Per-iteration RPM multiplier when boosting speed to find a feasible arc (unitless).
     *
     * <p>TODO(PHYSICS_TUNE): treat as coarse search step; tie to {@link #MIN_SPEED_MARGIN} if you
     * want a fixed relative step.
     */
    public static final double BALLISTIC_RPM_BOOST_FACTOR = 1.07;

    /** Upper clamp for measured/command RPM ratio (diagnostics + efficiency scale). */
    public static final double MEASURED_TO_COMMAND_RPM_RATIO_MAX = 2.0;

    /** Output bounds for {@link LaunchCalculator#getPhysicsLaunchEfficiencyScale()}. */
    public static final double PHYSICS_LAUNCH_EFFICIENCY_SCALE_OUT_MIN = 0.9;
    public static final double PHYSICS_LAUNCH_EFFICIENCY_SCALE_OUT_MAX = 1.35;

    /**
     * Reference angle (deg): mechanical hood uses {@code 90° − θ + offset} vs launch elevation θ.
     */
    public static final double MECHANICAL_RIGHT_ANGLE_DEG = 90.0;

    /** Equal blend between main and hood wheel contributions to effective surface speed. */
    public static final double DUAL_WHEEL_SURFACE_BLEND = 0.5;

    /** RPM ⟷ RPS conversion factor. */
    public static final double SECONDS_PER_MINUTE = 60.0;

    /** Unitless ratio placeholder before first solve or when RPM command is degenerate. */
    public static final double NEUTRAL_UNIT_RATIO = 1.0;
  }

  /** Defaults for flywheel idle goals and launch-efficiency scaling. */
  public static final class FlywheelShotConstants {
    public static final double FLYWHEEL_GOAL_AUTOIDLE_RPM = 3000.0;
    public static final double FLYWHEEL_GOAL_TELEIDLE_RPM = 2500.0;
    /** Default custom goal - TODO(PHYSICS_TUNE). */
    public static final double FLYWHEEL_GOAL_CUSTOM_RPM = 2300.0;

    /**
     * Clamp on {@link frc.robot.subsystems.shooter.flywheel.Flywheel#setPhysicsLaunchEfficiencyScale}
     * inputs.
     */
    public static final double PHYSICS_LAUNCH_EFFICIENCY_INPUT_MIN = 0.5;
    public static final double PHYSICS_LAUNCH_EFFICIENCY_INPUT_MAX = 2.5;

    /** Unit scale when launch physics efficiency is not applied (not aiming). */
    public static final double PHYSICS_LAUNCH_EFFICIENCY_SCALE_NEUTRAL = 1.0;
  }
}
