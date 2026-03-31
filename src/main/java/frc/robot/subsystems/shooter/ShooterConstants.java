package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

      public static final double FLYWHEEL_MAX_RPM = 7000;
      public static final boolean FLYWHEEL_CURRENT_LIMIT_ENABLE = true;
      public static final double FLYWHEEL_CURRENT_LIMIT = 75;
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
         * Rotations per second^2 profile limit for {@link com.ctre.phoenix6.controls.VelocityVoltage}
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

      public static final double AVERAGE_WHEEL_RADIUS_METERS =
          ((MAIN_WHEEL_DIAMETER_METERS*0.5) + (HOOD_WHEEL_DIAMETER_METERS*0.5)) * 0.5;
      /**
       * Ball geometry/properties:
       * Diameter: 150mm, material: high-density polyurethane foam.
       */
      public static final double BALL_DIAMETER_METERS = 0.15;
      public static final double BALL_COMPRESSION_METERS = Units.inchesToMeters(0.625);
      /**
       * Exit transfer efficiency from wheel-surface speed to ball exit speed.
       * Derived from compression ratio as a conservative first-order estimate.
       */

      public static final class ShotConstants {
        public static final double BALL_EXIT_TRANSFER_EFFICIENCY = 0.8;
        /**
         * Shared flywheel speed when not shooting (auto and tele use the same idle for consistent
         * warm-up).
         */
        public static final double FLYWHEEL_GOAL_IDLE_RPM = 2500.0;
      }
    }

    // Hood
    public static class Hood {
      public static final int HOOD_CAN_ID = 22;

      public static final double MAX_DEGREE = 24;
      public static final double MIN_DEGREE = 0;

      /**
       * Mechanical hood angle {@code m} (what the Talon commands, deg) vs shot elevation {@code theta}
       * above horizontal (deg, ballistics). Positive constant {@code k} matches the field convention
       * "add {@code k} to motor reading for slope-from-vertical" and "subtract {@code k} when going from
       * right-angle reference to motor degrees":
       *
       * <p>{@code slopeFromVertical_deg = m + k} (at {@code m = 0}, slope is {@code k} deg from vertical).
       *
       * <p>{@code theta = 90 deg - m - k} and {@code m = 90 deg - theta - k}.
       *
       * <p>Example: {@code k = 14} gives {@code theta = 76 deg} when {@code m = 0}.
       *
       * <p>Convert to/from physics for solves and commands only through {@link
       * frc.robot.subsystems.shooter.ShooterCalculator#physicsThetaRadFromMechanicalHoodDeg} and {@link
       * frc.robot.subsystems.shooter.ShooterCalculator#mechanicalHoodAngleRadFromPhysicsTheta} so limits
       * and ballistics stay consistent.
       */
      public static final double MECHANICAL_ANGLE_OFFSET_DEG = 14;

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
      // Control Gains
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

      public static final class shootingConstants {
        /** Max hood angle (deg) for shooting; if hood is above this, shooting is suppressed. */
        public static final double PASS_ANGLE_DEG = 24.0;
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
  public static final double GRAVITY = 9.80665;
  /** Base half-width RPM window for flywheel {@code nearGoal}. */
  public static final double READY_TO_SHOOT_FLYWHEEL_RPM_TOLERANCE = 450;
  
  public static final double READY_TO_SHOOT_HOOD_DEG_TOLERANCE = 2.0;
  /** Max |hood slew rate| (deg/s) to still count as settled. */
  public static final double READY_TO_SHOOT_HOOD_MAX_DEG_PER_SEC = 50.0;

  public static final double READY_TO_SHOOT_TURRET_DEG_TOLERANCE = 20;
  /** Max |turret slew rate| (deg/s) to still count as "settled" during slow aim tracking. */
  public static final double READY_TO_SHOOT_TURRET_MAX_DEG_PER_SEC = 80;

  /** Field aim geometry (turret offset, pass targets, trench protection). */
  public static final class ShooterAimConstants {
    // TODO(PHYSICS_TUNE): tune pass/hub state switch distance for strategy + make rate.
    public static double passPoint = 4.425;

    /** Field Z (m) at or below this uses geometric height only in the hood arc solve (no extra rise). */
    public static final double PASS_TARGET_Z_METERS = 0.0;

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
      /**
       * Hood angle threshold (mechanical deg, Talon setpoint frame) for folded/safe trench crossing.
       * Not physics theta; compare to {@link ComponentsConstants.Hood#MIN_DEGREE} / measured position.
       */
      public static final double SAFE_HOOD_ANGLE_DEG = ShooterConstants.ComponentsConstants.Hood.MIN_DEGREE;
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
    public static boolean SHOOTER_VERBOSE_AIMING = true;
    public static boolean SHOOTER_VERBOSE_TRENCH = true;
    /**
     * When false (and {@link #SHOOTER_VERBOSE_AIMING} is false), skips calculator trajectory/hood
     * diagnostic channels in {@link frc.robot.subsystems.shooter.ShooterCalculator}.
     */
    public static boolean LOG_SHOOTER_CALC_HOOD_COMP = true;
    /** When false, skips verbose per-loop Shooter dashboard logging (shoot mode, trench, etc.). */
    public static boolean LOG_SHOOTER_COORD_EVERY_CYCLE = true;
  }

  /**
   * Ballistic solver tuning for {@link frc.robot.subsystems.shooter.ShooterCalculator}.
   *
   * <p><b>WPILib note:</b> {@link Units} handles length/angle conversions. There is no WPILib constant
   * for "RPM to rotations per second"; Talon velocity is in RPS, so code uses {@code rpm / 60.0}
   * inline. The {@code EPSILON_*} values are tiny thresholds so divide-by-zero and degenerate
   * geometry do not explode floating-point math.
   *
   * <ul>
   *   <li>{@link #EPSILON_SURFACE_SPEED_MPS}, {@link #EPSILON_METERS} - Near-zero thresholds for speeds (m/s) and distances (m).
   *   <li>{@link #EPSILON_DENOMINATOR} - Floor when dividing by reductions or geometry.
   *   <li>{@link #EPSILON_TIME_AND_RATIO} - Small threshold for time/ratio comparisons.
   *   <li>{@link #MOVING_TARGET_LEAD_ITERATIONS} - Moving-target lead refinement iterations.
   *   <li>{@link #MECHANICAL_RIGHT_ANGLE_DEG} - Relates mechanical hood angle to shot elevation theta.
   *   <li>{@link #DUAL_WHEEL_SURFACE_BLEND} - Blend weight for dual-wheel surface speed (0..1 each).
   * </ul>
   */
  public static final class ShooterCalculatorConstants {
    /** Moving-average window (s) for turret/hood angular velocity from filtered angle deltas. */
    public static final double ANGLE_VELOCITY_FILTER_WINDOW_SEC = 0.1;

    public static final double EPSILON_SURFACE_SPEED_MPS = 1e-6;
    
    /**
     * Near-zero threshold for horizontal range, height tolerances, and similar geometry checks (m).
     */
    public static final double EPSILON_METERS = 1e-6;
    public static final double EPSILON_DENOMINATOR = 1e-9;
    /** Used for timer / ratio comparisons where exact zero is ambiguous. */
    public static final double EPSILON_TIME_AND_RATIO = 1e-6;

    /** Fallback shot-angle limits (deg) if hood mechanical limits are non-finite or inverted. */
    public static final double FALLBACK_THETA_MIN_DEG = 5.0;
    public static final double FALLBACK_THETA_MAX_DEG = 85.0;

    /**
     * Applied after {@link frc.robot.subsystems.shooter.ShooterCalculator#minimumExitVelocity} when
     * solving SHOOT/HUB trajectories (not PASS fixed-angle). Example: 0.10 = +10% speed headroom.
     */
    public static final double MIN_EXIT_VELOCITY_HEADROOM_RATIO = 0.30;

    /**
     * Iterations for moving-target lead (successive ballistic refinement). Match {@code physics-way}
     * fixed-point lead (20); drag theta solve is heavier than vacuum discriminant.
     */
    public static final int MOVING_TARGET_LEAD_ITERATIONS = 8;

    /**
     * Reference angle (deg): with hood offset {@code k} (positive), {@code theta = 90 - m - k}.
     */
    public static final double MECHANICAL_RIGHT_ANGLE_DEG = 90.0;
  }
}
