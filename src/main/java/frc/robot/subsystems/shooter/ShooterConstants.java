package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  public static final class ComponentsConstants {
    // FlyWheel
    public static class Flywheel {
      public static final int FLYWHEEL_LEAD_CAN_ID = 20;
      public static final int FLYWHEEL_FOLLOW_CAN_ID = 21;

      public static final double FLYWHEEL_MAX_RPM = 6500;
      public static final boolean FLYWHEEL_CURRENT_LIMIT_ENABLE = true;
      public static final double FLYWHEEL_CURRENT_LIMIT = 60;
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
      public static final double BALL_COMPRESSION_METERS = Units.inchesToMeters(0.62);
      /**
       * Exit transfer efficiency from wheel-surface speed to ball exit speed.
       * Derived from compression ratio as a conservative first-order estimate.
       */

      public static final class ShotConstants {
        /**
         * Wheel-to-ball exit speed ratio. Slightly below 1 models slip/compliance so the solver asks for
         * more wheel RPM (helps far hub line when hood is at mechanical maximum flat). Tune with
         * logs vs measured exit if needed.
         */
        public static final double BALL_EXIT_TRANSFER_EFFICIENCY = 0.98;

        /**
         * Added to the calculator flywheel RPM setpoint (hub/pass/3D tracking) after the ballistic solve.
         * Positive bias for long-field shots when angle is hood-limited (e.g. mechanical max ~23 deg).
         * Live: {@code TunableNumbers/Shooter/Physics/FlywheelCommandRpmOffset}.
         */
        public static final double FLYWHEEL_COMMAND_RPM_OFFSET = 10.0;

        /**
         * Shared flywheel speed when not shooting (auto and tele use the same idle for consistent
         * warm-up).
         */
        public static final double FLYWHEEL_GOAL_IDLE_RPM = 2000.0;
      }
    }

    // Hood
    public static class Hood {
      public static final int HOOD_CAN_ID = 22;

      public static final double MAX_DEGREE = 23.75;
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
      public static final double HoodEncoderMagnetOffset = 0.305908203125; // TUNE ALOT!!
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
    }

    // Turret
    public static class Turret {
      public static final int TURRET_CAN_ID = 24;
      public static final double TURRET_MAX_DEGREE = 180;
      public static final double TURRET_MIN_DEGREE = -155;

      public static final int TURRET_ENCODER_CAN_ID = 25;
      public static final double sensorToMechanismRatio = 1;
      public static final double rotorToSensorRatio = 62.5;
      public static final double TurretEncoderMagnetOffset = 0.401611328125; //Tune Alot
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
      public static final double SOFT_LIMIT_CONSTRAINT_TOLERANCE_DEG = 2.0;
    }
  }

  // General Constants
  public static final double GRAVITY = 9.80665;
  /** Base half-width RPM window for flywheel {@code nearGoal}. */
  public static final double READY_TO_SHOOT_FLYWHEEL_RPM_TOLERANCE = 300;
  
  public static final double READY_TO_SHOOT_HOOD_DEG_TOLERANCE = 5;
  /** Max |hood slew rate| (deg/s) to still count as settled. */
  public static final double READY_TO_SHOOT_HOOD_MAX_DEG_PER_SEC = 30.0;

  public static final double READY_TO_SHOOT_TURRET_DEG_TOLERANCE = 25;
  /** Max |turret slew rate| (deg/s) to still count as "settled" during slow aim tracking. */
  public static final double READY_TO_SHOOT_TURRET_MAX_DEG_PER_SEC = 50;

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

    /**
     * Hub shoot aim only. {@link frc.robot.subsystems.shooter.ShooterCalculator} uses a lob arc when the
     * hub is close to the lookahead shooter pose and retries with HIGH if the default LOW solve is
     * invalid.
     */
    public static final class Hub {
      /**
       * Prefer {@link frc.robot.subsystems.shooter.ShooterState.ShootingArc#HIGH} for {@link
       * frc.robot.subsystems.shooter.ShooterState.ShooterMode#HUB} when hub-to-lookahead shooter horizontal
       * distance is at most this value (meters). Lookahead uses {@code Shooter/Physics/HorizontalLookaheadTimeSec}
       * with current shooter XY velocity.
       */
      public static final double HIGH_ARC_LOOKAHEAD_DISTANCE_METERS = 2.5;
    }

    public static final class Trench {
      /**
       * Expands trench protection strips in X and Y (m). See ShooterCalculator {@code isUnderTrenchOverhang}
       * and {@link frc.robot.FieldConstants} hub and trench openings; only margin lives here for tuning.
       */
      public static final double PROTECTION_MARGIN_METERS = 0.0375;
      /**
       * Constant-velocity horizon for {@code shooterPose + v t} (field frame): trench pre-checks and
       * logged look-ahead pose. Larger values predict farther along current translation; tune live via
       * {@code TunableNumbers/Shooter/Physics/HorizontalLookaheadTimeSec}.
       */
      public static final double LOOKAHEAD_TIME_SEC = 1.35;
      /**
       * Folded hood (mechanical deg) commanded while {@link frc.robot.subsystems.shooter.ShooterState}
       * trench protection is active. Matches {@link ComponentsConstants.Hood#MIN_DEGREE}.
       */
      public static final double SAFE_HOOD_ANGLE_DEG = ShooterConstants.ComponentsConstants.Hood.MIN_DEGREE;
    }
  }

  public static final class Logging {
    /**
     * Rich calculator aim/geometry logs. Default off to reduce loop time; enable on bench or when tuning
     * in AdvantageScope.
     */
    public static boolean SHOOTER_VERBOSE_AIMING = true;
    /** Trench zone diagnostic channels in {@link frc.robot.subsystems.shooter.ShooterCalculator}. */
    public static boolean SHOOTER_VERBOSE_TRENCH = true;
    /**
     * When true (or {@link #SHOOTER_VERBOSE_AIMING}), logs calculator trajectory/hood diagnostics. Default
     * off for match/real-time cycles.
     */
    public static boolean LOG_SHOOTER_CALC_HOOD_COMP = true;
    /**
     * Compact {@code Shooter/Calculator/Sanity/} bundle. Default off; turn on for quick field checks.
     */
    public static boolean LOG_SHOOTER_SANITY_BUNDLE = true;
    /**
     * Extra {@link org.littletonrobotics.junction.Logger#recordOutput} calls in Flywheel, Hood, and Turret
     * {@code periodic()} (does not affect {@code Logger.processInputs}). Default off to save cycle time;
     * logs still ingest normally when replaying if inputs were logged.
     */
    public static boolean LOG_SHOOTER_MECHANISM_TELEM = true;
  }

  /**
   * Fuel (game piece) and atmosphere constants for drag/Magnus projectile models.
   *
   * <p>Defaults below are a <b>conservative baseline</b> after hub/target frame fixes: moderate drag
   * (sphere-like foam), modest Magnus, and bounded spin geometry so NT tunables start near physical
   * scale. {@link ShooterPhysicsTunables} mirrors these; change keys under {@code Shooter/Physics/} on
   * the robot without redeploy.
   */
  public static final class ProjectileConstants {
    /** Fuel ball mass in kilograms (team-measured nominal). */
    public static final double FUEL_MASS_KG = 0.215;
    /** Fuel ball diameter in meters (nominal). */
    public static final double FUEL_DIAMETER_METERS = ComponentsConstants.Flywheel.BALL_DIAMETER_METERS;
    /** Fuel ball radius in meters. */
    public static final double FUEL_RADIUS_METERS = FUEL_DIAMETER_METERS * 0.5;
    /** Cross-sectional area in square meters for quadratic drag. */
    public static final double FUEL_CROSS_SECTIONAL_AREA_M2 =
        Math.PI * FUEL_RADIUS_METERS * FUEL_RADIUS_METERS;

    /** Air density (kg/m^3) near sea level, room-temperature baseline. */
    public static final double AIR_DENSITY_KG_PER_M3 = 1.12;

    /**
     * Quadratic-drag Cd (lumped). Values ~3+ force the solver to over-command speed vs a typical foam
     * ball and read as close-range overshoot; ~0.5–0.65 matches sphere-like foam for a better baseline.
     */
    public static final double DRAG_COEFFICIENT_SMOOTH_SPHERE = 1.12;

    /**
     * Magnus lift coefficient {@code Cl} (unitless, lumped backspin). Runtime: {@link
     * ShooterPhysicsTunables} {@code MagnusLiftCoefficientCl}. Tune after Cd: increase if shots land
     * low while range already matches; typical final band often ~0.03–0.15 for FRC-scale speeds.
     */
    public static final double MAGNUS_LIFT_COEFFICIENT = 0.15;

    /** Drag acceleration factor (1/m): 0.5 * rho * Cd * A / m. */
    public static final double DRAG_ACCEL_FACTOR_PER_M =
        0.5
            * AIR_DENSITY_KG_PER_M3
            * DRAG_COEFFICIENT_SMOOTH_SPHERE
            * FUEL_CROSS_SECTIONAL_AREA_M2
            / FUEL_MASS_KG;

    /** Magnus acceleration factor (1/m): 0.5 * rho * Cl * A / m. */
    public static final double MAGNUS_ACCEL_FACTOR_PER_M =
        0.5 * AIR_DENSITY_KG_PER_M3 * MAGNUS_LIFT_COEFFICIENT * FUEL_CROSS_SECTIONAL_AREA_M2 / FUEL_MASS_KG;

    /** Slip-ratio fallback for spin estimate: omega ~= (v / r) * slipRatio. */
    public static final double SPIN_SLIP_RATIO = 0.8;

    /**
     * Clamps on geometry-based spin scale (compressed vs free diameter). Keeps ω within ~±50% of v/r
     * band before wheel-delta gain; widen max if you need more Magnus coupling.
     */
    //0.894166634
    public static final double SPIN_GEOMETRY_SCALE_MIN = 0.1;

    public static final double SPIN_GEOMETRY_SCALE_MAX = 2.0;

    /**
     * rad·s⁻¹ per (m/s) hood−main surface mismatch; 0 disables the delta path. Default {@code ~1/r}
     * maps a relative surface speed to ball ω like rolling contact (~127 RPM per m/s of mismatch),
     * which lands in the team-measured ~500–800 RPM band for typical hood−main differentials (a few
     * m/s) when combined with geometry-scale ω; high launch speeds still hit the backspin clamp.
     */
    public static final double WHEEL_DELTA_SPIN_GAIN = 1.0 / FUEL_RADIUS_METERS;

    /**
     * Team-measured ball backspin band (RPM) for Magnus. The hybrid estimate from geometry and wheel
     * delta is clamped to {@code [min, max]} rad/s so the model does not use {@code v/r}-scale ω, which
     * is far above typical foam-ball backspin.
     */
    public static final double BALL_BACKSPIN_MEASURED_MIN_RPM = 500.0;
    public static final double BALL_BACKSPIN_MEASURED_MAX_RPM = 800.0;
  }

  /**
   * Ballistic solver tuning for {@link frc.robot.subsystems.shooter.ShooterCalculator}.
   *
   * <p><b>Live tuning:</b> physics-related doubles used by the drag solver are overridden by {@link
   * ShooterPhysicsTunables} (see keys under {@code TunableNumbers/Shooter/Physics/}).
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
   *   <li>{@link #MOVING_TARGET_LEAD_ITERATIONS} - Moving-target lead fixed-point iterations.
   *   <li>{@link #MOVING_TARGET_LEAD_TOF_SCALE}, {@link #MOVING_TARGET_LEAD_TOF_CAP_SEC} - Scale and cap
   *       on ballistic TOF when offsetting shooter pose for aim (reduces over-lead from large TOF).
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
    public static final double FALLBACK_THETA_MIN_DEG = 0.0;
    public static final double FALLBACK_THETA_MAX_DEG = 51.0;

    /**
     * Applied after {@link frc.robot.subsystems.shooter.ShooterCalculator#minimumExitVelocity} when
     * solving hub, pass, and 3D tracking trajectories. Example: 0.10 = +10% speed headroom.
     */
    /** Fraction above vacuum minimum exit speed for shoot/hub solves (e.g. 0.06 = +6%). */
    public static final double MIN_EXIT_VELOCITY_HEADROOM_RATIO = 0.05;

    /**
     * Moving-target lead fixed-point iterations. Each step runs a full shoot solve; default 3 trades a
     * little lead accuracy for cycle time. Tune via NT key
     * {@code TunableNumbers/Shooter/Physics/MovingTargetLeadIterations} or raise this constant if lead
     * lags when driving.
     */
    public static final int MOVING_TARGET_LEAD_ITERATIONS = 5;

    /**
     * Multiplier on ballistic time-of-flight when applying field-velocity lead ({@code v * tof}). Use
     * {@code < 1} if lead feels too aggressive; {@code 0} disables displacement lead (same as no robot
     * motion).
     */
    /** Slightly conservative lead vs raw ballistic TOF when driving at the hub. */
    public static final double MOVING_TARGET_LEAD_TOF_SCALE = 1.15;

    /**
     * Max seconds of displacement used for moving-target lead per iteration. A cap that is too low trims
     * {@code vRobot * dt} vs true ballistic TOF and reads as systematic miss (~1m class) when driving.
     * Set {@code <= 0} in the tunable to disable the cap (not recommended in match play).
     */
    public static final double MOVING_TARGET_LEAD_TOF_CAP_SEC = 1.5;

    /**
     * Trajectory simulation integration step (s). Coarser than 10 ms reduces CPU per shot solve; tighten
     * via {@code TunableNumbers/Shooter/Physics/TrajectorySimDtSec} for tuning.
     */
    public static final double TRAJECTORY_SIM_DT_SEC = 0.015;
    /** Hard cap on ballistic simulation horizon (seconds). */
    public static final double TRAJECTORY_SIM_MAX_TIME_SEC = 2.0;
    /** Max bisection iterations for numeric speed/angle solves. */
    public static final int TRAJECTORY_BISECTION_MAX_ITERS = 20;
    /** Theta scan samples used before selecting and refining low/high-arc candidate. */
    public static final int TRAJECTORY_THETA_SCAN_SAMPLES = 23;
   
    /** Max launch speed considered by numeric speed solves (m/s). */
    /** FRC-scale cap (~25–30 m/s); raise only if the model truly needs faster trial speeds. */
    public static final double TRAJECTORY_MAX_LAUNCH_SPEED_MPS = 200.0;
    
    /**
     * Vacuum minimum ball speed can be too low for the drag integrator to reach {@code d} before {@code v_x}
     * dies, which yields no launch-angle root and invalid shooter output. Each step multiplies trial ball speed by
     * this ratio until {@link frc.robot.subsystems.shooter.ShooterCalculator#ballisticThetaAboveHorizontalRad}
     * returns finite angle or {@link #TRAJECTORY_MAX_LAUNCH_SPEED_MPS} is hit.
     */
    public static final double DRAG_SHOOT_BALL_SPEED_BUMP_RATIO = 1.06;

    /** Max bump steps for shoot-mode drag angle solve (guards loop). */
    public static final int DRAG_SHOOT_BALL_SPEED_BUMP_MAX_STEPS = 28;

    /**
     * Reference angle (deg): with hood offset {@code k} (positive), {@code theta = 90 - m - k}.
     */
    public static final double MECHANICAL_RIGHT_ANGLE_DEG = 90.0;
  }
}
