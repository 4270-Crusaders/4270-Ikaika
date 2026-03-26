package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  // FlyWheel
  public static final class FlywheelConstants {
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
    public static double FlyWheelkP = 0.25;
    public static double FlyWheelkI = 0.0;
    public static double FlyWheelkD = 0.0;
    public static double FlyWheelkA = 0.0;
    public static double FlyWheelkV = 0.0976759;
    public static double FlyWheelkS = 0.849814;
    public static double FlyWheelMotionMagicVelocity = 0;
    public static double FlyWheelMotionMagicAcceleration = 250;
    public static double FlyWheelMotionMagicJerk = 0;

    
    // public static double TurretMotorToMainFlyWheelReduction = 0.58666666666; //0.952941176471;
    // public static double TurretMotorToHoodFlyWheelReduction = 0.431372549; //0.700692041522;  
    public static double TurretMotorToMainFlyWheelReduction = 0.952941176471;
    public static double TurretMotorToHoodFlyWheelReduction = 0.700692041522;  

    // 2,912 rpm at 153in or 3.8862m

    

    // diff old -> new 0.61563786007

    // 12->18  22->30  18->15 (new) 0.58666666666
    // 34->30  18->15

    // 12->18  34->18  15->18 (old)

    public static final double FLYWHEEL_GEAR_RATIO = 0.0; //TODO -> Change when mech change on bot
  }

  // Hood
  public static final class HoodConstants {
    public static final int HOOD_CAN_ID = 22;

    public static final double MAX_DEGREE = 24;

    /**
     * Extra vertical rise (m) in the ballistic hood solve only (above geometric target height).
     * Was 2.5 ft; reduced by ~1 ft for a lower arc (~1.5 ft margin).
     */
    public static final double BALLISTIC_EXTRA_HEIGHT_METERS = Units.feetToMeters(1.5);

    /**
     * Mechanical hood: {@code 0°} = exit toward the sky, {@code 90°} = exit horizontal forward.
     * Commanded angle is {@code 90° − θ + offset} where θ is launch angle above horizontal (deg).
     */
    public static final double MECHANICAL_ANGLE_OFFSET_DEG = -14;
    public static final int HOOD_ENCODER_CAN_ID = 23;
    public static double HoodSensorToMechanismRatio = -21.1428571; // 296/14
    public static double HoodRotorToSensorRatio = -5.25000001; // 42/8
    public static SensorDirectionValue hoodEncoderDirection =
        SensorDirectionValue.Clockwise_Positive;
    public static double HoodEncoderAbsoluteSensorDiscontinuityPoint = 0.5;
    public static double HoodEncoderMagnetOffset = -0.267333984375; // TUNE ALOT!!
    public static double HoodCurrentLimit = 60.0;
    public static InvertedValue HoodInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static boolean HoodSupplyCurrentLimitEnable = true;
    public static NeutralModeValue HoodNeutralModeValue = NeutralModeValue.Brake;
    public static double HoodMotionMagicVelocity = 0;
    public static double HoodMotionMagicAcceleration = 0;
    public static double HoodMotionMagicJerk = 0;
    public static double HoodMotionMagickA = 0.1;
    public static double HoodMotionMagickV = 0.12;
    public static double HoodkP = 1500;
    public static double HoodkI = 100;
    public static double HoodkD = 75;
    public static double HoodkG = 0.05;
    public static double HoodkA = 0.5;
    public static double HoodkV = 0.5;
    public static double HoodkS = 0;
  }

  // Turret
  public static final class TurretConstants {
    public static final int TURRET_CAN_ID = 24;
    public static final double TURRET_MAX_DEGREE = 170;
    public static final double TURRET_MIN_DEGREE = -146;

    public static final int TURRET_ENCODER_CAN_ID = 25;
    public static double TurretSensorToMechanismRatio = 1;
    public static double TurretRotorToSensorRatio = 62.5;
    public static double TurretEncoderMagnetOffset = 0.378662109375;
    public static SensorDirectionValue turretEncoderDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    public static double TurretEncoderAbsoluteSensorDiscontinuityPoint = 0.5;

    public static double TurretCurrentLimit = 65.0;
    public static InvertedValue TurretInvertedValue = InvertedValue.Clockwise_Positive;
    public static boolean TurretSupplyCurrentLimitEnable = true;
    public static NeutralModeValue TurretNeutralModeValue = NeutralModeValue.Brake;
    public static double TurretMotionMagicVelocity = 0;
    public static double TurretMotionMagicAcceleration = 0;
    public static double TurretMotionMagicJerk = 0;
    public static double TurretMotionMagickA = 0.1;
    public static double TurretMotionMagickV = 0.12;
    public static double TurretkP = 450;
    public static double TurretkI = 20;
    public static double TurretkD = 40;
    public static double TurretkG = 0.61224;
    public static double TurretkA = 0.0;
    public static double TurretkV = 5;
    public static double TurretkS = 1.5;
  }

  // General Constants
  public static final double GRAVITY = 9.81;
  public static final double TURRET_HEIGHT = 0.4826; // inches multiplied by meter conversion
  public static final double GOAL_HEIGHT = 1.8288; // Same as above
  public static final double DELTA_HEIGHT = GOAL_HEIGHT - TURRET_HEIGHT;

  /**
   * Approximate radius of the ball/game piece used by the aerodynamic model (meters).
   *
   * <p>Tuned for the drag/lift approximation in {@link frc.robot.subsystems.shooter.LaunchCalculator}.
   */
  public static final double BALL_RADIUS_METERS = Units.inchesToMeters(1.375); // ~2.75" diameter

  /**
   * Linear air drag coefficient (1/s) for a simplified drag model used during flight.
   *
   * <p>Quadratic drag is more realistic but more expensive to solve; this linear model is tuned
   * to match observed velocity/arc loss.
   */
  public static final double AIR_DRAG_LINEAR_COEFF_1_PER_S = 0.35;

  /**
   * Backspin Magnus/lift coefficient for the simplified model.
   *
   * <p>Used as: {@code a_lift = MAGNUS_COEFF * omega_ball * v}. Tune so the ball "bites" / arc
   * feels right.
   */
  public static final double BACKSPIN_MAGNUS_LIFT_COEFF = 0.00027;

  /** Spin rate ratio between ball angular speed and wheel surface speed (unitless). */
  public static final double BACKSPIN_SPIN_RATE_RATIO = 1.0;

  /**
   * Extra scale applied to the vacuum minimum-exit velocity to account for losses not fully
   * captured by the simplified flight model.
   */
  public static final double AIR_DRAG_EXIT_VELOCITY_MULTIPLIER = 1.08;

  /**
   * Default (initial) RPM band for flywheel {@code nearGoal} / {@link Shooter#readyToShoot()}.
   * Overridden at runtime by {@code NearGoalRpmTolerance} × {@code PhysicsLaunchEfficiencyScale}
   * (from ideal-min/empirical-map RPM while aiming).
   */
  public static final double READY_TO_SHOOT_FLYWHEEL_RPM_TOLERANCE = 600;

  public static final double READY_TO_SHOOT_TURRET_DEG_TOLERANCE = 22;
  /** Max |turret slew rate| (deg/s) to still count as “settled” during slow aim tracking. */
  public static final double READY_TO_SHOOT_TURRET_MAX_DEG_PER_SEC = 25;

  public static final double ShooterXOffset = 0.2032;
  public static final double ShooterYOffset = -0.1905;
  public static Translation2d turretOffsetLocal = new Translation2d(ShooterXOffset, ShooterYOffset);
}
