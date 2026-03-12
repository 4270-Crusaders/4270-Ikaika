package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShooterConstants {
  // FlyWheel
  public static final class FlywheelConstants {
    public static final int FLYWHEEL_LEAD_CAN_ID = 20;
    public static final int FLYWHEEL_FOLLOW_CAN_ID = 21;

    public static final double TOP_FLYWHEEL_RADIUS_METERS = 0.0254;
    public static final double BOTTOM_FLYWHEEL_RADIUS_METER = 0.0381;
    public static final boolean FLYWHEEL_CURRENT_LIMIT_ENABLE = true;
    public static final double FLYWHEEL_CURRENT_LIMIT = 60;
    public static final boolean FLYWHEEL_LIMIT_ENABLE = false;
    public static final double FLYWHEEL_MAIN_ROLLER_REDUCTION = 1;
    public static final double FLYWHEEL_HOOD_ROLLER_REDUCTION = 1;
    public static final InvertedValue MAIN_FLYWHEEL_INVERTED_VALUE =
        InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue FLYWHEEL_NEUTRAL_MODE = NeutralModeValue.Coast;
    public static double FlyWheelkP = 4.375;
    public static double FlyWheelkI = 0.015;
    public static double FlyWheelkD = 0.001;
    public static double FlyWheelkA = 0.1;
    public static double FlyWheelkV = 0.5;
    public static double FlyWheelkS = 0.05;
    public static double FlyWheelMotionMagicVelocity = 0;
    public static double FlyWheelMotionMagicAcceleration = 250;
    public static double FlyWheelMotionMagicJerk = 0;

    // TODO: Tune these values
    public static final double FLYWHEEL_TOP_RPM_ALLIANCE = 2500;
    public static final double FLYWHEEL_BOTTOM_RPM_ALLIANCE = 2500;
    public static final double FLYWHEEL_TOP_RPM_OPP = 2500;
    public static final double FLYWHEEL_BOTTOM_RPM_OPP = 2500;
    public static final double FLYWHEEL_TOP_RPM_NEUTRAL = 2500;
    public static final double FLYWHEEL_BOTTOM_RPM_NEUTRAL = 2500;

    public static double TurretMotorToMainFlyWheelReduction = 0.952941176471;
    public static double TurretMotorToHoodFlyWheelReduction = 0.700692041522;
    // 12->18  34->18  15->18
    // 34->30  18->15

  }

  // Hood
  public static final class HoodConstants {
    public static final int HOOD_CAN_ID = 22;
    public static final int HOOD_ENCODER_CAN_ID = 23;
    public static double HoodSensorToMechanismRatio = -21.1428571; // 296/14
    public static double HoodRotorToSensorRatio = -5.25000001; // 42/8
    public static SensorDirectionValue hoodEncoderDirection =
        SensorDirectionValue.Clockwise_Positive;
    public static double HoodEncoderAbsoluteSensorDiscontinuityPoint = 0.5;
    public static double HoodEncoderMagnetOffset = 0.171630859375; // TUNE ALOT!!
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
    public static final double TURRET_MAX_DEGREE = 201;
    public static final double TURRET_MIN_DEGREE = -111;
    public static final double TURRET_LIMIT_DEGREE = 2.5;

    public static final int TURRET_ENCODER_CAN_ID = 25;
    public static double TurretSensorToMechanismRatio = 1;
    public static double TurretRotorToSensorRatio = 62.5;
    public static double TurretEncoderMagnetOffset = 0.381103515625;
    public static SensorDirectionValue turretEncoderDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    public static double TurretEncoderAbsoluteSensorDiscontinuityPoint = 0.5;

    public static double TurretCurrentLimit = 60.0;
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

  public static final double TEST_FLYWHEEL_VELOCITY = 2500;
  public static final double TEST_DISTANCE = 5;
  public static final double SHOOTER_ETA = 0.60; // Test Values 0.5~0.8

  public static final double ShooterXOffset = 0.2032;
  public static final double ShooterYOffset = -0.1905;

  public static Translation2d turretOffsetLocal = new Translation2d(ShooterXOffset, ShooterYOffset);

  // Calculations

  public static Pose2d getShooterPose2d(Pose2d driveTrainPose2d) {
    Pose2d hehePose = driveTrainPose2d;
    double x =
        hehePose.rotateBy(hehePose.getRotation().times(-1)).getTranslation().getX()
            - ShooterXOffset;
    double y =
        hehePose.rotateBy(hehePose.getRotation().times(-1)).getTranslation().getY()
            + ShooterYOffset;
    return new Pose2d(x, y, new Rotation2d()).rotateBy(driveTrainPose2d.getRotation());
  }

  /**
   * Function that converts Rpm to Meter Per Seconds with RPM value and Radius of rotating object
   *
   * @param rpm faka number val
   * @param radius Shiiii radius of hemmah
   * @return Stuf in Meters per Seconds
   */
  public static double rpmToMeterPerSec(double rpm, double radius) {
    return (2 * Math.PI * radius * rpm) / 60;
  }

  /**
   * Returns ball velocity based on top and bottom flywheel
   *
   * @param topFlyWheelRPM RPM of top flywheel
   * @param bottomFlyWheelRPM RPM of bottom flywheel
   * @return initial velocity of ball
   */
  public static double getBallVelocity(
      double topFlyWheelRPM, double bottomFlyWheelRPM, double shooterEta) {
    double avg =
        (rpmToMeterPerSec(topFlyWheelRPM, FlywheelConstants.TOP_FLYWHEEL_RADIUS_METERS)
                + rpmToMeterPerSec(
                    bottomFlyWheelRPM, FlywheelConstants.BOTTOM_FLYWHEEL_RADIUS_METER))
            / 2;
    return avg * shooterEta;
  }

  /**
   * Function that returns desired hood angle based on distance from turret to hub and initial
   * velocity of the flywheel
   *
   * @param distance distance to goal in meters
   * @param initialVelocity velocity of ball exiting in Meters per Seconds
   * @return desired hood angle radians
   */
  public static double getHoodAngle(double distance, double initialVelocity) {

    double distanceSq = Math.pow(distance, 2);

    double velocitySq = Math.pow(initialVelocity, 2);

    double A = (GRAVITY * distanceSq) / (2 * velocitySq);

    double radical = distanceSq - (4 * A * (A + DELTA_HEIGHT));

    return (Math.PI / 2) - Math.atan2((distance + Math.sqrt(radical)), (2 * A));
  }

  /**
   * Get angle centered at pose1 to pose 2
   *
   * @param pose1 pose of origin
   * @param pose2 pose of target
   * @return angle in rad
   */
  public static double getAngleFromTwoTranslation(Translation2d pos1, Translation2d pos2) {
    double deltaX = pos1.getX() - pos2.getX();
    double deltaY = pos1.getY() - pos2.getY();
    return Math.atan2(deltaY, deltaX);
  }

  public static Rotation2d calcTurAng(Pose2d shooterPose2d, Translation2d targetPosition) {
    Translation2d relativeTarget = targetPosition.minus(shooterPose2d.getTranslation());

    Rotation2d targetAngle = relativeTarget.getAngle();

    return shooterPose2d.getRotation().minus(targetAngle);
  }

  /**
   * Calculates the velocity of a point (e.g. turret base) attached to the robot in the world/field
   * frame.
   *
   * <p>This accounts for both the robot's translational velocity and the additional tangential
   * velocity caused by the robot's rotation at the offset location.
   *
   * <p>All linear values are in meters and meters per second. Angular velocity is in radians per
   * second. Assumes the provided {@code ChassisSpeeds} are field-oriented (world-frame vx, vy,
   * omega).
   *
   * @param chassisSpeeds The current chassis speeds of the robot. vxMetersPerSecond and
   *     vyMetersPerSecond represent the linear velocity of the robot center in the field coordinate
   *     system (meters per second). omegaRadiansPerSecond is the robot's angular velocity (positive
   *     = counterclockwise rotation).
   * @param turretOffsetLocal The position of the turret (or desired point) relative to the robot's
   *     center, expressed in the robot's local coordinate frame. Units: meters. Convention: +X =
   *     forward (along robot's heading), +Y = left.
   * @param robotPose The current estimated pose of the robot on the field. Used to obtain the
   *     robot's current heading (rotation) to transform the local offset into world coordinates.
   *     The translation part of the pose is not used — only the rotation matters here.
   * @return A {@link Translation2d} representing the velocity of the turret point in the
   *     world/field frame, in meters per second (x = field east-west component, y = field
   *     north-south component).
   */
  public static Translation2d getTurretVelocity(
      ChassisSpeeds chassisSpeeds, Translation2d turretOffsetLocal, Pose2d robotPose) {
    // Robot center velocity in world frame (m/s)
    Translation2d robotVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

    // Robot's current heading (used to rotate local offset into world space)
    Rotation2d robotHeading = robotPose.getRotation();

    // Robot's angular velocity (rad/s)
    double omega = chassisSpeeds.omegaRadiansPerSecond;

    // Local offset rotated into world coordinates (vector from robot center to turret, in meters)
    Translation2d r_world = turretOffsetLocal.rotateBy(robotHeading);

    // Tangential (rotational) velocity contribution at the turret location
    // In 2D: v_rot = ω × r = (-ω * ry, ω * rx)
    Translation2d v_rotation = new Translation2d(-omega * r_world.getY(), omega * r_world.getX());

    // Sum of translational + rotational velocity = total velocity of the point
    return robotVelocity.plus(v_rotation);
  }

  /**
   * ShooterAimCompensation - Simplified for REBUILT (2026 FRC season).
   *
   * <p>Provides: - Physics-based time-of-flight (TOF) estimation for high-arcing fuel shots. -
   * Iterative adjusted goal position to compensate for robot motion during flight.
   *
   * <p>Use getAdjustedGoalPosition(...) to get the aim point (add this offset internally).
   *
   * <p>Assumptions: - Goal position is horizontal (Translation2d in meters, relative to robot). -
   * You provide heightDifference (HUB opening height minus shooter exit height, typically positive
   * ~1-2m). - muzzleVelocity is characterized exit speed (m/s).
   */
  public static class ShooterAimCompensation {

    private static final double GRAVITY_MPS2 = 9.81;

    private static final int MAX_ITERATIONS = 12;
    private static final double TOF_CONVERGENCE_TOLERANCE = 0.005; // seconds

    /**
     * Main method: Get the motion-compensated goal position to aim at. Iteratively adjusts for
     * Shooter velocity changing the effective distance during TOF.
     *
     * @param shooterMidVelocityField Shooter velocity (m/s) in field coordinates
     * @param currentGoalRelative Current goal position relative to robot (Translation2d, meters)
     * @param heightDifferenceMeters Goal opening height - shooter exit height (m, positive if
     *     higher)
     * @param muzzleVelocityMps Projectile exit speed (m/s)
     * @return Adjusted goal position relative to robot (aim your turret here)
     */
    public static Translation2d getAdjustedGoalPosition(
        Translation2d shooterMidVelocityField,
        Translation2d currentGoalRelative,
        double heightDifferenceMeters,
        double muzzleVelocityMps) {

      if (muzzleVelocityMps <= 0) {
        return currentGoalRelative; // Safety: no valid speed → no change
      }

      Translation2d workingGoal = currentGoalRelative;
      double prevTOF = 0.0;
      int iteration = 0;

      do {
        double horizontalDistance = workingGoal.getNorm();

        double tof =
            calculatePhysicsTOF(horizontalDistance, heightDifferenceMeters, muzzleVelocityMps);

        // Fallback if physics has no solution (e.g., too close/fast or impossible arc)
        if (tof < 0) {
          tof = estimateSimpleTOF(horizontalDistance, muzzleVelocityMps);
        }

        if (tof <= 0) {
          break; // Can't compute valid TOF → return last working aim
        }

        // Offset = -robot velocity * TOF (goal "moves" opposite to your motion)
        Translation2d offset = shooterMidVelocityField.times(-tof);

        workingGoal = currentGoalRelative.plus(offset);

        if (Math.abs(tof - prevTOF) < TOF_CONVERGENCE_TOLERANCE) {
          break;
        }

        prevTOF = tof;
        iteration++;

      } while (iteration < MAX_ITERATIONS);

      return workingGoal;
    }

    /**
     * Get just the estimated TOF (useful for debugging or other calcs). Uses the same physics
     * method as the iterative compensator.
     *
     * @param horizontalDistanceM Horizontal distance to goal (m)
     * @param heightDifferenceM Height diff (goal - shooter exit, m)
     * @param muzzleVelocityMps Exit speed (m/s)
     * @return TOF in seconds, or -1 if impossible
     */
    public static double getEstimatedTOF(
        double horizontalDistanceM, double heightDifferenceM, double muzzleVelocityMps) {

      double tof = calculatePhysicsTOF(horizontalDistanceM, heightDifferenceM, muzzleVelocityMps);

      if (tof < 0) {
        tof = estimateSimpleTOF(horizontalDistanceM, muzzleVelocityMps);
      }

      return tof;
    }

    /**
     * Physics TOF for parabolic/high-arc trajectories (selects higher-angle solution). Returns -1
     * if no real trajectory exists.
     */
    private static double calculatePhysicsTOF(
        double horizontalDistanceM, double heightDifferenceM, double muzzleVelocityMps) {
      if (horizontalDistanceM <= 0 || muzzleVelocityMps <= 0) {
        return -1;
      }

      double vSquared = muzzleVelocityMps * muzzleVelocityMps;
      double discriminant =
          vSquared * vSquared
              - GRAVITY_MPS2
                  * (GRAVITY_MPS2 * horizontalDistanceM * horizontalDistanceM
                      + 2 * heightDifferenceM * vSquared);

      if (discriminant < 0) {
        return -1;
      }

      double sqrtDisc = Math.sqrt(discriminant);

      // Higher arc (longer TOF, typical for fuel lobs over distance)
      double tanTheta = (vSquared + sqrtDisc) / (GRAVITY_MPS2 * horizontalDistanceM);
      double thetaRad = Math.atan(tanTheta);

      double vx = muzzleVelocityMps * Math.cos(thetaRad);

      return horizontalDistanceM / vx;
    }

    /** Simple linear TOF fallback (ignores gravity/height, for very flat or debug use). */
    private static double estimateSimpleTOF(double distanceM, double muzzleVelocityMps) {
      return distanceM / muzzleVelocityMps;
    }
  }

  public static class shootyRPMgetters {
    // public static double rpmFarGet(double distanceToTarget){
    //     // return
    // (33.84669*(Math.pow(distanceToTarget,2)))-(25.68754*distanceToTarget)+2333.88349;
    // }

    public static double rpmFarGet(double distanceToTarget) {
      return (269.31004 * distanceToTarget) + 1691.29172;
    }

    // 269.31004x+1691.29172
    public static double rpmCloseGet(double distanceToTarget) {
      return (-266.17827 * (Math.pow(distanceToTarget, 2))) + (1576.67888 * distanceToTarget);
    }

    public static double getRPM(double distanceToTarget) {
      if (distanceToTarget > 4.9) {
        return 3000;
      } else if (distanceToTarget <= 4.9 && distanceToTarget > 4.1) {
        return rpmFarGet(distanceToTarget);
      } else if (distanceToTarget <= 4.1 && distanceToTarget > 3.6) {
        return 2750;
      } else if (distanceToTarget <= 3.6 && distanceToTarget > 3) {
        return rpmFarGet(distanceToTarget);
      } else if (distanceToTarget <= 3 && distanceToTarget > 2.6) {
        return 2300;
      } else if (distanceToTarget <= 2.6) {
        return rpmCloseGet(distanceToTarget);
      } else {
        return 2200;
      }
    }
  }
}
