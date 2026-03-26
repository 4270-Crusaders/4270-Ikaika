// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class LauncherConstants {
  /** Horizontal distance (m) to hub — valid shot band for launch interpolation. */
  public static final double LAUNCH_MIN_DISTANCE_M = 1.34;

  public static final double LAUNCH_MAX_DISTANCE_M = 5.60;

  /** Phase delay (s) for pose prediction along robot velocity. */
  public static final double LAUNCH_PHASE_DELAY_S = 0.03;

  /**
   * Scales how much of the turret exit velocity (field frame) along the horizontal shot vector is
   * treated as additive to ball speed. The RPM table is tuned stationary; subtracting {@code gain ×
   * v_along} from the commanded surface speed before converting back to RPM reduces overshoot when
   * driving toward the target. Tune 0.6–1.0 if still long/short on the move.
   */
  public static final double MOVING_SHOT_ALONG_V_CORRECTION_GAIN = 0.85;

  public static final double TURRET_HEIGHT = Units.inchesToMeters(20.5); // inches multiplied by meter conversion

  //TODO -> Double Check Values of offsets
  public static final double SHOOTER_X_OFFSET = Units.inchesToMeters(-7.75); //Right
  public static final double SHOOTER_Y_OFFSET = Units.inchesToMeters(-5.75); //Forward

  public static Transform3d robotToTurret = new Transform3d(SHOOTER_X_OFFSET, SHOOTER_Y_OFFSET, TURRET_HEIGHT, Rotation3d.kZero);

  public static final Translation3d pass3dTargetLeft = new Translation3d(3.67, 6.0, 0.0);
  public static final Translation3d pass3dTargetRight = new Translation3d(3.67, 2.043, 0.0);

  public static double passPoint = 4.425;

  public final static double TrenchZoneStart = 4;

  public final static double INCREASE = 0;

  /** Target Z for "pass" shots (ground / floor). */
  public static final double PASS_TARGET_Z_METERS = 0.0;

  /**
   * Effective radius (m) at the exit for ball leaving wheels: {@code v = rpm * 2π r / 60}.
   * Tune so physics RPM matches measured flywheel RPM at a known distance.
   */
  public static final double BALLISTIC_WHEEL_RADIUS_METERS =
      (ShooterConstants.FlywheelConstants.TOP_FLYWHEEL_RADIUS_METERS
              + ShooterConstants.FlywheelConstants.BOTTOM_FLYWHEEL_RADIUS_METER)
          / 2.0;

  /**
   * Legacy logistic RPM curve (unused — launch uses {@link #interpolateFlywheelRpm(double)}). Kept
   * for reference.
   */
  public static final double LAUNCH_RPM_LOGISTIC_A = 3.76689;

  public static final double LAUNCH_RPM_LOGISTIC_NUMERATOR_EXPONENT = 114;
  public static final double LAUNCH_RPM_LOGISTIC_K = 0.00364395;
  public static final double LAUNCH_RPM_LOGISTIC_D0 = 256.56495;

  private static final NavigableMap<Double, Double> FLYWHEEL_RPM_BY_DISTANCE_M = new TreeMap<>();
  private static final NavigableMap<Double, Double> HOOD_DEG_BY_DISTANCE_M = new TreeMap<>();
  private static final NavigableMap<Double, Double> TIME_OF_FLIGHT_BY_DISTANCE_M = new TreeMap<>();

  static {
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(71), 2050.0 + 200 - 50);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(81), 2100.0 + 200 - 50);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(93), 2300.0 + 200 - 50);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(105), 2350.0 + 200 - 50);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(117), 2400.0 + 200 - 100);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(129), 2475.0 + 200 - 100);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(141), 2650.0 + 200 - 100);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(153), 2675.0 + 200 - 100);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(154.5), 2688.0 + 200 - 100);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(158), 2695.0 + 200 - 100);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(162), 2696.0 + 200 - 100);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(165), (4700.0) / 1.62433155);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(181.3), (4900.0) / 1.62433155);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(193.6), (5150.0) / 1.62433155);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(205), (5500.0) / 1.62433155);
    put(FLYWHEEL_RPM_BY_DISTANCE_M, Units.inchesToMeters(216.9), (5700.0) / 1.62433155);

    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(50), 9.0);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(81), 9.25);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(93), 10.5);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(105), 11.0);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(117), 12.5);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(129), 13.5);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(141), 17.5);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(153), 19.0);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(154.5), 20.0);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(158), 20.5);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(162), 20.625);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(165), 21.0);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(167), 21.125);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(174), 21.5);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(177), 21.75);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(183.5), 21.9);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(189), 22.0);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(201), 23.0);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(206), 23.5);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(216.9), 24.0);
    put(HOOD_DEG_BY_DISTANCE_M, Units.inchesToMeters(220), 25.0);

    put(TIME_OF_FLIGHT_BY_DISTANCE_M, Units.inchesToMeters(84), 1.05333);
    put(TIME_OF_FLIGHT_BY_DISTANCE_M, Units.inchesToMeters(123), 1.16666);
    put(TIME_OF_FLIGHT_BY_DISTANCE_M, Units.inchesToMeters(155), 1.17333);
    put(TIME_OF_FLIGHT_BY_DISTANCE_M, Units.inchesToMeters(185), 1.19666);
    put(TIME_OF_FLIGHT_BY_DISTANCE_M, Units.inchesToMeters(222), 1.27000);
  }

  private static void put(NavigableMap<Double, Double> map, double keyMeters, double value) {
    map.put(keyMeters, value);
  }

  /** Piecewise-linear interpolation; clamps outside the sampled range. */
  public static double interpolateLinear(NavigableMap<Double, Double> map, double x) {
    if (map.isEmpty()) {
      return 0;
    }
    Map.Entry<Double, Double> first = map.firstEntry();
    Map.Entry<Double, Double> last = map.lastEntry();
    if (x <= first.getKey()) {
      return first.getValue();
    }
    if (x >= last.getKey()) {
      return last.getValue();
    }
    Map.Entry<Double, Double> lower = map.floorEntry(x);
    Map.Entry<Double, Double> upper = map.ceilingEntry(x);
    if (lower == null) {
      return upper.getValue();
    }
    if (upper == null) {
      return lower.getValue();
    }
    if (lower.getKey().equals(upper.getKey())) {
      return lower.getValue();
    }
    double t = (x - lower.getKey()) / (upper.getKey() - lower.getKey());
    return lower.getValue() + t * (upper.getValue() - lower.getValue());
  }

  /** Empirical flywheel RPM (base, before offset / multiplier) vs horizontal distance (m). */
  public static double interpolateFlywheelRpm(double distanceMeters) {
    return interpolateLinear(FLYWHEEL_RPM_BY_DISTANCE_M, distanceMeters);
  }

  /** Empirical hood mechanical angle (deg) vs horizontal distance (m). */
  public static double interpolateHoodMechanicalDeg(double distanceMeters) {
    return interpolateLinear(HOOD_DEG_BY_DISTANCE_M, distanceMeters);
  }

  /** Empirical time of flight (s) vs horizontal distance (m). */
  public static double interpolateTimeOfFlight(double distanceMeters) {
    return interpolateLinear(TIME_OF_FLIGHT_BY_DISTANCE_M, distanceMeters);
  }

  private LauncherConstants() {}
}