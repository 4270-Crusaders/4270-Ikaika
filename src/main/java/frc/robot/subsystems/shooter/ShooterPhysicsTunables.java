// Copyright (c) 2026 FRC Team 4270

package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTunableNumber;

/**
 * Live-tunable shooter physics and solver parameters via NetworkTables / AdvantageScope. Keys use
 * prefix {@code TunableNumbers/Shooter/Physics/}. Defaults match ShooterConstants; change NT values to
 * tune without redeploy.
 *
 * <p><b>Ballistic tuning order:</b> (1) {@code DragCoefficientCd} — range/decay with spin model off or
 * Cl at 0; (2) {@code MagnusLiftCoefficientCl} — unitless lumped lift coefficient {@code Cl} (not a length
 * in cm); raises trajectory height for typical backspin when {@code Cl > 0}. Start small (e.g. 0.03–0.08)
 * and increase if shots land short while drag is already matched; (3) if height vs wheel speed still
 * disagrees, adjust {@code SpinSlipRatio} and geometry spin clamps — they scale {@code omega} fed into
 * Magnus. {@code BallBackspinMeasuredMinRpm} / {@code MaxRpm} clamp hybrid spin to measured backspin
 * (e.g. ~500–800 RPM) before Magnus.
 *
 * <p><b>Lookahead / lead:</b> {@code HorizontalLookaheadTimeSec} is {@code r + v t} for trench prediction
 * and log-only geometry. {@code MovingTargetLeadTofScale} and {@code MovingTargetLeadTofCapSec} limit
 * how far the solver shifts the shooter in field space using ballistic TOF each lead iteration.
 */
public final class ShooterPhysicsTunables {
  private ShooterPhysicsTunables() {}

  private static final LoggedTunableNumber tAirDensityKgPerM3 =
      new LoggedTunableNumber(
          "Shooter/Physics/AirDensityKgPerM3",
          ShooterConstants.ProjectileConstants.AIR_DENSITY_KG_PER_M3);

  private static final LoggedTunableNumber tDragCoefficient =
      new LoggedTunableNumber(
          "Shooter/Physics/DragCoefficientCd",
          ShooterConstants.ProjectileConstants.DRAG_COEFFICIENT_SMOOTH_SPHERE);

  /** Lumped Magnus lift coefficient {@code Cl} (dimensionless); see class Javadoc for tuning after Cd. */
  private static final LoggedTunableNumber tMagnusLiftCoefficient =
      new LoggedTunableNumber(
          "Shooter/Physics/MagnusLiftCoefficientCl",
          ShooterConstants.ProjectileConstants.MAGNUS_LIFT_COEFFICIENT);

  private static final LoggedTunableNumber tFuelMassKg =
      new LoggedTunableNumber(
          "Shooter/Physics/FuelMassKg", ShooterConstants.ProjectileConstants.FUEL_MASS_KG);

  private static final LoggedTunableNumber tSpinSlipRatio =
      new LoggedTunableNumber(
          "Shooter/Physics/SpinSlipRatio", ShooterConstants.ProjectileConstants.SPIN_SLIP_RATIO);

  private static final LoggedTunableNumber tSpinGeometryScaleMin =
      new LoggedTunableNumber(
          "Shooter/Physics/SpinGeometryScaleMin",
          ShooterConstants.ProjectileConstants.SPIN_GEOMETRY_SCALE_MIN);

  private static final LoggedTunableNumber tSpinGeometryScaleMax =
      new LoggedTunableNumber(
          "Shooter/Physics/SpinGeometryScaleMax",
          ShooterConstants.ProjectileConstants.SPIN_GEOMETRY_SCALE_MAX);

  private static final LoggedTunableNumber tWheelDeltaSpinGain =
      new LoggedTunableNumber(
          "Shooter/Physics/WheelDeltaSpinGain",
          ShooterConstants.ProjectileConstants.WHEEL_DELTA_SPIN_GAIN);

  private static final LoggedTunableNumber tBallBackspinMeasuredMinRpm =
      new LoggedTunableNumber(
          "Shooter/Physics/BallBackspinMeasuredMinRpm",
          ShooterConstants.ProjectileConstants.BALL_BACKSPIN_MEASURED_MIN_RPM);

  private static final LoggedTunableNumber tBallBackspinMeasuredMaxRpm =
      new LoggedTunableNumber(
          "Shooter/Physics/BallBackspinMeasuredMaxRpm",
          ShooterConstants.ProjectileConstants.BALL_BACKSPIN_MEASURED_MAX_RPM);

  private static final LoggedTunableNumber tBallExitTransferEfficiency =
      new LoggedTunableNumber(
          "Shooter/Physics/BallExitTransferEfficiency",
          ShooterConstants.ComponentsConstants.Flywheel.ShotConstants.BALL_EXIT_TRANSFER_EFFICIENCY);

  private static final LoggedTunableNumber tFlywheelCommandRpmOffset =
      new LoggedTunableNumber(
          "Shooter/Physics/FlywheelCommandRpmOffset",
          ShooterConstants.ComponentsConstants.Flywheel.ShotConstants.FLYWHEEL_COMMAND_RPM_OFFSET);

  private static final LoggedTunableNumber tMinExitVelocityHeadroomRatio =
      new LoggedTunableNumber(
          "Shooter/Physics/MinExitVelocityHeadroomRatio",
          ShooterConstants.ShooterCalculatorConstants.MIN_EXIT_VELOCITY_HEADROOM_RATIO);

  private static final LoggedTunableNumber tTrajectoryMaxLaunchSpeedMps =
      new LoggedTunableNumber(
          "Shooter/Physics/TrajectoryMaxLaunchSpeedMps",
          ShooterConstants.ShooterCalculatorConstants.TRAJECTORY_MAX_LAUNCH_SPEED_MPS);

  private static final LoggedTunableNumber tDragShootBallSpeedBumpRatio =
      new LoggedTunableNumber(
          "Shooter/Physics/DragShootBallSpeedBumpRatio",
          ShooterConstants.ShooterCalculatorConstants.DRAG_SHOOT_BALL_SPEED_BUMP_RATIO);

  private static final LoggedTunableNumber tDragShootBallSpeedBumpMaxSteps =
      new LoggedTunableNumber(
          "Shooter/Physics/DragShootBallSpeedBumpMaxSteps",
          ShooterConstants.ShooterCalculatorConstants.DRAG_SHOOT_BALL_SPEED_BUMP_MAX_STEPS);

  private static final LoggedTunableNumber tTrajectorySimDtSec =
      new LoggedTunableNumber(
          "Shooter/Physics/TrajectorySimDtSec",
          ShooterConstants.ShooterCalculatorConstants.TRAJECTORY_SIM_DT_SEC);

  private static final LoggedTunableNumber tTrajectorySimMaxTimeSec =
      new LoggedTunableNumber(
          "Shooter/Physics/TrajectorySimMaxTimeSec",
          ShooterConstants.ShooterCalculatorConstants.TRAJECTORY_SIM_MAX_TIME_SEC);

  private static final LoggedTunableNumber tMovingTargetLeadIterations =
      new LoggedTunableNumber(
          "Shooter/Physics/MovingTargetLeadIterations",
          ShooterConstants.ShooterCalculatorConstants.MOVING_TARGET_LEAD_ITERATIONS);

  private static final LoggedTunableNumber tHorizontalLookaheadTimeSec =
      new LoggedTunableNumber(
          "Shooter/Physics/HorizontalLookaheadTimeSec",
          ShooterConstants.ShooterAimConstants.Trench.LOOKAHEAD_TIME_SEC);

  private static final LoggedTunableNumber tMovingTargetLeadTofScale =
      new LoggedTunableNumber(
          "Shooter/Physics/MovingTargetLeadTofScale",
          ShooterConstants.ShooterCalculatorConstants.MOVING_TARGET_LEAD_TOF_SCALE);

  private static final LoggedTunableNumber tMovingTargetLeadTofCapSec =
      new LoggedTunableNumber(
          "Shooter/Physics/MovingTargetLeadTofCapSec",
          ShooterConstants.ShooterCalculatorConstants.MOVING_TARGET_LEAD_TOF_CAP_SEC);

  /** 0.5 * rho * Cd * A / m (1/m); A from ShooterConstants.ProjectileConstants. */
  public static double dragAccelFactorPerM() {
    double rho = tAirDensityKgPerM3.get();
    double cd = tDragCoefficient.get();
    double m = Math.max(tFuelMassKg.get(), 1e-6);
    double a = ShooterConstants.ProjectileConstants.FUEL_CROSS_SECTIONAL_AREA_M2;
    return 0.5 * rho * cd * a / m;
  }

  /** 0.5 * rho * Cl * A / m (1/m). */
  public static double magnusAccelFactorPerM() {
    double rho = tAirDensityKgPerM3.get();
    double cl = tMagnusLiftCoefficient.get();
    double m = Math.max(tFuelMassKg.get(), 1e-6);
    double a = ShooterConstants.ProjectileConstants.FUEL_CROSS_SECTIONAL_AREA_M2;
    return 0.5 * rho * cl * a / m;
  }

  public static double spinSlipRatio() {
    return tSpinSlipRatio.get();
  }

  public static double spinGeometryScaleMin() {
    return tSpinGeometryScaleMin.get();
  }

  public static double spinGeometryScaleMax() {
    return tSpinGeometryScaleMax.get();
  }

  public static double wheelDeltaSpinGain() {
    return tWheelDeltaSpinGain.get();
  }

  /** Lower clamp (RPM) for ball backspin fed to Magnus after hybrid estimate. */
  public static double ballBackspinMeasuredMinRpm() {
    return Math.max(0.0, tBallBackspinMeasuredMinRpm.get());
  }

  /** Upper clamp (RPM) for ball backspin; at least {@link #ballBackspinMeasuredMinRpm()}. */
  public static double ballBackspinMeasuredMaxRpm() {
    double lo = ballBackspinMeasuredMinRpm();
    double hi = tBallBackspinMeasuredMaxRpm.get();
    return Math.max(lo, hi);
  }

  public static double ballExitTransferEfficiency() {
    return tBallExitTransferEfficiency.get();
  }

  /**
   * RPM bias added to tracked-shot flywheel setpoint (does not re-run ballistics). Typical range on field
   * is roughly ±200–800 depending on droop.
   */
  public static double flywheelCommandRpmOffset() {
    double o = tFlywheelCommandRpmOffset.get();
    return Math.min(3000.0, Math.max(-3000.0, o));
  }

  public static double minExitVelocityHeadroomRatio() {
    return tMinExitVelocityHeadroomRatio.get();
  }

  public static double trajectoryMaxLaunchSpeedMps() {
    return Math.max(tTrajectoryMaxLaunchSpeedMps.get(), 0.5);
  }

  public static double dragShootBallSpeedBumpRatio() {
    return Math.max(tDragShootBallSpeedBumpRatio.get(), 1.0);
  }

  public static int dragShootBallSpeedBumpMaxSteps() {
    int r = (int) Math.round(tDragShootBallSpeedBumpMaxSteps.get());
    return Math.min(100, Math.max(1, r));
  }

  public static int movingTargetLeadIterations() {
    int r = (int) Math.round(tMovingTargetLeadIterations.get());
    return Math.min(50, Math.max(1, r));
  }

  /** Constant-velocity prediction horizon (s) for trench checks and logged look-ahead pose. */
  public static double horizontalLookaheadTimeSec() {
    return Math.max(0.0, tHorizontalLookaheadTimeSec.get());
  }

  /** Scales ballistic TOF when advancing shooter position for moving-target aim (0 = no lead). */
  public static double movingTargetLeadTofScale() {
    double s = tMovingTargetLeadTofScale.get();
    return Math.min(2.0, Math.max(0.0, s));
  }

  /**
   * Upper bound (s) on displacement time for moving-target lead per iteration; {@code <= 0} means no cap
   * (uses raw scaled TOF).
   */
  public static double movingTargetLeadTofCapSec() {
    double c = tMovingTargetLeadTofCapSec.get();
    if (c <= 0.0) {
      return Double.POSITIVE_INFINITY;
    }
    return Math.min(c, 5.0);
  }

  public static double trajectorySimDtSec() {
    return Math.max(tTrajectorySimDtSec.get(), 1e-4);
  }

  public static double trajectorySimMaxTimeSec() {
    return Math.max(tTrajectorySimMaxTimeSec.get(), 0.05);
  }
}
