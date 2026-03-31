// Copyright (c) 2026 FRC Team 4270

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FieldConstants;
import frc.robot.util.geometry.AllianceFlipUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Shooter-specific runtime state and telemetry. Robot pose, velocity, and acceleration remain on {@link
 * frc.robot.RobotState}; {@link ShooterCalculator} reads those and publishes solve inputs here.
 */
public class ShooterState {
  /** High-level aim mode. {@link #IDLE} keeps mechanisms at home/idle setpoints. */
  public enum ShooterMode {
    IDLE,
    HUB,
    PASS,
    CUSTOM,
    POINT_3D
  }

  private static ShooterState instance;

  public static ShooterState getInstance() {
    if (instance == null) instance = new ShooterState();
    return instance;
  }

  private ShooterState() {}

  @Getter @Setter @AutoLogOutput private boolean shooterReadyToShoot = false;

  @Getter @Setter private ShooterMode shooterMode = ShooterMode.IDLE;

  /**
   * Field-consistent 3D aim point for {@link ShooterMode#POINT_3D} (blue perspective via {@link
   * #applyShooterPoint3dTargetBlue}).
   */
  @Getter @Setter private Translation3d shooterPoint3dTarget =
      AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);

  public boolean isShooterTracking() {
    return shooterMode == ShooterMode.HUB
        || shooterMode == ShooterMode.PASS
        || shooterMode == ShooterMode.POINT_3D;
  }

  /**
   * Blue-field target in meters; alliance flip applied to match {@link frc.robot.RobotState}, then
   * {@link ShooterMode#POINT_3D}.
   */
  public void applyShooterPoint3dTargetBlue(Translation3d targetFieldBluePerspective) {
    this.shooterPoint3dTarget = AllianceFlipUtil.apply(targetFieldBluePerspective);
    this.shooterMode = ShooterMode.POINT_3D;
  }

  /**
   * Measured average flywheel surface speed (m/s); updated from {@link
   * frc.robot.subsystems.shooter.flywheel.Flywheel#periodic}.
   */
  @Getter @AutoLogOutput private double shooterFlywheelSurfaceSpeedMps = 0.0;

  /**
   * Measured hood angle (rad, mechanical Talon/CANcoder frame); updated from {@link
   * frc.robot.subsystems.shooter.hood.Hood#periodic}.
   */
  @Getter @AutoLogOutput private double shooterHoodMeasuredAngleRad = 0.0;

  /** Internal ballistics inputs for {@link ShooterCalculator#refreshCachedParameters}. */
  @Getter private boolean shooterSolveInputsValid = false;

  @Getter private Pose3d shooterSolvePose3d = Pose3d.kZero;
  @Getter private Translation3d shooterSolveVelocity3d = Translation3d.kZero;
  @Getter private Translation3d shooterSolveAcceleration3d = Translation3d.kZero;
  @Getter private Translation3d shooterSolveTarget3d = Translation3d.kZero;

  /**
   * Geometric Δz (shooter to target) non-positive; combined in {@link ShooterCalculator} with hood-height
   * heuristic for low vs high drag root.
   */
  @Getter private boolean shooterSolveUseLowArc = false;

  @Getter @AutoLogOutput private boolean shooterFlywheelNearGoal = false;
  @Getter @AutoLogOutput private boolean shooterHoodNearGoal = false;
  @Getter @AutoLogOutput private boolean shooterTurretNearGoal = false;
  @Getter @AutoLogOutput private boolean shooterTurretConstrained = false;
  @Getter @AutoLogOutput private boolean shooterTrenchProtectionActive = false;

  public void recordShooterFlywheelSurfaceSpeedMps(double flywheelSurfaceSpeedMps) {
    this.shooterFlywheelSurfaceSpeedMps = flywheelSurfaceSpeedMps;
  }

  public void recordShooterHoodMeasuredAngleRad(double hoodAngleRad) {
    this.shooterHoodMeasuredAngleRad = hoodAngleRad;
  }

  /**
   * @param useLowArc low vs high arc when two drag roots exist.
   */
  public void setShooterSolveInputs(
      Pose3d shooterPose3d,
      Translation3d shooterVelocity3d,
      Translation3d shooterAcceleration3d,
      Translation3d target3d,
      boolean useLowArc) {
    this.shooterSolvePose3d = shooterPose3d;
    this.shooterSolveVelocity3d = shooterVelocity3d;
    this.shooterSolveAcceleration3d = shooterAcceleration3d;
    this.shooterSolveTarget3d = target3d;
    this.shooterSolveUseLowArc = useLowArc;
    this.shooterSolveInputsValid = true;
  }

  public void clearShooterSolveInputs() {
    shooterSolveInputsValid = false;
  }

  public void recordShooterMechanismProcess(
      boolean flywheelNearGoal,
      boolean hoodNearGoal,
      boolean turretNearGoal,
      boolean turretConstrained,
      boolean trenchProtectionActive) {
    this.shooterFlywheelNearGoal = flywheelNearGoal;
    this.shooterHoodNearGoal = hoodNearGoal;
    this.shooterTurretNearGoal = turretNearGoal;
    this.shooterTurretConstrained = turretConstrained;
    this.shooterTrenchProtectionActive = trenchProtectionActive;
  }
}
