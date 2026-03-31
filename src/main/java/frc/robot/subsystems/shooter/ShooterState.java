// Copyright (c) 2026 FRC Team 4270

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FieldConstants;
import frc.robot.util.geometry.AllianceFlipUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Shooter-specific runtime state and telemetry. Robot pose, velocity remain on {@link
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

  public enum ShootingArc {
    HIGH,
    LOW
  }

  private static ShooterState instance;

  public static ShooterState getInstance() {
    if (instance == null) instance = new ShooterState();
    return instance;
  }

  private ShooterState() {}

  @Getter @Setter @AutoLogOutput(key = "Shooter/State/ReadyToShoot") private boolean shooterReadyToShoot = false;

  @Getter @Setter @AutoLogOutput(key = "Shooter/State/Mode") private ShooterMode shooterMode = ShooterMode.IDLE;

  /**
   * Field-consistent 3D aim point for {@link ShooterMode#POINT_3D} (blue perspective via {@link
   * #applyShooterPoint3dTargetBlue}).
   */
  @Getter @Setter @AutoLogOutput(key = "Shooter/State/Point3dTarget") private Translation3d shooterPoint3dTarget =
      AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);

  @AutoLogOutput(key = "Shooter/State/Tracking") public boolean isShooterTracking() {
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
  @Getter @AutoLogOutput(key = "Shooter/State/FlywheelSurfaceSpeedMps") private double shooterFlywheelSurfaceSpeedMps = 0.0;

  /**
   * Measured hood angle (deg, mechanical Talon/CANcoder frame); updated from {@link
   * frc.robot.subsystems.shooter.hood.Hood#periodic}.
   */
  @Getter @AutoLogOutput(key = "Shooter/State/HoodMeasuredAngleDeg") private double shooterHoodMeasuredAngleDeg = 0.0;

  /** Internal ballistics inputs for {@link ShooterCalculator#refreshCachedParameters}. */
  @Getter private boolean shooterSolveInputsValid = false;

  @Getter private Pose3d shooterSolvePose3d = Pose3d.kZero;
  @Getter private Translation3d shooterSolveVelocity3d = Translation3d.kZero;
  @Getter private Translation3d shooterSolveTarget3d = Translation3d.kZero;

  /**
   * Geometric Δz (shooter to target) non-positive; combined in {@link ShooterCalculator} with hood-height
   * heuristic for low vs high drag root.
   */
  @Getter private ShootingArc shootingArc = ShootingArc.HIGH;

  @Getter @AutoLogOutput(key = "Shooter/State/FlywheelNearGoal") private boolean shooterFlywheelNearGoal = false;
  @Getter @AutoLogOutput(key = "Shooter/State/HoodNearGoal") private boolean shooterHoodNearGoal = false;
  @Getter @AutoLogOutput(key = "Shooter/State/TurretNearGoal") private boolean shooterTurretNearGoal = false;
  @Getter @AutoLogOutput(key = "Shooter/State/TrenchProtectionActive") private boolean shooterTrenchProtectionActive = false;

  public void recordShooterFlywheelSurfaceSpeedMps(double flywheelSurfaceSpeedMps) {
    this.shooterFlywheelSurfaceSpeedMps = flywheelSurfaceSpeedMps;
  }

  public void recordShooterHoodMeasuredAngle(Rotation2d hoodAngle) {
    this.shooterHoodMeasuredAngleDeg = hoodAngle.getDegrees();
  }

  /**
   * @param useLowArc low vs high arc when two drag roots exist.
   */
  public void setShooterSolveInputs(
      Pose3d shooterPose3d,
      Translation3d shooterVelocity3d,
      Translation3d target3d,
      ShootingArc shootingArc) {
    this.shooterSolvePose3d = shooterPose3d;
    this.shooterSolveVelocity3d = shooterVelocity3d;
    this.shooterSolveTarget3d = target3d;
    this.shootingArc = shootingArc;
    this.shooterSolveInputsValid = true;
  }

  public void clearShooterSolveInputs() {
    shooterSolveInputsValid = false;
  }

  public void recordShooterMechanismProcess(
      boolean flywheelNearGoal,
      boolean hoodNearGoal,
      boolean turretNearGoal,
      boolean trenchProtectionActive) {
    this.shooterFlywheelNearGoal = flywheelNearGoal;
    this.shooterHoodNearGoal = hoodNearGoal;
    this.shooterTurretNearGoal = turretNearGoal;
    this.shooterTrenchProtectionActive = trenchProtectionActive;
  }
}
