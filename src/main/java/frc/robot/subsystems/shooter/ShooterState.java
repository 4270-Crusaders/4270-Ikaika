// Copyright (c) 2026 FRC Team 4270

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
    START,
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

  /**
   * Hub shot aim (blue WPILib field frame, same as {@link frc.robot.RobotState#getEstimatedPose()}).
   * Blue aims at {@link FieldConstants.Hub#topCenterPoint}; red at {@link FieldConstants.Hub#oppTopCenterPoint}.
   */
  public static Translation3d hubShootTargetTranslation3d() {
    Translation3d hubTop =
        AllianceFlipUtil.shouldFlip()
            ? FieldConstants.Hub.oppTopCenterPoint
            : FieldConstants.Hub.topCenterPoint;
    return hubTop.minus(new Translation3d(0.0, 0.0, Units.inchesToMeters(2.0)));
  }

  /**
   * Pass target in blue-origin field frame (same as robot pose). Longitudinal X is always {@link
   * FieldConstants.Pass#AIM_X_METERS} for both alliances (red-side pass lane in blue coordinates). Chooses
   * field-left vs field-right Y from robot pose and {@link FieldConstants.Pass} trench lanes.
   */
  public static Translation3d passShootTargetTranslation3d(Pose2d robotEstimatedPose) {
    double halfWidth = FieldConstants.fieldWidth * 0.5;
    double robotY = robotEstimatedPose.getY();
    Translation3d targetPoint = robotY > halfWidth ? FieldConstants.Pass.LEFT_TARGET_BLUE : FieldConstants.Pass.RIGHT_TARGET_BLUE;
    return new Translation3d(
        AllianceFlipUtil.shouldFlip() ? FieldConstants.fieldLength - targetPoint.getX() : targetPoint.getX(),
        targetPoint.getY(),
        targetPoint.getZ());
  }

  /**
   * Teleop shoot (right trigger): {@link ShooterMode#HUB} while the robot is on <em>our</em> alliance
   * side, {@link ShooterMode#PASS} in the neutral band or opponent half. Pose is blue WPILib field
   * frame (same as {@link frc.robot.RobotState#getEstimatedPose()}); alliance side uses {@link
   * AllianceFlipUtil#shouldFlip()} (red vs blue).
   */
  public static ShooterMode teleopAimModeForOwnFieldSide(Pose2d poseBlueField) {
    double x = poseBlueField.getX();
    if (!AllianceFlipUtil.shouldFlip()) {
      return x < FieldConstants.LinesVertical.neutralZoneNear ? ShooterMode.HUB : ShooterMode.PASS;
    }
    return x > FieldConstants.LinesVertical.neutralZoneFar ? ShooterMode.HUB : ShooterMode.PASS;
  }

  /** Alliance hub hole center (blue frame); use for default 3D aim when not overridden. */
  public static Translation3d allianceHubInnerCenterAim() {
    return AllianceFlipUtil.shouldFlip()
        ? FieldConstants.Hub.oppInnerCenterPoint
        : FieldConstants.Hub.innerCenterPoint;
  }

  @Getter @Setter @AutoLogOutput(key = "Shooter/State/ReadyToShoot") private boolean shooterReadyToShoot = false;

  @Getter @Setter @AutoLogOutput(key = "Shooter/State/Mode") private ShooterMode shooterMode = ShooterMode.START;

  /**
   * Blue-field aim for {@link ShooterMode#POINT_3D}; set via {@link #applyShooterPoint3dTargetBlue}.
   * {@link #getPoint3dTargetPose()} logs a full {@link Pose3d} (identity rotation) to AdvantageKit.
   */
  @Getter @Setter private Translation3d shooterPoint3dTarget = FieldConstants.Hub.innerCenterPoint;

  @AutoLogOutput(key = "Shooter/State/Point3dTarget")
  public Pose3d getPoint3dTargetPose() {
    return new Pose3d(shooterPoint3dTarget, new Rotation3d());
  }

  /**
   * When {@link ShooterMode#POINT_3D} is active and the aim point is above the shooter height, the
   * solver may use the high (lob) arc instead of the default low arc. Hub, pass, and other modes always
   * use the low arc.
   */
  @Setter @AutoLogOutput(key = "Shooter/State/Point3dUseHighArc")
  private boolean point3dUseHighArc = false;

  /** @see #point3dUseHighArc */
  public boolean isPoint3dUseHighArc() {
    return point3dUseHighArc;
  }

  @AutoLogOutput(key = "Shooter/State/Tracking") public boolean isShooterTracking() {
    return shooterMode == ShooterMode.HUB
        || shooterMode == ShooterMode.PASS
        || shooterMode == ShooterMode.POINT_3D;
  }

  /**
   * Sets the point-3d aim target in blue field coordinates (do not alliance-flip; matches {@link
   * frc.robot.RobotState} pose frame).
   */
  public void applyShooterPoint3dTargetBlue(Translation3d targetFieldBluePerspective) {
    this.shooterPoint3dTarget = targetFieldBluePerspective;
    this.shooterMode = ShooterMode.POINT_3D;
  }

  /**
   * Measured average flywheel surface speed (m/s); updated from {@link
   * frc.robot.subsystems.shooter.flywheel.Flywheel#periodic}.
   */
  @Getter @AutoLogOutput(key = "Shooter/State/FlywheelSurfaceSpeedMps") private double shooterFlywheelSurfaceSpeedMps = 0.0;

  /**
   * Measured hood minus main wheel surface speed (m/s), both at the ball contact. Positive when the
   * hood wheel is faster; flip {@link ShooterPhysicsTunables#wheelDeltaSpinGain()} sign if your
   * mechanical top wheel is main. Updated from {@link
   * frc.robot.subsystems.shooter.flywheel.Flywheel#periodic}.
   */
  @Getter
  @AutoLogOutput(key = "Shooter/State/FlywheelTopBottomSurfaceDeltaMps")
  private double shooterFlywheelTopBottomSurfaceDeltaMps = 0.0;

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
   * LOW vs HIGH ballistic arc selected for the last solve. Pass uses the same ballistic hood/RPM solve
   * as hub but always {@link ShootingArc#LOW} (no hub lookahead-distance HIGH rule). Hub uses HIGH
   * inside {@link ShooterConstants.ShooterAimConstants.Hub#HIGH_ARC_LOOKAHEAD_DISTANCE_METERS} or when
   * the calculator falls back after a failed LOW solve. {@link ShooterMode#POINT_3D} uses HIGH when
   * {@link #point3dUseHighArc} is true and the target is above the shooter.
   */
  @Getter private ShootingArc shootingArc = ShootingArc.LOW;

  @Getter @AutoLogOutput(key = "Shooter/State/FlywheelNearGoal") private boolean shooterFlywheelNearGoal = false;
  @Getter @AutoLogOutput(key = "Shooter/State/HoodNearGoal") private boolean shooterHoodNearGoal = false;
  @Getter @AutoLogOutput(key = "Shooter/State/TurretNearGoal") private boolean shooterTurretNearGoal = false;
  @Getter @AutoLogOutput(key = "Shooter/State/TrenchProtectionActive") private boolean shooterTrenchProtectionActive = false;

  public void recordShooterFlywheelSurfaceSpeedMps(double flywheelSurfaceSpeedMps) {
    this.shooterFlywheelSurfaceSpeedMps = flywheelSurfaceSpeedMps;
  }

  /**
   * @param flywheelTopBottomSurfaceDeltaMps hood minus main surface speed (m/s); see field
   *     {@code shooterFlywheelTopBottomSurfaceDeltaMps}.
   */
  public void recordShooterFlywheelTopBottomSurfaceDeltaMps(double flywheelTopBottomSurfaceDeltaMps) {
    this.shooterFlywheelTopBottomSurfaceDeltaMps = flywheelTopBottomSurfaceDeltaMps;
  }

  public void recordShooterHoodMeasuredAngle(Rotation2d hoodAngle) {
    this.shooterHoodMeasuredAngleDeg = hoodAngle.getDegrees();
  }

  /**
   * @param shootingArc which drag root to prefer when both arcs are feasible ({@link ShootingArc#LOW}
   *     vs {@link ShootingArc#HIGH}).
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
