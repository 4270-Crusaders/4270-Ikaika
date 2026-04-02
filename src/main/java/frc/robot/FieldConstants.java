// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station
 */
public class FieldConstants {
  public static final FieldType fieldType = FieldType.WELDED;

  // AprilTag related constants
  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  // Field dimensions
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  /**
   * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
   */
  public static class LinesVertical {
    public static final double center = fieldLength / 2.0;
    public static final double starting =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
    public static final double allianceZone = starting;
    public static final double hubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0;
    /**
     * Blue-origin X: blue alliance side is {@code x < neutralZoneNear}; neutral is {@code [near, far]};
     * red side is {@code x > neutralZoneFar}. Used for teleop auto hub vs pass (see {@link
     * frc.robot.subsystems.shooter.ShooterState#teleopAimModeForOwnFieldSide}).
     */
    public static final double neutralZoneNear = center - Units.inchesToMeters(120);
    /** @see #neutralZoneNear */
    public static final double neutralZoneFar = center + Units.inchesToMeters(120);
    public static final double oppHubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0;
    public static final double oppAllianceZone =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get().getX();
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
   *
   * <p>NOTE: The field element start and end are always left to right from the perspective of the
   * alliance station
   */
  public static class LinesHorizontal {

    public static final double center = fieldWidth / 2.0;

    // Right of hub
    public static final double rightBumpStart = Hub.nearRightCorner.getY();
    public static final double rightBumpEnd = rightBumpStart - RightBump.width;
    public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
    public static final double rightTrenchOpenEnd = 0;

    // Left of hub
    public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
    public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
    public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
    public static final double leftTrenchOpenStart = fieldWidth;
  }

  /**
   * Hub related constants.
   *
   * <p>Plan / controlled dimensions from FIRST 2026 field dwgs (FE-2026 Rev B), e.g. <a
   * href="https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf">2026-field-dimension-dwgs.pdf</a>:
   * 47" O.D., 72" top height, 41.73" inside scoring opening, hole center 44.25" AG, mouth inner
   * radius 22.25" (2× 22.25 +0.25/−0.25 controlled on perimeter dwg; plan shows (22.25) from hub
   * centerline).
   */
  public static class Hub {

    public static final double width = Units.inchesToMeters(47.0);
    /** Top of hub / funnel roof plane (72.000 ± 0.500 on GE-26300). */
    public static final double height = Units.inchesToMeters(72.0);
    /** Inside scoring opening ⌀ (field plan reference). */
    public static final double innerWidth = Units.inchesToMeters(41.73);
    /** Hole center height AG (field plan / hub origin table). */
    public static final double innerHeight = Units.inchesToMeters(44.25);

    /**
     * Clear inner radius (m) at the funnel mouth plane ({@link #height}): nominal 22.25 in from
     * official hub / field drawings (not half of {@link #width}, which is outer structure).
     */
    public static final double funnelMouthInnerRadius = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d topCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);
    /** Aim here for scoring shots (hole center), not {@link #topCenterPoint}. */
    public static final Translation3d innerCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            innerHeight);

    /**
     * Upside-down funnel: wider aperture at {@link #height}, narrowing to the hole at {@link
     * #innerHeight}. Used for radial clearance along the shot.
     */
    public static double funnelInnerRadiusAtZ(double zMeters) {
      double zHole = innerCenterPoint.getZ();
      double zTop = height;
      double rHole = innerWidth * 0.5;
      double rMouth = funnelMouthInnerRadius;
      if (zMeters <= zHole) {
        return rHole;
      }
      if (zMeters >= zTop) {
        return rMouth;
      }
      double t = (zMeters - zHole) / (zTop - zHole);
      return rHole + t * (rMouth - rHole);
    }

    /**
     * Inner funnel radius allowed at horizontal distance {@code approachMetersFromHole} before the
     * hole along the shot (0 at hole center, larger when farther from hole). Linear frustum from hole
     * to mouth; beyond {@code funnelAxialDepthMeters} the robot is outside the funnel walls.
     */
    public static double funnelInnerRadiusAtApproach(
        double approachMetersFromHole, double funnelAxialDepthMeters) {
        double rHole = innerWidth * 0.5;
        double rMouth = funnelMouthInnerRadius;
        if (approachMetersFromHole <= 0.0) {
            return rHole;
        }
        if (approachMetersFromHole >= funnelAxialDepthMeters) {
            return rMouth;
        }
        return rHole + (rMouth - rHole) * (approachMetersFromHole / funnelAxialDepthMeters);
    }

    public static final Translation2d nearLeftCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Relevant reference points on the opposite side
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);

    /** Scoring hole center on the opposite-alliance hub (blue frame); same Z as {@link #innerCenterPoint}. */
    public static final Translation3d oppInnerCenterPoint =
        new Translation3d(
            oppTopCenterPoint.getX(), oppTopCenterPoint.getY(), innerHeight);

    public static final Translation2d oppNearLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppNearRightCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppFarRightCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Hub faces
    public static final Pose2d nearFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
    public static final Pose2d farFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
    public static final Pose2d rightFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();
    public static final Pose2d leftFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();
  }

  /** Left Bump related constants */
  public static class LeftBump {

    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    // Relevant reference points on opposing side
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Right Bump related constants */
  public static class RightBump {
    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    // Relevant reference points on opposing side
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Left Trench related constants */
  public static class LeftTrench {
    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

    // Relevant reference points on opposing side
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
  }

  public static class RightTrench {

    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

    // Relevant reference points on opposing side
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
  }

  /**
   * Pass / alley partner-shot aim points in the <b>blue-origin</b> field frame (same as {@link
   * frc.robot.RobotState}).
   *
   * <p>Longitudinal X is past {@link LinesVertical#neutralZoneFar} for <em>both</em> alliances (same
   * field waypoint in blue coordinates). Do not use {@link
   * frc.robot.util.geometry.AllianceFlipUtil#apply(Translation3d)} for passes—it mirrors into the wrong
   * half. Y uses {@link LinesHorizontal} trench band (field left vs right).
   */
  public static final class Pass {
    /** Past neutral toward +X (red wall in blue frame). Used for pass aim for blue and red alliance. */
    public static final double AIM_X_METERS =
        LinesVertical.neutralZoneFar + Units.inchesToMeters(24.0);

    /** Lateral aim toward left trench opening side (high Y, blue-left). */
    public static final double LEFT_LANE_Y_METERS =
        LinesHorizontal.leftTrenchOpenEnd - Units.inchesToMeters(18.0);

    /** Lateral aim toward right trench opening side (low Y, blue-right). */
    public static final double RIGHT_LANE_Y_METERS =
        LinesHorizontal.rightTrenchOpenStart + Units.inchesToMeters(18.0);

    /** Ground plane / geometric pass height for ballistics-only Z (m). */
    public static final double TARGET_Z_METERS = 0.0;

    public static final Translation3d LEFT_TARGET_BLUE =
        new Translation3d(AIM_X_METERS, LEFT_LANE_Y_METERS, TARGET_Z_METERS);

    public static final Translation3d RIGHT_TARGET_BLUE =
        new Translation3d(AIM_X_METERS, RIGHT_LANE_Y_METERS, TARGET_Z_METERS);
  }

  /** Tower related constants */
  public static class Tower {
    // Dimensions
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);

    public static final double uprightHeight = Units.inchesToMeters(72.1);

    // Rung heights from the floor
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint =
        new Translation2d(
            frontFaceX, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());
    public static final Translation2d leftUpright =
        new Translation2d(
            frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d rightUpright =
        new Translation2d(
            frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));

    // Relevant reference points on opposing side
    public static final Translation2d oppCenterPoint =
        new Translation2d(
            fieldLength - frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY());
    public static final Translation2d oppLeftUpright =
        new Translation2d(
            fieldLength - frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d oppRightUpright =
        new Translation2d(
            fieldLength - frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));
  }

  public static class Depot {
    // Dimensions
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    // Relevant reference points on alliance side
    public static final Translation3d depotCenter =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
    public static final Translation3d leftCorner =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
    public static final Translation3d rightCorner =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
  }

  public static class Outpost {
    // Dimensions
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint =
        new Translation2d(0, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
  }

  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    @Getter private final String jsonFolder;
  }

  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
    NONE("2026-none");

    private final String name;
    private volatile AprilTagFieldLayout layout;
    private volatile String layoutString;

    AprilTagLayoutType(String name) {
      this.name = name;
    }

    public AprilTagFieldLayout getLayout() {
      if (layout == null) {
        synchronized (this) {
          if (layout == null) {
            try {
              Path p =
                  Constants.disableHAL
                      ? Path.of(
                          "src",
                          "main",
                          "deploy",
                          "apriltags",
                          fieldType.getJsonFolder(),
                          name + ".json")
                      : Path.of(
                          Filesystem.getDeployDirectory().getPath(),
                          "apriltags",
                          fieldType.getJsonFolder(),
                          name + ".json");
              layout = new AprilTagFieldLayout(p);
              layoutString = new ObjectMapper().writeValueAsString(layout);
            } catch (IOException e) {
              throw new RuntimeException(e);
            }
          }
        }
      }
      return layout;
    }

    public String getLayoutString() {
      if (layoutString == null) {
        getLayout();
      }
      return layoutString;
    }
  }
}