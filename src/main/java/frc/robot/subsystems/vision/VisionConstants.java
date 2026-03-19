// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String cameraFrontName = "limelight-front";
  public static String cameraLeftName = "limelight-left";
  public static String cameraRightName = "limelight-right";


  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToFrontCam =
        new Transform3d(Units.inchesToMeters(-12.33976),       // Right
                        Units.inchesToMeters(-6.295478),       // Forward
                        Units.inchesToMeters(20.4200),  // Up
                        new Rotation3d(0, Units.degreesToRadians(10), 0));
    public static Transform3d robotToLeftCam =
        new Transform3d(Units.inchesToMeters(-12.45784),       // Right
                        Units.inchesToMeters(-9.440023),       // Forward
                        Units.inchesToMeters(20.68442), // Up
                        new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(205))); //clockwise positive
    public static Transform3d robotToRightCam =
        new Transform3d(Units.inchesToMeters(-12.6474354),     // Right
                        Units.inchesToMeters(-9.380305),       // Forward
                        Units.inchesToMeters(18.38072), // Up
                        new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(155))); //clockwise positive

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  //TODO -> Change priority for back camera overlap (lower prio for front facing cam)
  public static double[] cameraStdDevFactors =
      new double[] {
        0.75, // Front
        1.0, // Left
        1.0  // Right
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
