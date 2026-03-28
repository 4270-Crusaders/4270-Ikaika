// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.Comparator;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final ArrayList<Pose3d> allTagPoses = new ArrayList<>(64);
  private final ArrayList<Pose3d> allRobotPoses = new ArrayList<>(64);
  private final ArrayList<Pose3d> allRobotPosesAccepted = new ArrayList<>(64);
  private final ArrayList<Pose3d> allRobotPosesRejected = new ArrayList<>(64);

  private final ArrayList<Pose3d> tagPoses = new ArrayList<>(32);
  private final ArrayList<Pose3d> robotPoses = new ArrayList<>(32);
  private final ArrayList<Pose3d> robotPosesAccepted = new ArrayList<>(32);
  private final ArrayList<Pose3d> robotPosesRejected = new ArrayList<>(32);
  private final ArrayList<RobotState.VisionObservation> robotStateVisionObservations =
      new ArrayList<>(64);

  public Vision(VisionIO... io) {
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    if (VisionConstants.logDetailedPoses) {
      allTagPoses.clear();
      allRobotPoses.clear();
      allRobotPosesAccepted.clear();
      allRobotPosesRejected.clear();
    }
    robotStateVisionObservations.clear();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      tagPoses.clear();
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        linearStdDev =
            Math.min(linearStdDevMaxMeters, Math.max(linearStdDevMinMeters, linearStdDev));
        angularStdDev =
            Math.min(angularStdDevMaxRadians, Math.max(angularStdDevMinRadians, angularStdDev));

        // Send validated observation to RobotState (sorted globally by timestamp after loop).
        robotStateVisionObservations.add(
            new RobotState.VisionObservation(
                observation.timestamp(),
                observation.pose(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
      }

      if (VisionConstants.logDetailedPoses) {
        String camPrefix = "Vision/Camera" + cameraIndex;
        Logger.recordOutput(camPrefix + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(camPrefix + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(camPrefix + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput(camPrefix + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));
        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
        allRobotPosesRejected.addAll(robotPosesRejected);
      }
    }

    if (VisionConstants.logDetailedPoses) {
      Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    }

    // Apply observations in timestamp order so fusion is deterministic across cameras.
    robotStateVisionObservations.sort(Comparator.comparingDouble(RobotState.VisionObservation::timestamp));
    for (RobotState.VisionObservation observation : robotStateVisionObservations) {
      RobotState.getInstance().addVisionObservation(observation);
    }
  }
}
