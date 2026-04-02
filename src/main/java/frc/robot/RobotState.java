// Copyright (c) 2026 FRC Team 4270
// Credit: FRC 6328 Mechanical Advantage.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.*;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import frc.robot.generated.TunerConstants;
import frc.robot.util.geometry.GeomUtil;
import org.littletonrobotics.junction.AutoLogOutput;

@ExtensionMethod({GeomUtil.class})
public class RobotState {
  // Constants
  private static final double poseBufferSizeSec = 2.0;
  private static final double turretAngleBufferSizeSec = 2.0;
  private static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

  // MARK: - Class fields

  // Pose estimation fields (wheel integration + vision fusion; Drive feeds odometry observations only)
  @Getter
  @AutoLogOutput(key = "Odometry/OdometryPose")
  private Pose2d odometryPose = Pose2d.kZero;

  /** Fused field pose (wheels + gyro + vision via {@link #addVisionObservation}). */
  @Getter
  @AutoLogOutput(key = "Odometry/Robot")
  private Pose2d estimatedPose = Pose2d.kZero;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
  private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
      TimeInterpolatableBuffer.createBuffer(turretAngleBufferSizeSec);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  // Odometry fields
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d gyroOffset = Rotation2d.kZero;

  @Getter private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  /**
   * Robot-frame chassis acceleration from finite differencing {@link #setRobotVelocity} (vx, vy in
   * m/s^2, omega in rad/s^2). Used for moving-target lead with constant-acceleration correction.
   */
  @Getter private ChassisSpeeds robotAcceleration = new ChassisSpeeds();

  private boolean robotVelocityInitializedForAccel = false;

  // MARK: - Initialization

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }
    kinematics = new SwerveDriveKinematics(TunerConstants.moduleTranslations);
  }

  // MARK: - Drive & vision methods

  /** Reset the pose estimate and odometry pose to the given pose. */
  public void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
    // frame of rotation
    gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
    robotVelocityInitializedForAccel = false;
    robotAcceleration = new ChassisSpeeds();
  }

  /**
   * Like {@link #resetPose(Pose2d)} but records current module positions so the next {@link
   * #addOdometryObservation} does not apply a spurious whole-history twist (replaces WPILib {@code
   * SwerveDrivePoseEstimator.resetPosition}).
   */
  public void resetPose(Pose2d pose, SwerveModulePosition[] wheelPositionsMeters) {
    resetPose(pose);
    if (wheelPositionsMeters != null && wheelPositionsMeters.length == 4) {
      for (int i = 0; i < 4; i++) {
        lastWheelPositions[i] = wheelPositionsMeters[i];
      }
    }
  }

  /** Get the rotation of the estimated pose. */
  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  /**
   * Updates measured chassis velocity (from swerve odometry) and derives {@link #robotAcceleration}
   * via {@code (v - v_prev) / dt}. Call once per main loop ({@link frc.robot.Constants#loopPeriodSecs}).
   */
  public void setRobotVelocity(ChassisSpeeds newVelocity) {
    double dt = Constants.loopPeriodSecs;
    if (robotVelocityInitializedForAccel && dt > 1e-9) {
      robotAcceleration =
          new ChassisSpeeds(
              (newVelocity.vxMetersPerSecond - robotVelocity.vxMetersPerSecond) / dt,
              (newVelocity.vyMetersPerSecond - robotVelocity.vyMetersPerSecond) / dt,
              (newVelocity.omegaRadiansPerSecond - robotVelocity.omegaRadiansPerSecond) / dt);
    } else {
      robotVelocityInitializedForAccel = true;
      robotAcceleration = new ChassisSpeeds();
    }
    this.robotVelocity = newVelocity;
  }

  @AutoLogOutput
  public Optional<Rotation2d> getTurretAngle(double timestamp) {
    return turretAngleBuffer.getSample(timestamp);
  }

  /** Adds a new odometry sample from the drive subsystem. */
  public void addOdometryObservation(OdometryObservation observation) {
    // Update odometry pose
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);

    // Replace odometry pose with gyro if present
    observation.gyroAngle.ifPresent(
        gyroAngle -> {
          // Add offset to measured angle
          Rotation2d angle = gyroAngle.plus(gyroOffset);
          odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        });

    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);

    // Apply odometry delta to vision pose estimate
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  /**
   * Adds a turret angle sample (robot-centric azimuth); timestamps use FPGA time for interpolation with
   * vision and odometry buffers.
   */
  public void addTurretObservation(TurretObservation observation) {
    turretAngleBuffer.addSample(observation.timestamp(), observation.turretAngle);
  }

  /** Fuses a vision sample into {@link #estimatedPose} using the odometry pose buffer. */
  public void addVisionObservation(VisionObservation observation) {
    var internalBuffer = poseBuffer.getInternalBuffer();
    if (internalBuffer.isEmpty()) {
      return;
    }

    // Clamp timestamp into available interpolation window. This avoids silently dropping all vision
    // updates when camera timestamps use a different time base and appear "in the future."
    double newestTimestampSec = internalBuffer.lastKey();
    double oldestTimestampSec = internalBuffer.firstKey();
    double observationTimestampSec = observation.timestamp();
    if (!Double.isFinite(observationTimestampSec)) {
      return;
    }
    if (observationTimestampSec < oldestTimestampSec || observationTimestampSec < newestTimestampSec - poseBufferSizeSec) {
      return;
    }
    if (observationTimestampSec > newestTimestampSec) {
      observationTimestampSec = newestTimestampSec;
    }

    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observationTimestampSec);
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // Calculate transforms between odometry pose and vision sample pose
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

    // Shift estimated pose backwards to sample time
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }

    // Calculate the transform from the shifted estimate to the observation pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose().toPose2d());

    // Scale the transform by the Kalman gain
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate the current estimate by applying the scaled transform to the old estimate
    // then shifting forwards using odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  // MARK: - Type declarations

  public record OdometryObservation(
      double timestamp, SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle) {}

  public record VisionObservation(double timestamp, Pose3d visionPose, Matrix<N3, N1> stdDevs) {}

  public record TurretObservation(double timestamp, Rotation2d turretAngle) {}
}