// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.LauncherConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import lombok.experimental.ExtensionMethod;
import frc.robot.Constants;
// import frc.robot.FieldConstants;
// import frc.robot.util.geometry.AllianceFlipUtil;
import frc.robot.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class LaunchCalculator {
    private static LaunchCalculator instance;

    private final LinearFilter turretAngleFilter =
        LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
    private final LinearFilter hoodAngleFilter =
        LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    private Rotation2d lastTurretAngle;
    private double lastHoodAngle;
    private Rotation2d turretAngle;
    private double hoodAngle = Double.NaN;
    private double turretVelocity;
    private double hoodVelocity;

    public static LaunchCalculator getInstance() {
        if (instance == null) instance = new LaunchCalculator();
        return instance;
    }

    public record LaunchingParameters(
        boolean isValid,
        Rotation2d turretAngle,
        double turretVelocity,
        double hoodAngle,
        double hoodVelocity,
        double flywheelSpeed) {}

    // Cache parameters
    private LaunchingParameters latestParameters = null;

    private static double minDistance;
    private static double maxDistance;
    private static double phaseDelay;
    private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap =
        new InterpolatingDoubleTreeMap();

    static {
        minDistance = 1.34;
        maxDistance = 5.60;
        phaseDelay = 0.03;

        launchHoodAngleMap.put(Units.inchesToMeters(71),    Rotation2d.fromDegrees(9));
        launchHoodAngleMap.put(Units.inchesToMeters(81),    Rotation2d.fromDegrees(9.25));
        launchHoodAngleMap.put(Units.inchesToMeters(93),    Rotation2d.fromDegrees(10.5));
        launchHoodAngleMap.put(Units.inchesToMeters(105),   Rotation2d.fromDegrees(11));
        launchHoodAngleMap.put(Units.inchesToMeters(117),   Rotation2d.fromDegrees(12.5));
        launchHoodAngleMap.put(Units.inchesToMeters(129),   Rotation2d.fromDegrees(13.5));
        launchHoodAngleMap.put(Units.inchesToMeters(141),   Rotation2d.fromDegrees(17.5));
        launchHoodAngleMap.put(Units.inchesToMeters(153),   Rotation2d.fromDegrees(19));
        launchHoodAngleMap.put(Units.inchesToMeters(165),   Rotation2d.fromDegrees(21));
        launchHoodAngleMap.put(Units.inchesToMeters(177),   Rotation2d.fromDegrees(21.75));
        launchHoodAngleMap.put(Units.inchesToMeters(189),   Rotation2d.fromDegrees(22));
        launchHoodAngleMap.put(Units.inchesToMeters(201),   Rotation2d.fromDegrees(23));
        launchHoodAngleMap.put(Units.inchesToMeters(154.5), Rotation2d.fromDegrees(20));
        launchHoodAngleMap.put(Units.inchesToMeters(158),   Rotation2d.fromDegrees(20.5));
        launchHoodAngleMap.put(Units.inchesToMeters(162),   Rotation2d.fromDegrees(20.625));
        launchHoodAngleMap.put(Units.inchesToMeters(167),   Rotation2d.fromDegrees(21.125));
        launchHoodAngleMap.put(Units.inchesToMeters(174),   Rotation2d.fromDegrees(21.5));
        launchHoodAngleMap.put(Units.inchesToMeters(183.5), Rotation2d.fromDegrees(21.9));
        launchHoodAngleMap.put(Units.inchesToMeters(206),   Rotation2d.fromDegrees(23.5));

        
        launchFlywheelSpeedMap.put(Units.inchesToMeters(71),    2050.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(81),    2100.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(93),    2300.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(105),   2350.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(117),   2400.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(129),   2475.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(141),   2650.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(153),   2675.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(165),   2700.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(177),   2850.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(189),   2850.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(201),   2950.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(154.5), 2688.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(158),   2695.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(162),   2696.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(167),   2750.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(174),   2800.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(183.5), 2875.0*1.62433155);
        launchFlywheelSpeedMap.put(Units.inchesToMeters(206),   3000.0*1.62433155);

        timeOfFlightMap.put(Units.inchesToMeters(84),  1.05333);
        timeOfFlightMap.put(Units.inchesToMeters(123), 1.16666);
        timeOfFlightMap.put(Units.inchesToMeters(155), 1.17333);
        timeOfFlightMap.put(Units.inchesToMeters(185), 1.19666);
        timeOfFlightMap.put(Units.inchesToMeters(222), 1.27000);
    }

    public LaunchingParameters getParameters(Pose2d robotEstimatedPose2d, ChassisSpeeds robotRelativeVelocityChassisSpeed, Translation2d targeTranslation2d) {
        // if (latestParameters != null) {
        //     return latestParameters;
        // }

        // Calculate estimated pose while accounting for phase delay
        Pose2d estimatedPose = robotEstimatedPose2d;
        ChassisSpeeds robotRelativeVelocity = robotRelativeVelocityChassisSpeed;

        estimatedPose =
            estimatedPose.exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate distance from turret to target
        Translation2d target = targeTranslation2d; //AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        Pose2d turretPosition = estimatedPose.transformBy(robotToTurret.toTransform2d());
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocityChassisSpeed, robotEstimatedPose2d.getRotation());
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX =
            robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (robotToTurret.getY() * Math.cos(robotAngle)
                        - robotToTurret.getX() * Math.sin(robotAngle));
        double turretVelocityY =
            robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (robotToTurret.getX() * Math.cos(robotAngle)
                        - robotToTurret.getY() * Math.sin(robotAngle));

        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose =
                new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
        if (lastTurretAngle == null) lastTurretAngle = turretAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
        turretVelocity =
            turretAngleFilter.calculate(
                turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
        hoodVelocity =
            hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngle;
        latestParameters =
            new LaunchingParameters(
                lookaheadTurretToTargetDistance >= minDistance
                    && lookaheadTurretToTargetDistance <= maxDistance,
                turretAngle,
                turretVelocity,
                hoodAngle,
                hoodVelocity,
                launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

        // Log calculated values
        Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        return latestParameters;
    }

    public void clearLaunchingParameters() {
        latestParameters = null;
    }
}