package frc.robot.util;

public class SpeedUtil {
    public static double metersPerSecondFromRpm(double rpm, double wheelRadiusMeters) {
        return rpm * 2.0 * Math.PI * wheelRadiusMeters * (1/60.0);
    }

    public static double rpmFromMetersPerSecond(double metersPerSecond, double wheelRadiusMeters) {
        return metersPerSecond * 60.0 / (2.0 * Math.PI * wheelRadiusMeters);
    }
}
