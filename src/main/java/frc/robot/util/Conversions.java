package frc.robot.util;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Conversions {
    public static double radiansToMeters(double rad, double radiusMeters) { return rad * radiusMeters; }

    public static double metersToRadians(double meters, double radiusMeters) { return meters / radiusMeters; }

    public static double elevatorRadiansToMeters(double positionRad) {
        return radiansToMeters(positionRad, Constants.ElevatorConstants.kSprocketRadiusMeters) * 2;
    }

    public static double elevatorMetersToRadians(double meters) {
        return metersToRadians(meters, Constants.ElevatorConstants.kSprocketRadiusMeters) / 2;
    }

    // Conversions contained in Units

    public static double metersToFeet(double meters) { return Units.metersToFeet(meters); }

    public static double feetToMeters(double feet) { return Units.feetToMeters(feet); }

    public static double metersToInches(double meters) { return Units.metersToInches(meters); }

    public static double inchesToMeters(double inches) { return Units.inchesToMeters(inches); }

    public static double degreesToRadians(double degrees) { return Units.degreesToRadians(degrees); }

    public static double radiansToDegrees(double radians) { return Units.radiansToDegrees(radians); }

    public static double radiansToRotations(double rotations) { return Units.radiansToRotations(rotations); }

    public static double degreesToRotations(double degrees) { return Units.degreesToRotations(degrees); }

    public static double rotationsToDegrees(double rotations) { return Units.rotationsToDegrees(rotations); }

    public static double rotationsToRadians(double rotations) { return Units.rotationsToRadians(rotations); }

    public static double rotationsPerMinuteToRadiansPerSecond(double rpm) {
        return Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    public static double radiansPerSecondToRotationsPerMinute(double radiansPerSecond) {
        return Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond);
    }

    public static double millisecondsToSeconds(double milliseconds) {
        return Units.millisecondsToSeconds(milliseconds);
    }

    public static double secondsToMilliseconds(double seconds) { return Units.secondsToMilliseconds(seconds); }

    public static double kilogramsToLbs(double kilograms) { return Units.kilogramsToLbs(kilograms); }

    public static double lbsToKilograms(double lbs) { return Units.lbsToKilograms(lbs); }
}
