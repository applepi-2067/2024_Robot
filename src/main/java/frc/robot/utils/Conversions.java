package frc.robot.utils;


public class Conversions {
    public static double radiusToCircumference(double radius) {
        return radius * (2.0 * Math.PI);
    }

    public static double rotationsToArcLength(double rotations, double radius) {
        return rotations * radiusToCircumference(radius);
    }

    public static double arcLengthToRotations(double arcLength, double radius) {
        return arcLength / radiusToCircumference(radius);
    }
}
