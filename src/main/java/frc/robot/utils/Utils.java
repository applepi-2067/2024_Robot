package frc.robot.utils;

public class Utils {
    public static boolean withinThreshold(double currPosition, double targetPosition, double percentAllowableError) {
        double error = Math.abs(currPosition - targetPosition);
        double allowableError = targetPosition * percentAllowableError;
        return (error < allowableError);
    } 
}
