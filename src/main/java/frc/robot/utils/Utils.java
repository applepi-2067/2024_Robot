package frc.robot.utils;

public class Utils {
    public static boolean withinThreshold(double currPosition, double targetPosition, double allowableErrror) {
        double error = Math.abs(currPosition - targetPosition);
        return (error < allowableErrror);
    }
}
