package frc.robot.utils;

import java.text.DecimalFormat;

import edu.wpi.first.math.geometry.Pose2d;

public class Utils {
    public static boolean withinThreshold(double currPosition, double targetPosition, double allowableErrror) {
        double error = Math.abs(currPosition - targetPosition);
        return (error < allowableErrror);
    }

    public static String getPose2dDescription(Pose2d pose) {
        DecimalFormat rounder = new DecimalFormat("0.0000");
        
        String poseString = "x (m)=" + rounder.format(pose.getX()) + "    ";
        poseString += "y (m)=" + rounder.format(pose.getY()) + "    ";
        poseString += "rotation (deg)=" + rounder.format(pose.getRotation().getDegrees());
        return poseString;
    }
}
