package frc.robot.constants;


public final class RobotMap {
    public static class canIDs {
        public static class Drivetrain {
            // Follows back left, back right, front left, front right convention.
            public static final int[] DRIVE = {1, 2, 3, 4};

            public static final int[] STEER = {5, 6, 7, 8};
            public static final int[] CANCoder = {1, 2, 3, 4};

            public static final int GYRO = 1;
        }
        public static class Shooter {
            public static final int TOP_SHOOTER = 9;
            public static final int BOTTOMSHOOTER = 10;
            public static final int FEEDER = 11;
            public static final int SENSOR = 0;
        }
    }
}
