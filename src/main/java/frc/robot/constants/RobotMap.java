package frc.robot.constants;


public final class RobotMap {
    public static class canIDs {
        public static class Drivetrain {
            // Follows back left, back right, front left, front right convention.
            public static final int[] DRIVE = {1, 2, 3, 4};

            public static final int[] STEER = {5, 6, 7, 8};
            public static final int[] CANCoder = {1, 2, 3, 4};

            public static final int GYRO = 9;
        }

        public static class Elevator {
            public static final int MASTER = 10;
            public static final int FOLLOWER = 11;
        }
    }
}
