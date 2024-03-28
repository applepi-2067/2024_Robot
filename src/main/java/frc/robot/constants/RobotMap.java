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
        
        public static class Elevator {
            public static final int LEFT = 9;
            public static final int RIGHT = 10;
        }
        public static class Shooter {
            public static final int TOP_SHOOTER = 11;
            public static final int BOTTOM_SHOOTER = 12;
        }

        public static final int FEEDER = 13;

        
        public static class Intake {
            public static final int RIGHT = 14;
            public static final int LEFT = 15;
        }
        
        public static final int SHOULDER = 16;
    }

    public static class dios {
        public static final int FEEDER_SENSOR = 0;
    }

    public static class pwms {
        public static final int LIGHTS = 8;
    }
}
