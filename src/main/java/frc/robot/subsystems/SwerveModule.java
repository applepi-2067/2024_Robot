package frc.robot.subsystems;


import java.text.DecimalFormat;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.constants.RobotMap;


public class SwerveModule {
    // Reported CANCoder abs encoder position at wheel zero.
    private static final double[] STEER_WHEEL_ZERO_OFFSET_DEGREES = {25.05, 246.27, 77.43, 213.93};

    private static final DecimalFormat rounder = new DecimalFormat("0.0000");

    private final DriveMotor m_driveMotor;
    private final SteerMotor m_steerMotor;

    private final int location;
    
    public SwerveModule(int location) {
        this.location = location;

        m_driveMotor = new DriveMotor(RobotMap.canIDs.Drivetrain.DRIVE[location]);
        m_steerMotor = new SteerMotor(
            RobotMap.canIDs.Drivetrain.STEER[location],
            RobotMap.canIDs.Drivetrain.DRIVE[location],
            STEER_WHEEL_ZERO_OFFSET_DEGREES[location]
        );
    }

    public void setTargetState(SwerveModuleState targetState) {
        // Do nothing if target state is to not move.
        if (Math.abs(targetState.speedMetersPerSecond) < 0.01) {
            m_driveMotor.setTargetVelocityMetersPerSecond(0.0);
            return;
        }

        // Optimize state, inverting steer and drive rotations for shortest turn.
        targetState = SwerveModuleState.optimize(targetState, m_steerMotor.getPositionRotation2d());

        // Set steer and drive motors to targets.
        m_steerMotor.setTargetPositionRotation2d(targetState.angle);
        m_driveMotor.setTargetVelocityMetersPerSecond(targetState.speedMetersPerSecond);
    }

    public SwerveModuleState getState() {
        SwerveModuleState state = new SwerveModuleState(
            m_driveMotor.getVelocityMetersPerSecond(),
            m_steerMotor.getPositionRotation2d()
        );
        return state;
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition position = new SwerveModulePosition(
            m_driveMotor.getPositionMeters(),
            m_steerMotor.getPositionRotation2d()
        );
        return position;
    }

    public String toString() {
        String desc = "Loc " + location + ": ";
        
        SwerveModuleState currState = getState();
        desc += "v (m/s)=" + rounder.format(currState.speedMetersPerSecond) + "   ";
        desc += "angle (deg)=" + rounder.format(currState.angle.getDegrees());
        
        return desc;
    }
}
