package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;

import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;


public class DriveMotor {
    private final WPI_TalonFX m_motor;

    // Motor settings.
    private static final boolean ENABLE_CURRENT_LIMIT = true;
    private static final double CONTINUOUS_CURRENT_LIMIT_AMPS = 55.0;
    private static final double TRIGGER_THRESHOLD_LIMIT_AMPS = 60.0;
    private static final double TRIGGER_THRESHOLD_TIME_SECONDS = 0.5;

    private static final double PERCENT_DEADBAND = 0.001;
    
    // Conversion constants.
    private static final double TICKS_PER_REV = 2048.0;
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(4.0 / 2.0);

    private static final double GEAR_RATIO = 6.12;

    // PID.
    private static final int K_TIMEOUT_MS = 10;
    private static final int K_PID_LOOP = 0;

    private static final int K_PID_SLOT = 0;
    private static final Gains PID_GAINS = new Gains(0.01, 0.045, 1.0);


    public DriveMotor(int canID, boolean invertMotor) {
        // Motor.
        m_motor = new WPI_TalonFX(canID);
        m_motor.configFactoryDefault();

        m_motor.setNeutralMode(NeutralMode.Coast);  // Coast to avoid tipping.

        // Limit current going to motor.
        SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(
            ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT_AMPS,
            TRIGGER_THRESHOLD_LIMIT_AMPS, TRIGGER_THRESHOLD_TIME_SECONDS
        );
        m_motor.configSupplyCurrentLimit(talonCurrentLimit);

        // Config velocity control.
        m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, K_PID_LOOP, K_TIMEOUT_MS);
        m_motor.setSelectedSensorPosition(0.0);
        m_motor.configNeutralDeadband(PERCENT_DEADBAND, K_TIMEOUT_MS);
        m_motor.setInverted(invertMotor);

        PID_GAINS.setGains(m_motor, K_PID_SLOT, K_PID_LOOP, K_TIMEOUT_MS);
    }

    public void setTargetVelocityMetersPerSecond(double wheelVelocityMetersPerSecond) {
        m_motor.selectProfileSlot(K_PID_SLOT, K_PID_LOOP);
        double velocityMetersPerSecond = wheelVelocityMetersPerSecond * GEAR_RATIO;

        double velocityRPM = Conversions.metersPerSecondToRPM(velocityMetersPerSecond, WHEEL_RADIUS_METERS);
        double velocityTicksPer100ms = Conversions.RPMToTicksPer100ms(velocityRPM, TICKS_PER_REV);
        m_motor.set(TalonFXControlMode.Velocity, velocityTicksPer100ms);
    }

    public double getVelocityMetersPerSecond() {
        double velocityTicksPer100ms = m_motor.getSelectedSensorVelocity(K_PID_LOOP) / GEAR_RATIO;
        double velocityRPM = Conversions.ticksPer100msToRPM(velocityTicksPer100ms, TICKS_PER_REV);
        double velocityMetersPerSecond = Conversions.rpmToMetersPerSecond(velocityRPM, WHEEL_RADIUS_METERS);

        return velocityMetersPerSecond;
    }

    public double getPositionMeters() {
        double ticks = m_motor.getSelectedSensorPosition() / GEAR_RATIO;
        double meters = Conversions.ticksToMeters(ticks, TICKS_PER_REV, WHEEL_RADIUS_METERS);
        return meters;
    }
}
