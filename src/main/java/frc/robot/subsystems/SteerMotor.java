package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.utils.Conversions;
import frc.robot.utils.Gains;


public class SteerMotor {
    private final WPI_TalonFX m_motor;
    private final CANCoder m_canCoder;

    // Motor settings.
    private static final boolean ENABLE_CURRENT_LIMIT = true;
    private static final double CONTINUOUS_CURRENT_LIMIT_AMPS = 55.0;
    private static final double TRIGGER_THRESHOLD_LIMIT_AMPS = 60.0;
    private static final double TRIGGER_THRESHOLD_TIME_SECONDS = 0.5;

    private static final double PERCENT_DEADBAND = 0.001;
    
    // Conversion constants.
    private static final double TICKS_PER_REV = 2048.0;
    private static final double GEAR_RATIO = 150.0 / 7.0;

    // PID.
    private static final int K_TIMEOUT_MS = 10;
    private static final int K_PID_LOOP = 0;

    private static final int K_PID_SLOT = 0;
    private static final Gains PID_GAINS = new Gains(0.5, 0.0, 1.0);

    private static final int CRUISE_VELOCITY_TICKS_PER_100MS = 20_000;
    private static final int MAX_ACCEL_TICKS_PER_100MS_PER_SEC = CRUISE_VELOCITY_TICKS_PER_100MS * 2;

    // For debugging.
    private int canID;
    private double wheelZeroOffsetDegrees;


    public SteerMotor(int canID, int canCoderID, double wheelZeroOffsetDegrees, boolean invertMotor) {
        this.canID = canID;
        this.wheelZeroOffsetDegrees = wheelZeroOffsetDegrees;

        // Motor.
        m_motor = new WPI_TalonFX(canID);
        m_motor.configFactoryDefault();
        m_motor.setInverted(invertMotor);

        // Coast allows for easier wheel offset tuning.
        // Note that brake mode isn't neeeded b/c pid loop runs in background continuously.
        m_motor.setNeutralMode(NeutralMode.Coast);

        // Limit current going to motor.
        SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(
            ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT_AMPS,
            TRIGGER_THRESHOLD_LIMIT_AMPS, TRIGGER_THRESHOLD_TIME_SECONDS
        );
        m_motor.configSupplyCurrentLimit(talonCurrentLimit);

        // Seed encoder w/ abs encoder (CAN Coder reading) + wheel zero offset.
        m_canCoder = new CANCoder(canCoderID);

        double initPositionTicks = Conversions.degreesToTicks(
            m_canCoder.getAbsolutePosition() - wheelZeroOffsetDegrees,
            TICKS_PER_REV
        ) * GEAR_RATIO;
        m_motor.setSelectedSensorPosition(initPositionTicks);

        // Config position control.
        m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, K_PID_LOOP, K_TIMEOUT_MS);
        m_motor.configNeutralDeadband(PERCENT_DEADBAND, K_TIMEOUT_MS);

        PID_GAINS.setGains(m_motor, K_PID_SLOT, K_PID_LOOP, K_TIMEOUT_MS);
        Gains.configMotionMagic(m_motor, CRUISE_VELOCITY_TICKS_PER_100MS, MAX_ACCEL_TICKS_PER_100MS_PER_SEC, K_TIMEOUT_MS);
    }

    public Rotation2d getPositionRotation2d() {
        double degrees = Conversions.ticksToDegrees(m_motor.getSelectedSensorPosition(), TICKS_PER_REV) / GEAR_RATIO;

        // Log CanCoder position for debugging.
        SmartDashboard.putNumber("Steer CanCoder " + canID + ": ", m_canCoder.getAbsolutePosition() - wheelZeroOffsetDegrees);
        return Rotation2d.fromDegrees(degrees);
    }

    public void setTargetPositionRotation2d(Rotation2d targetPositionRotation2d) {
        double ticks = Conversions.degreesToTicks(targetPositionRotation2d.getDegrees(), TICKS_PER_REV) * GEAR_RATIO;
        m_motor.set(TalonFXControlMode.Position, ticks);
    }
}
