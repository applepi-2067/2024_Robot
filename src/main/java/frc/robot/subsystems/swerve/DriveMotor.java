package frc.robot.subsystems.swerve;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Conversions;


public class DriveMotor {
    private final TalonFX m_motor;
    private final int m_canID;

    // Motor settings.
    private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withSupplyCurrentThreshold(60)
        .withSupplyTimeThreshold(0.5)
        .withSupplyCurrentLimit(40);
    private static final double PERCENT_DEADBAND = 0.001;
    
    // Conversion constants.
    private static final double GEAR_RATIO = 6.12;
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(4.0 / 2.0);

    // PID.
    private static final int K_TIMEOUT_MS = 10;
    private static final Slot0Configs PID_GAINS = new Slot0Configs().withKP(2.0).withKV(0.8);

    private static final double FALCON_500_MAX_SPEED_RPS = 6380.0 / 60.0;
    private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
        .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS * (100.0 / 15.0));

    // Max speeds.
    public static final double MAX_SPEED_METERS_PER_SEC = 5.0;

    public DriveMotor(int canID) {
        m_canID = canID;

        m_motor = new TalonFX(m_canID);

        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        m_motor.setNeutralMode(NeutralModeValue.Coast);

        m_motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS, K_TIMEOUT_MS);

        m_motor.getConfigurator().apply(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(GEAR_RATIO),
            K_TIMEOUT_MS
        );
        m_motor.getConfigurator().apply(
            new MotorOutputConfigs().withDutyCycleNeutralDeadband(PERCENT_DEADBAND),
            K_TIMEOUT_MS
        );

        m_motor.setPosition(0.0, K_TIMEOUT_MS);

        m_motor.getConfigurator().apply(PID_GAINS, K_TIMEOUT_MS);
        m_motor.getConfigurator().apply(MOTION_MAGIC_CONFIGS, K_TIMEOUT_MS);
    }
    
    public double getPositionMeters() {
        double rotations = m_motor.getPosition().getValueAsDouble();
        double meters = Conversions.rotationsToArcLength(rotations, WHEEL_RADIUS_METERS);
        return meters;
    }
    
    public double getVelocityMetersPerSecond() {
        double velocityRotationsPerSecond = m_motor.getVelocity().getValueAsDouble();
        double velocityMetersPerSecond = Conversions.rotationsToArcLength(velocityRotationsPerSecond, WHEEL_RADIUS_METERS);
        
        // SmartDashboard.putNumber("Drive motor accel (rps^2)" + m_canID, m_motor.getAcceleration().getValueAsDouble());
        // SmartDashboard.putNumber("Drive motor velocity (rps)" + m_canID, m_motor.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("Drive motor velocity (mps)" + m_canID, velocityMetersPerSecond);

        return velocityMetersPerSecond;
    }
    
    public void setTargetVelocityMetersPerSecond(double velocityMetersPerSecond) {
        double velocityRotationsPerSecond = Conversions.arcLengthToRotations(velocityMetersPerSecond, WHEEL_RADIUS_METERS);
        m_motor.setControl(new MotionMagicVelocityVoltage(velocityRotationsPerSecond).withSlot(0));
    }
}
