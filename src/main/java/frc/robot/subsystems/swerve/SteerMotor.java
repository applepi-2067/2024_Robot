package frc.robot.subsystems.swerve;


import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;


public class SteerMotor {
    private final TalonFX m_motor;
    private final CANcoder m_canCoder;

    // private final int canCoderID;
    private final double wheelZeroOffsetDegrees;

    // Motor settings.
    private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withSupplyCurrentThreshold(60)
        .withSupplyTimeThreshold(0.5)
        .withSupplyCurrentLimit(40);
    private static final double PERCENT_DEADBAND = 0.001;
    
    // Conversion constants.
    private static final double GEAR_RATIO = 150.0 / 7.0;

    // PID.
    private static final int K_TIMEOUT_MS = 10;
    private static final Slot0Configs PID_GAINS = new Slot0Configs().withKP(100.0);

    private static final double FALCON_500_MAX_SPEED_RPS = 6380.0 / 60;
    private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
        .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS * 2.0);

    public SteerMotor(int canID, int canCoderID, double wheelZeroOffsetDegrees) {
        // this.canCoderID = canCoderID;
        this.wheelZeroOffsetDegrees = wheelZeroOffsetDegrees;

        // Motor.
        m_motor = new TalonFX(canID);
        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        m_motor.setNeutralMode(NeutralModeValue.Coast);  // Coast for wheel offset tuning. Background PID loop holds position.
        m_motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS, K_TIMEOUT_MS);

        m_motor.getConfigurator().apply(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(GEAR_RATIO),
            K_TIMEOUT_MS
        );
        m_motor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withDutyCycleNeutralDeadband(PERCENT_DEADBAND)
                .withInverted(InvertedValue.CounterClockwise_Positive),
            K_TIMEOUT_MS
        );

        // Enable PID wrapping.
        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        closedLoopGeneralConfigs.ContinuousWrap = true;
        m_motor.getConfigurator().apply(closedLoopGeneralConfigs);

        // Seed rotor w/ abs CANCoder reading.
        m_canCoder = new CANcoder(canCoderID);
        m_motor.setPosition(getCANCoderPositionRotation2d().getRotations(), K_TIMEOUT_MS);

        // Config position control.
        m_motor.getConfigurator().apply(PID_GAINS, K_TIMEOUT_MS);
        m_motor.getConfigurator().apply(MOTION_MAGIC_CONFIGS, K_TIMEOUT_MS);
    }

    public Rotation2d getCANCoderPositionRotation2d() {
        double rotations = m_canCoder.getAbsolutePosition().getValueAsDouble() - (wheelZeroOffsetDegrees / 360.0);
        return Rotation2d.fromRotations(rotations);
    }

    public Rotation2d getPositionRotation2d() {        
        // Log CANCoder position for debugging.
        // SmartDashboard.putNumber("CANCoder " + canCoderID + ": ", getCANCoderPositionRotation2d().getDegrees());

        double rotations = m_motor.getPosition().getValueAsDouble() % 1.0;
        return Rotation2d.fromRotations(rotations);
    }

    public void setTargetPositionRotation2d(Rotation2d targetPositionRotation2d) {
        double rotations = targetPositionRotation2d.getRotations();
        m_motor.setControl(new MotionMagicVoltage(rotations).withSlot(0));
    }
}
