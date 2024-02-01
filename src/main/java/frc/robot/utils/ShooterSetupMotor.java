package frc.robot.utils;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ShooterSetupMotor {
    private static final int K_TIMEOUT_MS = 10;
    private static final Slot0Configs PID_GAINS = new Slot0Configs().withKP(0.01).withKV(0.12);

    private static final double PERCENT_DEADBAND = 0.001;
    private static final double FALCON_500_MAX_SPEED_RPS = 6380.0 / 60.0;

    private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
        .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS * 2.0)
        .withMotionMagicJerk(FALCON_500_MAX_SPEED_RPS * 20.0);

        private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withSupplyCurrentThreshold(60)
        .withSupplyTimeThreshold(0.5)
        .withSupplyCurrentLimit(40);

    public static void setupMotor(TalonFX motor, double GEAR_RATIO){
        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.setNeutralMode(NeutralModeValue.Brake);

        motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS, K_TIMEOUT_MS);

        motor.getConfigurator().apply(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(GEAR_RATIO),
            K_TIMEOUT_MS
        );
        motor.getConfigurator().apply(
            new MotorOutputConfigs().withDutyCycleNeutralDeadband(PERCENT_DEADBAND),
            K_TIMEOUT_MS
        );
            
        motor.getConfigurator().apply(PID_GAINS, K_TIMEOUT_MS);   
        motor.getConfigurator().apply(MOTION_MAGIC_CONFIGS, K_TIMEOUT_MS);
    }
}
