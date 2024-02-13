package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.constants.RobotMap;


public class Shooter extends SubsystemBase implements Loggable {
    private static Shooter instance = null;

    private final TalonFX m_shooterTop;
    private final TalonFX m_shooterBottom;

    // PIDs.
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
    
    // Speeds for shooting.
    public static final double SHOOTING_SPEED_RPM = 3_800.0;
    private static final double SHOOTING_SPEED_TOLERANCE_PERCENT = 0.05;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        m_shooterTop = new TalonFX(RobotMap.canIDs.Shooter.TOP_SHOOTER);
        m_shooterBottom = new TalonFX(RobotMap.canIDs.Shooter.BOTTOM_SHOOTER);
        setupMotor(m_shooterTop);
        setupMotor(m_shooterBottom);
    }

    public void setupMotor(TalonFX motor){
        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.setNeutralMode(NeutralModeValue.Brake);

        motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS, K_TIMEOUT_MS);

        motor.getConfigurator().apply(
            new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor),
            K_TIMEOUT_MS
        );
        motor.getConfigurator().apply(
            new MotorOutputConfigs().withDutyCycleNeutralDeadband(PERCENT_DEADBAND),
            K_TIMEOUT_MS
        );
            
        motor.getConfigurator().apply(PID_GAINS, K_TIMEOUT_MS);   
        motor.getConfigurator().apply(MOTION_MAGIC_CONFIGS, K_TIMEOUT_MS);
    }

    public void setPercentOutput(double percentOutput) {
        m_shooterTop.setControl(new DutyCycleOut(-1.0 * percentOutput));
        m_shooterBottom.setControl(new DutyCycleOut(percentOutput));
    }

    @Log (name = "motor v (rpm)")
    public double getMotorVelocityRPM() {
        return m_shooterTop.getVelocity().getValueAsDouble() * 60.0;
    }

    @Log (name = "Shooting v reached")
    public boolean shootingVelocityReached() {
        return velocityReached(SHOOTING_SPEED_RPM);
    }

    @Log (name = "Current amps top")
    public double getCurrentTop() {
        return m_shooterTop.getSupplyCurrent().getValueAsDouble();
    }

    @Log (name = "Current amps bottom")
    public double getCurrentBottom() {
        return m_shooterBottom.getSupplyCurrent().getValueAsDouble();
    }

    public boolean velocityReached(double targetVelocityRPM) {
        double error = Math.abs(Math.abs(getMotorVelocityRPM()) - targetVelocityRPM);
        return error < (targetVelocityRPM * SHOOTING_SPEED_TOLERANCE_PERCENT);
    }

    public void setTargetMotorRPM(double motorRPM) {
        double motorRPS = motorRPM / 60.0;

        m_shooterTop.setControl(new MotionMagicVelocityVoltage(-1.0 * motorRPS).withSlot(0));
        m_shooterBottom.setControl(new MotionMagicVelocityVoltage(motorRPS).withSlot(0));
    }
}
