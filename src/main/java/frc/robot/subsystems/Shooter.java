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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.constants.RobotMap;
import frc.robot.utils.Utils;


public class Shooter extends SubsystemBase implements Loggable {
    private static Shooter instance = null;

    // PIDs.
    private static final int K_TIMEOUT_MS = 10;
    private static final Slot0Configs PID_GAINS = new Slot0Configs()
        .withKP(0.15)
        .withKV(0.125);

    private static final double PERCENT_DEADBAND = 0.001;
    private static final double FALCON_500_MAX_SPEED_RPS = 6380.0 / 60.0;

    private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
        .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS * 4.0)
        .withMotionMagicJerk(FALCON_500_MAX_SPEED_RPS * 20.0);

    private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()  // TODO: make shooter faster.
        .withSupplyCurrentThreshold(60)
        .withSupplyTimeThreshold(0.5)
        .withSupplyCurrentLimit(40);
    
    // Speeds for shooting.
    public static final double SHOOTING_SPEED_RPM = 4_200.0;
    public static final double CLOSE_SHOOTING_SPEED_RPM = 3_000.0;
    public static final double ALLOWABLE_ERROR_RPM = 50.0;

    private final TalonFX m_topMotor;
    private final TalonFX m_bottomMotor;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        m_topMotor = new TalonFX(RobotMap.canIDs.Shooter.TOP_SHOOTER);
        m_bottomMotor = new TalonFX(RobotMap.canIDs.Shooter.BOTTOM_SHOOTER);
        setupMotor(m_topMotor, InvertedValue.CounterClockwise_Positive);
        setupMotor(m_bottomMotor, InvertedValue.Clockwise_Positive);
    }

    public void setupMotor(TalonFX motor, InvertedValue invert){
        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.setNeutralMode(NeutralModeValue.Brake);

        motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS, K_TIMEOUT_MS);

        motor.getConfigurator().apply(
            new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor),
            K_TIMEOUT_MS
        );
        motor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withDutyCycleNeutralDeadband(PERCENT_DEADBAND)
                .withInverted(invert),
            K_TIMEOUT_MS
        );
            
        motor.getConfigurator().apply(PID_GAINS, K_TIMEOUT_MS);   
        motor.getConfigurator().apply(MOTION_MAGIC_CONFIGS, K_TIMEOUT_MS);
    }

    public void setPercentOutput(double percentOutput) {
        m_topMotor.setControl(new DutyCycleOut(percentOutput));
        m_bottomMotor.setControl(new DutyCycleOut(percentOutput));
    }

    public void setTargetMotorRPM(double motorRPM) {
        double motorRPS = motorRPM / 60.0;

        m_topMotor.setControl(new MotionMagicVelocityVoltage(motorRPS).withSlot(0));
        m_bottomMotor.setControl(new MotionMagicVelocityVoltage(motorRPS).withSlot(0));
    }

    @Log (name = "motor v (rpm)")
    public double getMotorVelocityRPM() {
        return (getTopMotorVelocityRPM() + getBottomMotorVelocityRPM()) / 2.0;
    }

    @Log (name = "Top motor v (rpm)")
    public double getTopMotorVelocityRPM() {
        return m_topMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    @Log (name = "Bottom motor v (rpm)")
    public double getBottomMotorVelocityRPM() {
        return m_bottomMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    @Log (name = "Shooting v reached")
    public boolean shootingVelocityReached() {
        return Utils.withinThreshold(getMotorVelocityRPM(), SHOOTING_SPEED_RPM, ALLOWABLE_ERROR_RPM) && (getMotorVelocityRPM() > SHOOTING_SPEED_RPM);
    }

    @Log (name = "Current amps top")
    public double getCurrentTop() {
        return m_topMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Log (name = "Current amps bottom")
    public double getCurrentBottom() {
        return m_bottomMotor.getSupplyCurrent().getValueAsDouble();
    }

    // @Config
    // public void setPIDs(double kV, double kP) {
    //     Slot0Configs newPIDs = new Slot0Configs().withKV(kV).withKP(kP);
    //     m_topMotor.getConfigurator().apply(newPIDs, K_TIMEOUT_MS);
    //     m_bottomMotor.getConfigurator().apply(newPIDs, K_TIMEOUT_MS);
    // }
}
