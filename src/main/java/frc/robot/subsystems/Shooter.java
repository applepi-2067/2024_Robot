package frc.robot.subsystems;


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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.utils.Conversions;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


public class Shooter extends SubsystemBase implements Loggable {
    private final TalonFX m_shooterTop;
    private final TalonFX m_shooterBottom;
    private final TalonFX m_feeder;

    private static Shooter instance = null;

    // Motor settings.
    private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withSupplyCurrentThreshold(60)
        .withSupplyTimeThreshold(0.5)
        .withSupplyCurrentLimit(40);
    private static final double PERCENT_DEADBAND = 0.001;

    // PID.
    private static final int K_TIMEOUT_MS = 10;
    private static final Slot0Configs PID_GAINS = new Slot0Configs().withKP(0.02).withKV(0.071);

    private static final double FALCON_500_MAX_SPEED_RPS = 6380.0 / 60.0;
    private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
        .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS * 2.0)
        .withMotionMagicJerk(FALCON_500_MAX_SPEED_RPS * 20.0);

    // Conversion constants.
    private static final double GEAR_RATIO = 18.0 / 30.0;
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0 / 2.0);

    private Shooter() {
        m_shooterTop = new TalonFX(RobotMap.canIDs.Shooter.TOP_SHOOTER);
        m_shooterBottom = new TalonFX(RobotMap.canIDs.Shooter.BOTTOMSHOOTER);
        m_feeder = new TalonFX(RobotMap.canIDs.Shooter.FEEDER);
        setupMotor(m_shooterTop);
        setupMotor(m_shooterBottom);
        setupMotor(m_feeder);
    }

    public static Shooter getInstance(){
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private void setupMotor(TalonFX motor){
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

    public void setFeederSpeed(double speed){
        m_feeder.set(-speed);
    }

    @Log (name = "wheel v (m/s)")
    public double getVelocityMPS() {
        return Conversions.rotationsToArcLength(getWheelVelocityRPS(), WHEEL_RADIUS_METERS);
    }

    @Log (name = "wheel v (rps)")
    public double getWheelVelocityRPS() {
        return m_shooterTop.getVelocity().getValueAsDouble();
    }

    @Log (name = "motor v (rpm)")
    public double getMotorVelocityRPM() {
        return getWheelVelocityRPS() * 60.0 * GEAR_RATIO;
    }

    public void setTargetMotorRPM(double motorRPM) {
        double wheelRPM = motorRPM / GEAR_RATIO;
        double wheelRPS = wheelRPM / 60.0;
        setTargetWheelRPS(wheelRPS);
    }

    public void setTargetWheelRPS(double wheelRPS) {
        m_shooterTop.setControl(new MotionMagicVelocityVoltage(-wheelRPS).withSlot(0));
        m_shooterBottom.setControl(new MotionMagicVelocityVoltage(wheelRPS).withSlot(0));
    }
}
