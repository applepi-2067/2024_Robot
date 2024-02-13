package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;

public class Intake extends SubsystemBase implements Loggable {
    private static Intake instance = null; 

    private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withSupplyCurrentThreshold(60)
        .withSupplyTimeThreshold(0.5)
        .withSupplyCurrentLimit(40);

    private static final Slot0Configs PID_GAINS = new Slot0Configs()  // TODO: tune intake PIDs.
        .withKV(0.0)
        .withKP(0.0);
    
    private static final double FALCON_500_MAX_SPEED_RPS = 6380.0 / 60.0;
    private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
        .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS * 2.0);

    private final TalonFX m_rightMotor;
    private final TalonFX m_leftMotor;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        m_rightMotor = new TalonFX(RobotMap.canIDs.Intake.RIGHT);
        m_leftMotor = new TalonFX(RobotMap.canIDs.Intake.LEFT);

        setUpMotor(m_rightMotor, InvertedValue.Clockwise_Positive);
        setUpMotor(m_leftMotor, InvertedValue.CounterClockwise_Positive);
    }

    private void setUpMotor(TalonFX motor, InvertedValue invert) { 
        motor.getConfigurator().apply(new TalonFXConfiguration());

        motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
        motor.setNeutralMode(NeutralModeValue.Coast);
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(invert));

        motor.getConfigurator().apply(PID_GAINS);
        motor.getConfigurator().apply(MOTION_MAGIC_CONFIGS);
    }

    public void setPercentOutput(double percentOutput) {
        m_rightMotor.setControl(new DutyCycleOut(percentOutput));
        m_leftMotor.setControl(new DutyCycleOut(percentOutput));
    }

    public void setTargetMotorRPM(double motorRPM) {
        m_rightMotor.setControl(new MotionMagicVelocityVoltage(motorRPM));
        m_leftMotor.setControl(new MotionMagicVelocityVoltage(motorRPM));
    }

    @Log (name="Right motor v (rpm))")
    public double getRightMotorVelocityRPM() {
        return m_rightMotor.getVelocity().getValueAsDouble();
    }

    @Log (name="Left motor v (rpm))")
    public double getLeftMotorVelocityRPM() {
        return m_leftMotor.getVelocity().getValueAsDouble();
    }

    @Log (name="Right Current (A)")
    public double getRightCurrent() {
        return m_rightMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Log (name="Left Current (A)")
    public double getleftCurrent() {
        return m_leftMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Config (name="PIDs")
    public void configPIDs(double kV, double kP) {
        Slot0Configs pids = new Slot0Configs().withKV(kV).withKP(kP);
        m_rightMotor.getConfigurator().apply(pids);
        m_leftMotor.getConfigurator().apply(pids);
    }
}
