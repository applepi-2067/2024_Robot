package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;

public class Intake extends SubsystemBase implements Loggable {
    private static Intake instance = null;

    private final TalonFX m_rightMotor;
    private final TalonFX m_leftMotor;

    private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withSupplyCurrentThreshold(60)
        .withSupplyTimeThreshold(0.5)
        .withSupplyCurrentLimit(40);
    
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
        motor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(invert)
        );
    }

    public void setPercentOutput(double percentOutput) {
        m_rightMotor.setControl(new DutyCycleOut(percentOutput));
        m_leftMotor.setControl(new DutyCycleOut(percentOutput));
    }

    @Log (name="Right Current (A)")
    public double getRightCurrent() {
        return m_rightMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Log (name="Left Current (A)")
    public double getleftCurrent() {
        return m_leftMotor.getSupplyCurrent().getValueAsDouble();
    }
}
