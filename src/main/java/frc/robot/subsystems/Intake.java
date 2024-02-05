package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;

public class Intake extends SubsystemBase implements Loggable {
    private static Intake instance = null;
    private final TalonFX m_motor;

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
        m_motor = new TalonFX(RobotMap.canIDs.INTAKE);

        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        m_motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
        m_motor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)  // TODO: check inversion.
        );
    }

    public void setPercentOutput(double percentOutput) {
        m_motor.setControl(new DutyCycleOut(percentOutput));
    }

    @Log (name="Current (A)")
    public double getCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }
}
