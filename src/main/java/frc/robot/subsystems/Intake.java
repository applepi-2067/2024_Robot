package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


public class Intake extends SubsystemBase implements Loggable {
    private static Intake instance = null;

    private static final boolean INVERT_MOTOR = true;
    private static final int CURRENT_LIMIT_AMPS = 60;

    private final CANSparkMax m_motor;
    
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        m_motor = new CANSparkMax(RobotMap.canIDs.INTAKE, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
        m_motor.setInverted(INVERT_MOTOR);
    }

    public void setPercentOutput(double percentOutput) {
        m_motor.set(percentOutput);
    }

    @Log (name="intakecurrent")
    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }
}
