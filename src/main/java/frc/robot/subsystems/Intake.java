package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;


public class Intake extends SubsystemBase {
    private static Intake instance = null;

    private static final boolean INVERT_MOTOR = false;

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
        m_motor.setInverted(INVERT_MOTOR);
    }

    public void setPercentOutput(double percentOutput) {
        m_motor.set(percentOutput);
    }
}
