package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;

public class Lights extends SubsystemBase{
    public static Lights instance = null;
    
    private Spark m_blinkinLEDs;
    
    public Lights(){
        m_blinkinLEDs = new Spark(RobotMap.pwms.LIGHTS);
    }
    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }
        return instance;
    }

    @Override
    public void periodic(){
        if (Feeder.getInstance().gamePieceDetected()){
            m_blinkinLEDs.set(0.93);
        }
        else {
            m_blinkinLEDs.set(0.0);
        }
    }
}
