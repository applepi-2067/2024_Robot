package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinLEDs  extends SubsystemBase{
    private Spark m_blinkinLEDs;
    private double m_currentLEDMode;
    public static BlinkinLEDs instance = null;
    
    public BlinkinLEDs(){
        m_blinkinLEDs = new Spark(8);
    }
    public static BlinkinLEDs getInstance() {
        if (instance == null) {
            instance = new BlinkinLEDs();
        }
        return instance;
    }
    @Override
    public void periodic(){
        if (Feeder.getInstance().gamePieceDetected()){
            
        }else{
            
        }
    }

    public void setLEDManual(double pattern){
        m_currentLEDMode = pattern;
    }
}
