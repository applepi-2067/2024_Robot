package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinLeds extends SubsystemBase {
   private Spark m_blinkinLeds;
   private double m_currentLedMode;
   public static BlinkinLeds instance = null;

   public BlinkinLeds(){
    m_blinkinLeds = new Spark(8);
      
   }
   public static BlinkinLeds getInstance() {
      if (instance == null) {
        instance = new BlinkinLeds();
      }
      return instance;
    }

   @Override
   public void periodic() {
     m_blinkinLeds.set(m_currentLedMode);
   }


   public void setLEDManual(double pattern) {
      m_currentLedMode = pattern;
   }
}
