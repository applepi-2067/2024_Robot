package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinLeds extends SubsystemBase {
   private Spark m_blinkinLeds;
   private double m_currentLedMode;


   public BlinkinLeds(){
    m_blinkinLeds = new Spark(8);
    m_currentLedMode = LEDMode.rainbow.value;
      
   }

   public void setMode(LEDMode mode){
      m_currentLedMode = mode.value;
   }
   
   public enum LEDMode{
      rainbow(-0.97);

      public final double value;
      private LEDMode(double value){
      this.value = value;
      }
   }

   @Override
   public void periodic() {
     m_blinkinLeds.set(m_currentLedMode);
   }

}
