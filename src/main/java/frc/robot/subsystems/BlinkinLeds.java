package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinLeds extends SubsystemBase {
   private Spark m_blinkinLeds;
   private double m_currentLedMode;
   public static BlinkinLeds instance = null;

   public BlinkinLeds(){
    m_blinkinLeds = new Spark(8);
    m_currentLedMode = LEDMode.orange.value;
      
   }

   public void setMode(LEDMode mode){
      m_currentLedMode = mode.value;
   }
   
   public enum LEDMode {
      //colors
      rainbow(-0.97), blue(0.00), white(0.00), green(0.00), orange(0.65), 
      //paterns
      blinking(0.07), solid(0.00);

      public final double value;
      private LEDMode(double value){
      this.value = value;
      }
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
