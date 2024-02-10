package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;

public class Feeder extends SubsystemBase implements Loggable {
  private static Feeder instance;

  private final TalonFX m_feeder;
  private final DigitalInput m_gamePieceSensor;

  private static final double GEAR_RATIO = 18.0 / 30.0;


  private Feeder() {
    m_gamePieceSensor = new DigitalInput(RobotMap.dios.FEEDER_SENSOR);
    
    m_feeder = new TalonFX(RobotMap.canIDs.FEEDER);
    m_feeder.getConfigurator().apply(new TalonFXConfiguration());
    m_feeder.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO));
    m_feeder.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
  }

  public static Feeder getInstance() {
    if (instance == null) {
      instance = new Feeder();
    } 
    return instance;
  }

  public void setPercentOutput(double percentOutput){
    m_feeder.set(percentOutput);
  }

  public double getWheelPositionRotations() {
    return m_feeder.getPosition().getValueAsDouble();
  }

  @Log (name = "Game piece sensor")
  public boolean gamePieceDetected() {
    return !m_gamePieceSensor.get();
  }
}
