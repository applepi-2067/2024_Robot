package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

  private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
    .withSupplyCurrentThreshold(60)
    .withSupplyTimeThreshold(0.5)
    .withSupplyCurrentLimit(40);

  private final TalonFX m_motor;
  private final DigitalInput m_gamePieceSensor;

  private static final double GEAR_RATIO = 18.0 / 30.0;


  private Feeder() {
    m_gamePieceSensor = new DigitalInput(RobotMap.dios.FEEDER_SENSOR);
    
    m_motor = new TalonFX(RobotMap.canIDs.FEEDER);
    m_motor.getConfigurator().apply(new TalonFXConfiguration());
    m_motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
    m_motor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO));
    m_motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
  }

  public static Feeder getInstance() {
    if (instance == null) {
      instance = new Feeder();
    } 
    return instance;
  }

  public void setPercentOutput(double percentOutput){
    m_motor.set(percentOutput);
  }

  public double getWheelPositionRotations() {
    return m_motor.getPosition().getValueAsDouble();
  }

  @Log (name = "Game piece sensor")
  public boolean gamePieceDetected() {
    return !m_gamePieceSensor.get();
  }

  @Log (name = "Current amps")
  public double getCurrent() {
      return m_motor.getSupplyCurrent().getValueAsDouble();
  }
}
