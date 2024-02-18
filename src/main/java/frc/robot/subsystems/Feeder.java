package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.RobotMap;
import frc.robot.utils.Conversions;

public class Feeder extends SubsystemBase implements Loggable {
  private static Feeder instance;

  private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
    .withSupplyCurrentThreshold(60)
    .withSupplyTimeThreshold(0.5)
    .withSupplyCurrentLimit(40);

  private static final double GEAR_RATIO = 30.0 / 18.0;
  private static final double WHEEL_RADIUS_INCHES = 0.75;
  
  private static final Slot0Configs PID_GAINS = new Slot0Configs()
    .withKV(0.19)
    .withKP(0.5);

  private static final double FALCON_500_MAX_SPEED_RPS = 6380.0 / 60.0;
  private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
    .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
    .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS * 8.0)
    .withMotionMagicJerk(FALCON_500_MAX_SPEED_RPS * 8.0 * 4.0);

  private final TalonFX m_motor;
  private final DigitalInput m_gamePieceSensor;

  public static Feeder getInstance() {
    if (instance == null) {
      instance = new Feeder();
    } 
    return instance;
  }

  private Feeder() {
    m_gamePieceSensor = new DigitalInput(RobotMap.dios.FEEDER_SENSOR);
    
    m_motor = new TalonFX(RobotMap.canIDs.FEEDER);
    m_motor.getConfigurator().apply(new TalonFXConfiguration());

    m_motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);

    m_motor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO));
    m_motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    m_motor.getConfigurator().apply(PID_GAINS);
    m_motor.getConfigurator().apply(MOTION_MAGIC_CONFIGS);

    m_motor.setPosition(0.0);
  }

  public void setPercentOutput(double percentOutput){
    m_motor.setControl(new DutyCycleOut(percentOutput));
  }

  public void setTargetVelocityRPM(double rpm) {
    double rps = rpm / 60.0;
    m_motor.setControl(new MotionMagicVelocityVoltage(rps));
  }

  public void setTargetPositionInches(double inches) {
    double rotations = Conversions.arcLengthToRotations(inches, WHEEL_RADIUS_INCHES);
    m_motor.setControl(new MotionMagicVoltage(rotations));
  }

  @Log (name="Velocity (rpm))")
  public double getVelocityRPM() {
    return m_motor.getVelocity().getValueAsDouble() * 60.0;
  }

  @Log (name="Position (in))")
  public double getPositionInches() {
    double rotations = m_motor.getPosition().getValueAsDouble();
    return Conversions.rotationsToArcLength(rotations, WHEEL_RADIUS_INCHES);
  }

  @Log (name = "Game piece sensor")
  public boolean gamePieceDetected() {
    return !m_gamePieceSensor.get();
  }

  @Log (name = "Current amps")
  public double getCurrent() {
      return m_motor.getSupplyCurrent().getValueAsDouble();
  }

  // @Config (name="PIDs")
  // public void configPIDs(double kV, double kP) {
  //   Slot0Configs pids = new Slot0Configs().withKV(kV).withKP(kP);
  //   m_motor.getConfigurator().apply(pids);
  // }
}
