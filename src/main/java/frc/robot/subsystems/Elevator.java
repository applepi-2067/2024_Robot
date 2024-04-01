package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.utils.Conversions;

import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Elevator extends SubsystemBase implements Loggable {
  private static Elevator instance;
  
  // Physical properties.
  public static final double MAX_EXTENSION_INCHES = 15.0;
  private static final double OUTPUT_SPROCKET_PITCH_RADIUS_INCHES = 1.751 / 2.0;
  private static final double GEAR_RATIO = 9.0 * 4.0;

  private static final double HOLD_POSITION_VOLTAGE = 0.25;

  public static final double ALLOWABLE_ERROR_INCHES = 0.1;

  private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
    .withSupplyCurrentThreshold(25)
    .withSupplyTimeThreshold(0.5)
    .withSupplyCurrentLimit(25);

  // Motors.
  private final TalonFX m_leftMotor;
  private final TalonFX m_rightMotor;

  // PID.
  private static final int K_TIMEOUT_MS = 10;
  private static final double PERCENT_DEADBAND = 0.001;

  private static final Slot0Configs PID_GAINS = new Slot0Configs()
    .withKP(200.0)
    .withKV(1.85);

  private static final double FALCON_500_MAX_SPEED_RPS = 6380.0 / 60.0;
  private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
      .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS * 8.0);

  private static final MotionMagicConfigs SLOW_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS / 25.0)
      .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS / 25.0);

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  private Elevator() {
    m_leftMotor = new TalonFX(RobotMap.canIDs.Elevator.LEFT);
    m_rightMotor = new TalonFX(RobotMap.canIDs.Elevator.RIGHT);
    setUpMotor(m_leftMotor, InvertedValue.Clockwise_Positive);
    setUpMotor(m_rightMotor, InvertedValue.CounterClockwise_Positive);
  }
  
  private void setUpMotor(TalonFX motor, InvertedValue invert) {
    motor.getConfigurator().apply(new TalonFXConfiguration(), K_TIMEOUT_MS);
    motor.setNeutralMode(NeutralModeValue.Brake);

    motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);

    motor.getConfigurator().apply(
      new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO),
      K_TIMEOUT_MS
    );

    motor.getConfigurator().apply(
      new MotorOutputConfigs()
        .withInverted(invert)
        .withDutyCycleNeutralDeadband(PERCENT_DEADBAND)
    );

    motor.getConfigurator().apply(PID_GAINS, K_TIMEOUT_MS);
    motor.getConfigurator().apply(MOTION_MAGIC_CONFIGS, K_TIMEOUT_MS);

    motor.setPosition(0.0, K_TIMEOUT_MS);
  }

  public void setTargetPositionInches(double inches, boolean slow) {
    MotionMagicConfigs speedConfigs = slow ? SLOW_MOTION_MAGIC_CONFIGS : MOTION_MAGIC_CONFIGS;
    m_leftMotor.getConfigurator().apply(speedConfigs);
    m_rightMotor.getConfigurator().apply(speedConfigs);

    double rotations = Conversions.arcLengthToRotations(inches, OUTPUT_SPROCKET_PITCH_RADIUS_INCHES);
    MotionMagicVoltage request = new MotionMagicVoltage(rotations).withFeedForward(HOLD_POSITION_VOLTAGE);
    m_leftMotor.setControl(request);
    m_rightMotor.setControl(request);
  }

  @Log (name = "Position (in)")
  public double getPositionInches() {
    double rotations = m_leftMotor.getPosition().getValueAsDouble();
    return Conversions.rotationsToArcLength(rotations, OUTPUT_SPROCKET_PITCH_RADIUS_INCHES);
  }

  @Log (name="Left current (amps)")
  public double getLeftCurrentAmps() {
    return m_leftMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Log (name="Right current (amps)")
  public double getRightCurrentAmps() {
    return m_rightMotor.getSupplyCurrent().getValueAsDouble();
  }

  // @Config (name = "PIDs")
  // public void setPIDs(double kV, double kP) {
  //   Slot0Configs gains = new Slot0Configs().withKV(kV).withKP(kP);
  //   m_leftMotor.getConfigurator().apply(gains, K_TIMEOUT_MS);
  //   m_rightMotor.getConfigurator().apply(gains, K_TIMEOUT_MS);
  // }
}
