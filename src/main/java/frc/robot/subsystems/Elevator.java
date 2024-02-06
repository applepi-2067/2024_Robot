package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.utils.Conversions;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Elevator extends SubsystemBase implements Loggable {
  private static Elevator instance;
  
  // Physical properties.
  public static final double MAX_EXTENSION_INCHES = 16.0;
  private static final double OUTPUT_SPROCKET_PITCH_RADIUS_INCHES = 1.751 / 2.0;
  private static final double GEAR_RATIO = 5.0 * 3.0;

  private static final double HOLD_POSITION_VOLTAGE = 0.25;  // TODO: find voltage to pull robot up.

  // Motors.
  private final TalonFX m_masterMotor;
  private final TalonFX m_followerMotor;

  // PID.
  private static final int K_TIMEOUT_MS = 10;
  private static final double PERCENT_DEADBAND = 0.001;

  private static final Slot0Configs PID_GAINS = new Slot0Configs().withKP(600.0).withKV(2.0); // TODO: tune PIDs.

  private static final double FALCON_500_MAX_SPEED_RPS = 100.0;  // 6380 rpm.
  private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
      .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS / 2.0);

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  private Elevator() {
    m_masterMotor = new TalonFX(RobotMap.canIDs.Elevator.MASTER);
    m_followerMotor = new TalonFX(RobotMap.canIDs.Elevator.FOLLOWER);

    Follower follower = new Follower(m_masterMotor.getDeviceID(), true);
    m_followerMotor.setControl(follower);

    m_masterMotor.getConfigurator().apply(new TalonFXConfiguration(), K_TIMEOUT_MS);
    m_masterMotor.setNeutralMode(NeutralModeValue.Brake);

    m_masterMotor.getConfigurator().apply(
      new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO),
      K_TIMEOUT_MS
    );

    m_masterMotor.getConfigurator().apply(
      new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withDutyCycleNeutralDeadband(PERCENT_DEADBAND)
    );

    m_masterMotor.getConfigurator().apply(PID_GAINS, K_TIMEOUT_MS);
    m_masterMotor.getConfigurator().apply(MOTION_MAGIC_CONFIGS, K_TIMEOUT_MS);

    m_masterMotor.setPosition(0.0, K_TIMEOUT_MS);
  }

  public void setTargetPositionInches(double inches) {
    double rotations = Conversions.arcLengthToRotations(inches, OUTPUT_SPROCKET_PITCH_RADIUS_INCHES);
    MotionMagicVoltage request = new MotionMagicVoltage(rotations);

    if (inches > getPositionInches()) {
      request = request.withFeedForward(HOLD_POSITION_VOLTAGE);
    }
    m_masterMotor.setControl(request);
  }

  @Log (name = "Position (in)")
  public double getPositionInches() {
    double rotations = m_masterMotor.getPosition().getValueAsDouble();
    return Conversions.rotationsToArcLength(rotations, OUTPUT_SPROCKET_PITCH_RADIUS_INCHES);
  }

  @Config (name = "PIDs")
  public void setPIDs(double kV, double kP) {
    Slot0Configs gains = new Slot0Configs().withKV(kV).withKP(kP);
    m_masterMotor.getConfigurator().apply(gains, K_TIMEOUT_MS);
  }
}
