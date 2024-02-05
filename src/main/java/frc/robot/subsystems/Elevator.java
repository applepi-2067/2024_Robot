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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.utils.Conversions;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Elevator extends SubsystemBase implements Loggable {
  private static Elevator instance;
  
  // Physical properties.
  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(16.0);
  private static final double OUTPUT_SPROCKET_PITCH_RADIUS_METERS = Units.inchesToMeters(1.751 / 2.0);
  private static final double GEAR_RATIO = 5.0 * 3.0;

  // Motors.
  private final TalonFX m_masterMotor;
  private final TalonFX m_followerMotor;

  // PID.
  private static final int K_TIMEOUT_MS = 10;
  private static final double PERCENT_DEADBAND = 0.001;

  private static final Slot0Configs PID_GAINS = new Slot0Configs().withKP(0.0).withKV(0.0); // TODO: tune PIDs.

  private static final double FALCON_500_MAX_SPEED_RPS = 100.0;  // 6380 rpm.
  private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
      .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS * 2.0);

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
    m_masterMotor.setNeutralMode(NeutralModeValue.Coast);

    m_masterMotor.getConfigurator().apply(
      new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO),
      K_TIMEOUT_MS
    );

    m_masterMotor.getConfigurator().apply(
      new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)  // TODO: check inversion.
        .withDutyCycleNeutralDeadband(PERCENT_DEADBAND)
    );

    m_masterMotor.getConfigurator().apply(PID_GAINS, K_TIMEOUT_MS);
    m_masterMotor.getConfigurator().apply(MOTION_MAGIC_CONFIGS, K_TIMEOUT_MS);

    m_masterMotor.setPosition(0.0, K_TIMEOUT_MS);
  }

  public void setTargetPositionMeters(double meters) {
    double rotations = Conversions.arcLengthToRotations(meters, OUTPUT_SPROCKET_PITCH_RADIUS_METERS);
    m_masterMotor.setControl(new MotionMagicVoltage(rotations));
  }

  @Log (name = "Position (rotations)")
  public double getPositionRotations() {
    return m_masterMotor.getPosition().getValueAsDouble();
  }

  @Log (name = "Position (meters)")
  public double getPositionMeters() {
    return Conversions.rotationsToArcLength(getPositionRotations(), OUTPUT_SPROCKET_PITCH_RADIUS_METERS);
  }

  @Log (name = "Position (in)")
  public double getPositionInches() {
    return Units.metersToInches(getPositionMeters());
  }
}
