// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Conversions;

public class Climber extends SubsystemBase {
  // TODO MEASUREMENTS
  private static final double METERS_PER_REV = 1.0;
  private static final double CLIMBER_RADIUS_METERS = 1.0;
  private static final double GEAR_RATIO = 1.0;


  private Climber instance;

  private final TalonFX m_motor_1;
  private final TalonFX m_motor_2;

  private final PositionVoltage m_request = new PositionVoltage(0);
  private final Slot0Configs PID_GAINS = new Slot0Configs();
  private final Follower m_follower;

  private Climber() {
    // TODO CHECK IDS
    m_motor_1 = new TalonFX(9);
    m_motor_2 = new TalonFX(10);

    m_follower = new Follower(m_motor_1.getDeviceID(), true);
    m_motor_2.setControl(m_follower);

    // TODO PID
    PID_GAINS.kV = 0.0;
    PID_GAINS.kP = 0.0;
    PID_GAINS.kI = 0.0;
    PID_GAINS.kD = 0.0;

    m_motor_1.getConfigurator().apply(PID_GAINS, 0.050);
    m_motor_1.getConfigurator().apply(new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
      .withSensorToMechanismRatio(GEAR_RATIO)
    );

    // m_motor_2.getConfigurator().apply(PID_GAINS, 0.050);
  }

  public Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  private double metersToMotorRotations(double meters) {
    return meters / METERS_PER_REV;
  }

  private double motorRotationsToMeters(double rotations) {
    return rotations * METERS_PER_REV;
  }

  public double getPositionMeters() {
    double rotations = m_motor_1.getPosition().getValueAsDouble();
    double meters = Conversions.rotationsToArcLength(rotations, CLIMBER_RADIUS_METERS);
    return meters;
  }

  public void setTargetPositionRotations(double rotations) {
    m_motor_1.setControl(m_request.withPosition(rotations));
  }

  public void setTargetPositionMeters(double meters) {
    setTargetPositionRotations(metersToMotorRotations(meters));
  }

  public double getTargetPositionRotations() {
    return m_motor_1.getPosition().getValueAsDouble();
  }

  public double getTargetPositionMeters() {
    return motorRotationsToMeters(getTargetPositionRotations());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
