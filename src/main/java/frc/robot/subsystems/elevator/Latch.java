// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Latch extends SubsystemBase {
  // TODO MEASUREMENTS
  private static final double LATCHED_ROTATIONS = 1.0;
  private static final double EXCEEDED_CURRENT = 0.0;
  private static final double GEAR_RATIO = 1.0;

  private static Latch instance = null;
  private final TalonSRX m_motor;

  private boolean isOpen = false;

  private Latch() {
    // TODO CHECK IDS
    m_motor = new TalonSRX(9);
    // m_motor(TalonSRXControlMode.MotionMagic, PID_GAINS);

    // TODO Configure gains
    m_motor.config_kF(0, 0.0);
    m_motor.config_kP(0, 0.0);
    m_motor.config_kD(0, 0.0);
    m_motor.config_kI(0, 0.0);
  }

  public static Latch getInstance() {
    if (instance == null) {
      instance = new Latch();
    }
    return instance;
  }

  // Set Position
  private void setTargetPositionRotations(double rotations) {

    m_motor.set(TalonSRXControlMode.Position, rotations);
  }

  // TODO add max velocity
  // (MAX_VELOCITY  / 600) * (4096 / GEAR_RATIO)

  public void setOpen(boolean _isOpen) {
    isOpen = _isOpen;
    if (isOpen) {
      setTargetPositionRotations(0);
    } else {
      setTargetPositionRotations(LATCHED_ROTATIONS);
    }
  }

  // Get Position
  public double getTargetPosition() {
    return m_motor.getSelectedSensorPosition();
  }

  public boolean isOpen() {
    return isOpen;
  }

  public boolean isCurrentExceeded() {
    return m_motor.getStatorCurrent() > EXCEEDED_CURRENT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
