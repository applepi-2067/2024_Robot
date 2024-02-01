// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.utils.ShooterSetupMotor;

public class Feeder extends SubsystemBase {
  private final TalonFX m_feeder;
  private final DigitalInput m_gamePieceSensor;

  private static final double GEAR_RATIO = 18.0 / 30.0;

  private static Feeder instance;

  private Feeder() {
    m_feeder = new TalonFX(RobotMap.canIDs.Shooter.FEEDER);
    m_gamePieceSensor = new DigitalInput(RobotMap.canIDs.Shooter.SENSOR);
    ShooterSetupMotor.setupMotor(m_feeder, GEAR_RATIO);

    m_feeder.setInverted(true);
  }

  public static Feeder getInstance() {
    if (instance == null) {
      instance = new Feeder();
    } 
    return instance;
  }

  public void setFeederSpeed(double speed){
    m_feeder.set(speed);
  }

  public double getMotorPosition() {
    return m_feeder.getPosition().getValueAsDouble() / GEAR_RATIO; // TODO CHECK
  }

  public boolean isGamePieceDetected() {
    return !m_gamePieceSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
