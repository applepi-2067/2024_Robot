// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class WaitForFeederRotations extends Command {
  private static Feeder m_feeder;

  private double rotations;
  private double startingPosition;

  public WaitForFeederRotations(double _rotations) {
    rotations = _rotations;
    m_feeder = Feeder.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPosition = m_feeder.getMotorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_feeder.getMotorPosition() - startingPosition) >= rotations;
  }
}
