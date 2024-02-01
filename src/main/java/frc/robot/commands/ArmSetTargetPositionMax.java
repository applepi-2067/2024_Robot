// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Arm;

public class ArmSetTargetPositionMax extends Command {
  private static Arm m_elevator;

  public ArmSetTargetPositionMax() {
    m_elevator = Arm.getInstance();
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setTargetPositionMax();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
