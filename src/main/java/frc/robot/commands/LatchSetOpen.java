// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Latch;;

public class LatchSetOpen extends Command {
  private static Latch m_latch;

  private boolean isOpen;

  public LatchSetOpen(boolean _isOpen) {
    m_latch = Latch.getInstance();
    isOpen = _isOpen;
    addRequirements(m_latch);
  }

  @Override
  public void initialize() {
    m_latch.setOpen(isOpen);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
