// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.BlinkinLEDs;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class LEDsEmpty extends InstantCommand {
  private final BlinkinLEDs m_blinkinLEDs;
  private final Feeder m_feeder;
  
  public LEDsEmpty() {
    m_blinkinLEDs = BlinkinLEDs.getInstance();
    m_feeder = Feeder.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_blinkinLEDs.setLEDManual(0.89);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_blinkinLEDs.setLEDManual(0.07);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_feeder.gamePieceDetected()) {
      return true;
    } else {
      return false;

    }
  }
}

