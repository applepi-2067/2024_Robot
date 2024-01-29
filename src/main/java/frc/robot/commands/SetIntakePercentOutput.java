package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Intake;


public class SetIntakePercentOutput extends InstantCommand {
  private final double m_percentOutput;

  public SetIntakePercentOutput(double percentOutput) {
    m_percentOutput = percentOutput;
  }
  
  @Override
  public void initialize() {
    Intake.getInstance().setPercentOutput(m_percentOutput);
  }
}
