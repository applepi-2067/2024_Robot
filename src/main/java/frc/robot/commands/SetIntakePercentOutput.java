package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class SetIntakePercentOutput extends InstantCommand {
  private final Intake m_intake;
  private final double m_percentOutput;

  public SetIntakePercentOutput(double percentOutput) {
    m_percentOutput = percentOutput;

    m_intake = Intake.getInstance();
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setPercentOutput(m_percentOutput);
  }
}    
