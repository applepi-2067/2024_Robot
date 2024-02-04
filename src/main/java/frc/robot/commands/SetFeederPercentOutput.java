package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Feeder;

public class SetFeederPercentOutput extends InstantCommand {
  private final Feeder m_feeder;
  private final double m_percentOutput;

  public SetFeederPercentOutput(double percentOutput) {
    m_percentOutput = percentOutput;

    m_feeder = Feeder.getInstance();
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feeder.setPercentOutput(m_percentOutput);
  }
}
