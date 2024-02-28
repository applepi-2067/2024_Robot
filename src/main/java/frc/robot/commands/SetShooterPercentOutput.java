package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class SetShooterPercentOutput extends InstantCommand {
  private final double m_percentOutput;
  private final Shooter m_shooter;

  public SetShooterPercentOutput(double percentOutput) {
    m_percentOutput = percentOutput;

    m_shooter = Shooter.getInstance();
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setPercentOutput(m_percentOutput);
  }
}
