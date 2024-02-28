package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

public class AutoAimShoulder extends Command {
  private final Shoulder m_shoulder;

  public AutoAimShoulder() {
    m_shoulder = Shoulder.getInstance();
    addRequirements(m_shoulder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Continuosly aim at speaker.
    double speakerScoreAngle = m_shoulder.getSpeakerScoreAngleDegrees();
    m_shoulder.setTargetPositionDegrees(speakerScoreAngle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // Never finish. Auto-aim until interrupted by another command.
    return false;
  }
}
