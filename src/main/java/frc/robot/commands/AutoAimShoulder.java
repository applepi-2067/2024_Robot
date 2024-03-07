package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.utils.Utils;

public class AutoAimShoulder extends Command {
  private final Shoulder m_shoulder;
  private final boolean m_continuous;

  public AutoAimShoulder(boolean continuous) {
    m_shoulder = Shoulder.getInstance();
    addRequirements(m_shoulder);

    m_continuous = continuous;
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
    if (m_continuous) {
      return false;
    }
    
    return Utils.withinThreshold(
      m_shoulder.getPositionDegrees(),
      m_shoulder.getSpeakerScoreAngleDegrees(),
      Shoulder.ALLOWABLE_ERROR_DEGREES
    );
  }
}
