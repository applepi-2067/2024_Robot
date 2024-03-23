package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Utils;

public class SetShooterVelocity extends Command {
  private final Shooter m_shooter;

  private final double m_velocityRPM;
  private final boolean m_block;

  public SetShooterVelocity(double velocityRPM, boolean block) {
    m_velocityRPM = velocityRPM;
    m_block = block;

    m_shooter = Shooter.getInstance();
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setTargetMotorRPM(m_velocityRPM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_block) {
      return true;
    }
    
    boolean withinThreshold = Utils.withinThreshold(
      m_shooter.getMotorVelocityRPM(),
      m_velocityRPM,
      Shooter.ALLOWABLE_ERROR_RPM
    );
    return withinThreshold && (m_shooter.getMotorVelocityRPM() > m_velocityRPM);
  }
}
