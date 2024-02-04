package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class SetFeederRotations extends Command {
  private final Feeder m_feeder;

  private final double m_rotations;
  private double m_startingPositionRotations;

  public SetFeederRotations(double rotations) {
    m_rotations = rotations;
    
    m_feeder = Feeder.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startingPositionRotations = m_feeder.getWheelPositionRotations();
    m_feeder.setPercentOutput(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.setPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double delta = Math.abs(m_feeder.getWheelPositionRotations() - m_startingPositionRotations); 
    return (delta >= m_rotations);
  }
}
