package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Feeder;

public class SetFeederVelocity extends InstantCommand {
  private final Feeder m_feeder;
  private final double m_rpm;

  public SetFeederVelocity(double rpm) {
    m_rpm = rpm;

    m_feeder = Feeder.getInstance();
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feeder.setTargetVelocityRPM(m_rpm);
  }
}
