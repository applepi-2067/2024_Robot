package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class SetIntakeVelocity extends InstantCommand {
  private final Intake m_intake;
  private final double m_rpm;

  public SetIntakeVelocity(double rpm) {
    m_rpm = rpm;

    m_intake = Intake.getInstance();
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setTargetVelocityRPM(m_rpm);
  }
}    
