package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class SetElevatorVelocity extends InstantCommand {
  private final Elevator m_elevator;
  private final double m_rpm;

  public SetElevatorVelocity(double rpm) {
    m_rpm = rpm;

    m_elevator = Elevator.getInstance();
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setTargetVelocityRPM(m_rpm);
  }
}    