package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BlinkinLEDs;

public class SetLEDPattern extends InstantCommand {
  private final BlinkinLEDs m_blinkinLEDs;

  private final double m_pattern;
  
  public SetLEDPattern(double pattern) {
    m_blinkinLEDs = BlinkinLEDs.getInstance();

    m_pattern = pattern;
  }

  @Override
  public void initialize() {
    m_blinkinLEDs.setLEDManual(m_pattern);
  }
}