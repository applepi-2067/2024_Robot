package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BlinkinLEDs;
import frc.robot.subsystems.Feeder;

public class LEDsEmpty extends InstantCommand {
    private final BlinkinLEDs m_blinkinLEDs;
    private final Feeder m_feeder;

    public LEDsEmpty(){
        m_blinkinLEDs = BlinkinLEDs.getInstance();
        m_feeder = Feeder.getInstance();

    }
    
}
