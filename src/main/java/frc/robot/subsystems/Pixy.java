package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;

public class Pixy extends SubsystemBase {
    
    Pixy2 pixy = new Pixy2(Pixy2.(LinkType));
    pixy.init(0);
}