package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Pixy extends SubsystemBase {
    public static void initialize() {
        Link link = new SPILink();
        Pixy2 pixy = Pixy2.createInstance(link);
        pixy.init();
        pixy.setLamp((byte) 1, (byte) 1);
        pixy.setLED(200, 30, 255);
    }
}