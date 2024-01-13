package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pixy extends SubsystemBase {
    Pixy2 pixy = Pixy2.createInstance(0);
    pixy.init(0);
}