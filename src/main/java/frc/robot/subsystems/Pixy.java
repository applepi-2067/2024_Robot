package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Pixy extends SubsystemBase {
    public static Block initialize() {
        Link link = new SPILink();
        Pixy2 pixy = Pixy2.createInstance(link);
        pixy.init();
        pixy.setLamp((byte) 0, (byte) 1);
        pixy.setLED(0, 0, 0);

        int blockCount = pixy.getCCC().getBlocks(true, Pixy2CCC.CCC_SIG_ALL, 25);
        System.out.println("Blocks " + blockCount);
        if (blockCount <= 0) {
            return null;
        }
        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        Block largestBlock = null;
        for (Block block : blocks) {
            if (largestBlock == null) {
				largestBlock = block;
			} else if (block.getWidth() > largestBlock.getWidth()) {
				largestBlock = block;
			}
        }
        return largestBlock;
    }
}