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
    private static Pixy instance = null;
    private Pixy2 m_pixy;
    private Link m_link;

    public static Pixy getInstance() {
        if (instance == null) {
            instance = new Pixy();
        }
        return instance;
    }

    private Pixy() {
        m_link = new SPILink();
        m_pixy = Pixy2.createInstance(m_link);
        initialize();
    }

    public void initialize() {
        m_pixy.init();
        m_pixy.setLamp((byte) 0, (byte) 1);
        m_pixy.setLED(0, 0, 0);
    }

    public void setLEDColor(int r, int g, int b) {
        m_pixy.setLamp((byte) 0, (byte) 1);
        m_pixy.setLED(r, g, b);
    }

    public int countBlocks() {
        int blockCount = m_pixy.getCCC().getBlocks(true, Pixy2CCC.CCC_SIG_ALL, 25);
        return blockCount;
    }

    public Block findBiggestBlock() {
        ArrayList<Block> blocks = m_pixy.getCCC().getBlockCache();
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

    @Override
    public void periodic() {
        System.out.println("Found " + countBlocks() + " blocks");
        if (!(findBiggestBlock() == null)) {
            System.out.println(findBiggestBlock().getX());
        } else {
            System.out.println("No blocks found :(");
        }
    }
}