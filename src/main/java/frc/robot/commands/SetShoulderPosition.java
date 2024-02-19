package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.utils.Utils;

public class SetShoulderPosition extends Command {
    private final Shoulder m_shoulder;

    private double m_positionDegrees;
    private final boolean m_autoAim;
    private final boolean m_block;

    public SetShoulderPosition(double positionDegrees, boolean block) {
        this(positionDegrees, false, block);
    }

    public SetShoulderPosition(boolean autoAim, boolean block) {
        this(0.0, autoAim, block);
    }

    private SetShoulderPosition(double positionDegrees, boolean autoAim, boolean block) {
        m_positionDegrees = positionDegrees;
        m_autoAim = autoAim;
        m_block = block;

        m_shoulder = Shoulder.getInstance();
        addRequirements(m_shoulder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_autoAim) {
            m_positionDegrees = m_shoulder.getSpeakerScoreAngleDegrees();
        }

        m_shoulder.setTargetPositionDegrees(m_positionDegrees);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!m_block) {
            return true;
        }

        return Utils.withinThreshold(
            m_shoulder.getPositionDegrees(),
            m_positionDegrees,
            Shoulder.ALLOWABLE_ERROR_DEGREES
        );
    }
}
