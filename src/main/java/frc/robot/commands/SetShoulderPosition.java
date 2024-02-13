package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.utils.Utils;

public class SetShoulderPosition extends Command {
    private final Shoulder m_shoulder;

    private final double m_positionDegrees;
    private final boolean m_block;

    public SetShoulderPosition(double positionDegrees, boolean block) {
        m_positionDegrees = positionDegrees;
        m_block = block;

        m_shoulder = Shoulder.getInstance();
        addRequirements(m_shoulder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
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
            Shoulder.PERCENT_ALLOWABLE_ERROR
        );
    }
}
