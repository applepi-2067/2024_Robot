package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.utils.Utils;

public class SetShoulderPosition extends Command {
    private final Shoulder m_shoulder;

    private double m_positionDegrees;
    private final boolean m_block;
    private final boolean m_useDevAngle;

    public SetShoulderPosition(double positionDegrees, boolean block) {
        this(positionDegrees, block, false);
    }

    public SetShoulderPosition(double positionDegrees, boolean block, boolean useDevAngle) {
        m_positionDegrees = positionDegrees;
        m_block = block;
        m_useDevAngle = useDevAngle;

        m_shoulder = Shoulder.getInstance();
        addRequirements(m_shoulder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_useDevAngle) {
            m_positionDegrees = SmartDashboard.getNumber("Dev angle degrees", 136.0);
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
