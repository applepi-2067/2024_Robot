package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Utils;

public class SetElevatorPosition extends Command {
    private final Elevator m_elevator;

    private final double m_positionInches;
    private final boolean m_block;

    public SetElevatorPosition(double positionInches, boolean block) {
        m_positionInches = positionInches;
        m_block = block;

        m_elevator = Elevator.getInstance();
        addRequirements(m_elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elevator.setTargetPositionInches(m_positionInches);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!m_block) {
            return true;
        }

        return Utils.withinThreshold(
            m_elevator.getPositionInches(),
            m_positionInches,
            Elevator.PERCENT_ALLOWABLE_ERROR
        );
    }
}
