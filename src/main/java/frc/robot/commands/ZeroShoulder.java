/* Moves the shoulder up until it the zeroing sensor no longer reads the magnet.
 * Once the magnet is no longer detected, the shoulder should be at a known angle
*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

public class ZeroShoulder extends Command {
    private Shoulder m_shoulder;
    private double ZERO_POSITION_DEGREES = 60;
    private double targetPositionDegrees;

    public ZeroShoulder() {
        m_shoulder = Shoulder.getInstance();
        addRequirements(m_shoulder);
    }

    @Override
    public void initialize() {
        targetPositionDegrees = m_shoulder.getPositionDegrees() - 20.0;
    }

    @Override
    public void execute() {
        m_shoulder.setTargetPositionDegrees(targetPositionDegrees);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the shoulder by setting the target angle to the current angle
        m_shoulder.setTargetPositionDegrees(m_shoulder.getPositionDegrees());

        // Seed the shoulder angle
        m_shoulder.setSeedPositionDegrees(ZERO_POSITION_DEGREES);
    }

    // Returns true when we have raised the arm high enough to lose the sensor
    @Override
    public boolean isFinished() {
        return !m_shoulder.zeroSensorTriggered();
    }
}
