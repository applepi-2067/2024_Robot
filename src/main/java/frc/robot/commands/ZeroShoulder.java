/* Moves the shoulder up until it the zeroing sensor no longer reads the magnet.
 * Once the magnet is no longer detected, the shoulder should be at a known angle
*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

public class ZeroShoulder extends Command {
    private static final double ZERO_POSITION_DEGREES = 56.75;

    private final Shoulder m_shoulder;

    public ZeroShoulder() {
        m_shoulder = Shoulder.getInstance();
        addRequirements(m_shoulder);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_shoulder.setPercentOutput(-0.1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the shoulder
        m_shoulder.setPercentOutput(0.0);

        // Seed the shoulder angle
        m_shoulder.setSeedPositionDegrees(ZERO_POSITION_DEGREES);
    }

    // Returns true when we have raised the arm high enough to lose the sensor
    @Override
    public boolean isFinished() {
        return !m_shoulder.zeroSensorTriggered();  // TODO: failsafe if misses magnet.
    }
}
