package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Controllers;

public class Rumble extends SequentialCommandGroup {
    public Rumble(double seconds) {
        Controllers controllers = Controllers.getInstance();

        addCommands(
            new InstantCommand(() -> controllers.setRumble(1.0), controllers),
            new WaitCommand(seconds),
            new InstantCommand(() -> controllers.setRumble(0.0), controllers)
        );
    }
}
