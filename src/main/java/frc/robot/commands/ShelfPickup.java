package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;

public class ShelfPickup extends SequentialCommandGroup {
    public ShelfPickup() {
        addCommands(
            new ParallelCommandGroup(
                new SetElevatorPosition(0.0, false),
                new SetShoulderPosition(0, false),
                new SetShooterVelocity(-1.0, false)
            ),

            new WaitUntilCommand(Feeder.getInstance()::gamePieceDetected),

            new ParallelCommandGroup(
                new SetElevatorPosition(0.0, false),
                new SetShoulderPosition(0.0, false),
                new SetShooterVelocity(0.0, false)
            ),

            //get note into feeder
            new SequentialCommandGroup(
                new SetFeederVelocity(-200.0),
                wait(0.2)
            )
        );
    }

    private Command wait(double d) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'wait'");
    }
}
