package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shoulder;

public class FeedShot extends SequentialCommandGroup {
  public FeedShot() {
    addCommands(
        new ParallelCommandGroup(
            new SetShoulderPosition(45.0, true),  // TODO: check feed angle.
            new SetShooterVelocity(Shooter.CLOSE_SHOOTING_SPEED_RPM, true)
        ),
            
        new SetFeederVelocity(3_000.0),
        new WaitUntilCommand(() -> !Feeder.getInstance().gamePieceDetected()),
        new WaitCommand(0.5),

        new SetFeederVelocity(0.0),
        new SetShooterVelocity(0.0, false),
        new SetShoulderPosition(Shoulder.MIN_ANGLE_DEGREES, false)
    );
  }
}