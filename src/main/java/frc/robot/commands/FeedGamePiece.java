package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;

public class FeedGamePiece extends SequentialCommandGroup {
  public FeedGamePiece() {
    Feeder feeder = Feeder.getInstance();

    addCommands(
      new SetFeederPercentOutput(0.3),
      new WaitUntilCommand(feeder::gamePieceDetected),
      new SetFeederRotations(100.0)  // TODO: check that note is stowed in feeder.
    );

    addRequirements(feeder);
  }
}
