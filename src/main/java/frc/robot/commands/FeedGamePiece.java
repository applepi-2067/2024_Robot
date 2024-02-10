package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class FeedGamePiece extends SequentialCommandGroup {
  public FeedGamePiece() {
    Feeder feeder = Feeder.getInstance();
    Intake intake = Intake.getInstance();

    addCommands(
      new ParallelCommandGroup(
        new SetFeederPercentOutput(0.75),
        new InstantCommand(() -> intake.setPercentOutput(0.75))
      ),

      new WaitUntilCommand(feeder::gamePieceDetected),

      new ParallelCommandGroup(
        new SetFeederPercentOutput(0.0),
        new InstantCommand(() -> intake.setPercentOutput(0.0))
      )
    );

    addRequirements(feeder, intake);
  }
}
