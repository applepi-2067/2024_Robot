package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shoulder;

public class ScoreAmp extends SequentialCommandGroup {
  public ScoreAmp() {
    addCommands(
      new ParallelCommandGroup(
        new SetElevatorPosition(8.0, true),
        new SetShoulderPosition(0, true)  // TODO: find shoulder angle for amp scoring.
      ),

      new SetFeederVelocity(-1_000),
      new WaitUntilCommand(() -> !Feeder.getInstance().gamePieceDetected()),
      new WaitUntilCommand(0.5),

      new ParallelCommandGroup(
        new SetElevatorPosition(0, false),
        new SetShoulderPosition(0, false),
        new SetFeederVelocity(0.0)
      )
    );

    addRequirements(
      Elevator.getInstance(),
      Shoulder.getInstance(),
      Feeder.getInstance()
    );
  }
}
