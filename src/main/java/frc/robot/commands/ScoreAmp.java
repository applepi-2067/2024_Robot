package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shoulder;

public class ScoreAmp extends SequentialCommandGroup {
  public ScoreAmp() {
    addCommands(
      new ParallelCommandGroup(
        new SetShoulderPosition(12.0, true),
        new SetElevatorPosition(13.0, false, true)
      ),

      new SetFeederVelocity(-3_000.0),
      new WaitUntilCommand(() -> !Feeder.getInstance().gamePieceDetected()),
      new WaitCommand(0.05),

      new SetShoulderPosition(Shoulder.ZERO_POSITION_DEGREES, false),
      new SetFeederVelocity(0.0),
      new SetElevatorPosition(0.0, false, false)
    );
  }
}
