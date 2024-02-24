package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shoulder;

public class ScoreAmp extends SequentialCommandGroup {
  public ScoreAmp() {
    addCommands(
      new SetShoulderPosition(-11.7, true),

      new SetFeederVelocity(-1_000.0),
      new WaitUntilCommand(() -> !Feeder.getInstance().gamePieceDetected()),
      new WaitCommand(0.5),

      new SetShoulderPosition(Shoulder.ZERO_POSITION_DEGREES, false),
      new SetFeederVelocity(0.0)
    );

    addRequirements(
      Shoulder.getInstance(),
      Feeder.getInstance()
    );
  }
}
