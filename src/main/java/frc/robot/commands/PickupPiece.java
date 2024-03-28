package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shoulder;

public class PickupPiece extends SequentialCommandGroup {
  public PickupPiece() {
    addCommands(
      new ParallelCommandGroup(
        new SetFeederVelocity(900.0),
        new SetIntakeVelocity(3_500.0),
        new SetShoulderPosition(Shoulder.ZERO_POSITION_DEGREES, true),
        new SetElevatorPosition(0.0, true)
      ),

      new WaitUntilCommand(Feeder.getInstance()::gamePieceDetected),

      new ParallelCommandGroup(
        new SetFeederVelocity(0.0),
        new SetIntakeVelocity(0.0)
      ),
      new ParallelCommandGroup(  // FIXME: need continuous setting to 0?
        new SetFeederVelocity(0.0),
        new SetIntakeVelocity(0.0)
      )
    );
  }
}
