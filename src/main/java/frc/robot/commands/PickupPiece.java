package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shoulder;

public class PickupPiece extends SequentialCommandGroup {
  public PickupPiece() {
    addCommands(
      new ParallelCommandGroup(
        new SetFeederVelocity(700.0),
        new SetIntakeVelocity(4_000.0),
        new SetShoulderPosition(Shoulder.MIN_ANGLE_DEGREES, true),
        new SetElevatorPosition(0.0, false, true)
      ),

      new WaitUntilCommand(Feeder.getInstance()::gamePieceDetected),
      new InstantCommand(() -> CommandScheduler.getInstance().schedule(new Rumble(0.5))),

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
