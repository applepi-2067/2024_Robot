package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootGamePiece extends SequentialCommandGroup {
  /** Creates a new PassGamePieceToShooter. */
  public ShootGamePiece() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RampUpShooter(), // Enable Shooter and block until spun up
      new SetFeederPower(0.3),  // Feed piece into shooter
      new WaitCommand(0.8),  // Piece should be gone by now!
      new SetFeederPower(0.0),  // Turn off Feeder
      new SetShooterVelocity(0.0)  // Turn off Shooter
    );
  }
}
