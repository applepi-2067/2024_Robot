package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootGamePiece extends SequentialCommandGroup {
  public ShootGamePiece(boolean closeShot, boolean keepShooterSpinning) {
    addCommands(
      new SetShooterVelocity((closeShot ? Shooter.CLOSE_SHOOTING_SPEED_RPM : Shooter.SHOOTING_SPEED_RPM), true),
      new SetFeederVelocity(3_000),
      
      new WaitUntilCommand(() -> !Feeder.getInstance().gamePieceDetected()),
      new WaitCommand(0.5),  // Piece should be gone by now!
      
      new SetFeederVelocity(0.0)
    );

    if (!keepShooterSpinning) {
      addCommands(new SetShooterPercentOutput(0.0));  // Coast to 0.
    }
    else if (closeShot) {
      addCommands(new SetShooterVelocity(Shooter.SHOOTING_SPEED_RPM, false));
    }
  }
}
