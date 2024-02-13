package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootGamePiece extends SequentialCommandGroup {
  public ShootGamePiece() {
    addCommands(
      new SetShooterVelocity(Shooter.SHOOTING_SPEED_RPM, true),
      new SetFeederPercentOutput(0.85),
      
      new WaitUntilCommand(() -> !Feeder.getInstance().gamePieceDetected()),
      new WaitCommand(0.5),  // Piece should be gone by now!
      
      new SetFeederPercentOutput(0.0),
      new SetShooterVelocity(0.0, false)
    );

    addRequirements(
      Feeder.getInstance(),
      Shooter.getInstance()
    );
  }
}
