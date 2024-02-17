package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootGamePiece extends SequentialCommandGroup {
  public ShootGamePiece() {
    Shooter shooter = Shooter.getInstance();
    Feeder feeder = Feeder.getInstance();

    addCommands(
      new SetShooterVelocity(Shooter.SHOOTING_SPEED_RPM, true),
      new SetFeederVelocity(3_000),
      
      new WaitUntilCommand(() -> !feeder.gamePieceDetected()),
      new WaitCommand(0.5),  // Piece should be gone by now!
      
      new SetFeederVelocity(0.0),
      new InstantCommand(() -> shooter.setPercentOutput(0.0), shooter)  // Coast to 0.
    );

    addRequirements(feeder, shooter);
  }
}
