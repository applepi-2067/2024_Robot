package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class AimShoot extends SequentialCommandGroup {
  public AimShoot(boolean closeShot) {
    addCommands(
      new ParallelCommandGroup(
        new SetShooterVelocity((closeShot ? Shooter.CLOSE_SHOOTING_SPEED_RPM : Shooter.SHOOTING_SPEED_RPM), false),
        new AutoAimShoulder(false),
        new FaceSpeaker()
      ),
      new ShootGamePiece(closeShot, true)
    );
  }
}
