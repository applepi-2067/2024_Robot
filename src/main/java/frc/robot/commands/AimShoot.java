package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AimShoot extends SequentialCommandGroup {
  public AimShoot(boolean closeShot) {
    addCommands(
      new ParallelCommandGroup(
        new AutoAimShoulder(false),
        new FaceSpeaker()
      ),
      new ShootGamePiece(closeShot, true)
    );
  }
}
