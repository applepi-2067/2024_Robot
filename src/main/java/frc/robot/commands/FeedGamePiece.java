// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedGamePiece extends SequentialCommandGroup {
  /** Creates a new RunFeederUntilPieceDetected. */
  public FeedGamePiece(boolean reverse) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double feederMultipler = 1;
    double feederRotations = 0;
    if (reverse) {
      feederMultipler = -1;
      feederRotations = 100;
    }

    addCommands(
      new SetFeederPower(0.3 * feederMultipler),
      new WaitUntilGamePieceInFeeder(),
      new WaitForFeederRotations(feederRotations),
      new SetFeederPower(0.0)
    );
  }
}
