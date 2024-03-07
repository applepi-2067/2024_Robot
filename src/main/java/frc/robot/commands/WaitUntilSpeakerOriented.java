// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.AprilTag;
import frc.robot.utils.Utils;

public class WaitUntilSpeakerOriented extends Command {
  /** Creates a new WaitUntil. */
  public WaitUntilSpeakerOriented() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Drivetrain drivetrain = Drivetrain.getInstance();
    return Utils.withinThreshold(
      drivetrain.getRobotPose2d().getRotation().getDegrees(),
      drivetrain.getRobotToPoseRotation(drivetrain.getAprilTagPose(AprilTag.SPEAKER)).getDegrees(),
      1.0
    );
  }
}
