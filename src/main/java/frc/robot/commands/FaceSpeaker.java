package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.AprilTag;
import frc.robot.utils.Utils;

public class FaceSpeaker extends Command {
  private final Drivetrain drivetrain;

  private static final double ALLOWABLE_ERROR_DEGREES = 1.0;

  public FaceSpeaker() {
    drivetrain = Drivetrain.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setTargetAprilTag(Optional.of(AprilTag.SPEAKER));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0.0, 0.0, 0.0);  // Rotation is automatically overridden.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setTargetAprilTag(Optional.empty());
    drivetrain.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Utils.withinThreshold(
      drivetrain.getRobotPose2d().getRotation().getDegrees(),
      drivetrain.getRobotToPoseRotation(drivetrain.getAprilTagPose(AprilTag.SPEAKER)).getDegrees(),
      ALLOWABLE_ERROR_DEGREES
    );
  }
}
