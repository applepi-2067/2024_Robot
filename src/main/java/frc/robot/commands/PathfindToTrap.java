package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.AprilTag;

public class PathfindToTrap extends Command {
  private final PathConstraints m_pathConstraints;

  public PathfindToTrap(PathConstraints pathConstraints) {
    m_pathConstraints = pathConstraints;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Drivetrain drivetrain = Drivetrain.getInstance();
    CommandScheduler.getInstance().schedule(
      AutoBuilder.pathfindToPose(
        drivetrain.getAprilTagPose(drivetrain.getAprilTagID(AprilTag.TRAP)).transformBy(new Transform2d(1.0, 0.0, new Rotation2d())),
        m_pathConstraints
      )
    );

    drivetrain.resetFieldOriented(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
