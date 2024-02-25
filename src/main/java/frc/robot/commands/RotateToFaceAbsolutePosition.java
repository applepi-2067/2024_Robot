package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain;

public class RotateToFaceAbsolutePosition extends Command {
  private final Drivetrain m_drivetrain;

  private final Pose2d m_targetPose2d;

  private static final double ROTATION_TOLERANCE_DEGREES = 1.0;  // TODO: find rotation tolerance.

  private static final double kP = 0.001;  // TODO: tune rotation kP.
  private static final double kS = 0.1;  // TODO: find kS, minimum power to rotate robot.
  
  private Rotation2d m_targetRotation2d;

  public RotateToFaceAbsolutePosition(Pose2d targetPose2d) {
    m_targetPose2d = targetPose2d;

    m_drivetrain = Drivetrain.getInstance();
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPose2d = m_drivetrain.getRobotPose2d();

    double dx = m_targetPose2d.getX() - robotPose2d.getX();
    double dy = m_targetPose2d.getY() - robotPose2d.getY();

    m_targetRotation2d = Rotation2d.fromRadians(Math.atan2(dy, dx));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = getRotationDeltaDegrees() * kP;
    power += Math.signum(power) * kS;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, power);
    m_drivetrain.driveRobotRelative(chassisSpeeds);
  }

  private double getRotationDeltaDegrees() {
    Rotation2d currRotation2d = m_drivetrain.getRobotPose2d().getRotation();
    Rotation2d deltaRotation2d = currRotation2d.minus(m_targetRotation2d); 
    return deltaRotation2d.getDegrees();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    m_drivetrain.driveRobotRelative(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(getRotationDeltaDegrees()) < ROTATION_TOLERANCE_DEGREES;
  }
}
