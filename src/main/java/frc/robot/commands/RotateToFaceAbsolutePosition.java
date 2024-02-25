package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain;

public class RotateToFaceAbsolutePosition extends Command {
  private final Drivetrain m_drivetrain;

  private final Pose2d m_targetPose2d;

  private static final double ROTATION_TOLERANCE_DEGREES = 0.5;

  private static final double kP = 1.35;
  private static final double kS = 0.15;
  
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
    double power = getRobotToTargetRotation2d().getRadians() * (1.0 / Math.PI) * kP;
    power += Math.signum(power) * kS;

    m_drivetrain.drive(0.0, 0.0, -1.0 * power);
  }

  private Rotation2d getRobotToTargetRotation2d() {
    Rotation2d currRotation2d = m_drivetrain.getRobotPose2d().getRotation();
    Rotation2d deltaRotation2d = currRotation2d.minus(m_targetRotation2d); 
    return deltaRotation2d.unaryMinus();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(getRobotToTargetRotation2d().getDegrees()) < ROTATION_TOLERANCE_DEGREES;
  }
}
