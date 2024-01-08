package frc.robot;


import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import io.github.oblarg.oblog.Logger;

import frc.robot.subsystems.Drivetrain;


public class RobotContainer {
  // Subsystems.
  private final Drivetrain m_drivetrain;

  // Controllers.
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private final CommandXboxController m_driverController;


  public RobotContainer() {
    m_drivetrain = Drivetrain.getInstance();
    
    m_driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    configureBindings();

    Logger.configureLoggingAndConfig(this, false);
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
      Commands.run(
        () -> m_drivetrain.drive(
          m_driverController.getLeftX(),
          m_driverController.getLeftY(),
          m_driverController.getRightX()
        ),
        m_drivetrain
      )
    );

    m_driverController.a().onTrue(
      new InstantCommand(
        () -> m_drivetrain.resetGyro()
      )
    );
  }

  // Use this to pass the autonomous command to the main Robot.java class.
  public Command getAutonomousCommand() {
    // Create trajectory settings.
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      2.0,
      4.0
      // Drivetrain.MAX_TRANSLATION_SPEED_METERS_PER_SEC,
      // Drivetrain.MAX_ACCEL_METERS_PER_SEC_SQUARED
    );
    trajectoryConfig.setKinematics(Drivetrain.SWERVE_DRIVE_KINEMATICS);

    // Generate trajectory.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(),
      List.of(
        new Translation2d(1.0, 0.0),
        new Translation2d(1.0, -1.0)
      ),
      new Pose2d(2.0, -1.0, Rotation2d.fromDegrees(180.0)),
      trajectoryConfig
    );

    // TODO: tune pid controllers.
    // Create PID controllers.
    PIDController xController = new PIDController(0.0, 0.0, 0.0);
    PIDController yController = new PIDController(0.0, 0.0, 0.0);


    // TODO: find theta controller constraints.
    ProfiledPIDController thetaController = new ProfiledPIDController(
      0.0, 0.0, 0.0,
      new Constraints(
        Drivetrain.MAX_ROTATION_SPEED_RADIANS_PER_SEC,
        Drivetrain.MAX_ROTATION_ACCEL_RADIANS_PER_SEC_SQUARED
      )
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    HolonomicDriveController driveController = new HolonomicDriveController(xController, yController, thetaController);

    // Create command.
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      m_drivetrain::getRobotPose2d,
      Drivetrain.SWERVE_DRIVE_KINEMATICS,
      driveController,
      m_drivetrain::setSwerveModuleStates,
      m_drivetrain
    );

    // TODO: wrap command?
    return swerveControllerCommand;
  }
}
