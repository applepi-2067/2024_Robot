package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import io.github.oblarg.oblog.Logger;

import frc.robot.commands.FeedGamePiece;
import frc.robot.commands.ShootGamePiece;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Elevator;


public class RobotContainer {
  // Subsystems.
  private final Drivetrain m_drivetrain;
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Intake m_intake;
  private final Shoulder m_shoulder;
  private final Elevator m_elevator;

  // Controllers.
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private final CommandXboxController m_driverController;


  public RobotContainer() {
    m_drivetrain = Drivetrain.getInstance();
    m_shooter = Shooter.getInstance();
    m_feeder = Feeder.getInstance();
    m_intake = Intake.getInstance();
    m_shoulder = Shoulder.getInstance();
    m_elevator = Elevator.getInstance();
    
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

    m_driverController.a().onTrue(new InstantCommand(m_drivetrain::resetGyro));

    m_driverController.x().onTrue(new FeedGamePiece());
    m_driverController.y().onTrue(new ShootGamePiece());

    m_driverController.b().onTrue(new InstantCommand(() -> m_intake.setPercentOutput(0.75), m_intake));
    m_driverController.b().onFalse(new InstantCommand(() -> m_intake.setPercentOutput(0.0), m_intake));

    // m_driverController.b().onTrue(
    //   new InstantCommand(
    //     () -> m_shoulder.setTargetPositionDegrees(45.0),
    //     m_shoulder
    //   )
    // );
    // m_driverController.b().onFalse(
    //   new InstantCommand(
    //     () -> m_shoulder.setTargetPositionDegrees(110.0),
    //     m_shoulder
    //   )
    // );

    // m_driverController.x().onTrue(
    //   new InstantCommand(
    //     () -> m_elevator.setTargetPositionInches(Elevator.MAX_EXTENSION_INCHES / 2.0),
    //     m_elevator
    //   )
    // );
    // m_driverController.x().onFalse(
    //   new InstantCommand(
    //     () -> m_elevator.setTargetPositionInches(0.0),
    //     m_elevator
    //   )
    // );
  }

  // Use this to pass the autonomous command to the main Robot.java class.
  public Command getAutonomousCommand() {
    return null;
  }
}
