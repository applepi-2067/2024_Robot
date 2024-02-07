package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import io.github.oblarg.oblog.Logger;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;


public class RobotContainer {
  // Subsystems.
  private final Drivetrain m_drivetrain;
  private final Intake m_intake;

  // Controllers.
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private final CommandXboxController m_driverController;


  public RobotContainer() {
    m_drivetrain = Drivetrain.getInstance();
    m_intake = Intake.getInstance();
    
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

    m_driverController.b().onTrue(new InstantCommand(() -> m_intake.setPercentOutput(0.5), m_intake));
    m_driverController.b().onFalse(new InstantCommand(() -> m_intake.setPercentOutput(0.0), m_intake));
  }

  // Use this to pass the autonomous command to the main Robot.java class.
  public Command getAutonomousCommand() {
    return null;
  }
}
