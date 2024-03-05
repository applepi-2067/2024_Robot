package frc.robot;


import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import io.github.oblarg.oblog.Logger;

import frc.robot.commands.PickupPiece;
import frc.robot.commands.AutoAimShoulder;
import frc.robot.commands.SetShooterPercentOutput;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.SetFeederVelocity;
import frc.robot.commands.SetIntakeVelocity;
import frc.robot.commands.SetShooterVelocity;
import frc.robot.commands.SetShoulderPosition;
import frc.robot.commands.ShootGamePiece;
import frc.robot.commands.WaitUntilSpeakerOriented;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain.AprilTag;
import frc.robot.subsystems.swerve.DriveMotor;
import frc.robot.subsystems.Elevator;


public class RobotContainer {
  // Subsystems.
  private final Drivetrain m_drivetrain;
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Intake m_intake;
  private final Shoulder m_shoulder;
  private final Elevator m_elevator;
  private final Vision m_vision;

  // Controllers.
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private final CommandXboxController m_driverController;

  private static final int OPERATOR_CONTROLLER_PORT = 1;
  private final CommandXboxController m_operatorController;
  
  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    // Subsystem init.
    m_drivetrain = Drivetrain.getInstance();
    m_shooter = Shooter.getInstance();
    m_feeder = Feeder.getInstance();
    m_intake = Intake.getInstance();
    m_shoulder = Shoulder.getInstance();
    m_elevator = Elevator.getInstance();
    m_vision = Vision.getInstance();
  
    // PathPlanner.
    SequentialCommandGroup aimShoot = new SequentialCommandGroup(
      new AutoAimShoulder(false),
      new WaitUntilSpeakerOriented(),
      new ShootGamePiece(true)
    );

    NamedCommands.registerCommand("AimShoot", aimShoot);
    NamedCommands.registerCommand("PickupAimShoot",
      new SequentialCommandGroup(new PickupPiece(), aimShoot)
    );

    NamedCommands.registerCommand("Pickup", new PickupPiece());
    NamedCommands.registerCommand("RampupShooter", new SetShooterPercentOutput(Shooter.SHOOTING_SPEED_RPM));
    NamedCommands.registerCommand("WaitUntilSpeakerOriented", new WaitUntilSpeakerOriented());
    NamedCommands.registerCommand("KillShooter", new SetShooterVelocity(0.0, false));

    AutoBuilder.configureHolonomic(
      m_drivetrain::getRobotPose2d,
      m_drivetrain::setRobotPose2d,
      m_drivetrain::getRobotRelativeChassisSpeeds,
      m_drivetrain::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        new PIDConstants(2.5),
        new PIDConstants(10.0),
        DriveMotor.MAX_SPEED_METERS_PER_SEC,
        Drivetrain.CENTER_TO_WHEEL_OFFSET_METERS,
        new ReplanningConfig()
      ),
      () -> {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      m_drivetrain
    );

    PPHolonomicDriveController.setRotationTargetOverride(this::getTargetRotationOverride);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto chooser", autoChooser);

    // Controls init.
    m_driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    m_operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    configureBindings();

    Logger.configureLoggingAndConfig(this, false);
  }

  public Optional<Rotation2d> getTargetRotationOverride() {
    SmartDashboard.putBoolean("Feeder piece detected", Feeder.getInstance().gamePieceDetected());
    if (Feeder.getInstance().gamePieceDetected()) {
      return Optional.of(m_drivetrain.getRobotToPoseRotation(m_drivetrain.getAprilTagPose(AprilTag.SPEAKER)));
    }
    
    return Optional.empty();
  }

  private void configureBindings() {
    // Driver.
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

    m_driverController.rightTrigger().onTrue(new InstantCommand(() -> m_drivetrain.setTargetAprilTag(Optional.of(AprilTag.SPEAKER))));
    m_driverController.b().onTrue(new InstantCommand(() -> m_drivetrain.setTargetAprilTag(Optional.of(AprilTag.AMP))));
    m_driverController.x().onTrue(new InstantCommand(() -> m_drivetrain.setTargetAprilTag(Optional.of(AprilTag.TRAP))));
    m_driverController.leftTrigger().onTrue(new InstantCommand(() -> m_drivetrain.setTargetAprilTag(Optional.empty())));

    // Operator.
    m_operatorController.a().onTrue(new ScoreAmp());
    m_operatorController.x().onTrue(new PickupPiece());

    m_operatorController.leftBumper().onTrue(new SetShooterVelocity(Shooter.SHOOTING_SPEED_RPM, false));
    m_operatorController.rightBumper().onTrue(
      new ParallelCommandGroup(
        new SetShooterPercentOutput(0.0),
        new SetShoulderPosition(Shoulder.ZERO_POSITION_DEGREES, false),
        new SetFeederVelocity(0.0),
        new SetIntakeVelocity(0.0)
      )
    );

    m_operatorController.leftTrigger().onTrue(new AutoAimShoulder(true));
    m_operatorController.rightTrigger().onTrue(new ShootGamePiece(false));
  }
  

  // Use this to pass the autonomous command to the main Robot.java class.
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
