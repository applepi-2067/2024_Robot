package frc.robot;


import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.commands.AimShoot;
import frc.robot.commands.AutoAimShoulder;
import frc.robot.commands.PathfindToTrap;
import frc.robot.commands.SetShooterPercentOutput;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetFeederVelocity;
import frc.robot.commands.SetIntakeVelocity;
import frc.robot.commands.SetShooterVelocity;
import frc.robot.commands.SetShoulderPosition;
import frc.robot.commands.ShootGamePiece;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
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
  private final Lights m_lights;

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
    m_lights = Lights.getInstance();
  
    // PathPlanner.
    NamedCommands.registerCommand("Pickup", new PickupPiece());
    NamedCommands.registerCommand("AimShoot", new AimShoot(false).onlyIf(m_feeder::gamePieceDetected));
    NamedCommands.registerCommand("CloseAimShoot", new AimShoot(true).onlyIf(m_feeder::gamePieceDetected));
    NamedCommands.registerCommand("KillShooter", new SetShooterVelocity(0.0, false));

    AutoBuilder.configureHolonomic(
      m_drivetrain::getRobotPose2d,
      m_drivetrain::setRobotPose2d,
      m_drivetrain::getRobotRelativeChassisSpeeds,
      m_drivetrain::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        new PIDConstants(3.0),
        new PIDConstants(11.0),
        DriveMotor.MAX_SPEED_METERS_PER_SEC,
        Drivetrain.CENTER_TO_WHEEL_OFFSET_METERS,
        new ReplanningConfig()
      ),
      () -> {return !m_drivetrain.isBlue();},
      m_drivetrain
    );

    // Populate auto chooser.
    autoChooser = new SendableChooser<Command>();
    autoChooser.addOption("Amp auto", new PathPlannerAuto("Amp auto"));
    autoChooser.addOption("Center upper auto", new PathPlannerAuto("Center upper 4-note"));
    autoChooser.addOption("Center lower auto", new PathPlannerAuto("Center lower 4-note"));
    autoChooser.addOption("Source auto", new PathPlannerAuto("Source auto"));

    SmartDashboard.putData("Auto chooser", autoChooser);

    // Controls init.
    m_driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    m_operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    configDriverBindings();
    configOperatorBindings();

    Logger.configureLoggingAndConfig(this, false);
  }

  private void configDriverBindings() {
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

    m_driverController.povUp().onTrue(new InstantCommand(() -> m_drivetrain.drive(0.0, 0.0, 0.0), m_drivetrain));

    m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_drivetrain.resetGyro(true)));
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_drivetrain.resetGyro(false)));

    m_driverController.rightTrigger().onTrue(new InstantCommand(() -> m_drivetrain.setTargetAprilTag(Optional.of(AprilTag.SPEAKER))));
    m_driverController.leftTrigger().onTrue(new InstantCommand(() -> m_drivetrain.setTargetAprilTag(Optional.empty())));

    // Pathfinding.
    PathConstraints pathConstraints = new PathConstraints(
      3.0,
      3.0,
      Units.degreesToRadians(360.0),
      Units.degreesToRadians(720.0)
    );

    m_driverController.a().onTrue(
      AutoBuilder.pathfindToPose(
        m_drivetrain.getAprilTagPose(m_drivetrain.getAprilTagID(AprilTag.SPEAKER)).transformBy(new Transform2d(2.0, 0.0, Rotation2d.fromDegrees(180.0))),
        pathConstraints
      ).andThen(new InstantCommand(() -> m_drivetrain.setTargetAprilTag(Optional.of(AprilTag.SPEAKER))))
    );

    Pose2d ampPose = m_drivetrain.getAprilTagPose(m_drivetrain.getAprilTagID(AprilTag.AMP));
    m_driverController.b().onTrue(
      new SequentialCommandGroup(
        AutoBuilder.pathfindToPose(
          ampPose.transformBy(new Transform2d(1.0, 0.0, new Rotation2d())),
          pathConstraints,
          1.0
        ),
        AutoBuilder.pathfindToPose(
          ampPose.transformBy(new Transform2d(0.4, 0.0, new Rotation2d())),
          pathConstraints
        )
      )
    );
    m_driverController.x().onTrue(new PathfindToTrap(pathConstraints));
  }

  private void configOperatorBindings() {
    // DEV: shoulder tuning.
    // SmartDashboard.putNumber("targetPosition", Shoulder.ZERO_POSITION_DEGREES);
    // m_operatorController.a().onTrue(new SetShoulderPosition(Shoulder.ZERO_POSITION_DEGREES, false, true));
    
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
    m_operatorController.rightTrigger().onTrue(new ShootGamePiece(false, false));

    m_operatorController.povDown().onTrue(new ParallelCommandGroup(new SetFeederVelocity(-1_000.0), new SetIntakeVelocity(-1_000.0)));
    m_operatorController.povRight().onTrue(new SetShoulderPosition(50.0, false));

    m_operatorController.povLeft().onTrue(new SetShoulderPosition(0.0, false));

    // Trap score.
    m_operatorController.b().onTrue(new SetElevatorPosition(Elevator.MAX_EXTENSION_INCHES, false));
    m_operatorController.b().onFalse(new SetElevatorPosition(0.0, false));

    m_operatorController.y().onTrue(new SetShoulderPosition(-11.0, false));
    m_operatorController.y().onFalse(new SetShoulderPosition(Shoulder.ZERO_POSITION_DEGREES, false));

    m_operatorController.povUp().onTrue(new SetFeederVelocity(-1_000.0));
    m_operatorController.povUp().onFalse(new SetFeederVelocity(0.0));
  }
  
  // Use this to pass the autonomous command to the main Robot.java class.
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
