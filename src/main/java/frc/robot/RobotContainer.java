package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import io.github.oblarg.oblog.Logger;

import frc.robot.commands.PickupPiece;
import frc.robot.commands.AimShoot;
import frc.robot.commands.SetFeederVelocity;
import frc.robot.commands.SetIntakeVelocity;
import frc.robot.commands.SetShooterVelocity;
import frc.robot.commands.ZeroShoulder;

import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Vision;
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
  private final Controllers m_controllers; 
  
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
    NamedCommands.registerCommand("CloseAimShoot", new ZeroShoulder().andThen(new AimShoot(true)));
    NamedCommands.registerCommand("KillShooter", new SetShooterVelocity(0.0, false));

    NamedCommands.registerCommand(
      "SpitNotes",
      new ParallelCommandGroup(
        new SetIntakeVelocity(4_000.0),
        new SetFeederVelocity(3_000.0),
        new SetShooterVelocity(3_600.0, false)
      )
    );

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

    // Create controllers. Must come after autobuilder configuration.
    m_controllers = Controllers.getInstance();

    // Populate auto chooser.
    autoChooser = new SendableChooser<Command>();
    autoChooser.addOption("Amp auto", new PathPlannerAuto("Amp auto"));
    autoChooser.addOption("Center upper auto", new PathPlannerAuto("Center upper 4-note"));
    autoChooser.addOption("Center lower auto", new PathPlannerAuto("Center lower 4-note"));
    autoChooser.addOption("Source auto", new PathPlannerAuto("Source auto"));
    autoChooser.addOption("Jim auto", new PathPlannerAuto("Jim auto"));
    autoChooser.addOption("Center 4 note close", new PathPlannerAuto("Close 4 note"));

    SmartDashboard.putData("Auto chooser", autoChooser); 

    Logger.configureLoggingAndConfig(this, false);
  }
 
  // Use this to pass the autonomous command to the main Robot.java class.
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
