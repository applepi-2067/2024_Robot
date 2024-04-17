package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.AutoAimShoulder;
import frc.robot.commands.FeedShot;
import frc.robot.commands.PathfindToTrap;
import frc.robot.commands.PickupPiece;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetFeederVelocity;
import frc.robot.commands.SetIntakeVelocity;
import frc.robot.commands.SetShooterPercentOutput;
import frc.robot.commands.SetShooterVelocity;
import frc.robot.commands.SetShoulderPosition;
import frc.robot.commands.ShootGamePiece;
import frc.robot.commands.ZeroShoulder;

import frc.robot.subsystems.Drivetrain.AprilTag;

public class Controllers extends SubsystemBase {
  private static Controllers instance = null;

  private static final int DRIVER_CONTROLLER_PORT = 0;
  private final CommandXboxController m_driverController;

  private static final int OPERATOR_CONTROLLER_PORT = 1;
  private final CommandXboxController m_operatorController;

  public static Controllers getInstance() {
    if (instance == null) {
      instance = new Controllers();
    }
    return instance;
  }

  private Controllers() {
    m_driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    m_operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    configDriverBindings();
    configOperatorBindings();
  }

  private void configDriverBindings() {
    Drivetrain drivetrain = Drivetrain.getInstance();

    drivetrain.setDefaultCommand(
      Commands.run(
        () -> drivetrain.drive(
          m_driverController.getLeftX(),
          m_driverController.getLeftY(),
          m_driverController.getRightX()
        ),
        drivetrain
      )
    );

    m_driverController.povUp().onTrue(new InstantCommand(() -> drivetrain.drive(0.0, 0.0, 0.0), drivetrain));

    m_driverController.rightBumper().onTrue(new InstantCommand(() -> drivetrain.resetFieldOriented(true)));
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> drivetrain.resetFieldOriented(false)));

    m_driverController.rightTrigger().onTrue(new InstantCommand(() -> drivetrain.setTargetFacingPose(AprilTag.SPEAKER, false)));
    m_driverController.leftTrigger().onTrue(new InstantCommand(() -> drivetrain.setTargetFacingPose(Optional.empty(), false)));
    m_driverController.a().onTrue(new InstantCommand(() -> drivetrain.setTargetFacingPose(AprilTag.AMP, true)));

    Pose2d feedShotPose = drivetrain.getAprilTagPose(drivetrain.getAprilTagID(AprilTag.SPEAKER)).transformBy(  // TODO: check feed shot transform.
      new Transform2d(0.0, drivetrain.isBlue() ? 1.0 : -1.0, new Rotation2d())
    );
    m_driverController.y().onTrue(new InstantCommand(() -> drivetrain.setTargetFacingPose(Optional.of(feedShotPose), false)));

    // Pathfinding.
    PathConstraints pathConstraints = new PathConstraints(
      3.0,
      3.0,
      Units.degreesToRadians(360.0),
      Units.degreesToRadians(720.0)
    );

    Pose2d ampPose = drivetrain.getAprilTagPose(drivetrain.getAprilTagID(AprilTag.AMP));
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
    // SmartDashboard.putNumber("targetPosition", Shoulder.MIN_ANGLE_DEGREES);
    // m_operatorController.a().onTrue(new SetShoulderPosition(Shoulder.MIN_ANGLE_DEGREES, false, true));
    
    m_operatorController.a().onTrue(new ScoreAmp());
    m_operatorController.x().onTrue(new PickupPiece());

    m_operatorController.leftBumper().onTrue(new SetShooterVelocity(Shooter.SHOOTING_SPEED_RPM, false));
    m_operatorController.rightBumper().onTrue(
      new ParallelCommandGroup(
        new SetShooterPercentOutput(0.0),
        new SetShoulderPosition(Shoulder.MIN_ANGLE_DEGREES, false),
        new SetFeederVelocity(0.0),
        new SetIntakeVelocity(0.0)
      )
    );

    m_operatorController.leftTrigger().onTrue(new AutoAimShoulder(true));
    m_operatorController.rightTrigger().onTrue(new ShootGamePiece(false, false));

    m_operatorController.povDown().onTrue(
      new ParallelCommandGroup(
        new SetFeederVelocity(-1_000.0),
        new SetIntakeVelocity(-1_000.0),
        new SetShooterVelocity(-400.0, false)
      )
    );
    m_operatorController.povDown().onFalse(
      new ParallelCommandGroup(
        new SetFeederVelocity(0.0),
        new SetIntakeVelocity(0.0),
        new SetShooterVelocity(0.0, false)
      )
    );

    m_operatorController.back().onTrue(new ZeroShoulder());
    m_operatorController.povLeft().onTrue(new FeedShot());
    m_operatorController.povRight().onTrue(new SetShoulderPosition(50.0, false));
    
    // Trap score.
    m_operatorController.b().onTrue(new SetElevatorPosition(Elevator.MAX_EXTENSION_INCHES, false, false));
    m_operatorController.b().onFalse(new SetElevatorPosition(0.0, true, false));

    m_operatorController.y().onTrue(new SetShoulderPosition(-1.0, false));
    m_operatorController.y().onFalse(new SetShoulderPosition(Shoulder.MIN_ANGLE_DEGREES, false));

    m_operatorController.povUp().onTrue(new SetShoulderPosition(-30.0, false));
    m_operatorController.povUp().onFalse(new SetShoulderPosition(Shoulder.MIN_ANGLE_DEGREES, false));
  }

  public void setRumble(double rumbleValue) {
    m_driverController.getHID().setRumble(RumbleType.kBothRumble, rumbleValue);
    m_operatorController.getHID().setRumble(RumbleType.kBothRumble, rumbleValue);
  }

  @Override
  public void periodic() {} 
}
