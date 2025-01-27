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

    Pose2d feedShotPose = drivetrain.getAprilTagPose(drivetrain.getAprilTagID(AprilTag.SPEAKER)).transformBy(
      new Transform2d(0.0, drivetrain.isBlue() ? -1.0 : 1.0, new Rotation2d())
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
  }

  public void setRumble(double rumbleValue) {
    m_driverController.getHID().setRumble(RumbleType.kBothRumble, rumbleValue);
    m_operatorController.getHID().setRumble(RumbleType.kBothRumble, rumbleValue);
  }

  @Override
  public void periodic() {} 
}
