package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Utils;

public class Vision extends SubsystemBase {
  public static Vision instance = null;

  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private static final Transform3d ROBOT_TO_CAMERA_TRANSFORM3D = new Transform3d(
    new Translation3d(Units.inchesToMeters(11.25), 0.0, Units.inchesToMeters(20.75)),
    new Rotation3d(0.0, 0.0, 0.0)
  );

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_photonPoseEstimator;

  private final Field2d m_field;

  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  private Vision() {
    m_camera = new PhotonCamera("Arducam_0");

    m_photonPoseEstimator = new PhotonPoseEstimator(
      APRIL_TAG_FIELD_LAYOUT,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      m_camera,
      ROBOT_TO_CAMERA_TRANSFORM3D
    );
    m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    m_field = new Field2d();
    SmartDashboard.putData("Vision field", m_field);
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> result = m_photonPoseEstimator.update();
    if (!result.isPresent()) {return;}

    EstimatedRobotPose robotPose = result.get();
    Pose2d estimatedRobotPose2d = robotPose.estimatedPose.toPose2d();
    double timestampSeconds = result.get().timestampSeconds;
    
    Drivetrain.getInstance().addVisionMeaurement(estimatedRobotPose2d, timestampSeconds);

    // Log vision position on shuffleboard.
    m_field.setRobotPose(estimatedRobotPose2d);
    SmartDashboard.putString("Vision pose", Utils.getPose2dDescription(estimatedRobotPose2d));
  }
}

