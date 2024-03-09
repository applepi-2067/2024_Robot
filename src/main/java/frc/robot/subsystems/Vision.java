package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

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

public class Vision extends SubsystemBase {
  public static Vision instance = null;

  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final Field2d m_field;

  // Cameras.
  private final PhotonPoseEstimator[] m_photonPoseEstimators = new PhotonPoseEstimator[2];

  private static final String[] CAMERA_NAMES = {
    "Arducam_0",
    "Arducam_1"
  };
  
  private static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = {
    new Transform3d(
      new Translation3d(Units.inchesToMeters(9.75), Units.inchesToMeters(0.0), Units.inchesToMeters(22.0)),
      new Rotation3d(0.0, Units.degreesToRadians(-26.0), Units.degreesToRadians(-2.0))  // TODO: check camera rotation offset.
    ),
    new Transform3d(
      new Translation3d(Units.inchesToMeters(-12.0), 0.0, Units.inchesToMeters(9.0)),
      new Rotation3d(0.0, Units.degreesToRadians(-41.0), Units.degreesToRadians(180.0))
    )
  };

  // Single-tag pose estimate rejection thresholds.
  private static final double MAX_TARGET_AMBIGUITY = 0.15;

  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  private Vision() {
    for (int i = 0; i < CAMERA_NAMES.length; ++i) {
      PhotonCamera camera = new PhotonCamera(CAMERA_NAMES[i]);

      m_photonPoseEstimators[i] = new PhotonPoseEstimator(
        APRIL_TAG_FIELD_LAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        ROBOT_TO_CAMERA_TRANSFORMS[i]
      );

      m_photonPoseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    m_field = new Field2d();
    SmartDashboard.putData("Vision field", m_field);
  }

  @Override
  public void periodic() {
    for (PhotonPoseEstimator photonPoseEstimator : m_photonPoseEstimators) {
      addVisionMeaurementToDrivetrain(photonPoseEstimator);
    }
  }

  public void addVisionMeaurementToDrivetrain(PhotonPoseEstimator photonPoseEstimator) {
    Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
    if (!result.isPresent()) {return;}

    EstimatedRobotPose robotPose = result.get();
    // Estimates based on a single tag must pass ambiguity test.
    if (robotPose.targetsUsed.size() == 1) {
      PhotonTrackedTarget target = robotPose.targetsUsed.get(0);
      if (target.getPoseAmbiguity() > MAX_TARGET_AMBIGUITY) {return;}
    }
    
    Pose2d estimatedRobotPose2d = robotPose.estimatedPose.toPose2d();
    double timestampSeconds = result.get().timestampSeconds;
    Drivetrain.getInstance().addVisionMeaurement(estimatedRobotPose2d, timestampSeconds);

    // Log vision position on shuffleboard.
    m_field.setRobotPose(estimatedRobotPose2d);
  }
}

