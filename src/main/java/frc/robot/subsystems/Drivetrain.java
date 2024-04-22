package frc.robot.subsystems;


import java.util.Optional;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.constants.RobotMap;
import frc.robot.subsystems.swerve.DriveMotor;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.utils.Conversions;
import frc.robot.utils.Utils;


public class Drivetrain extends SubsystemBase implements Loggable {
  private static Drivetrain instance = null;

  // Swerve module offsets from center. +x = front of robot, +y = left of robot. 
  public static final double CENTER_TO_WHEEL_OFFSET_METERS = Units.inchesToMeters(13.0 - 2.5);
  public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(-CENTER_TO_WHEEL_OFFSET_METERS, CENTER_TO_WHEEL_OFFSET_METERS),
    new Translation2d(-CENTER_TO_WHEEL_OFFSET_METERS, -CENTER_TO_WHEEL_OFFSET_METERS),
    new Translation2d(CENTER_TO_WHEEL_OFFSET_METERS, CENTER_TO_WHEEL_OFFSET_METERS),
    new Translation2d(CENTER_TO_WHEEL_OFFSET_METERS, -CENTER_TO_WHEEL_OFFSET_METERS)
  );

  private final SwerveModule[] m_swerveModules;

  // Max speeds.
  public static final double MAX_ROTATION_SPEED_RADIANS_PER_SEC = Units.rotationsToRadians(
    Conversions.arcLengthToRotations(
      DriveMotor.MAX_SPEED_METERS_PER_SEC,
      CENTER_TO_WHEEL_OFFSET_METERS
    )
  );
  
  // Odometry (stds are x, y, heading).
  private final SwerveDrivePoseEstimator m_odometry;
  private static final Matrix<N3, N1> DRIVETRAIN_STDS = new Matrix<>(Nat.N3(), Nat.N1(), new double[] {0.01, 0.01, 0.005});
  private static final Matrix<N3, N1> SINGLE_VISION_STDS = new Matrix<>(Nat.N3(), Nat.N1(), new double[] {0.1, 0.1, 0.05});
  private static final Matrix<N3, N1> MULTI_VISION_STDS = new Matrix<>(Nat.N3(), Nat.N1(), new double[] {0.025, 0.025, 0.0125});

  private Pose2d m_pose;
  private final PigeonIMU m_gyro;
  private final Field2d m_field;

  private double m_fieldOrientedHeadingOffsetDegrees = 0.0;

  // Pose facing.
  private Optional<Pose2d> m_targetFacingPose = Optional.empty();
  private boolean m_faceAway = false;
  
  private final PIDController m_poseFacingPIDController = new PIDController(0.015, 0.05, 0.0);
  private static final double POSE_FACING_kS = -0.12;
  private static final double POSE_FACING_IZONE = 4.0;
  public static final double POSE_FACING_ALLOWABLE_ERROR_DEGREES = 0.5;

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  private Drivetrain() {
    // Create swerve modules.
    m_swerveModules = new SwerveModule[4];
    for (int location = 0; location < 4; location++) {
      m_swerveModules[location] = new SwerveModule(location);
    }

    // Create and reset gyro.
    m_gyro = new PigeonIMU(RobotMap.canIDs.Drivetrain.GYRO);
    m_gyro.setYaw(0.0);

    // Odometry.
    m_odometry = new SwerveDrivePoseEstimator(
      SWERVE_DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(getHeadingDegrees()),
      getSwerveModulePositions(),
      new Pose2d(),
      DRIVETRAIN_STDS,
      SINGLE_VISION_STDS
    );

    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    m_pose = getRobotPose2d();

    m_poseFacingPIDController.setIZone(POSE_FACING_IZONE);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      swerveModulePositions[i] = m_swerveModules[i].getPosition();
    }
    return swerveModulePositions;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      swerveModuleStates[i] = m_swerveModules[i].getState();
    }
    return swerveModuleStates;
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    // Convert to swerve module states, and set states.
    SwerveModuleState[] states = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setSwerveModuleStates(states);
  }

  public void drive(double leftStickX, double leftStickY, double rightStickX) {
    // Override rotation command if targeting pose.
    if (m_targetFacingPose.isPresent()) {
      rightStickX = getPoseFacingRightStickX();
    }
    
    // Deadband and square stick values.
    double absDeadbandThreshold = 0.04;
    leftStickX = deadbandSquareStickInput(leftStickX, absDeadbandThreshold);
    leftStickY = deadbandSquareStickInput(leftStickY, absDeadbandThreshold);
    rightStickX = deadbandSquareStickInput(rightStickX, absDeadbandThreshold);

    // Speeds in robot coords. Negatives account for stick signs.
    double yVelocityMetersPerSecond = -1.0 * leftStickX * DriveMotor.MAX_SPEED_METERS_PER_SEC;
    double xVelocityMetersPerSecond = -1.0 * leftStickY * DriveMotor.MAX_SPEED_METERS_PER_SEC;
    double rotationVelocityRadiansPerSecond = -1.0 * rightStickX * MAX_ROTATION_SPEED_RADIANS_PER_SEC;

    // Robot to field oriented speeds.
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xVelocityMetersPerSecond,
      yVelocityMetersPerSecond,
      rotationVelocityRadiansPerSecond,
      Rotation2d.fromDegrees(getHeadingDegrees() - m_fieldOrientedHeadingOffsetDegrees)
    );

    driveRobotRelative(chassisSpeeds);
  }

  private double getPoseFacingRightStickX() {
    Rotation2d targetRotation = getRobotToPoseRotation(m_targetFacingPose.get());
    Rotation2d robotToTargetRotationError = getRobotPose2d().getRotation().minus(targetRotation).unaryMinus();

    if (m_faceAway) {
      robotToTargetRotationError = robotToTargetRotationError.plus(Rotation2d.fromDegrees(180.0));
    }

    if (Math.abs(robotToTargetRotationError.getDegrees()) < POSE_FACING_ALLOWABLE_ERROR_DEGREES) {
      return 0.0;
    }

    double rightStickX = m_poseFacingPIDController.calculate(robotToTargetRotationError.getDegrees(), 0.0);
    rightStickX += Math.signum(robotToTargetRotationError.getDegrees()) * POSE_FACING_kS;
    return rightStickX;
  }

  public void setTargetFacingPose(AprilTag targetAprilTag, boolean faceAway) {
    setTargetFacingPose(Optional.of(getAprilTagPose(getAprilTagID(targetAprilTag))), faceAway);
  }

  public void setTargetFacingPose(Optional<Pose2d> targetFacingPose, boolean faceAway) {
    m_targetFacingPose = targetFacingPose;
    m_faceAway = faceAway;
  }

  private double deadbandSquareStickInput(double value, double absDeadbandThreshold) {
    // Deadband while preserving sensitivity.
    if (Math.abs(value) < absDeadbandThreshold) {
      return 0.0;
    }

    double m = 1.0 / (1 - absDeadbandThreshold);
    value = Math.signum(value) * (Math.abs(value) - absDeadbandThreshold) * m;
    return Math.pow(value, 2.0) * Math.signum(value);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states) {
    // Normalize swerve module states.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveMotor.MAX_SPEED_METERS_PER_SEC);

    // Pass states to each module.
    for (int location = 0; location < 4; location++) {
      SwerveModule swerveModule = m_swerveModules[location];
      SwerveModuleState state = states[location];

      swerveModule.setTargetState(state);
    }
  }

  public void resetFieldOriented(boolean fieldOriented) {
    if (fieldOriented) {
      Rotation2d fieldOrientedHeadingOffsetRotation = Rotation2d.fromDegrees(getHeadingDegrees())
        .minus(getRobotPose2d().getRotation());
      if (!isBlue()) {fieldOrientedHeadingOffsetRotation = fieldOrientedHeadingOffsetRotation.plus(Rotation2d.fromDegrees(180.0));}
      m_fieldOrientedHeadingOffsetDegrees = fieldOrientedHeadingOffsetRotation.getDegrees();
    }
    else {
      m_fieldOrientedHeadingOffsetDegrees = getHeadingDegrees();
    }
  }

  @Log (name="Heading (deg)")
  public double getHeadingDegrees() {
    return m_gyro.getYaw();
  }

  public Pose2d getRobotPose2d() {
    Pose2d robotPose2d = m_odometry.update(
      Rotation2d.fromDegrees(getHeadingDegrees()),
      getSwerveModulePositions()
    );
    return robotPose2d; 
  }

  public void setRobotPose2d(Pose2d pose) {
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getHeadingDegrees()),
      getSwerveModulePositions(),
      pose
    );
  }
  
  public void addVisionMeaurement(Pose2d visionEstimatedRobotPose2d, double timestampSeconds, boolean singleTarget) {
    Matrix<N3, N1> visionStds = singleTarget ? SINGLE_VISION_STDS : MULTI_VISION_STDS;
    m_odometry.addVisionMeasurement(visionEstimatedRobotPose2d, timestampSeconds, visionStds);

    double visionToDrivetrainPoseDistMeters = visionEstimatedRobotPose2d.getTranslation().getDistance(m_pose.getTranslation());
    SmartDashboard.putNumber("Dist to vision measurement", visionToDrivetrainPoseDistMeters);
    SmartDashboard.putBoolean("VisionCloseToDrivetrain", visionToDrivetrainPoseDistMeters < 0.5);
  }

  @Override
  public void periodic() {
    m_pose = getRobotPose2d();
    m_field.setRobotPose(m_pose);
    SmartDashboard.putString("Robot pose", Utils.getPose2dDescription(m_pose));
    
    SmartDashboard.putNumber("Dist to speaker (in)", getDistToSpeakerInches());
  }

  // Log state.
  @Log (name="Swerve Module 0")
  public String getSwerveModule0Description() {
    return m_swerveModules[0].toString();
  }

  @Log (name="Swerve Module 1")
  public String getSwerveModule1Description() {
    return m_swerveModules[1].toString();
  }

  @Log (name="Swerve Module 2")
  public String getSwerveModule2Description() {
    return m_swerveModules[2].toString();
  }

  @Log (name="Swerve Module 3")
  public String getSwerveModule3Description() {
    return m_swerveModules[3].toString();
  }

  public boolean isBlue() {
    return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
  }

  public enum AprilTag {
    SPEAKER,
    AMP,
    TRAP
  }

  public int getAprilTagID(AprilTag aprilTag) {
    boolean isBlue = isBlue();
    
    int aprilTagID;
    if (aprilTag == AprilTag.AMP) {
      aprilTagID = isBlue ? 6 : 5;
    }
    else if (aprilTag == AprilTag.TRAP) {
      int[] blueTrapTags = {14, 15, 16};
      int[] redTrapTags = {11, 12, 13};
      aprilTagID = isBlue ? getClosestAprilTagID(blueTrapTags) : getClosestAprilTagID(redTrapTags);
    }
    else {
      aprilTagID = isBlue ? 7 : 4;  // Default at speaker.
    }

    return aprilTagID;
  }

  private int getClosestAprilTagID(int[] aprilTagIDs) {
    int closestAprilTagID = aprilTagIDs[0];
    for (int i = 1; i < aprilTagIDs.length; i++) {
      if (getDistToAprilTagMeters(aprilTagIDs[i]) < getDistToAprilTagMeters(closestAprilTagID)) {
        closestAprilTagID = aprilTagIDs[i];
      }
    }
    return closestAprilTagID;
  }

  public Pose2d getAprilTagPose(int aprilTagID) {
    return Vision.APRIL_TAG_FIELD_LAYOUT.getTagPose(aprilTagID).get().toPose2d();
  }

  public double getDistToAprilTagMeters(int aprilTagID) {
    Translation2d aprilTagTranslation2d = getAprilTagPose(aprilTagID).getTranslation();
    Translation2d robotTranslation2d = getRobotPose2d().getTranslation();
    return aprilTagTranslation2d.getDistance(robotTranslation2d);
  }

  @Log (name="Dist to speaker (in)")
  public double getDistToSpeakerInches() {
    double distToSpeakerMeters = getDistToAprilTagMeters(getAprilTagID(AprilTag.SPEAKER));
    return Units.metersToInches(distToSpeakerMeters);
  }

  public Rotation2d getRobotToPoseRotation(Pose2d targetPose) {
    Pose2d robotPose2d = getRobotPose2d();

    double dx = targetPose.getX() - robotPose2d.getX();
    double dy = targetPose.getY() - robotPose2d.getY();
    Rotation2d targetRotation = Rotation2d.fromRadians(Math.atan2(dy, dx));
    return targetRotation;
  }
}
