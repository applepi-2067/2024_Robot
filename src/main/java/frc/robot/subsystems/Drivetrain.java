package frc.robot.subsystems;


import java.text.DecimalFormat;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

  private static final DecimalFormat rounder = new DecimalFormat("0.0000");

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
  
  // Odometry.
  private final SwerveDrivePoseEstimator m_odometry;
  private static final double[] drivetrainStds = {0.02, 0.02, 0.01};  // x, y, heading.
  private static final double[] visionStds = {0.5, 0.5, 0.2};  // TODO: verify pose stds.

  private Pose2d m_pose;
  private final PigeonIMU m_gyro;
  private final Field2d m_field;

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
    resetGyro();

    // Odometry.
    m_odometry = new SwerveDrivePoseEstimator(
      SWERVE_DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(getHeadingDegrees()),
      getSwerveModulePositions(),
      new Pose2d(),
      new Matrix<>(Nat.N3(), Nat.N1(), drivetrainStds),
      new Matrix<>(Nat.N3(), Nat.N1(), visionStds)
    );

    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    m_pose = getRobotPose2d();
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
    // Deadband and square stick values.
    double absDeadbandThreshold = 0.10;
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
      Rotation2d.fromDegrees(getHeadingDegrees())
    );

    driveRobotRelative(chassisSpeeds);
  }

  public double deadbandSquareStickInput(double value, double absDeadbandThreshold) {
    // Add abs deadband and square values.
    value = deadband(absDeadbandThreshold, value);
    return Math.pow(value, 2.0) * Math.signum(value);
  }
  
  public double deadband(double absDeadbandThreshold, double x) {
    if (Math.abs(x) < absDeadbandThreshold) {
      return 0.0;
    }

    double m = 1.0 / (1 - absDeadbandThreshold);
    return Math.signum(x) * (Math.abs(x) - absDeadbandThreshold) * m;
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

  public void resetGyro() {
    m_gyro.setYaw(0.0);
  }

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
  
  public void addVisionMeaurement(Pose2d visionEstimatedRobotPose2d, double timestampSeconds) {
    m_odometry.addVisionMeasurement(visionEstimatedRobotPose2d, timestampSeconds);
  }

  @Override
  public void periodic() {
    m_pose = getRobotPose2d();
    m_field.setRobotPose(m_pose);
    SmartDashboard.putString("Robot pose", Utils.getPose2dDescription(m_pose));
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

  @Log (name="Gyro (deg)")
  public String getGyroDescription() {
    // Pitch = hand up, yaw = hand left, roll = hand in.
    String description = "Yaw=" + rounder.format(m_gyro.getYaw()) + "    ";
    description += "Pitch=" + rounder.format(m_gyro.getPitch()) + "    ";
    description += "Roll=" + rounder.format(m_gyro.getRoll());
    return description;
  }

  public Pose2d getSpeakerPose2d() {
    int speakerTag = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue? 8 : 4;
    Pose2d speakerPose2d = Vision.APRIL_TAG_FIELD_LAYOUT.getTagPose(speakerTag).get().toPose2d();
    return speakerPose2d;
  }

  @Log (name="Dist to speaker (m)")
  public double getDistToSpeakerMeters() {
    Pose2d speakerPose2d = getSpeakerPose2d();
    double dx = speakerPose2d.getX() - m_pose.getX();
    double dy = speakerPose2d.getY() - m_pose.getY();
    double distToSpeaker = Math.sqrt((dx * dx) + (dy * dy));
    return distToSpeaker;
  }

  @Log (name="Dist to speaker (in)")
  public double getDistToSpeakerInches() {
    return Units.metersToInches(getDistToSpeakerMeters());
  }
  
  public Rotation2d getRobotToSpeakerRotation2d() {
    Pose2d robotPose2d = getRobotPose2d();

    Pose2d speakerPose2d = getSpeakerPose2d();
    double dx = speakerPose2d.getX() - robotPose2d.getX();
    double dy = speakerPose2d.getY() - robotPose2d.getY();
    Rotation2d targetRotation2d = Rotation2d.fromRadians(Math.atan2(dy, dx));

    Rotation2d currRotation2d = getRobotPose2d().getRotation();
    Rotation2d deltaRotation2d = currRotation2d.minus(targetRotation2d); 
    return deltaRotation2d.unaryMinus();
  }

  @Log (name="Robot to speaker rotation (deg)")
  public double getRobotToSpeakerRotationDegrees() {
    return getRobotToSpeakerRotation2d().getDegrees();
  }
}
