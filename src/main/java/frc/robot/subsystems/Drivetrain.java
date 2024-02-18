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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.constants.RobotMap;
import frc.robot.subsystems.swerve.SwerveModule;


public class Drivetrain extends SubsystemBase implements Loggable {
  private static Drivetrain instance = null;

  private static final DecimalFormat rounder = new DecimalFormat("0.0000");

  // Swerve module offsets from center. +x = front of robot, +y = left of robot. 
  private static final double centerToWheelOffsetMeters = Units.inchesToMeters(13.0 - 2.5);
  public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(-centerToWheelOffsetMeters, centerToWheelOffsetMeters),
    new Translation2d(-centerToWheelOffsetMeters, -centerToWheelOffsetMeters),
    new Translation2d(centerToWheelOffsetMeters, centerToWheelOffsetMeters),
    new Translation2d(centerToWheelOffsetMeters, -centerToWheelOffsetMeters)
  );

  // Max speeds.
  public static final double MAX_TRANSLATION_SPEED_METERS_PER_SEC = 5.55;
  public static final double MAX_ACCEL_METERS_PER_SEC_SQUARED = MAX_TRANSLATION_SPEED_METERS_PER_SEC * 2.0;

  public static final double MAX_ROTATION_SPEED_RADIANS_PER_SEC = Math.PI * 4.0;
  public static final double MAX_ROTATION_ACCEL_RADIANS_PER_SEC_SQUARED = MAX_ROTATION_SPEED_RADIANS_PER_SEC * 2.0;
  
  private final SwerveModule[] m_swerveModules;
  
  // Odometry.
  private final SwerveDrivePoseEstimator m_odometry;
  private static final double[] drivetrainStds = {0.0, 0.0, 0.0};  // x, y, heading.
  private static final double[] visionStds = {0.0, 0.0, 0.0};  // TODO: find pose stds.

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

  public void drive(double leftStickX, double leftStickY, double rightStickX) {
    // Deadband and square stick values.
    double absDeadbandThreshold = 0.10;
    leftStickX = deadbandSquareStickInput(leftStickX, absDeadbandThreshold);
    leftStickY = deadbandSquareStickInput(leftStickY, absDeadbandThreshold);
    rightStickX = deadbandSquareStickInput(rightStickX, absDeadbandThreshold);

    // Speeds in robot coords. Negatives account for stick signs.
    double yVelocityMetersPerSecond = -1.0 * leftStickX * MAX_TRANSLATION_SPEED_METERS_PER_SEC;
    double xVelocityMetersPerSecond = -1.0 * leftStickY * MAX_TRANSLATION_SPEED_METERS_PER_SEC;
    double rotationVelocityRadiansPerSecond = -1.0 * rightStickX * MAX_ROTATION_SPEED_RADIANS_PER_SEC;

    // Robot to field oriented speeds.
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xVelocityMetersPerSecond,
      yVelocityMetersPerSecond,
      rotationVelocityRadiansPerSecond,
      Rotation2d.fromDegrees(getHeadingDegrees())
    );

    // Convert to swerve module states, and set states.
    SwerveModuleState[] states = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    setSwerveModuleStates(states);
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
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_TRANSLATION_SPEED_METERS_PER_SEC);

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

  public void addVisionMeaurement(Pose2d visionEstimatedRobotPose2d, double timestampSeconds) {
    m_odometry.addVisionMeasurement(visionEstimatedRobotPose2d, timestampSeconds);
  }

  @Override
  public void periodic() {
    m_pose = getRobotPose2d();
    m_field.setRobotPose(m_pose);
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

  @Log (name="Robot Pose")
  public String getPoseDescription() {
    String poseString = "x (m)=" + rounder.format(m_pose.getX()) + "    ";
    poseString += "y (m)=" + rounder.format(m_pose.getY()) + "    ";
    poseString += "rotation (deg)=" + rounder.format(m_pose.getRotation().getDegrees());
    return poseString;
  }
}
