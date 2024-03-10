package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
// import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

// import frc.robot.utils.Utils;
import frc.robot.constants.RobotMap;

public class Shoulder extends SubsystemBase implements Loggable {
    private static Shoulder instance = null;

    private final TalonFX m_motor;

    // Motor settings.
    private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withSupplyCurrentThreshold(60)
        .withSupplyTimeThreshold(0.5)
        .withSupplyCurrentLimit(40);
    private static final double PERCENT_DEADBAND = 0.001;
    
    // Conversion constants.
    private static final double GEAR_RATIO = (54.0 / 17.0) * (5.0 * 4.0);

    // PID.
    private static final int K_TIMEOUT_MS = 10;
    private static final Slot0Configs PID_GAINS = new Slot0Configs()
        .withKP(100.0)
        .withKV(4.0);

    private static final double FALCON_500_MAX_SPEED_RPS = 6380.0 / 60.0;
    private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(FALCON_500_MAX_SPEED_RPS)
        .withMotionMagicAcceleration(FALCON_500_MAX_SPEED_RPS / 16.0);

    public static final double HORIZONTAL_FEED_FORWARD_VOLTAGE = -0.25;

    public static final double ZERO_POSITION_DEGREES = 60.0;
    public static final double ALLOWABLE_ERROR_DEGREES = 1.0;

    // Dist -> theta quadratic fit coefficients.
    private static final double A = 0.001376;
    private static final double B = -0.5706;
    private static final double C = 78.48 - 1.5;

    public static Shoulder getInstance() {
        if (instance == null) {
          instance = new Shoulder();
        }
        return instance;
      }

    private Shoulder() {
        m_motor = new TalonFX(RobotMap.canIDs.SHOULDER);

        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        m_motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS, K_TIMEOUT_MS);

        m_motor.getConfigurator().apply(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(GEAR_RATIO),
            K_TIMEOUT_MS
        );
        m_motor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withDutyCycleNeutralDeadband(PERCENT_DEADBAND)
                .withInverted(InvertedValue.CounterClockwise_Positive),
            K_TIMEOUT_MS
        );

        // Config position control.
        m_motor.getConfigurator().apply(PID_GAINS, K_TIMEOUT_MS);
        m_motor.getConfigurator().apply(MOTION_MAGIC_CONFIGS, K_TIMEOUT_MS);

        m_motor.setPosition(Units.degreesToRotations(ZERO_POSITION_DEGREES), K_TIMEOUT_MS);
    }

    @Log (name = "Position (rot)")
    public double getPositionRotations() {
        return m_motor.getPosition().getValueAsDouble();
    }

    @Log (name = "Position (deg)")
    public double getPositionDegrees() {        
        return getPositionRotations() * 360.0;
    }

    private double getFeedForwardVoltage(double degrees) {
        return Math.cos(Units.degreesToRadians(degrees)) * HORIZONTAL_FEED_FORWARD_VOLTAGE;
    }

    public void setTargetPositionDegrees(double degrees) {
        double rotations = degrees / 360.0;
        MotionMagicVoltage request = new MotionMagicVoltage(rotations)
            .withFeedForward(getFeedForwardVoltage(degrees));
        m_motor.setControl(request);
    }

    @Log (name = "Current (A)")
    public double getCurrentAmps() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getSpeakerScoreAngleDegrees() {
        double x = Drivetrain.getInstance().getDistToSpeakerInches();
        double theta = (A * (x * x)) + (B * x) + C;

        // Clamp theta to (0, zero_position).
        theta = Math.max(theta, 0.0);
        theta = Math.min(theta, ZERO_POSITION_DEGREES);
        return theta;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Speaker score angle (deg)", getSpeakerScoreAngleDegrees());
    }

    // @Config
    // public void setPIDs(double kV, double kP) {
    //     m_motor.getConfigurator().apply(new Slot0Configs().withKV(kV).withKP(kP), K_TIMEOUT_MS);
    // }
}
