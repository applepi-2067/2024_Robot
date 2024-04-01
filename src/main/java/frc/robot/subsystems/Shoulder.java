package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
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

    private final DigitalInput m_zeroingSensor;


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

    public static final double MIN_ANGLE_DEGREES = 60.0;
    public static final double ALLOWABLE_ERROR_DEGREES = 1.0;

    // Dist -> theta quadratic fit coefficients.
    private static final double A = 0.00231639;
    private static final double B = -0.721311;
    private static final double C = 80.4049 + 1.3;

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

        m_zeroingSensor = new DigitalInput(RobotMap.dios.SHOULDER_ZERO_SENSOR);
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

    public void setSeedPositionDegrees(double degrees) {
        m_motor.setPosition(Units.degreesToRotations(degrees));
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
        theta = Math.min(theta, MIN_ANGLE_DEGREES);
        return theta;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Speaker score angle (deg)", getSpeakerScoreAngleDegrees());
    }

    @Log (name = "Shoulder Zero Sensor")
    public boolean zeroSensorTriggered() {
      return ! m_zeroingSensor.get();
    }

    public void setPercentOutput(double percentOutput) {
        m_motor.setControl(new DutyCycleOut(percentOutput));
    }

    // @Config
    // public void setPIDs(double kV, double kP) {
    //     m_motor.getConfigurator().apply(new Slot0Configs().withKV(kV).withKP(kP), K_TIMEOUT_MS);
    // }
}
