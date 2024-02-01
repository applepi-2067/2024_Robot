package frc.robot.subsystems;


// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.ShooterSetupMotor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotMap;
import frc.robot.utils.Conversions;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


public class Shooter extends SubsystemBase implements Loggable {
    private final TalonFX m_shooterTop;
    private final TalonFX m_shooterBottom;

    private static Shooter instance = null;

    // Conversion constants.
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0 / 2.0);
    
    private static final double SPEED_FOR_SHOOTING_RPM = 4800.0;
    private static final double SPEED_FOR_SHOOTING_THRESHOLD = 0.05; // Percent

    private Shooter() {
        m_shooterTop = new TalonFX(RobotMap.canIDs.Shooter.TOP_SHOOTER);
        m_shooterBottom = new TalonFX(RobotMap.canIDs.Shooter.BOTTOMSHOOTER);
        ShooterSetupMotor.setupMotor(m_shooterTop, GEAR_RATIO);
        ShooterSetupMotor.setupMotor(m_shooterBottom, GEAR_RATIO);
    }

    public static Shooter getInstance(){
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    @Log (name = "wheel v (m/s)")
    public double getVelocityMPS() {
        return Conversions.rotationsToArcLength(getMotorVelocityRPS(), WHEEL_RADIUS_METERS);
    }

    @Log (name = "wheel v (rps)")
    public double getMotorVelocityRPS() {
        return m_shooterTop.getVelocity().getValueAsDouble();
    }

    @Log (name = "motor v (rpm)")
    public double getMotorVelocityRPM() {
        return getMotorVelocityRPS() * 60.0;
    }

    public double getMotorRotations() {
        return m_shooterTop.getRotorPosition().getValueAsDouble(); // TODO CHECK
    }

    public boolean shootingVelocityReached() {
        double error = Math.abs(Math.abs(getMotorVelocityRPM()) - SPEED_FOR_SHOOTING_RPM);
        return error < SPEED_FOR_SHOOTING_RPM * SPEED_FOR_SHOOTING_THRESHOLD;
    }

    public void setTargetMotorRPM(double motorRPM) {
        double wheelRPM = motorRPM;
        double wheelRPS = wheelRPM / 60.0;
        setTargetMotorRPS(wheelRPS);
    }

    public void setTargetMotorRPS(double motorRPS) {
        m_shooterTop.setControl(new MotionMagicVelocityVoltage(-motorRPS).withSlot(0));
        m_shooterBottom.setControl(new MotionMagicVelocityVoltage(motorRPS).withSlot(0));
    }

    public void rampUpTargetMotor() {
        setTargetMotorRPM(SPEED_FOR_SHOOTING_RPM);
    }
}
