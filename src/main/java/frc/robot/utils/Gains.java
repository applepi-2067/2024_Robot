package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.SparkMaxPIDController;


public class Gains {
    public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final double kIzone;
	public final double kPeakOutput;
	
	public Gains(double _kP, double _kF, double _kPeakOutput) {
		this(_kP, 0.0, 0.0, _kF, 0.0, _kPeakOutput);
	}

	public Gains(
		double _kP, double _kI, double _kD,
		double _kF, double _kIzone, double _kPeakOutput
	) {
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
    }

	public void setGains(SparkMaxPIDController pidController, int pidSlot) {
		pidController.setP(kP, pidSlot);
        pidController.setI(kI, pidSlot);
        pidController.setD(kD, pidSlot);
        pidController.setFF(kF, pidSlot); 
        pidController.setIZone(kIzone, pidSlot);
        pidController.setOutputRange(-kPeakOutput, kPeakOutput, pidSlot);
	}

	public void setGains(WPI_TalonFX motor, int pidSlot, int pidLoop, int timeoutMS) {
		motor.selectProfileSlot(pidSlot, pidLoop);

        motor.config_kF(pidSlot, kF, timeoutMS);
        motor.config_kP(pidSlot, kP, timeoutMS);
        motor.config_kI(pidSlot, kI, timeoutMS);
        motor.config_kD(pidSlot, kD, timeoutMS);
        motor.config_IntegralZone(pidSlot, kIzone, timeoutMS);

        // Set peak (max) and nominal (min) outputs.
        motor.configNominalOutputForward(0.0, timeoutMS);
        motor.configNominalOutputReverse(0.0, timeoutMS);
        motor.configPeakOutputForward(kPeakOutput, timeoutMS);
        motor.configPeakOutputReverse(-1.0 * kPeakOutput, timeoutMS);
	}

	public static void configSmartMotion(
		SparkMaxPIDController pidController,
		double maxVelocityRPM,
		double minVelocityRPM,
		double maxAccelRPMPerSec,
		double allowedErrorRotations,
		int pidSlot
	) {
        pidController.setSmartMotionMaxVelocity(maxVelocityRPM, pidSlot);
        pidController.setSmartMotionMinOutputVelocity(minVelocityRPM, pidSlot);
        pidController.setSmartMotionMaxAccel(maxAccelRPMPerSec, pidSlot);
        pidController.setSmartMotionAllowedClosedLoopError(allowedErrorRotations, pidSlot);
    }

	public static void configMotionMagic(
		WPI_TalonFX motor,
		int cruiseVelocityTicksPer100MS,
		int maxAccelTicksPer100MS_PerSec,
		int timeoutMS
	) {
		motor.configMotionCruiseVelocity(cruiseVelocityTicksPer100MS, timeoutMS);
		motor.configMotionAcceleration(maxAccelTicksPer100MS_PerSec, timeoutMS);
	}
}
