package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.io.MotorIO;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorIOSim;
import frc.robot.Ports;
import frc.robot.Robot;


public class TurretConstants {
	
	public static final int id = 0;
	//public static final double anglePerCount = 360; //TODO find this
	//public static final Translation2d turretOffset = new Translation2d(0, 0);

	public static final double toleranceDeg = 1;
	public static final double debounceTime = 0.15;

	public static final double maxLimit = 90;
	public static final double minLimit = -90;


	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 120.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;


		config.MotionMagic.MotionMagicCruiseVelocity = 1400;//  200 * 2 * Math.PI; // rotations/sec at mechanism
   		config.MotionMagic.MotionMagicAcceleration = 1400;//290 * 2 * Math.PI; // rotations/sec² at mechanism
    	//config.MotionMagic.MotionMagicJerk = 1100;
		

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;
		
		config.Feedback.SensorToMechanismRatio = 41.666667;
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


		config.Slot0.kP = 50;//25.88; 
		config.Slot0.kI = 0.002;
		config.Slot0.kD = 0.005;
		config.Slot0.kS = 1.90;
		//config.Slot0.kA = 0.20;
		

		
		return config;
	}

	public static frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig getIOConfig() {
		frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig config = new frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.TURRET.getDeviceNumber();
		config.mainBus = Ports.TURRET.getBus();
		return config;
	}

	public static MotorIO getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOSim(getSimConfig());
		}
	}

	public static MotorIOSim.MotorIOSimConfig getSimConfig() {
		MotorIOSim.MotorIOSimConfig config = new MotorIOSim.MotorIOSimConfig();
		config.unit = Units.Rotations;
		config.time = Units.Minute;
		config.gearing = 40; // Must match SensorToMechanismRatio
		config.jKgMetersSquared = 0.01; // Moment of inertia for turret (increased for stability)
		config.inverted = false; // Normal direction for field-relative tracking (turret counter-rotates)

		// PID gains tuned for simulation (lower than real hardware but still responsive)
		config.positionKp = 15.0;
		config.positionKi = 0.0;
		config.positionKd = 0.5;
		config.positionTolerance = 0.02;

		config.velocityKp = 0.5;
		config.velocityKi = 0.0;
		config.velocityKd = 0.0;

		// Motion profile constraints (converted from rotations/sec to rad/s)
		config.maxVelocity = 80 * 2 * Math.PI; // 80 rot/s = ~502 rad/s
		config.maxAcceleration = 160 * 2 * Math.PI; // 160 rot/s² = ~1005 rad/s²

		// Soft limits (in radians)
		config.forwardSoftLimit = java.lang.Math.toRadians(90);
		config.reverseSoftLimit = java.lang.Math.toRadians(-90);

		return config;
	}

}