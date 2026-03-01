package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.lib.Util;
import frc.robot.lib.io.MotorIOTalonFX;

public class HoodConstants {
    
    public static final Distance kStowPosition = Units.Centimeters.of(0);
    public static final Distance kEpsilonThreshold = Units.Centimeters.of(0.1);

    public static final Util.DistanceAngleConverter converter = new Util.DistanceAngleConverter(Units.Centimeters.of(2.0));
	
	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 120.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;
		
		config.Feedback.SensorToMechanismRatio = 0.3;
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		// PID gains for position control 
		config.Slot0.kP = 10;
		config.Slot0.kI = 0.0;
		config.Slot0.kD = 0.0;
		config.Slot0.kS = 0.8;
		config.Slot0.kV = 0.0;
		config.Slot0.kG = 0.0;

		// Motion Magic profile constraints
		config.MotionMagic.MotionMagicCruiseVelocity =  10; // rotations/sec at mechanism
   		config.MotionMagic.MotionMagicAcceleration = 10; // rotations/sec² at mechanism
    	

		return config;
	}

	public static frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig getIOConfig() {
		frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig config = new frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.HOOD.getDeviceNumber();
		config.mainBus = Ports.HOOD.getBus();
		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		
		return new MotorIOTalonFX(getIOConfig());
		
	}
}
