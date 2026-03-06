package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;


import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.Robot;


public class ClimbConstants {
	
	public static final Voltage kClimbVoltage = Units.Volts.of( 12);
	public static final Voltage kReverseVoltage = Units.Volts.of(-3);

	public static final Voltage kStopVoltage = Units.Volts.of(0);

	// Position setpoints (in rotations)
	public static final Angle kStowedPosition = Units.Rotations.of(0.0);
	public static final Angle kClimbedPosition = Units.Rotations.of(10.0); // Tune this on the robot

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

		// Position control (Slot 0) — tune kP on the robot
		config.Slot0.kP = 20.0;
		config.Slot0.kD = 0.0;
		config.Slot0.kS = 0.0;

		// Motion Magic — conservative speeds for climbing
		config.MotionMagic.MotionMagicCruiseVelocity = 5.0;
		config.MotionMagic.MotionMagicAcceleration = 10.0;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return config;
	}

	public static frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig getIOConfig() {
		frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig config = new frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.CLIMB.getDeviceNumber();
		config.mainBus = Ports.CLIMB.getBus();

		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		
		return new MotorIOTalonFX(getIOConfig());
		
	}

}