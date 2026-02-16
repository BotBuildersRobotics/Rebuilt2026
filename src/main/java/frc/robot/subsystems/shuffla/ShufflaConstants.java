package frc.robot.subsystems.shuffla;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.Robot;


public class ShufflaConstants {
	
	public static final Voltage kShootVoltage = Units.Volts.of( 12);
	public static final Voltage kReverseVoltage = Units.Volts.of(-1);

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
		

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return config;
	}

	public static frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig getIOConfig() {
		frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig config = new frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.SHUFFLA.getDeviceNumber();
		config.mainBus = Ports.SHUFFLA.getBus();
		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		
		return new MotorIOTalonFX(getIOConfig());
		
	}

}