package frc.robot.subsystems.rollerFloor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.Robot;


public class RollerFloorConstants {

	public static final Voltage kReverseVoltage = Units.Volts.of(-1);
	public static final Voltage kFloorVoltage = Units.Volts.of(-6);

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

		config.Slot1.kS = 0.25;
		config.Slot1.kV = 0.12;
		config.Slot1.kP = 0.1;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return config;
	}

	public static MotorIOTalonFX.MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFX.MotorIOTalonFXConfig config = new MotorIOTalonFX.MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.ROLLER_FLOOR.getDeviceNumber();
		config.mainBus = Ports.ROLLER_FLOOR.getBus();
		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		return new MotorIOTalonFX(getIOConfig());
	}

}
