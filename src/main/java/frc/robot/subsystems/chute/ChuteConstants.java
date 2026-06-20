package frc.robot.subsystems.chute;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorIOTalonFXS;
import frc.robot.Ports;
import frc.robot.Robot;


public class ChuteConstants {

	public static final Voltage kReverseVoltage = Units.Volts.of(-2);

	// ── Roller pair (top + bottom, TalonFXS / Minion) ──────────────────────

	/** Shared electrical limits and kP for the roller Minions. Per-motor feedforward (kS/kV) and inversion are set by the callers. */
	private static TalonFXSConfiguration getBaseRollerFXSConfig() {
		TalonFXSConfiguration config = new TalonFXSConfiguration();

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 40.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 20.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 20.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.Slot1.kP = 0.1;

		return config;
	}

	public static TalonFXSConfiguration getTopRollerFXSConfig() {
		TalonFXSConfiguration config = getBaseRollerFXSConfig();
		config.Slot1.kS = 0.2;
		config.Slot1.kV = 0.098;
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		return config;
	}

	public static TalonFXSConfiguration getBottomRollerFXSConfig() {
		TalonFXSConfiguration config = getBaseRollerFXSConfig();
		config.Slot1.kS = 0.2;
		config.Slot1.kV = 0.098;
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		return config;
	}

	public static TalonFXSConfiguration getSideFeederFXSConfig() {
		TalonFXSConfiguration config = getBaseRollerFXSConfig();
		config.Slot1.kS = 0.2;
		config.Slot1.kV = 0.098;
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		return config;
	}

	public static MotorIOTalonFXS.MotorIOTalonFXSConfig getRollerIOConfig() {
		MotorIOTalonFXS.MotorIOTalonFXSConfig config = new MotorIOTalonFXS.MotorIOTalonFXSConfig();
		config.mainConfig = getTopRollerFXSConfig();
		config.motorArrangement = MotorArrangementValue.Minion_JST;
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.CHUTE_TOP_ROLLER.getDeviceNumber();
		config.mainBus = Ports.CHUTE_TOP_ROLLER.getBus();

		return config;
	}

	public static MotorIOTalonFXS.MotorIOTalonFXSConfig getBottomRollerIOConfig() {
		MotorIOTalonFXS.MotorIOTalonFXSConfig config = new MotorIOTalonFXS.MotorIOTalonFXSConfig();
		config.mainConfig = getBottomRollerFXSConfig();
		config.motorArrangement = MotorArrangementValue.Minion_JST;
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.CHUTE_BOTTOM_ROLLER.getDeviceNumber();
		config.mainBus = Ports.CHUTE_BOTTOM_ROLLER.getBus();

		return config;
	}

	public static MotorIOTalonFXS.MotorIOTalonFXSConfig getVertRollerIOConfig() {
		MotorIOTalonFXS.MotorIOTalonFXSConfig config = new MotorIOTalonFXS.MotorIOTalonFXSConfig();
		config.mainConfig = getSideFeederFXSConfig();
		config.motorArrangement = MotorArrangementValue.Minion_JST;
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.CHUTE_SIDE_FEEDER.getDeviceNumber();
		config.mainBus = Ports.CHUTE_SIDE_FEEDER.getBus();

		return config;
	}

	public static MotorIOTalonFXS getRollerIO() {
		return new MotorIOTalonFXS(getRollerIOConfig());
	}
	public static MotorIOTalonFXS getBottomRollerIO() {
		return new MotorIOTalonFXS(getBottomRollerIOConfig());
	}

	// ── Feeder roller (TalonFX) ─────────────────────────────────────────────

	public static TalonFXConfiguration getFeederFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 50.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 20.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 20.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.Slot1.kS = 0.25;
		config.Slot1.kV = 0.12;
		config.Slot1.kP = 0.1;

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return config;
	}

	public static MotorIOTalonFX.MotorIOTalonFXConfig getFeederIOConfig() {
		MotorIOTalonFX.MotorIOTalonFXConfig config = new MotorIOTalonFX.MotorIOTalonFXConfig();
		config.mainConfig = getFeederFXConfig();
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.CHUTE_FEEDER.getDeviceNumber();
		config.mainBus = Ports.CHUTE_FEEDER.getBus();
		return config;
	}


	public static MotorIOTalonFXS getVertFeederIO() {
		return new MotorIOTalonFXS(getVertRollerIOConfig());
	}

	public static MotorIOTalonFX getFeederIO() {
		return new MotorIOTalonFX(getFeederIOConfig());
	}

}
