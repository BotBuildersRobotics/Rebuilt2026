package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.Robot;


public class IntakeConstants {
	
	public static final Voltage kIntakeVoltage = Units.Volts.of( 9);
	public static final Voltage kReverseVoltage = Units.Volts.of(-5);

	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		// NOTE: these limits are PER MOTOR and the intake runs two opposing motors,
		// so the battery sees roughly double each of these numbers.
		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 80.0; // torque/heat cap (was 120)

		// Allow a 50 A burst to grab/unjam a game piece, then clamp to 25 A steady
		// so a stall doesn't sit at full current draining the battery.
		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 50.0; // burst ceiling (was 60)
		config.CurrentLimits.SupplyCurrentLowerLimit = 25.0; // steady-state (was 60 = no step-down)
		config.CurrentLimits.SupplyCurrentLowerTime = 0.5; // seconds at burst before clamping

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;
		

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		return config;
	}

	public static frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig getIOConfig() {
		frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig config = new frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.INTAKE.getDeviceNumber();
		config.mainBus = Ports.INTAKE.getBus();

		config.followerConfig = getFXConfig();
		config.followerOpposeMain = new boolean[] {true};
		config.followerBuses = new String[] {Ports.INTAKE_2.getBus()};
		config.followerIDs = new int[] {Ports.INTAKE_2.getDeviceNumber()};
		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		
		return new MotorIOTalonFX(getIOConfig());
		
	}

}