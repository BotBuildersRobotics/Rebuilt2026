package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.lib.io.MotorIOTalonFX;

public class ShooterConstants {

	public static final Voltage kShootVoltage = Units.Volts.of( 4.5);
    
    public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 120.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		
		config.MotionMagic.MotionMagicAcceleration = 100;
		config.MotionMagic.MotionMagicCruiseVelocity = 1100;
		//config.MotionMagic.MotionMagicExpo_kA = 0.0074516;
		config.MotionMagic.MotionMagicExpo_kA = 0.074516;
		config.MotionMagic.MotionMagicExpo_kV = 0.12034;
		
		// Slot 1: Voltage-based velocity control (VelocityVoltage)
		config.Slot1.kS = 0.30532;
		config.Slot1.kP = 0.453558;
		config.Slot1.kV = 0.12034;//0.12034;

		// Slot 0: Torque current FOC velocity control (VelocityTorqueCurrentFOC)
		// Units are in Amps instead of Volts
		config.Slot0.kS = 0.085;   // static friction compensation in amps
		config.Slot0.kP = 3.35;   // amps per RPS of error
		config.Slot0.kV = 0.25;   // amps per RPS of target velocity
		
		

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return config;
	}

	public static frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig getIOConfig() {
		frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig config = new frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.SHOOTER.getDeviceNumber();
		config.mainBus = Ports.SHOOTER.getBus();
		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		
		return new MotorIOTalonFX(getIOConfig());
		
	}
}
