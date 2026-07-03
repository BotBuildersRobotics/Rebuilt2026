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

	// Rotor rotations per turret rotation. If the turret physically under/over-travels
	// vs commanded, this is the value to correct (see TurretSubsystem calibration buttons).
	public static final double SENSOR_TO_MECHANISM_RATIO = 41.666667;

	// --- Homing (drift-correction) sensor -------------------------------------------------
	// roboRIO analog input channel for the fixed active-high digital sensor that detects the
	// magnet on the moving turret (digital sensor wired into an analog port). Used to re-zero
	// the turret when it returns to stow.
	public static final int HOMING_ANALOG_CHANNEL = 3;
	// True turret angle (radians, robot-relative) when the homing magnet is centered on the
	// sensor. Stow == turret zero on this robot, so this is 0. If the sensor is physically
	// mounted a few degrees off stow, put that offset here instead.
	public static final double HOMING_INDEX_POSITION_RAD = 0.0;


	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.StatorCurrentLimit = 120.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;


		// Mechanism units (after SensorToMechanismRatio). Kraken X60 free speed ≈ 100 rot/s rotor
		// ÷ 41.67 ≈ 2.4 rot/s at the turret. The 07-03 tuning log showed cruise at 2.4 == free speed,
		// which left no voltage headroom for kP and let the turret carry max momentum into the target
		// (33%+ overshoot, ~2s ring). Dropped to ~75% of free speed to restore headroom and cut
		// arrival momentum — the biggest lever on the overshoot. Retune after re-logging.
		config.MotionMagic.MotionMagicCruiseVelocity = 1.8; // rot/s at mechanism (~75% of free speed)
		config.MotionMagic.MotionMagicAcceleration = 12.0; // rot/s² at mechanism (reaches cruise in ~0.15s)
    	//config.MotionMagic.MotionMagicJerk = 1100;
		

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;
		
		config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


		// First-pass gains from the 07-03 tuning-log analysis (overshoot + ~2s ring, kD contribution
		// was ~0V). Lowered kP now that cruise gives headroom, added real kD for damping, and added
		// kA so the profile's acceleration is fed forward instead of leaning on kP. Starting points —
		// re-log and trim (watch Turret Motor/ClosedLoop/Error and overshoot).
		config.Slot0.kP = 20;   // was 30 — reduce aggression; let kD do the damping
		config.Slot0.kI = 0.002;
		config.Slot0.kD = 1.5;  // was 0.005 (effectively zero) — add derivative braking
		config.Slot0.kS = 1.90;
		// Volts per mechanism rot/s. (12 - kS) / 2.4 rot/s free speed ≈ 4.2. Feeds the profile
		// velocity forward instead of leaning on kP alone.
		config.Slot0.kV = 4.2;
		config.Slot0.kA = 0.15; // was 0 — feed profile acceleration forward (SysId would refine this)
		

		
		return config;
	}

	public static frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig getIOConfig() {
		frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig config = new frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Minute;
		config.unit = Units.Rotations;
		config.mainID = Ports.TURRET.getDeviceNumber();
		config.mainBus = Ports.TURRET.getBus();
		// High-rate closed-loop telemetry for PID / Motion Magic tuning (turret only).
		config.closedLoopUpdateHz = 100.0;
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