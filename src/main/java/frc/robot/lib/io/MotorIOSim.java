package frc.robot.lib.io;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Simulated MotorIO implementation using WPILib's DCMotorSim for physics simulation.
 * Supports position control, velocity control, motion magic, voltage control, and duty cycle control.
 */
public class MotorIOSim extends MotorIO {
	private final DCMotorSim motorSim;
	private final PIDController positionController;
	private final PIDController velocityController;
	private final TrapezoidProfile motionProfile;
	private TrapezoidProfile.State motionProfileState;
	private TrapezoidProfile.State motionProfileGoal;

	private double appliedVoltage = 0.0;
	private boolean isInBrakeMode = true;
	private boolean softLimitsEnabled = false;
	private boolean inverted = false;
	private double forwardSoftLimit = Double.POSITIVE_INFINITY;
	private double reverseSoftLimit = Double.NEGATIVE_INFINITY;

	/**
	 * Creates a simulated motor IO.
	 *
	 * @param config Configuration for the simulated motor.
	 */
	public MotorIOSim(MotorIOSimConfig config) {
		super(config.unit, config.time);

		// Create the DC motor system using LinearSystemId
		LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(
			config.motor,
			config.jKgMetersSquared,
			config.gearing
		);

		this.motorSim = new DCMotorSim(plant, config.motor);

		this.positionController = new PIDController(config.positionKp, config.positionKi, config.positionKd);
		this.velocityController = new PIDController(config.velocityKp, config.velocityKi, config.velocityKd);

		this.motionProfile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(config.maxVelocity, config.maxAcceleration)
		);

		this.motionProfileState = new TrapezoidProfile.State(0, 0);
		this.motionProfileGoal = new TrapezoidProfile.State(0, 0);

		this.forwardSoftLimit = config.forwardSoftLimit;
		this.reverseSoftLimit = config.reverseSoftLimit;
		this.inverted = config.inverted;

		positionController.setTolerance(config.positionTolerance);
	}

	@Override
	public void updateInputs() {
		// Update the motor simulation
		motorSim.update(Constants.loopPeriodSecs);

		// Apply brake friction when in brake mode and no voltage applied
		if (isInBrakeMode && Math.abs(appliedVoltage) < 0.01) {
			double currentVelocity = motorSim.getAngularVelocityRadPerSec();
			double brakeVoltage = -Math.signum(currentVelocity) * 0.5;
			if (Math.abs(currentVelocity) < 0.1) {
				brakeVoltage = 0;
			}
			applyVoltageToSim(brakeVoltage);
		}

		// Update inputs from simulation
		inputs.position = unitType.of(motorSim.getAngularPositionRotations());
		inputs.velocity = unitType.per(time).of(motorSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI));
		inputs.motorVoltage = Units.Volts.of(appliedVoltage);

		// Simulate current draw based on voltage and velocity
		double currentDraw = motorSim.getCurrentDrawAmps();
		inputs.statorCurrent = Units.Amps.of(currentDraw);
		inputs.supplyCurrent = Units.Amps.of(currentDraw);

		// Simulate motor temperature (simple model)
		double tempIncrease = Math.abs(currentDraw) * 0.01;
		inputs.motorTemperature = Units.Celsius.of(20.0 + tempIncrease);

		// Telemetry for debugging
		SmartDashboard.putNumber("Sim/MotorPositionRad", motorSim.getAngularPositionRad());
		SmartDashboard.putNumber("Sim/MotorVelocityRadPerSec", motorSim.getAngularVelocityRadPerSec());
		SmartDashboard.putNumber("Sim/AppliedVoltage", appliedVoltage);
		SmartDashboard.putNumber("Sim/CurrentDraw", currentDraw);
	}

	@Override
	public void setPositionVelocitySetpoint(Angle mechAngle, AngularVelocity velocity){
		
	}

	@Override
	public void setCurrentPosition(Angle mechanismPosition) {
		double positionRadians = mechanismPosition.in(Units.Radians);
		motorSim.setState(positionRadians, motorSim.getAngularVelocityRadPerSec());
		motionProfileState = new TrapezoidProfile.State(positionRadians, motorSim.getAngularVelocityRadPerSec());
		positionController.reset();
	}

	@Override
	public void zeroSensors() {
		setCurrentPosition(Units.Rotations.of(0.0));
	}

	@Override
	public void setNeutralBrake(boolean wantsBrake) {
		this.isInBrakeMode = wantsBrake;
	}

	@Override
	public void useSoftLimits(boolean enable) {
		this.softLimitsEnabled = enable;
	}

	@Override
	protected void setNeutralSetpoint() {
		appliedVoltage = 0.0;
		if (isInBrakeMode) {
			// Apply brake friction
			double currentVelocity = motorSim.getAngularVelocityRadPerSec();
			double brakeVoltage = -Math.signum(currentVelocity) * 0.5;
			if (Math.abs(currentVelocity) < 0.1) {
				brakeVoltage = 0;
			}
			applyVoltageToSim(brakeVoltage);
		} else {
			applyVoltageToSim(0.0);
		}
	}

	@Override
	protected void setCoastSetpoint() {
		appliedVoltage = 0.0;
		applyVoltageToSim(0.0);
	}

	@Override
	protected void setVoltageSetpoint(Voltage voltage) {
		appliedVoltage = voltage.in(Units.Volts);
		appliedVoltage = clampVoltageToLimits(appliedVoltage);
		applyVoltageToSim(appliedVoltage);
	}

	@Override
	protected void setMotionMagicSetpoint(Angle mechanismPosition) {
		double goalPositionRadians = mechanismPosition.in(Units.Radians);
		goalPositionRadians = clampPositionToLimits(goalPositionRadians);

		double currentPosition = motorSim.getAngularPositionRad();
		double currentVelocity = motorSim.getAngularVelocityRadPerSec();

		// Only resync profile if diverged significantly (> 0.1 rad error)
		double positionError = Math.abs(motionProfileState.position - currentPosition);
		if (positionError > 0.1) {
			motionProfileState = new TrapezoidProfile.State(currentPosition, currentVelocity);
		}

		motionProfileGoal = new TrapezoidProfile.State(goalPositionRadians, 0);

		// Calculate next motion profile state
		motionProfileState = motionProfile.calculate(
			Constants.loopPeriodSecs,
			motionProfileState,
			motionProfileGoal
		);

		// Use position controller to track the motion profile with feedforward
		double feedback = positionController.calculate(currentPosition, motionProfileState.position);

		// Simple velocity feedforward (kV * velocity in rad/s)
		double kV = 0.2; // Velocity feedforward gain
		double feedforward = motionProfileState.velocity * kV;

		appliedVoltage = feedback + feedforward;
		appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));

		// Telemetry for motion magic
		SmartDashboard.putNumber("Sim/ProfileSetpoint", motionProfileState.position);
		SmartDashboard.putNumber("Sim/ProfileVelocity", motionProfileState.velocity);
		SmartDashboard.putNumber("Sim/Goal", goalPositionRadians);
		SmartDashboard.putNumber("Sim/PositionError", positionError);

		applyVoltageToSim(appliedVoltage);
	}

	@Override
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
		double goalVelocityRadPerSec = mechanismVelocity.in(Units.RadiansPerSecond);
		double currentVelocity = motorSim.getAngularVelocityRadPerSec();

		appliedVoltage = velocityController.calculate(currentVelocity, goalVelocityRadPerSec);
		appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));
		applyVoltageToSim(appliedVoltage);
	}

	@Override
	protected void setDutyCycleSetpoint(Dimensionless percent) {
		double percentValue = percent.in(Units.Percent) / 100.0;
		appliedVoltage = percentValue * 12.0;
		appliedVoltage = clampVoltageToLimits(appliedVoltage);
		applyVoltageToSim(appliedVoltage);
	}

	@Override
	protected void setPositionSetpoint(Angle mechanismPosition) {
		double goalPositionRadians = mechanismPosition.in(Units.Radians);
		goalPositionRadians = clampPositionToLimits(goalPositionRadians);

		double currentPosition = motorSim.getAngularPositionRad();
		appliedVoltage = positionController.calculate(currentPosition, goalPositionRadians);
		appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));
		applyVoltageToSim(appliedVoltage);
	}

	/**
	 * Applies voltage to the motor sim with inversion if needed.
	 *
	 * @param voltage Voltage to apply
	 */
	private void applyVoltageToSim(double voltage) {
		double finalVoltage = inverted ? -voltage : voltage;
		motorSim.setInputVoltage(finalVoltage);
	}

	/**
	 * Clamps position to soft limits if enabled.
	 *
	 * @param position Position in radians
	 * @return Clamped position
	 */
	private double clampPositionToLimits(double position) {
		if (!softLimitsEnabled) {
			return position;
		}
		return Math.max(reverseSoftLimit, Math.min(forwardSoftLimit, position));
	}

	/**
	 * Clamps voltage based on current position relative to soft limits.
	 *
	 * @param voltage Voltage to apply
	 * @return Clamped voltage
	 */
	private double clampVoltageToLimits(double voltage) {
		if (!softLimitsEnabled) {
			return voltage;
		}

		double currentPosition = motorSim.getAngularPositionRad();

		// Prevent moving further if at soft limit
		if (currentPosition >= forwardSoftLimit && voltage > 0) {
			return 0.0;
		}
		if (currentPosition <= reverseSoftLimit && voltage < 0) {
			return 0.0;
		}

		return voltage;
	}

	/**
	 * Configuration for MotorIOSim.
	 */
	public static class MotorIOSimConfig {
		public AngleUnit unit = Units.Rotations;
		public TimeUnit time = Units.Seconds;
		public DCMotor motor = DCMotor.getFalcon500(1);
		public double gearing = 1.0;
		public double jKgMetersSquared = 0.001;
		public boolean inverted = false;

		// PID gains for position control
		public double positionKp = 10.0;
		public double positionKi = 0.0;
		public double positionKd = 0.0;
		public double positionTolerance = 0.01;

		// PID gains for velocity control
		public double velocityKp = 0.5;
		public double velocityKi = 0.0;
		public double velocityKd = 0.0;

		// Motion profile constraints
		public double maxVelocity = 10.0; // rad/s
		public double maxAcceleration = 20.0; // rad/s^2

		// Soft limits (in radians)
		public double forwardSoftLimit = Double.POSITIVE_INFINITY;
		public double reverseSoftLimit = Double.NEGATIVE_INFINITY;
	}
}
