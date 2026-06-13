package frc.robot.lib.io;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

/**
 * Class used to control a main TalonFXS (with Minion motor) and any number of followers.
 * TorqueCurrentFOC is not supported by TalonFXS; velocity/position FOC uses VoltageOut-based requests.
 */
public class MotorIOTalonFXS extends MotorIO {
	protected final TalonFXS main;
	protected final TalonFXS[] followers;
	protected TalonFXSConfiguration config;
	protected TalonFXSConfiguration followerConfig;
	private final MotorIOTalonFX.ControlRequestGetter requestGetter;
	private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
	private ThreadPoolExecutor threadPoolExecutor =
			new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

	public void applyConfig(TalonFXS fxs, TalonFXSConfiguration config) {
		threadPoolExecutor.submit(() -> {
			for (int i = 0; i < 5; i++) {
				StatusCode result = fxs.getConfigurator().apply(config);
				if (result.isOK()) {
					break;
				}
			}
		});
	}

	@Override
	public void updateInputs() {
		updateMotorInputs(inputs, main);
		for (int i = 0; i < followers.length; i++) {
			updateMotorInputs(followerInputs[i], followers[i]);
		}
	}

	protected void updateMotorInputs(Inputs inputsToUpdate, TalonFXS motor) {
		inputsToUpdate.position = motor.getPosition().getValue();
		inputsToUpdate.velocity = motor.getVelocity().getValue();
		inputsToUpdate.statorCurrent = motor.getStatorCurrent().getValue();
		inputsToUpdate.supplyCurrent = motor.getSupplyCurrent().getValue();
		inputsToUpdate.motorVoltage = motor.getMotorVoltage().getValue();
		inputsToUpdate.motorTemperature = motor.getDeviceTemp().getValue();
		inputsToUpdate.isOK = motor.isConnected();
	}

	private void setControl(ControlRequest request) {
		main.setControl(request);
	}

	@Override
	public void setNeutralSetpoint() {
		setControl(new NeutralOut());
	}

	@Override
	public void setCoastSetpoint() {
		setControl(new CoastOut());
	}

	@Override
	protected void setVoltageSetpoint(Voltage voltage) {
		setControl(requestGetter.getVoltageRequest(voltage));
	}

	@Override
	protected void setDutyCycleSetpoint(Dimensionless percent) {
		setControl(requestGetter.getDutyCycleRequest(percent));
	}

	@Override
	protected void setMotionMagicSetpoint(Angle mechanismPosition) {
		setControl(requestGetter.getMotionMagicRequest(mechanismPosition));
	}

	@Override
	protected void setPositionVelocitySetpoint(Angle mechanismPosition, AngularVelocity velocity) {
		setControl(requestGetter.getPositionVelocityRequest(mechanismPosition, velocity));
	}

	@Override
	protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
		setControl(requestGetter.getVelocityRequest(mechanismVelocity));
	}

	@Override
	protected void setVelocityFOCSetpoint(AngularVelocity mechanismVelocity) {
		// TalonFXS does not support TorqueCurrentFOC — use voltage-based FOC velocity instead
		setControl(requestGetter.getVelocityRequest(mechanismVelocity));
	}

	@Override
	protected void setPositionSetpoint(Angle mechanismPosition) {
		// TalonFXS does not support TorqueCurrentFOC — use voltage-based position instead
		setControl(requestGetter.getPositionVelocityRequest(mechanismPosition, Units.RotationsPerSecond.of(0)));
	}

	public void setMotorControl(ControlRequest request) {
		setControl(request);
	}

	@Override
	public void setCurrentPosition(Angle mechanismPosition) {
		threadPoolExecutor.submit(() -> {
			main.setPosition(mechanismPosition);
		});
	}

	@Override
	public void zeroSensors() {
		setCurrentPosition(Units.Rotations.of(0.0));
	}

	private void setNeutralMode(TalonFXS fxs, NeutralModeValue neutralMode) {
		SmartDashboard.putNumber("TALON FXS NEUTRAL MODE SET!!", Timer.getFPGATimestamp());
		threadPoolExecutor.submit(() -> {
			fxs.setNeutralMode(neutralMode);
		});
	}

	@Override
	public void setNeutralBrake(boolean wantsBrake) {
		NeutralModeValue neutralMode = wantsBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		config.MotorOutput.NeutralMode = neutralMode;
		setNeutralMode(main, neutralMode);
		for (TalonFXS fxs : followers) {
			setNeutralMode(fxs, neutralMode);
		}
	}

	@Override
	public void useSoftLimits(boolean enable) {
		changeMainConfig(config -> {
			config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
			config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
			return config;
		});
	}

	public void setMainConfig(TalonFXSConfiguration configuration) {
		config = configuration;
		applyConfig(main, config);
	}

	public void changeMainConfig(UnaryOperator<TalonFXSConfiguration> configChanger) {
		setMainConfig(configChanger.apply(config));
	}

	public void setFollowerConfig(TalonFXSConfiguration configuration) {
		followerConfig = configuration;
		for (TalonFXS fxs : followers) {
			applyConfig(fxs, followerConfig);
		}
	}

	public void changeFollowerConfig(UnaryOperator<TalonFXSConfiguration> configChanger) {
		setFollowerConfig(configChanger.apply(followerConfig));
	}

	public MotorIOTalonFXS(MotorIOTalonFXSConfig config) {
		super(config.unit, config.time, config.followerIDs.length);
		requestGetter = config.requestGetter;
		main = new TalonFXS(config.mainID, new CANBus(config.mainBus));

		config.mainConfig.Commutation.MotorArrangement = config.motorArrangement;
		setMainConfig(config.mainConfig);

		if (config.velocityUpdateHz > 0) {
			main.getVelocity().setUpdateFrequency(config.velocityUpdateHz);
		}

		followers = new TalonFXS[config.followerIDs.length];
		for (int i = 0; i < config.followerIDs.length; i++) {
			followers[i] = new TalonFXS(config.followerIDs[i], new CANBus(config.followerBuses[i]));
			boolean oppose = config.followerOpposeMain.length > i && config.followerOpposeMain[i];
			MotorAlignmentValue alignment = oppose
				? MotorAlignmentValue.Opposed
				: MotorAlignmentValue.Aligned;
			followers[i].setControl(new Follower(config.mainID, alignment));
			if (config.velocityUpdateHz > 0) {
				followers[i].getVelocity().setUpdateFrequency(config.velocityUpdateHz);
			}
		}

		setFollowerConfig(config.followerConfig);
	}

	public static class MotorIOTalonFXSConfig {
		public AngleUnit unit = Units.Rotations;
		public TimeUnit time = Units.Seconds;
		public int mainID = -1;
		public String mainBus = "ASSIGN_BUS";
		public TalonFXSConfiguration mainConfig = new TalonFXSConfiguration();
		public MotorArrangementValue motorArrangement = MotorArrangementValue.Minion_JST;
		public int[] followerIDs = new int[0];
		public String[] followerBuses = new String[0];
		public TalonFXSConfiguration followerConfig = new TalonFXSConfiguration();
		public boolean[] followerOpposeMain = new boolean[0];
		public MotorIOTalonFX.ControlRequestGetter requestGetter = new MotorIOTalonFX.ControlRequestGetter();
		public double velocityUpdateHz = 0;
	}
}
