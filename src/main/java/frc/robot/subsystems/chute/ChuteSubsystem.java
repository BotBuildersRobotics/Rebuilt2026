package frc.robot.subsystems.chute;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorIOTalonFXS;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChuteSubsystem extends SubsystemBase {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(ChuteConstants.kReverseVoltage);

	private static final LoggedTunableNumber shootSpeed = new LoggedTunableNumber("Chute/ShootSpeed");

	public static final ChuteSubsystem mInstance = new ChuteSubsystem();

	private final MotorIOTalonFXS rollers;
	private final MotorIOTalonFXS bottomRoller;
	private final MotorIOTalonFXS sideFeeder;
	private final MotorIOTalonFX feeder;

	public ChuteSubsystem() {
		super("Chute");
		rollers = ChuteConstants.getRollerIO();
		bottomRoller = ChuteConstants.getBottomRollerIO();
		feeder = ChuteConstants.getFeederIO();
		sideFeeder = ChuteConstants.getVertFeederIO();
		shootSpeed.initDefault(80); // RPS
	}

	@Override
	public void periodic() {
		rollers.updateInputs();
		feeder.updateInputs();
		bottomRoller.updateInputs();;
	}

	public void applySetpoint(Setpoint setpoint) {
		rollers.applySetpoint(setpoint);
		feeder.applySetpoint(setpoint);
		sideFeeder.applySetpoint(setpoint);
		bottomRoller.applySetpoint(setpoint);
	}

	/** Runs all three motors at the closed-loop velocity setpoint. Re-evaluates the tunable each cycle. */
	public Command runShootCommand() {
		return runOnce(() -> {
			Setpoint s = Setpoint.withVelocitySetpoint(RotationsPerSecond.of(shootSpeed.get()));
			applySetpoint(s);
		});
	}

	/**
	 * Runs all three motors at shoot speed when {@code ready} is true, idles otherwise.
	 */
	public Command runShootCommandGated(BooleanSupplier ready) {
		return run(() -> {
			//if (ready.getAsBoolean()) {
				//Setpoint s = Setpoint.withVelocitySetpoint(RotationsPerSecond.of(shootSpeed.get()));
				Setpoint s = Setpoint.withVoltageSetpoint(Units.Volts.of(12));
				
				applySetpoint(s);
			//} else {
			//	applySetpoint(IDLE);
			//}
		});
	}

	public Command setpointCommand(Setpoint setpoint) {
		return runOnce(() -> applySetpoint(setpoint));
	}

	/** Stops all three motors. */
	public Command stopCommand() {
		return setpointCommand(IDLE);
	}

}
