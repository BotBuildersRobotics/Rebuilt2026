package frc.robot.subsystems.chute;

import frc.robot.lib.LoggedTracer;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorIOTalonFXS;
import frc.robot.lib.io.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChuteSubsystem extends SubsystemBase {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(ChuteConstants.kReverseVoltage);

	// Per-motor shoot speeds (RPS) so each motor can be tuned independently.
	private static final LoggedTunableNumber topRollerSpeed = new LoggedTunableNumber("Chute/TopRollerSpeed");
	private static final LoggedTunableNumber bottomRollerSpeed = new LoggedTunableNumber("Chute/BottomRollerSpeed");
	private static final LoggedTunableNumber sideFeederSpeed = new LoggedTunableNumber("Chute/SideFeederSpeed");
	private static final LoggedTunableNumber feederSpeed = new LoggedTunableNumber("Chute/FeederSpeed");

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
		topRollerSpeed.initDefault(90); // RPS
		bottomRollerSpeed.initDefault(90); // RPS
		sideFeederSpeed.initDefault(50); // RPS
		feederSpeed.initDefault(80); // RPS
	}

	@Override
	public void periodic() {
		rollers.updateInputs();
		bottomRoller.updateInputs();
		sideFeeder.updateInputs();
		feeder.updateInputs();

		// Log the standard motor telemetry (isOK, temperature, currents, velocity, voltage) for each
		// motor. This subsystem holds MotorIOs directly instead of extending MotorSubsystem, so it
		// has to invoke the shared logging itself.
		MotorSubsystem.recordMotorTelemetry("Chute/Rollers", rollers);
		MotorSubsystem.recordMotorTelemetry("Chute/BottomRoller", bottomRoller);
		MotorSubsystem.recordMotorTelemetry("Chute/SideFeeder", sideFeeder);
		MotorSubsystem.recordMotorTelemetry("Chute/Feeder", feeder);
		LoggedTracer.record("Chute");
	}

	public void applySetpoint(Setpoint setpoint) {
		rollers.applySetpoint(setpoint);
		feeder.applySetpoint(setpoint);
		sideFeeder.applySetpoint(setpoint);
		bottomRoller.applySetpoint(setpoint);
	}

	/** Drives each motor at its own tunable velocity setpoint. Re-evaluates the tunables each call. */
	private void applyShootSpeeds() {
		rollers.applySetpoint(Setpoint.withVelocitySetpoint(RotationsPerSecond.of(topRollerSpeed.get())));
		bottomRoller.applySetpoint(Setpoint.withVelocitySetpoint(RotationsPerSecond.of(bottomRollerSpeed.get())));
		sideFeeder.applySetpoint(Setpoint.withVelocitySetpoint(RotationsPerSecond.of(sideFeederSpeed.get())));
		feeder.applySetpoint(Setpoint.withVelocitySetpoint(RotationsPerSecond.of(feederSpeed.get())));
	}

	/** Runs every motor at its own closed-loop velocity setpoint. Re-evaluates the tunables each cycle. */
	public Command runShootCommand() {
		return runOnce(this::applyShootSpeeds);
	}

	/**
	 * Runs every motor at its own shoot speed when {@code ready} is true, idles otherwise.
	 */
	public Command runShootCommandGated(BooleanSupplier ready) {
		return run(() -> {
			if (ready.getAsBoolean()) {
				applyShootSpeeds();
			} else {
				applySetpoint(IDLE);
			}
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
