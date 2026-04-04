package frc.robot.subsystems.chute;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class ChuteSubsystem extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(ChuteConstants.kReverseVoltage);

	private static final LoggedTunableNumber shootSpeed = new LoggedTunableNumber("Chute/ShootSpeed");

	public static final ChuteSubsystem mInstance = new ChuteSubsystem();

	public ChuteSubsystem() {
		super(ChuteConstants.getMotorIO(), "Chute Rollers");
		shootSpeed.initDefault(20); // RPS 
	}

	/** Runs the chute at a closed-loop velocity setpoint. Re-evaluates the tunable each cycle. */
	public Command runShootCommand() {
		return run(() -> applySetpoint(
			Setpoint.withVelocitySetpoint(RotationsPerSecond.of(shootSpeed.get()))
		));
	}

	/**
	 * Runs the chute at shoot speed when {@code ready} is true, idles otherwise.
	 * Automatically pauses feeding if the shooter bogs down and resumes when it recovers.
	 */
	public Command runShootCommandGated(BooleanSupplier ready) {
		return run(() -> {
			if (ready.getAsBoolean()) {
				applySetpoint(Setpoint.withVelocitySetpoint(RotationsPerSecond.of(shootSpeed.get())));
			} else {
				applySetpoint(IDLE);
			}
		});
	}

}
