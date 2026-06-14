package frc.robot.subsystems.rollerFloor;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class RollerFloorSubsystem extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(RollerFloorConstants.kReverseVoltage);

	private static final LoggedTunableNumber shootSpeed = new LoggedTunableNumber("RollerFloor/ShootSpeed");

	public static final RollerFloorSubsystem mInstance = new RollerFloorSubsystem();

	public RollerFloorSubsystem() {
		super(RollerFloorConstants.getMotorIO(), "Roller Floor");
		shootSpeed.initDefault(80); // RPS
	}

	/** Runs the roller floor at a closed-loop velocity setpoint. Re-evaluates the tunable each cycle. */
	public Command runShootCommand() {
		return runOnce(() -> applySetpoint(
			Setpoint.withVelocitySetpoint(RotationsPerSecond.of(shootSpeed.get()))
		));
	}

	/**
	 * Runs the roller floor at shoot speed when {@code ready} is true, idles otherwise.
	 */
	public Command runShootCommandGated(BooleanSupplier ready) {
		return run(() -> {
			if (ready.getAsBoolean()) {
				applySetpoint(Setpoint.withVoltageSetpoint(RollerFloorConstants.kFloorVoltage));//.withVelocitySetpoint(RotationsPerSecond.of(shootSpeed.get())));
			} else {
				applySetpoint(IDLE);
			}
		});
	}

}
