package frc.robot.subsystems.shuffla;

import java.util.function.BooleanSupplier;

import frc.robot.lib.io.MotorSubsystem;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class ShufflaSubsystem extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint SHOOT = Setpoint.withVoltageSetpoint(ShufflaConstants.kShootVoltage);
	
	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(ShufflaConstants.kReverseVoltage);
	
	public static final ShufflaSubsystem mInstance = new ShufflaSubsystem();

	public ShufflaSubsystem() {
		super(ShufflaConstants.getMotorIO(), "Shuffla Wheels");
	}

	/**
	 * Runs the shuffla at shoot voltage when {@code ready} is true, idles otherwise.
	 * Automatically pauses feeding if the shooter bogs down and resumes when it recovers.
	 */
	public Command runShootCommandGated(BooleanSupplier ready) {
		return run(() -> applySetpoint(ready.getAsBoolean() ? SHOOT : IDLE));
	}

}