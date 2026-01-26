package frc.robot.subsystems.shuffla;

import frc.robot.lib.io.MotorSubsystem;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;

public class ShufflaSubsystem extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint SHOOT = Setpoint.withVoltageSetpoint(ShufflaConstants.kShootVoltage);
	
	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(ShufflaConstants.kReverseVoltage);
	
	public static final ShufflaSubsystem mInstance = new ShufflaSubsystem();

	public ShufflaSubsystem() {
		super(ShufflaConstants.getMotorIO(), "Shuffla Wheels");
	}

}