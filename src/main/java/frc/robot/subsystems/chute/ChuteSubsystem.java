package frc.robot.subsystems.chute;

import frc.robot.lib.io.MotorSubsystem;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;

public class ChuteSubsystem extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint SHOOT = Setpoint.withVoltageSetpoint(ChuteConstants.kShootVoltage);
	
	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(ChuteConstants.kReverseVoltage);
	
	public static final ChuteSubsystem mInstance = new ChuteSubsystem();

	public ChuteSubsystem() {
		super(ChuteConstants.getMotorIO(), "Chute Rollers");
	}

}