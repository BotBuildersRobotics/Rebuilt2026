package frc.robot.subsystems.climb;

import frc.robot.lib.io.MotorSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.lib.io.MotorIO.Setpoint;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.MotorIOTalonFX;

public class ClimbSubsystem extends MotorSubsystem<MotorIOTalonFX> {
	
	public static final Setpoint STOP = Setpoint.withNeutralSetpoint();
	public static final Setpoint CLIMB = Setpoint.withVoltageSetpoint(ClimbConstants.kClimbVoltage);
	
	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(ClimbConstants.kReverseVoltage);
	

	public static final ClimbSubsystem mInstance = new ClimbSubsystem();

	public ClimbSubsystem() {
		super(ClimbConstants.getMotorIO(), "Climber");
	}

	

	
}