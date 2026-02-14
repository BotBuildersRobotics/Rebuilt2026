package frc.robot.subsystems.intake;

import frc.robot.lib.io.MotorSubsystem;
import frc.robot.lib.io.MotorIO.Setpoint;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.MotorIOTalonFX;

public class IntakeSubsystem extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(IntakeConstants.kIntakeVoltage);
	
	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(IntakeConstants.kReverseVoltage);
	
	public static final IntakeSubsystem mInstance = new IntakeSubsystem();

	public IntakeSubsystem() {
		super(IntakeConstants.getMotorIO(), "Intake Rollers");
	}

	

	
}