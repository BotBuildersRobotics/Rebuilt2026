package frc.robot.subsystems.climb;

import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.lib.io.MotorIO.Setpoint;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.MotorIOTalonFX;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Rotations;

public class ClimbSubsystem extends MotorSubsystem<MotorIOTalonFX> {

	// Voltage-based setpoints (kept for manual override)
	public static final Setpoint STOP = Setpoint.withNeutralSetpoint();
	public static final Setpoint CLIMB = Setpoint.withVoltageSetpoint(ClimbConstants.kClimbVoltage);
	public static final Setpoint REVERSE = Setpoint.withVoltageSetpoint(ClimbConstants.kReverseVoltage);

	// Position-based setpoints
	public static final Setpoint STOWED = Setpoint.withMotionMagicSetpoint(ClimbConstants.kStowedPosition);
	public static final Setpoint CLIMBED = Setpoint.withMotionMagicSetpoint(ClimbConstants.kClimbedPosition);

	// Tunable climb distance for field testing
	private static final LoggedTunableNumber climbExtendRotations =
		new LoggedTunableNumber("Climb/ExtendRotations", -160.0);

	private static final LoggedTunableNumber climbRotations =
		new LoggedTunableNumber("Climb/ClimbRotations", -100.0);

	public static final ClimbSubsystem mInstance = new ClimbSubsystem();

	public ClimbSubsystem() {
		super(ClimbConstants.getMotorIO(), "Climber");
	}

	@Override
	public void periodic() {
		super.periodic();

		SmartDashboard.putNumber("Climb/PositionRot", getPosition().in(Rotations));
		Logger.recordOutput("Climb/PositionRot", getPosition().in(Rotations));
	}

	/** Zeros the climb position to the current location (stowed). */
	public Command zeroCommand() {
		return runOnce(() -> setCurrentPosition(ClimbConstants.kStowedPosition))
			.ignoringDisable(true);
	}

	/** Move to the stowed position (zero). */
	public Command stowCommand() {
		return setpointCommand(STOWED);
	}

	/** Move to the climb position using the tunable rotation count. */
	public Command climbCommand() {
		return run(() -> applySetpoint(
			Setpoint.withMotionMagicSetpoint(Units.Rotations.of(climbRotations.get()))));
	}

	public Command extendCommand() {
		return run(() -> applySetpoint(
			Setpoint.withMotionMagicSetpoint(Units.Rotations.of(climbExtendRotations.get()))));
	}

	public Command extendAutoCommand(){
		return Commands.runOnce(() -> applySetpoint(
			Setpoint.withMotionMagicSetpoint(Units.Rotations.of(climbExtendRotations.get()))));
	}

	public Command climeAutoCommand(){
		return Commands.runOnce(() -> applySetpoint(
			Setpoint.withMotionMagicSetpoint(Units.Rotations.of(climbRotations.get()))));
	}
}
