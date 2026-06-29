package frc.robot.subsystems.drive;


import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.ControllerType;

public class ControlBoardConstants {

	public static final ControllerMap mDriverController =
			makeController(0, OperatorConstants.kDriverControllerType);
	public static final ControllerMap mOperatorController =
			makeController(1, OperatorConstants.kOperatorControllerType);

	private static ControllerMap makeController(int port, ControllerType type) {
		return type == ControllerType.PS5
				? new PS5ControllerMap(port)
				: new XboxControllerMap(port);
	}

	public static final Time kIntakeRumbleTime = Units.Seconds.of(0.2);

	public static final double stickDeadband = 0.05;
}