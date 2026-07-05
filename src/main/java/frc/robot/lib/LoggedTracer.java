package frc.robot.lib;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

/** Utility class for logging code execution times. */
public class LoggedTracer {
	private LoggedTracer() {}

	private static double startTime = -1.0;

	/** Reset the clock. */
	public static void reset() {
		startTime = Timer.getFPGATimestamp();
	}

	/** Save the time elapsed since the last reset or record. */
	public static void record(String epochName) {
		double now = Timer.getFPGATimestamp();
		// Logger (not SmartDashboard) so timings land in the wpilog for offline analysis.
		Logger.recordOutput(
				"LoggedTracer/" + epochName + "MS", Units.secondsToMilliseconds(now - startTime));
		startTime = now;
	}
}