package frc.robot;

import frc.robot.lib.LoggedTunableNumber;

/**
 * Tuning knobs for outreach/demo ("show") mode — the Ekka build.
 *
 * <p>Show mode assumes there is no real field: no AprilTags, no reliable pose estimate, and no hub
 * to track. Everything the robot does is therefore open-loop with respect to the field. The shot is
 * a fixed turret angle + fixed flywheel speed + fixed hood angle, and the turret angle is
 * robot-relative, so the whole aim geometry rides with the chassis and the driver aims by parking
 * the robot. Nothing here reads the pose estimator.
 *
 * <p>All values are live-tunable over NetworkTables under {@code Show/} so the shot can be dialled in
 * at the venue without a laptop rebuild. {@link Constants#tuningMode} does not need to be on — these
 * are read every loop through {@link LoggedTunableNumber#get()}.
 */
public final class ShowConstants {

	private ShowConstants() {}

	/**
	 * Robot-relative turret angle for the show shot, in degrees. Positive is <b>left</b> — CCW looking
	 * down on the robot, i.e. the robot's left as seen standing behind it looking forward. Matches the
	 * sign of {@code TurretSubsystem.offsetLeft()}.
	 *
	 * <p>90 fires straight out the left side, into the net. The turret follows the chassis, so this
	 * stays pointing at the same place relative to the robot however the driver spins; it does not
	 * hold a field bearing. Between shots the turret returns to 0 (stowed, over the homing sensor).
	 */
	public static final LoggedTunableNumber kTurretAngleDeg =
			new LoggedTunableNumber("Show/TurretAngleDeg", 90.0);

	/**
	 * Fixed flywheel velocity for the show shot, in rotations/sec.
	 *
	 * <p>Reference points from the competition distance regression
	 * ({@code ShotCalculator.launchFlywheelSpeedMap}): ~165 rps at 1.7 m, ~183 rps at 3.35 m,
	 * ~195 rps at 4.1 m. The default here is roughly a 3.3 m shot.
	 */
	public static final LoggedTunableNumber kFlywheelRPS =
			new LoggedTunableNumber("Show/FlywheelRPS", 185.0);

	/**
	 * Fixed hood angle for the show shot, in <em>mechanism</em> degrees.
	 *
	 * <p>Careful: this is not the same number as the angles in
	 * {@code ShotCalculator.launchHoodAngleMap}. The hood's tracking command multiplies the map angle
	 * by {@code HoodConstants.fudgeFactor} (2.5) before commanding it, and {@code runFixedCommand}
	 * does not. So mechanism degrees = map degrees x 2.5 — the 20 deg map entry for a 3.3 m shot is
	 * 50 here. 0.0 is the stowed/flat position.
	 */
	public static final LoggedTunableNumber kHoodAngleDeg =
			new LoggedTunableNumber("Show/HoodAngleDeg", 50.0);

	/**
	 * Extra dwell after the flywheel first reports at-speed before the feed is allowed to run, in
	 * seconds. Stops a ball being fed into a flywheel that has only just touched its setpoint and is
	 * still settling, which is what makes the first shot of a burst drop short.
	 */
	public static final LoggedTunableNumber kFeedSettleSec =
			new LoggedTunableNumber("Show/FeedSettleSec", 0.25);

	/** Fraction of max translational speed the driver is allowed in show mode. */
	public static final LoggedTunableNumber kDriveSpeedFraction =
			new LoggedTunableNumber("Show/DriveSpeedFraction", 0.10);

	/**
	 * Fraction of max angular rate the driver is allowed in show mode. Deliberately higher than the
	 * translation cap: the chassis is the only aiming device in show mode, so pinning rotation to 10%
	 * as well would make lining up the shot painfully slow.
	 */
	public static final LoggedTunableNumber kDriveRotationFraction =
			new LoggedTunableNumber("Show/DriveRotationFraction", 0.30);
}
