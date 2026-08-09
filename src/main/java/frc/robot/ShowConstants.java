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
	 * <p>-90 fires straight out the <b>right</b> side, into the net. The turret follows the chassis, so
	 * this stays pointing at the same place relative to the robot however the driver spins; it does not
	 * hold a field bearing. Between shots the turret returns to 0 (stowed, over the homing sensor).
	 */
	public static final LoggedTunableNumber kTurretAngleDeg =
			new LoggedTunableNumber("Show/TurretAngleDeg", -90.0);

	// --- Intake -----------------------------------------------------------------------------------

	/**
	 * Intake roller voltage in show mode. The competition setpoint
	 * ({@code IntakeConstants.kIntakeVoltage}) is 9 V, sized for sweeping fuel up at speed during a
	 * match; that is more aggressive than a demo needs, where the robot is crawling at 10% and people
	 * are hand-feeding it.
	 *
	 * <p>Too low and fuel will sit on the rollers without being drawn in — if it stops picking up
	 * reliably, put this back up rather than making the driver nudge the robot into the ball.
	 */
	public static final LoggedTunableNumber kIntakeVolts =
			new LoggedTunableNumber("Show/IntakeVolts", 5.0);

	// --- Catch shot (right bumper) ----------------------------------------------------------------
	// A soft pop straight ahead for a spectator to catch, rather than the hard shot into the net.

	/** Turret angle for the catch shot, robot-relative degrees. 0 = straight forward. */
	public static final LoggedTunableNumber kCatchTurretAngleDeg =
			new LoggedTunableNumber("Show/CatchTurretAngleDeg", 0.0);

	/** Hood angle for the catch shot, mechanism degrees. 0 = no hood, the stowed/flat position. */
	public static final LoggedTunableNumber kCatchHoodAngleDeg =
			new LoggedTunableNumber("Show/CatchHoodAngleDeg", 0.0);

	/**
	 * Flywheel velocity for the catch shot, in rotations/sec.
	 *
	 * <p>Deliberately far below anything the competition regression covers — its slowest entry is 160
	 * rps for a 1 m shot, and this needs to be gentle enough for a person to catch by hand. 110 is a
	 * starting guess, not a measured value.
	 *
	 * <p><b>Dial this in from below, not above.</b> Start here or lower, on an empty field, and creep
	 * up until the ball carries far enough. Coming down from a hard shot means the first few test
	 * shots are the dangerous ones.
	 */
	public static final LoggedTunableNumber kCatchFlywheelRPS =
			new LoggedTunableNumber("Show/CatchFlywheelRPS", 110.0);

	// --- Arrival gating ---------------------------------------------------------------------------

	/**
	 * How close the turret must be to its commanded angle before the feed is allowed to run, in
	 * degrees. Applies to both shots.
	 *
	 * <p>Deliberately much tighter than the competition {@code Turret/OnTargetToleranceDeg} (10.5),
	 * which is sized for keeping a moving hub shot flowing, not for a static aim into a net. At the
	 * ranges involved 10 deg of turret error is most of the net width.
	 *
	 * <p>Not tightened further than this on purpose: if the turret's settled error is larger than the
	 * tolerance the feed never opens at all and the robot simply will not shoot, which is a far worse
	 * failure at a show than a slightly off shot. Watch {@code Show/TurretErrorDeg} on the dashboard —
	 * it reports the real settled error, so this can be tightened or loosened against actual numbers.
	 */
	public static final LoggedTunableNumber kTurretToleranceDeg =
			new LoggedTunableNumber("Show/TurretToleranceDeg", 4.0);

	/**
	 * How slowly the turret must be moving to count as arrived, in degrees/sec.
	 *
	 * <p>Position tolerance alone is not enough: a turret sweeping through 90 deg passes through the
	 * tolerance band at full speed, and a purely positional gate can open on that pass-through. This
	 * requires it to have actually stopped there.
	 */
	public static final LoggedTunableNumber kTurretMaxVelDegPerSec =
			new LoggedTunableNumber("Show/TurretMaxVelDegPerSec", 8.0);

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
	 *
	 * <p>The default is deliberately at the steep end: 100 is 40 map degrees, and the regression only
	 * goes to 41 (its 5.7 m entry), so this is about the steepest shot the competition code ever
	 * commands and therefore known to be mechanically reachable. That puts the ball on a high arc that
	 * drops down into the net rather than driving flat at it. There is no soft limit configured on the
	 * hood, so if you wind this past ~102 at the venue you are past anything the robot has been asked
	 * for before — go up in small steps and watch the mechanism.
	 */
	public static final LoggedTunableNumber kHoodAngleDeg =
			new LoggedTunableNumber("Show/HoodAngleDeg", 100.0);

	/**
	 * How close the hood must be to {@link #kHoodAngleDeg} before the feed is allowed to run, in
	 * mechanism degrees. Same reasoning as the turret gate: the hood has a long way to travel to a
	 * steep show angle, and a ball fed before it arrives leaves on a much flatter trajectory than the
	 * one the net is positioned for.
	 */
	public static final LoggedTunableNumber kHoodToleranceDeg =
			new LoggedTunableNumber("Show/HoodToleranceDeg", 3.0);

	/**
	 * Extra dwell after the flywheel first reports at-speed before the feed is allowed to run, in
	 * seconds. Stops a ball being fed into a flywheel that has only just touched its setpoint and is
	 * still settling, which is what makes the first shot of a burst drop short.
	 */
	public static final LoggedTunableNumber kFeedSettleSec =
			new LoggedTunableNumber("Show/FeedSettleSec", 0.25);

	/**
	 * How long the turret keeps pointing at the net after the trigger is released, in seconds.
	 *
	 * <p>On release the feed is commanded to stop immediately, but the chute and roller floor take
	 * time to actually spin down and a ball already in the path keeps travelling. For this long
	 * afterwards the turret, hood and flywheel all stay exactly where they were, so anything still on
	 * its way out leaves in the right direction at the right speed. Only then does the turret stow.
	 *
	 * <p>Raise it if you still see the occasional stray on release; the only cost is that the turret
	 * takes longer to park between shots.
	 */
	public static final LoggedTunableNumber kFeedStopSec =
			new LoggedTunableNumber("Show/FeedStopSec", 0.5);

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
