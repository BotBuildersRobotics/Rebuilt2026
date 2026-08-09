package frc.robot.subsystems;

import frc.robot.lib.LoggedTracer;
import edu.wpi.first.util.sendable.SendableBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.ShiftHelpers;
import frc.robot.ShowConstants;
import frc.robot.lib.AllianceFlipUtil;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.subsystems.chute.ChuteSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.rollerFloor.RollerFloorSubsystem;
import frc.robot.subsystems.turret.ShotCalculator;
import frc.robot.subsystems.turret.TurretSubsystem;



public class SuperSystem extends SubsystemBase {

    public static SuperSystem mInstance;

    private static final LoggedTunableNumber intakePulseOnTime =
        new LoggedTunableNumber("SuperSystem/IntakePulseOnTimeSec");
    private static final LoggedTunableNumber intakePulseOffTime =
        new LoggedTunableNumber("SuperSystem/IntakePulseOffTimeSec");
    // Feed readiness uses a hysteretic latch so it works both for stationary shots and shoot-on-
    // the-move. ARM: on-target + at-speed must hold for feedSettleSeconds before the first ball
    // (stops a ball being fed mid-slew during a big turn). HOLD: once armed, the feed keeps going
    // through brief tracking/at-speed flickers until they're lost for feedHoldSeconds — this is what
    // lets it keep shooting while the turret is continuously tracking a moving target.
    private static final LoggedTunableNumber feedSettleSeconds =
        new LoggedTunableNumber("SuperSystem/FeedSettleSeconds", 0.12);
    private static final LoggedTunableNumber feedHoldSeconds =
        new LoggedTunableNumber("SuperSystem/FeedHoldSeconds", 0.30);
    private Debouncer feedArmDebounce = new Debouncer(0.12, Debouncer.DebounceType.kRising);
    private Debouncer feedHoldDebounce = new Debouncer(0.30, Debouncer.DebounceType.kFalling);
    private double feedArmConfigured = 0.12;
    private double feedHoldConfigured = 0.30;
    private boolean feedReady = false;

	private TurretSubsystem turret;

	private ShooterSubsystem shooter;

	private ShotCalculator shotCalc;

	private HoodSubsystem hood;

	private boolean defenceModeActive = false;

	//private AprilTagFieldLayout kAprilTagMap = AprilTagFieldLayout.loadField(AprilTagFields);


    public static SuperSystem getInstance() {

        //Rethink this for how advantage kit does 
		if (mInstance == null) {
			mInstance = new SuperSystem();
		}
		return mInstance;
	}

	public SuperSystem(){

		intakePulseOnTime.initDefault(0.5);
		intakePulseOffTime.initDefault(0.5);

		shooter = new ShooterSubsystem();
		turret = new TurretSubsystem();
		hood = new HoodSubsystem();
		shotCalc = new ShotCalculator();
		shotCalc.getParameters(); // THIS IS IMPORTANT -  it will cause the field json to load, before teleop init

		hood.setShotCalculator(shotCalc);
		turret.setShotCalculator(shotCalc);
		shooter.setShotCalculator(shotCalc);
		shotCalc.setTurretFieldAngleSupplier(() ->
			frc.robot.subsystems.drive.DriveSubsystem.mInstance.getDrivetrain().getState().Pose.getRotation().getRadians()
			+ turret.getTurretAngle());
		
		// SHOW MODE: the flywheel idles unless the shot is actually being held. The competition default
		// (runTrackTargetActiveShootingCommand) spins off the distance regression whenever the robot is
		// outside the neutral zone, and with no valid pose at a demo that means spinning constantly all
		// day next to the public. Note this also means autos won't spin the flywheel on this branch.
		shooter.setDefaultCommand(shooter.runIdleCommand());
		turret.setDefaultCommand(turret.runTrackTargetActiveShootingCommand());
		hood.setDefaultCommand(hood.runTrackTargetActiveShootingCommand());

	}

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		
        builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);

    }

    @Override
	public void periodic() {

		updateFeedReady();
		updateShowFeedReady();

		 // Clear shooting parameters so they are recalculated each tick
    	shotCalc.clearShootingParameters();
		SmartDashboard.putBoolean("SuperSystem/DefenceMode", defenceModeActive);
		SmartDashboard.putBoolean("Drive/OnOpponentSide", ShiftHelpers.isOnOpponentSide());
		SmartDashboard.putBoolean("Drive/InOpponentZone", ShiftHelpers.isInOpponentZone());
		SmartDashboard.putBoolean("Hub/BlueActive", ShiftHelpers.isBlueHubActive());
		SmartDashboard.putBoolean("Hub/RedActive", ShiftHelpers.isRedHubActive());
		SmartDashboard.putBoolean("Hub/AreWeRedActive", ShiftHelpers.isRedHubActive() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
		SmartDashboard.putBoolean("Hub/AreWeActive", (ShiftHelpers.isRedHubActive() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) || (ShiftHelpers.isBlueHubActive() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue));
		LoggedTracer.record("SuperSystem");
	}

	 public Command idleIntakes() {
		
		return IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.IDLE);
					
	}

	public Command Intake() {
		return IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.INTAKE);
	}

	/** Continuously holds the intake running. Use for driver trigger bindings so it overrides pulse idle phases. */
	public Command intakeContinuousCommand() {
		return IntakeSubsystem.mInstance.followSetpointCommand(() -> IntakeSubsystem.INTAKE);
	}

	/**
	 * Show-mode intake: same continuous hold, but at the reduced {@code Show/IntakeVolts} rather than
	 * the 9 V competition setpoint. Read live, so the voltage can be dialled in from the dashboard
	 * while someone feeds the robot by hand.
	 */
	public Command showIntakeCommand() {
		return IntakeSubsystem.mInstance.followSetpointCommand(
			() -> frc.robot.lib.io.MotorIO.Setpoint.withVoltageSetpoint(
				edu.wpi.first.units.Units.Volts.of(ShowConstants.kIntakeVolts.get())));
	}


	/**
	 * Hysteretic feed-readiness latch, evaluated once per loop so the roller floor and chute share
	 * identical timing. ARM requires sustained on-target+at-speed (feedSettleSeconds) so a ball is
	 * not fed mid-slew; once armed, the feed HOLDS through brief flickers and only drops after the
	 * conditions are lost for feedHoldSeconds — which keeps it feeding while tracking a moving target.
	 */
	private void updateFeedReady() {
		// Rebuild debouncers if their tunables changed (rare, only while tuning).
		if (feedSettleSeconds.get() != feedArmConfigured) {
			feedArmConfigured = feedSettleSeconds.get();
			feedArmDebounce = new Debouncer(feedArmConfigured, Debouncer.DebounceType.kRising);
		}
		if (feedHoldSeconds.get() != feedHoldConfigured) {
			feedHoldConfigured = feedHoldSeconds.get();
			feedHoldDebounce = new Debouncer(feedHoldConfigured, Debouncer.DebounceType.kFalling);
		}

		boolean raw = shooter.isAtSpeed() && turret.isOnTarget();
		boolean armed = feedArmDebounce.calculate(raw);  // kRising: true after raw held for arm time
		boolean held = feedHoldDebounce.calculate(raw);  // kFalling: stays true until raw false for hold time
		// Hysteresis: need a sustained lock to start feeding, then tolerate brief losses while feeding.
		feedReady = feedReady ? held : armed;

		org.littletonrobotics.junction.Logger.recordOutput("SuperSystem/FeedRaw", raw);
		org.littletonrobotics.junction.Logger.recordOutput("SuperSystem/FeedReady", feedReady);
	}

	private BooleanSupplier shooterFeedReady() {
		return () -> feedReady;
	}

	public Command Shoot(){
		return
		Commands.parallel(
			RollerFloorSubsystem.mInstance.runShootCommandGated(shooterFeedReady()),
			ChuteSubsystem.mInstance.runShootCommandGated(shooterFeedReady())
		)
		// Tell the shooter we're actively shooting so it doesn't spin down in the neutral zone.
		.beforeStarting(() -> shooter.setActivelyShooting(true))
		.finallyDo(interrupted -> shooter.setActivelyShooting(false));
	}

	/**
	 * Hold-to-shoot-straight-ahead. While held, pins the turret to its stowed zero
	 * (0&deg; robot-relative, straight forward &mdash; the same spot it parks at when idle) instead of
	 * auto-tracking the hub, while the hood and flywheel keep spooling for the current distance and
	 * the feed runs gated on at-speed + on-target ({@link #Shoot()}). The driver aims by pointing the
	 * chassis; releasing restores the turret's (and hood's) previous stow state.
	 *
	 * <p>Intended as a manual fallback when turret auto-aim isn't trusted: point the robot at the hub
	 * and fire dead ahead. Only the turret <em>azimuth</em> is overridden &mdash; the hood is forced
	 * active so its angle still tracks distance, keeping the shot arc/velocity correct. If you'd rather
	 * also physically stow the hood (fixed close-range shot), drop the {@code hood.setStowed(false)}.
	 *
	 * <p>Bind with {@code whileTrue} so it holds only while the button is pressed.
	 */
	public Command shootStraightAhead(){
		// [0] = turret stow, [1] = hood stow, captured on start and restored on release.
		final boolean[] prevStow = { false, false };
		return Shoot()
			.beforeStarting(() -> {
				prevStow[0] = turret.isStowed();
				prevStow[1] = hood.isStowed();
				turret.setStowed(true);   // pin turret to stow zero (straight forward)
				hood.setStowed(false);    // keep the hood tracking so the arc stays correct
			})
			.finallyDo(interrupted -> {
				turret.setStowed(prevStow[0]);
				hood.setStowed(prevStow[1]);
			});
	}

	// --- Show / outreach mode ---------------------------------------------------------------

	private final edu.wpi.first.wpilibj.Timer showAtSpeedTimer = new edu.wpi.first.wpilibj.Timer();
	private boolean showFeedReady = false;

	/**
	 * Feed gate for the show shot: flywheel at speed <em>and</em> the turret arrived at its held
	 * angle, both sustained for {@code Show/FeedSettleSec}. The turret term matters — the show shot
	 * swings the turret 90 deg to the net on the trigger pull, and a 90 deg slew is not instant, so
	 * without it the first ball of each burst leaves mid-swing and misses.
	 *
	 * <p>Turret arrival is checked against {@code Show/TurretToleranceDeg} rather than
	 * {@code turret.isOnTarget()}, whose 10.5 deg competition tolerance is most of a net's width at
	 * these ranges, and it additionally requires the turret to have stopped
	 * ({@code Show/TurretMaxVelDegPerSec}) — a 90 deg sweep crosses the tolerance band at full speed,
	 * and a position-only gate can open on that pass-through and fire mid-swing.
	 *
	 * <p>Simpler than the competition {@link #updateFeedReady()} latch on purpose: no hysteresis to
	 * hold the feed open through tracking flickers, because nothing here is tracking a moving target.
	 */
	private void updateShowFeedReady() {
		double turretErrorDeg = turret.getTargetErrorDeg();
		double turretVelDegPerSec =
			Math.abs(edu.wpi.first.math.util.Units.radiansToDegrees(turret.getTurretVelocity()));

		boolean turretArrived = turretErrorDeg <= ShowConstants.kTurretToleranceDeg.get()
			&& turretVelDegPerSec <= ShowConstants.kTurretMaxVelDegPerSec.get();
		boolean flywheelReady = shooter.isAtSpeed();
		boolean hoodArrived = hood.getGoalErrorDeg() <= ShowConstants.kHoodToleranceDeg.get();

		if (turretArrived && hoodArrived && flywheelReady) {
			showAtSpeedTimer.start();
			showFeedReady = showAtSpeedTimer.hasElapsed(ShowConstants.kFeedSettleSec.get());
		} else {
			showAtSpeedTimer.stop();
			showAtSpeedTimer.reset();
			showFeedReady = false;
		}

		// Broken out so a feed that never opens can be diagnosed from the dashboard at the venue.
		org.littletonrobotics.junction.Logger.recordOutput("Show/FeedReady", showFeedReady);
		org.littletonrobotics.junction.Logger.recordOutput("Show/TurretArrived", turretArrived);
		org.littletonrobotics.junction.Logger.recordOutput("Show/TurretErrorDeg", turretErrorDeg);
		org.littletonrobotics.junction.Logger.recordOutput("Show/TurretVelDegPerSec", turretVelDegPerSec);
		org.littletonrobotics.junction.Logger.recordOutput("Show/FlywheelReady", flywheelReady);
		org.littletonrobotics.junction.Logger.recordOutput("Show/HoodArrived", hoodArrived);
		org.littletonrobotics.junction.Logger.recordOutput("Show/HoodErrorDeg", hood.getGoalErrorDeg());
		SmartDashboard.putBoolean("Show/FeedReady", showFeedReady);
		SmartDashboard.putBoolean("Show/TurretArrived", turretArrived);
		SmartDashboard.putNumber("Show/TurretErrorDeg", turretErrorDeg);
		SmartDashboard.putBoolean("Show/FlywheelReady", flywheelReady);
		SmartDashboard.putBoolean("Show/HoodArrived", hoodArrived);
		SmartDashboard.putNumber("Show/HoodErrorDeg", hood.getGoalErrorDeg());
	}

	/**
	 * Puts the shooting mechanisms into their show-mode resting state: turret and hood stowed. In show
	 * mode nothing ever un-stows the turret, so it stays at its zero (straight ahead, robot-relative)
	 * for the whole event and the driver aims by pointing the chassis.
	 */
	public Command enterShowModeCommand() {
		return Commands.runOnce(() -> {
			turret.setStowed(true);
			hood.setStowed(true);
			shooter.clearflywheelPreset();
		}).ignoringDisable(true);
	}

	/**
	 * Hold-to-shoot for outreach events. Swings the turret to a fixed robot-relative angle
	 * ({@code Show/TurretAngleDeg}, 90 deg right by default — out the side into the net) and holds a
	 * fixed flywheel speed and hood angle from {@link ShowConstants}. No distance regression, no pose,
	 * no vision. The feed is gated on the flywheel being at speed <em>and</em> the turret having
	 * arrived, sustained for {@code Show/FeedSettleSec}.
	 *
	 * <p><b>Release is staged, not instant.</b> Bind this with {@code onTrue} and pass the trigger
	 * itself as {@code held} — the command outlives the button press on purpose. The moment the
	 * trigger is released the feed is commanded to stop, but the turret, hood and flywheel all hold
	 * position for {@code Show/FeedStopSec} afterwards. The chute and roller floor take time to spin
	 * down and a ball already in the path keeps travelling; if the turret started slewing home the
	 * instant the button came up, that ball would be thrown wherever the turret happened to be
	 * pointing. Only once the path is clear does the turret stow.
	 *
	 * <p>Re-pressing the trigger during that window simply resumes shooting — the timer resets and the
	 * turret never leaves the net.
	 *
	 * @param held supplier that is true while the shot button is held
	 */
	public Command showNetShotCommand(BooleanSupplier held) {
		return showShotCommand(
			held,
			ShowConstants.kTurretAngleDeg::get,
			ShowConstants.kHoodAngleDeg::get,
			ShowConstants.kFlywheelRPS::get);
	}

	/**
	 * Hold-to-shoot straight ahead, flat, and gently — a soft pop for a spectator to catch, rather
	 * than the hard shot into the net. Turret at 0 (straight forward, robot-relative), hood at 0, and
	 * a much lower flywheel speed, all from the {@code Show/Catch*} tunables.
	 *
	 * <p>Identical machinery to {@link #showNetShotCommand(BooleanSupplier)} — same arrival gating,
	 * same staged release — just different numbers. The two share every subsystem, so pressing one
	 * while the other runs cleanly interrupts it.
	 *
	 * @param held supplier that is true while the shot button is held
	 */
	public Command showCatchShotCommand(BooleanSupplier held) {
		return showShotCommand(
			held,
			ShowConstants.kCatchTurretAngleDeg::get,
			ShowConstants.kCatchHoodAngleDeg::get,
			ShowConstants.kCatchFlywheelRPS::get);
	}

	private Command showShotCommand(
			BooleanSupplier held,
			DoubleSupplier turretDeg,
			DoubleSupplier hoodDeg,
			DoubleSupplier flywheelRPS) {
		// Time since the trigger came up. Only started once released, reset on every re-press.
		final edu.wpi.first.wpilibj.Timer releaseTimer = new edu.wpi.first.wpilibj.Timer();

		// The feed also requires the trigger to still be held, so releasing stops it on the same loop
		// while everything else carries on holding.
		BooleanSupplier feeding = () -> showFeedReady && held.getAsBoolean();

		return Commands.parallel(
			turret.runRobotRelativeHoldCommand(turretDeg),
			shooter.runShowShotCommand(flywheelRPS),
			hood.runFixedCommand(hoodDeg),
			RollerFloorSubsystem.mInstance.runShootCommandGated(feeding),
			ChuteSubsystem.mInstance.runShootCommandGated(feeding)
		)
		.beforeStarting(() -> {
			releaseTimer.stop();
			releaseTimer.reset();
			shooter.setActivelyShooting(true);
		})
		.until(() -> {
			if (held.getAsBoolean()) {
				// Still shooting (or shooting again) — cancel any pending shutdown.
				releaseTimer.stop();
				releaseTimer.reset();
				return false;
			}
			releaseTimer.start();
			return releaseTimer.hasElapsed(ShowConstants.kFeedStopSec.get());
		})
		.finallyDo(interrupted -> {
			// Runs on normal end and on interruption, so the turret can never be left aimed sideways.
			shooter.setActivelyShooting(false);
			turret.releaseRobotRelativeHold();
			RollerFloorSubsystem.mInstance.applySetpoint(RollerFloorSubsystem.IDLE);
			ChuteSubsystem.mInstance.applySetpoint(ChuteSubsystem.IDLE);
		});
	}

	public Command ShootAuto(){
		return
		
		Commands.parallel(
			RollerFloorSubsystem.mInstance.runShootCommand(),
			ChuteSubsystem.mInstance.runShootCommand()
		);
	}

	public Command idleShooter(){

		return
		Commands.parallel(
			RollerFloorSubsystem.mInstance.setpointCommand(RollerFloorSubsystem.IDLE),
			ChuteSubsystem.mInstance.setpointCommand(ChuteSubsystem.IDLE)
		);
	}

	/**
	 * Pre-feed jam clear. Runs the roller floor in REVERSE for {@code seconds}, then holds it there —
	 * the {@link #Shoot()} that follows in the shoot sequence overrides it to feed speed. This replaces
	 * the old fixed pre-shoot wait: reversing during the flywheel-settle window automatically backs a
	 * ball off a jam at the shooter entry before feeding starts. If the shot is released mid-clear, the
	 * shoot binding's {@code onFalse} idles the floor, so nothing is left running in reverse.
	 *
	 * @param seconds supplier for the reverse duration, read at schedule time so the tunable applies live
	 */
	public Command reverseFloorThenFeedDelay(DoubleSupplier seconds){
		return RollerFloorSubsystem.mInstance.setpointCommand(RollerFloorSubsystem.REVERSE)
			.andThen(Commands.defer(() -> Commands.waitSeconds(seconds.getAsDouble()), Set.of()));
	}

	public Command DeployIntake(){
		return PivotSubsystem.mInstance.findDeployLimitCommand();
	}

	public Command turretStow(){
		return turret.stowCommand();
	}

	public Command AimAtCenterHub(){
		return turret.pointAtFieldPosition(frc.robot.FieldConstants.Hub.innerCenterPoint.toTranslation2d());
	}

	public Command AimAtPassingZone(){
		return turret.pointAtFieldPosition((frc.robot.FieldConstants.Outpost.centerPoint));
	}

	public Command stowTurret(){
		return turret.stowCommand();
	}

	public Command zeroTurretCommand(){
		return turret.zeroCommand();
	}

	public Command toggleStow(){
		return Commands.runOnce(() -> {
			boolean newState = !turret.isStowed();
			turret.setStowed(newState);
			hood.setStowed(newState);
		});
	}

	public Command stowTurretHood(){
		return Commands.runOnce(() -> {
			turret.setStowed(true);
			hood.setStowed(true);
		});
	}

	public Command activeHood(){
		return Commands.runOnce(() -> {
			hood.setStowed(false);
		});
	}

	public Command stowHood(){
		return Commands.runOnce(() -> {
			hood.setStowed(true);
		});
	}

	public Command activeTurret(){
		return Commands.runOnce(() -> {
			turret.setStowed(false);
		});
	}

	public Command stowTurrets(){
		return Commands.runOnce(() -> {
			turret.setStowed(true);
		});
	}

	public Command activeTurretHood(){
		return Commands.runOnce(() -> {
			turret.setStowed(false);
			hood.setStowed(false);
		});
	}

	public Command enableStow(){
		return Commands.runOnce(() -> {
			turret.setStowed(true);
			hood.setStowed(true);
		});
	}

	public Command disableStow(){
		return Commands.runOnce(() -> {
			turret.setStowed(false);
			hood.setStowed(false);
		});
	}

	public Command passLeft(){
		return Commands.parallel(
			turret.passLeftCommand(),
			shooter.runPassingCommand(turret::getPassDistance),
			hood.runPassingCommand()
		);
	}

	public Command passRight(){
		return Commands.parallel(
			turret.passRightCommand(),
			shooter.runPassingCommand(turret::getPassDistance),
			hood.runPassingCommand()
		);
	}

	public Command ShooterRunPassingCommand() {
		return shooter.runPassingCommand(turret::getPassDistance);
	}

	public Command PassSCR() {
		return Commands.parallel(
			ShooterRunPassingCommand(),
			stowTurrets()
		);
	}

	public Command passAutoSCR() {
		return Commands.parallel(
			turret.passAutoCommand(),
			Commands.waitSeconds(1),
			shooter.runPassingCommand(turret::getPassDistance),
			hood.runPassingCommand(),
			RollerFloorSubsystem.mInstance.runShootCommand(),
			ChuteSubsystem.mInstance.runShootCommand()
		).beforeStarting(disableStow()).finallyDo(()->{
			turret.setStowed(true);
			hood.setStowed(true);
		});
	}

	public Command passAuto(){
		return Commands.parallel(
			turret.passAutoCommand(),
			hood.runPassingCommand(),
			Commands.waitSeconds(1),
			shooter.runPassingCommand(turret::getPassDistance)
			
		).beforeStarting(disableStow()).finallyDo(()->{
			turret.setStowed(true);
			hood.setStowed(true);
		});
	}

	/**
	 * Pulses the intake on and off to agitate balls in the hopper while shooting.
	 * Intentionally does NOT require IntakeSubsystem so the driver can also manually
	 * run the intake simultaneously without cancelling the shoot command.
	 * Runs indefinitely until interrupted.
	 */
	public Command intakePulseCommand() {
		return Commands.sequence(
			Commands.runOnce(() -> IntakeSubsystem.mInstance.applySetpoint(IntakeSubsystem.INTAKE)),
			Commands.defer(() -> Commands.waitSeconds(intakePulseOnTime.get()), Set.of()),
			Commands.runOnce(() -> IntakeSubsystem.mInstance.applySetpoint(IntakeSubsystem.IDLE)),
			Commands.defer(() -> Commands.waitSeconds(intakePulseOffTime.get()), Set.of())
		).repeatedly();
	}

	/**
	 * Long lob pass from the opponent side back into our alliance zone.
	 * Uses higher flywheel speed and steeper hood angle than the normal pass.
	 * Turret still auto-selects left/right target based on robot Y position.
	 */
	public Command passLobAuto(){
		return Commands.parallel(
			turret.passAutoCommand(),
			hood.runLobPassingCommand(),
			Commands.waitSeconds(1),
			shooter.runLobPassingCommand(turret::getPassDistance),
			
			RollerFloorSubsystem.mInstance.runShootCommand(),
			ChuteSubsystem.mInstance.runShootCommand()
		).beforeStarting(disableStow()).finallyDo(()->{
			turret.setStowed(true);
			hood.setStowed(true);
		});
	}


	// --- Turret calibration test commands ---
	public Command testTurretStraightAhead(){
		return turret.testAimStraightAhead();
	}

	public Command testTurretRobotRelative(){
		return turret.testAimRobotRelative();
	}

	public Command testTurretAimAtHub(){
		return turret.testAimAtHub();
	}

	public Command shootAtVelocity(double rps) {
		return shooter.runAtVelocityCommand(rps);
	}

	public Command shooterSysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction direction) {
		return shooter.sysIdQuasistatic(direction)
			.beforeStarting(Commands.runOnce(() -> {
				edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().removeDefaultCommand(shooter);
				turret.setStowed(true);
				hood.setStowed(true);
			}))
			.finallyDo(() -> {
				shooter.setDefaultCommand(shooter.runIdleCommand()); // show mode: never restore constant spin
				turret.setStowed(false);
				hood.setStowed(false);
			});
	}

	public Command shooterSysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction direction) {
		return shooter.sysIdDynamic(direction)
			.beforeStarting(Commands.runOnce(() -> {
				edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().removeDefaultCommand(shooter);
				turret.setStowed(true);
				hood.setStowed(true);
			}))
			.finallyDo(() -> {
				shooter.setDefaultCommand(shooter.runIdleCommand()); // show mode: never restore constant spin
				turret.setStowed(false);
				hood.setStowed(false);
			});
	}

	public Command setTurretAnglePreset(double fieldRelativeDeg) {
		return Commands.runOnce(() -> turret.setTurretAnglePreset(Rotation2d.fromDegrees(fieldRelativeDeg)));
	}

	public Command clearTurretAnglePreset() {
		return Commands.runOnce(() -> turret.clearTurretAnglePreset());
	}

	public Command setFlywheelPreset(double rps) {
		return shooter.setFlywheelPresetCommand(rps);
	}

	public Command clearFlywheelPreset() {
		return shooter.clearFlywheelPresetCommand();
	}

	public void clearflywheelPreset(){
		shooter.clearflywheelPreset();
	}

	public Command incrementFlywheelSpeed() {
		return Commands.runOnce(() -> shooter.incrementFlywheelOffset());
	}

	public Command decrementFlywheelSpeed() {
		return Commands.runOnce(() -> shooter.decrementFlywheelOffset());
	}


	public Command offsetTurretLeft(){
		return Commands.runOnce(() ->
			turret.offsetLeft()
		);
	}

	public Command offsetTurretRight(){
		return Commands.runOnce(() ->
			turret.offsetRight()
		);
	}

	public Command zeroTurretOffset(){
		return Commands.runOnce(() ->
					turret.zeroOffset()
				);
	}

	/** Jog the turret's true-zero position left/right (use while stowed to correct drift). */
	public Command nudgeTurretZeroLeft(){
		return turret.nudgeZeroLeftCommand();
	}

	public Command nudgeTurretZeroRight(){
		return turret.nudgeZeroRightCommand();
	}

	public Command reverseAllSystems(){
		return Commands.parallel(
			// Intake stays off while reversing; only the roller floor and chute run in reverse.
			IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.IDLE),
			RollerFloorSubsystem.mInstance.setpointCommand(RollerFloorSubsystem.REVERSE),
			ChuteSubsystem.mInstance.setpointCommand(ChuteSubsystem.REVERSE)
		);
	}

	public Command idleAllSystems(){
		return Commands.parallel(
			IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.IDLE),
			RollerFloorSubsystem.mInstance.setpointCommand(RollerFloorSubsystem.IDLE),
			ChuteSubsystem.mInstance.setpointCommand(ChuteSubsystem.IDLE)
		);
	}

	public Command stowIntake(){
		return PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.AGITATE);
	}

	/**
	 * Enables persistent defence mode: stows intake pivot, stops flywheel, stows
	 * turret/hood. State persists after command ends — call disableDefenceModeCommand() to exit.
	 */
	public Command enableDefenceModeCommand() {
		return Commands.runOnce(() -> {
			defenceModeActive = true;
			shooter.setFlywheelPreset(0);
			turret.setStowed(true);
			hood.setStowed(true);
			Commands.waitSeconds(1);
			PivotSubsystem.mInstance.applySetpoint(PivotSubsystem.STOW_DEFENCE);
			SmartDashboard.putBoolean("SuperSystem/DefenceMode", true);
		});
	}

	/**
	 * Disables defence mode and restores normal shooter/turret/hood behaviour.
	 */
	public Command disableDefenceModeCommand() {
		return Commands.runOnce(() -> {
			defenceModeActive = false;
			PivotSubsystem.mInstance.applySetpoint(PivotSubsystem.DEPLOY);
			Commands.waitSeconds(0.2);
			shooter.setDefaultCommand(shooter.runIdleCommand()); // show mode: never restore constant spin
			SmartDashboard.putBoolean("SuperSystem/DefenceMode", false);
		});
	}

	/** Toggles defence mode on or off. */
	public Command toggleDefenceModeCommand() {
		return Commands.runOnce(() -> {
			if (defenceModeActive) {
				disableDefenceModeCommand();
			} else {
				enableDefenceModeCommand();
			}
		});
	}

	public Command agitateChassisCommand() {
		final double kRotationRateRadPerSec = Math.toRadians(12.5); // 12.5 deg/s → 10° in 0.8s
		final double kDurationSecs = 0.8;
		SwerveRequest.FieldCentric rotateRequest = new SwerveRequest.FieldCentric();

		return Commands.sequence(
			DriveSubsystem.mInstance.run(() ->
				DriveSubsystem.mInstance.setSwerveRequest(
					rotateRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(kRotationRateRadPerSec)
				)
			).withTimeout(kDurationSecs),
			DriveSubsystem.mInstance.run(() ->
				DriveSubsystem.mInstance.setSwerveRequest(
					rotateRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(-kRotationRateRadPerSec)
				)
			).withTimeout(kDurationSecs)
		);
	}

	/** Repeatedly cycles the intake pivot between AGITATE and DEPLOY while shooting. */
	public Command pivotAgitateLoopCommand() {
		return Commands.sequence(
			PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.AGITATE),
			Commands.waitSeconds(0.15),
			PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.DEPLOY),
			Commands.waitSeconds(0.15)
		).repeatedly();
	}

	public Command agitateCommand(){
		return
					
						Commands.sequence( 
							Shoot(),
						//	Intake(),
							Commands.sequence(
								PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.AGITATE)),
								Commands.waitSeconds(0.2),
								PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.DEPLOY))
					.repeatedly()
				.handleInterrupt( () ->
					Commands.sequence(
									PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.DEPLOY),
									idleIntakes(),
									idleShooter()
					)

				);
	}

}
