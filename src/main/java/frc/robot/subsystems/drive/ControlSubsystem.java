package frc.robot.subsystems.drive;


import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.pivot.PivotSubsystem;



public class ControlSubsystem {

    public static final ControlSubsystem mInstance = new ControlSubsystem();

	private ControllerMap driver = ControlBoardConstants.mDriverController;


	/**
	 * SHOW MODE (Ekka outreach build) &mdash; deliberately minimal bindings.
	 *
	 * <p>The competition control set has been stripped out: no passing, no hub tracking, no defence
	 * mode, no turret offsets, no flywheel trim, no turbo, and no operator controls at all. Everything
	 * removed depended on a pose estimate, AprilTags, alliance side, or hub-shift timing &mdash; none
	 * of which exist at a demo. What the driver has left:
	 *
	 * <ul>
	 *   <li><b>Left trigger</b> &mdash; hold to deploy the intake pivot and run the intake.
	 *   <li><b>Right trigger</b> &mdash; hold for the net shot: turret swings 90 deg left into the
	 *       net, fixed flywheel and hood. See {@link SuperSystem#showNetShotCommand}.
	 *   <li><b>Right bumper</b> &mdash; hold for the catch shot: straight ahead, no hood, gentle, for
	 *       a spectator to catch. See {@link SuperSystem#showCatchShotCommand}.
	 *   <li><b>Back</b> &mdash; re-seed field-centric heading (the robot gets carried around at a show).
	 * </ul>
	 *
	 * <p>Setup actions that are needed once per event but must not be on the controller mid-show
	 * (pivot homing, turret zeroing) live on the dashboard instead &mdash; see {@code RobotContainer}.
	 *
	 * <p>The competition bindings are not lost, they are on the {@code OffseasonRobot} branch this one
	 * was cut from.
	 */
    public void configureBindings() {
		DriveSubsystem.mInstance.setDefaultCommand(DriveSubsystem.mInstance.followSwerveRequestCommand(
				DriveConstants.teleopRequest, DriveConstants.teleopRequestUpdater));

		driver.back()
				.onTrue(Commands.runOnce(
								() -> DriveSubsystem.mInstance.getGeneratedDrive().seedFieldCentric(), DriveSubsystem.mInstance)
						.ignoringDisable(true));

		// Park the turret and hood stowed. Only the show shot moves the turret off zero, and it puts it
		// back on release, so between shots the turret sits over its homing sensor.
		SuperSystem.mInstance.enterShowModeCommand().schedule();

		driverControls();
	}

    public void driverControls() {

		SuperSystem s = SuperSystem.mInstance;

		// Left trigger: intake. The pivot deploy is folded into this binding rather than sitting on its
		// own button, because the dedicated deploy button is gone in show mode. This is a plain setpoint
		// move, not the current-sensing homing routine — home once on the dashboard at setup, then this
		// just returns to the found deploy angle. The pivot is left deployed on release so it is not
		// slamming up and down all day.
		driver.leftTrigger().whileTrue(
			Commands.parallel(
				PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.DEPLOY),
				s.intakeContinuousCommand()
			)
		).onFalse(
			s.idleIntakes()
		);

		// Right trigger: the show shot. Turret swings to the net, fixed flywheel speed and hood angle,
		// feed gated on everything having arrived. Nothing here reads the pose estimator.
		//
		// onTrue, not whileTrue: the command deliberately outlives the button so it can stop the feed
		// first and only then bring the turret home (see showShotCommand). The trigger is passed in so
		// the command can see the release itself.
		driver.rightTrigger().onTrue(
			s.showNetShotCommand(driver.rightTrigger())
		);

		// Right bumper: the catch shot. Straight ahead, no hood, gentle — a soft pop for a spectator to
		// catch. Same gating and staged release as the net shot, just different numbers. Shares every
		// subsystem with it, so whichever is pressed second cleanly interrupts the first.
		driver.rightBumper().onTrue(
			s.showCatchShotCommand(driver.rightBumper())
		);
    }

    public void setRumble(boolean on) {
		//ControlBoardConstants.mDriverController.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}

}
