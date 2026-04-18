package frc.robot.subsystems.drive;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Set;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.ShiftHelpers;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.chute.ChuteSubsystem;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shuffla.ShufflaSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Limelight;
import static edu.wpi.first.units.Units.RadiansPerSecond;



public class ControlSubsystem {

    public static final ControlSubsystem mInstance = new ControlSubsystem();

	private CommandXboxController driver = ControlBoardConstants.mDriverController;
	private CommandXboxController operator = ControlBoardConstants.mOperatorController;
	

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	// When false, right trigger always shoots regardless of field position.
	// Toggle with driver B button when localisation is unreliable.
	private boolean passingEnabled = true;

	// How long after pressing the trigger before the chute/shuffla start feeding.
	// Gives the turret and hood time to reach their target before the ball enters the shooter.
	private static final LoggedTunableNumber shootDelay =
		new LoggedTunableNumber("Driver/ShootDelaySec");


    public void configureBindings() {
		DriveSubsystem.mInstance.setDefaultCommand(DriveSubsystem.mInstance.followSwerveRequestCommand(
				DriveConstants.teleopRequest, DriveConstants.teleopRequestUpdater));
		
        //back button to re-seed heading       
        driver.back()
				.onTrue(Commands.runOnce(
								() -> DriveSubsystem.mInstance.getGeneratedDrive().seedFieldCentric(), DriveSubsystem.mInstance)
						.ignoringDisable(true));

		SmartDashboard.putBoolean("Passing Enabled", passingEnabled);
		driverControls();
		operatorControls();

		// Rumble the driver controller 5 seconds before our hub activates
		ShiftHelpers.hubAboutToActivate(5.0)
			.onTrue(Commands.startEnd(
				() -> driver.getHID().setRumble(RumbleType.kBothRumble, 1.0),
				() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.0)
			).withTimeout(1.0));
		
		ShiftHelpers.hubAboutToActivate(10.0)
			.onTrue(Commands.startEnd(
				() -> operator.getHID().setRumble(RumbleType.kBothRumble, 1.0),
				() -> operator.getHID().setRumble(RumbleType.kBothRumble, 0.0)
			).withTimeout(1.0));
	
	}

    public void driverControls() {

		SuperSystem s = SuperSystem.mInstance;

		driver.leftTrigger().whileTrue(
			s.intakeContinuousCommand()
		).onFalse(
			s.idleIntakes()
		);

		shootDelay.initDefault(0.3);

		// Trigger pull (opponent side + passing enabled): pass over the bump.
		// Separate binding so the pass commands' turret requirement doesn't bleed into
		// the shoot binding and cancel the turret's tracking default command.
		driver.rightTrigger()
			.and(() -> passingEnabled && ShiftHelpers.isOnOpponentSide())
			.whileTrue(new ConditionalCommand(s.passLobAuto(), s.passAutoSCR(), ShiftHelpers::isInOpponentZone))
			.onFalse(Commands.parallel(s.stowTurretHood(), s.idleShooter(), s.idleIntakes()));

		// Trigger pull (alliance side, or passing disabled): normal shoot.
		// No turret requirement here — turret default command provides field-relative tracking.
		driver.rightTrigger()
			.and(() -> !passingEnabled || !ShiftHelpers.isOnOpponentSide())
			.whileTrue(
				Commands.sequence(
					Commands.parallel(
						s.activeTurretHood(),
						Commands.runOnce(() -> DriveConstants.setShootingSpeedLimited(true))
					),
					Commands.defer(() -> Commands.waitSeconds(shootDelay.get()), Set.of()),
					Commands.parallel(
						s.Shoot(),
						s.intakePulseCommand()
					)
				)
			).onFalse(
				Commands.parallel(
					s.stowTurretHood(),
					s.idleShooter(),
					s.idleIntakes(),
					Commands.runOnce(() -> DriveConstants.setShootingSpeedLimited(false))
				)
			);

		// Right bumper: shoot + agitate intake pivot simultaneously
		driver.rightBumper().whileTrue(
			Commands.sequence(
				Commands.parallel(
					s.activeTurretHood(),
					Commands.runOnce(() -> DriveConstants.setShootingSpeedLimited(true))
				),
				Commands.defer(() -> Commands.waitSeconds(shootDelay.get()), Set.of()),
				Commands.parallel(
					s.Shoot(),
					s.intakePulseCommand(),
					s.pivotAgitateLoopCommand()
				)
			)
		).onFalse(
			Commands.parallel(
				s.stowTurretHood(),
				s.idleShooter(),
				s.idleIntakes(),
				PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.DEPLOY),
				Commands.runOnce(() -> DriveConstants.setShootingSpeedLimited(false))
			)
		);

		// B button: toggle passing on/off (disable when localisation is unreliable)
		driver.b().onTrue(Commands.runOnce(() -> {
			passingEnabled = !passingEnabled;
			SmartDashboard.putBoolean("Passing Enabled", passingEnabled);
		}).ignoringDisable(true));


		driver.povUp().onTrue(s.incrementFlywheelSpeed());
		driver.povDown().onTrue(s.decrementFlywheelSpeed());


		driver.leftBumper().onTrue(
			PivotSubsystem.mInstance.findDeployLimitCommand()
		);

		driver.start().onTrue(DriveSubsystem.mInstance.runOnce( () ->DriveSubsystem.mInstance.getDrivetrain().seedFieldCentric()));

		
    }

	public void operatorControls(){

		SuperSystem s = SuperSystem.mInstance;

	//	operator.b().onTrue(s.enableStow());

		// Turret offset correction
		operator.leftTrigger().onTrue(s.offsetTurretLeft());
		operator.rightTrigger().onTrue(s.offsetTurretRight());

		// Climb: left bumper = climb to position, right bumper = stow back to zero
		operator.leftBumper().onTrue(s.climbExtend());
		operator.rightBumper().onTrue(s.climbStow());

		operator.b().onTrue(s.climb());

		operator.y()
			.onTrue(Commands.runOnce(() -> {
			
					IntakeSubsystem.mInstance.applySetpoint(IntakeSubsystem.REVERSE);
					ChuteSubsystem.mInstance.applySetpoint(ChuteSubsystem.REVERSE);
					ShufflaSubsystem.mInstance.applySetpoint(ShufflaSubsystem.REVERSE);
			
			}))
			.onFalse(Commands.runOnce(() -> {

					IntakeSubsystem.mInstance.applySetpoint(IntakeSubsystem.IDLE);
					ChuteSubsystem.mInstance.applySetpoint(ChuteSubsystem.IDLE);
					ShufflaSubsystem.mInstance.applySetpoint(ShufflaSubsystem.IDLE);
			}
			));
		
		operator.povDown().onTrue(ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.CLIMB)).onFalse(
			ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.STOP)
		);

		operator.povUp().onTrue(ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.REVERSE)).onFalse(
			ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.STOP)
		);


		// Zero turret
		operator.x().onTrue(s.zeroTurretCommand());

		// Reverse all systems (intake, shuffla, chute) — hold to reverse
		//operator.y()
		//	.onTrue(s.reverseAllSystems())
		//	.onFalse(s.idleAllSystems());

		// Stow intake
		operator.a().onTrue(s.stowIntake());

		// Vision calibration mode
		operator.back().onTrue(Limelight.mInstance.toggleCalibrationMode());
	}

    public void setRumble(boolean on) {
		//ControlBoardConstants.mDriverController.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}



}
