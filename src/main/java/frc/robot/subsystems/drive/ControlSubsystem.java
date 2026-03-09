package frc.robot.subsystems.drive;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ShiftHelpers;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Limelight;
import static edu.wpi.first.units.Units.RadiansPerSecond;



public class ControlSubsystem {

    public static final ControlSubsystem mInstance = new ControlSubsystem();

	private CommandXboxController driver = ControlBoardConstants.mDriverController;
	private CommandXboxController operator = ControlBoardConstants.mOperatorController;
	

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    public void configureBindings() {
		DriveSubsystem.mInstance.setDefaultCommand(DriveSubsystem.mInstance.followSwerveRequestCommand(
				DriveConstants.teleopRequest, DriveConstants.teleopRequestUpdater));
		
        //back button to re-seed heading       
        driver.back()
				.onTrue(Commands.runOnce(
								() -> DriveSubsystem.mInstance.getGeneratedDrive().seedFieldCentric(), DriveSubsystem.mInstance)
						.ignoringDisable(true));

		driverControls();
		operatorControls();

		// Rumble the driver controller 5 seconds before our hub activates
		ShiftHelpers.hubAboutToActivate(5.0)
			.onTrue(Commands.startEnd(
				() -> driver.getHID().setRumble(RumbleType.kBothRumble, 1.0),
				() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.0)
			).withTimeout(1.0));
	}

    public void driverControls() {

		SuperSystem s = SuperSystem.mInstance;

		driver.leftTrigger().onTrue(
			s.Intake()
		).onFalse(
			s.idleIntakes()
		);

		driver.rightTrigger().onTrue(
			Commands.parallel(
				s.Shoot(),
				Commands.runOnce(() -> DriveConstants.setShootingSpeedLimited(true))
			)
		).onFalse(
			Commands.parallel(
				s.idleShooter(),
				Commands.runOnce(() -> DriveConstants.setShootingSpeedLimited(false))
			)
		);

		driver.a().onTrue(s.toggleStow());

		driver.b().whileTrue(s.passAuto());

		//driver.x().onTrue(TurretSubsystem.mInstance.zeroCommand());

		driver.leftBumper().onTrue(
			PivotSubsystem.mInstance.findDeployLimitCommand()
		);

		driver.start().onTrue(DriveSubsystem.mInstance.runOnce( () ->DriveSubsystem.mInstance.getDrivetrain().seedFieldCentric()));

		//driver.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
		//driver.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

		/*
		* Joystick Y = quasistatic forward
		* Joystick A = quasistatic reverse
		* Joystick B = dynamic forward
		* Joystick X = dyanmic reverse
		*/
		/*driver.y().whileTrue(ShooterSubsystem.mInstance.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		driver.a().whileTrue(ShooterSubsystem.mInstance.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		driver.b().whileTrue(ShooterSubsystem.mInstance.sysIdDynamic(SysIdRoutine.Direction.kForward));
		driver.x().whileTrue(ShooterSubsystem.mInstance.sysIdDynamic(SysIdRoutine.Direction.kReverse));*/
				
    }

	public void operatorControls(){

		SuperSystem s = SuperSystem.mInstance;

		operator.b().onTrue(s.enableStow());

		// Turret offset correction
		operator.leftTrigger().onTrue(s.offsetTurretLeft());
		operator.rightTrigger().onTrue(s.offsetTurretRight());

		// Climb: left bumper = climb to position, right bumper = stow back to zero
		operator.leftBumper().onTrue(s.climbUp());
		operator.rightBumper().onTrue(s.climbStow());
		
		operator.povDown().onTrue(ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.CLIMB)).onFalse(
			ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.STOP)
		);

		operator.povUp().onTrue(ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.REVERSE)).onFalse(
			ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.STOP)
		);


		// Zero turret
		operator.x().onTrue(s.zeroTurretCommand());

		// Reverse all systems (intake, shuffla, chute) — hold to reverse
		operator.y()
			.onTrue(s.reverseAllSystems())
			.onFalse(s.idleAllSystems());

		// Stow intake
		operator.a().onTrue(s.stowIntake());

		// Vision calibration mode
		operator.back().onTrue(Limelight.mInstance.toggleCalibrationMode());
	}

    public void setRumble(boolean on) {
		//ControlBoardConstants.mDriverController.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}



}
