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
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
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

		driver.rightTrigger().onTrue(
			s.Intake()
		).onFalse(
			s.idleIntakes()
		);

		driver.leftTrigger().onTrue(
			s.Shoot()
		).onFalse(
			s.idleShooter()
		);

		//driver.x().onTrue(TurretSubsystem.mInstance.zeroCommand());
		//driver.y().onTrue(TurretSubsystem.mInstance.runTrackTargetCommand());

		//driver.x().onTrue(PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.mInstance.DEPLOY));
		//driver.y().onTrue(PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.mInstance.AGITATE));

		//driver.b().onTrue(SuperSystem.mInstance.AimAtCenterHub());
		//driver.rightBumper().onTrue(SuperSystem.mInstance.AimAtPassingZone());

	
		//driver.leftBumper().onTrue(ShooterSubsystem.mInstance.runFixedCommand(() -> 10.0));
		/*driver.leftBumper().onTrue(ShooterSubsystem.mInstance.setpointCommand(
			Setpoint.withVelocitySetpoint(AngularVelocity.ofBaseUnits(10, RadiansPerSecond))
		));*/
		
		//driver.leftBumper().onTrue(ShooterSubsystem.mInstance.setpointCommand(ShooterSubsystem.SHOOT));
		//driver.leftBumper().onTrue(ShooterSubsystem.mInstance.runTrackTargetActiveShootingCommand());
		//driver.a().onTrue(ShooterSubsystem.mInstance.stopCommand());
		//driver.a().onTrue(ShooterSubsystem.mInstance.setpointCommand(ShooterSubsystem.IDLE));

		/*driver.leftBumper().onTrue(
			//Commands.parallel(
				SuperSystem.mInstance.agitateCommand()//,
			//	s.Shoot()
			//)
		);*/

		driver.rightBumper().onTrue(
			PivotSubsystem.mInstance.resetDeployPosition()
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

		operator.a().onTrue(
			SuperSystem.mInstance.setManualShooterVelocity()
		);	
		operator.b().onTrue(
			SuperSystem.mInstance.resetAutoMap()
		);	

		operator.leftTrigger().onTrue(
			SuperSystem.mInstance.offsetTurretLeft()
		);

		operator.rightTrigger().onTrue(
			SuperSystem.mInstance.offsetTurretRight()
		);

		operator.x().onTrue(LedSubsystem.mInstance.setBlue());

		operator.leftBumper().onTrue(ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.CLIMB) ).onFalse(ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.STOP));
		operator.rightBumper().onTrue(ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.REVERSE) ).onFalse(ClimbSubsystem.mInstance.setpointCommand(ClimbSubsystem.STOP));
	}

    public void setRumble(boolean on) {
		//ControlBoardConstants.mDriverController.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}



}
