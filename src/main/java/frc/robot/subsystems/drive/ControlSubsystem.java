package frc.robot.subsystems.drive;


import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;



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
	
	}

    public void driverControls() {

		SuperSystem s = SuperSystem.mInstance;

		driver.rightTrigger().onTrue(
			s.Intake()
		).onFalse(
			s.idleIntakes()
		);

		driver.x().onTrue(TurretSubsystem.mInstance.zeroCommand());
		driver.y().onTrue(TurretSubsystem.mInstance.runTrackTargetCommand());

		driver.b().onTrue(SuperSystem.mInstance.AimAtCenterHub());
		driver.rightBumper().onTrue(SuperSystem.mInstance.AimAtPassingZone());

	
		driver.leftBumper().onTrue(ShooterSubsystem.mInstance.runTrackTargetActiveShootingCommand());
		driver.a().onTrue(ShooterSubsystem.mInstance.stopCommnad());
		
    }

    public void setRumble(boolean on) {
		//ControlBoardConstants.mDriverController.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}



}
