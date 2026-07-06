// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;



import frc.robot.auto.BLineAutos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.chute.ChuteSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.ControlSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.rollerFloor.RollerFloorSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.photon.FuelDetectionSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 @SuppressWarnings("unused")
public class RobotContainer {

	// get an instance of our subsystem, either sim or pheonix.

	private SuperSystem superSystem = SuperSystem.getInstance();


	/* Path follower */
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		
		//Super system owns the shooter, hood, turret
		for (SubsystemBase s : new SubsystemBase[] {
			
			DriveSubsystem.mInstance,
			Limelight.mInstance,
			IntakeSubsystem.mInstance,
			PivotSubsystem.mInstance,
			ChuteSubsystem.mInstance,
			RollerFloorSubsystem.mInstance,
			SuperSystem.mInstance,
			//FuelDetectionSubsystem.mInstance

		}) {
			SmartDashboard.putData(s);
		}

		Limelight.mInstance.disable(false);

		// Dashboard buttons for zeroing mechanisms
		SmartDashboard.putData("Zero Turret", SuperSystem.mInstance.zeroTurretCommand());

		// SignalLogger control
		SmartDashboard.putData("SysId/Signal Logger Start", Commands.runOnce(SignalLogger::start).ignoringDisable(true).withName("Signal Logger Start"));
		SmartDashboard.putData("SysId/Signal Logger Stop",  Commands.runOnce(SignalLogger::stop).ignoringDisable(true).withName("Signal Logger Stop"));

		// Shooter SysId — run these in test mode only, one at a time
		//SmartDashboard.putData("SysId/Shooter Quasistatic Fwd", SuperSystem.mInstance.shooterSysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
		//SmartDashboard.putData("SysId/Shooter Quasistatic Rev", SuperSystem.mInstance.shooterSysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
		//SmartDashboard.putData("SysId/Shooter Dynamic Fwd",     SuperSystem.mInstance.shooterSysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
		//SmartDashboard.putData("SysId/Shooter Dynamic Rev",     SuperSystem.mInstance.shooterSysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));

		CommandScheduler.getInstance().setPeriod(0.02);

		NamedCommands.registerCommand("DeployIntake",
		
			SuperSystem.mInstance.DeployIntake()
		
		);

		NamedCommands.registerCommand("Intake",
		
			SuperSystem.mInstance.Intake()
		
		);

		NamedCommands.registerCommand("idleIntake",
		
			SuperSystem.mInstance.idleIntakes()
		
		);

		NamedCommands.registerCommand("Shoot",
		
			SuperSystem.mInstance.ShootAuto()
		
		);

		NamedCommands.registerCommand("idleShooter",
		
			SuperSystem.mInstance.idleShooter()
		
		);

		NamedCommands.registerCommand("stowTurret",
		
			SuperSystem.mInstance.enableStow()
		
		);

		NamedCommands.registerCommand("aimTurret",

			SuperSystem.mInstance.disableStow()

		);

		NamedCommands.registerCommand("idleAllSystems",

			SuperSystem.mInstance.idleAllSystems()

		);

		NamedCommands.registerCommand("agitateChassisCommand",
			SuperSystem.mInstance.agitateChassisCommand().repeatedly().withTimeout(3.0)
		);

		NamedCommands.registerCommand("intakePulse",
			SuperSystem.mInstance.intakePulseCommand()
		);

		NamedCommands.registerCommand("setFlywheelSlow",
			SuperSystem.mInstance.setFlywheelPreset(180.0)
		);

		NamedCommands.registerCommand("setFlywheelWall",
			SuperSystem.mInstance.setFlywheelPreset(205.0)
		);

		NamedCommands.registerCommand("setFlywheelOCS",
			SuperSystem.mInstance.setFlywheelPreset(170)
		);

		NamedCommands.registerCommand("setFlywheelHub",  //used for auto
			SuperSystem.mInstance.setFlywheelPreset(220)
		);

		NamedCommands.registerCommand("setFlywheelFast",
			SuperSystem.mInstance.setFlywheelPreset(230.0)
		);

		NamedCommands.registerCommand("clearFlywheelPreset",
			SuperSystem.mInstance.clearFlywheelPreset()
		);

		NamedCommands.registerCommand("setTurretAngle1",
			SuperSystem.mInstance.setTurretAnglePreset(0.0)
		);

		NamedCommands.registerCommand("setTurretAngleLeftWall",
			SuperSystem.mInstance.setTurretAnglePreset(-73.0)
		);

		NamedCommands.registerCommand("setTurretAngleDepot",
			SuperSystem.mInstance.setTurretAnglePreset(-25.0)
		);

		NamedCommands.registerCommand("clearTurretAnglePreset",
			SuperSystem.mInstance.clearTurretAnglePreset()
		);

		NamedCommands.registerCommand("turretRightOffset",
			SuperSystem.mInstance.offsetTurretRight()
		);

		NamedCommands.registerCommand("turretLeftOffset",
			SuperSystem.mInstance.offsetTurretLeft()
		);

		NamedCommands.registerCommand("clearTurretAngleOffset",
			SuperSystem.mInstance.zeroTurretOffset()
		);

		autoChooser = AutoBuilder.buildAutoChooser();
		// Add BLine autos into the same chooser (prefixed "BLine: ") so getAutonomousCommand() is unchanged.
		BLineAutos.getInstance().registerBLineAutos(autoChooser);
		/*if(SmartDashboard.containsKey("Auto Mode")) {
			SmartDashboard.getEntry("Auto Mode").close();
		}*/
		SmartDashboard.putData("Auto Mode", autoChooser);

		configureBindings();
		

	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {


		ControlSubsystem.mInstance.configureBindings();		
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 * 
	 */
	public Command getAutonomousCommand() {
		
		return autoChooser.getSelected();
		// return Commands.print("Auto command selected");
	}
}
