package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
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
		
		shooter.setDefaultCommand(shooter.runTrackTargetActiveShootingCommand());
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

		 // Clear shooting parameters so they are recalculated each tick
    	shotCalc.clearShootingParameters();
		SmartDashboard.putBoolean("SuperSystem/DefenceMode", defenceModeActive);
		SmartDashboard.putBoolean("Drive/OnOpponentSide", ShiftHelpers.isOnOpponentSide());
		SmartDashboard.putBoolean("Drive/InOpponentZone", ShiftHelpers.isInOpponentZone());
		SmartDashboard.putBoolean("Hub/BlueActive", ShiftHelpers.isBlueHubActive());
		SmartDashboard.putBoolean("Hub/RedActive", ShiftHelpers.isRedHubActive());
		SmartDashboard.putBoolean("Hub/AreWeRedActive", ShiftHelpers.isRedHubActive() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
		SmartDashboard.putBoolean("Hub/AreWeActive", (ShiftHelpers.isRedHubActive() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) || (ShiftHelpers.isBlueHubActive() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue));
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


	public Command Shoot(){
		return
		Commands.parallel(
			RollerFloorSubsystem.mInstance.runShootCommandGated(shooter::isAtSpeed),
			ChuteSubsystem.mInstance.runShootCommandGated(shooter::isAtSpeed)
		);
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
				shooter.setDefaultCommand(shooter.runTrackTargetActiveShootingCommand());
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
				shooter.setDefaultCommand(shooter.runTrackTargetActiveShootingCommand());
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

	public Command reverseAllSystems(){
		return Commands.parallel(
			IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.REVERSE),
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
			shooter.setDefaultCommand(shooter.runTrackTargetActiveShootingCommand());
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
