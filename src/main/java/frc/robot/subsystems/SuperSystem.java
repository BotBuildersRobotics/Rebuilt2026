package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.AllianceFlipUtil;
import frc.robot.subsystems.chute.ChuteSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shuffla.ShufflaSubsystem;
import frc.robot.subsystems.turret.ShotCalculator;
import frc.robot.subsystems.turret.TurretSubsystem;



public class SuperSystem extends SubsystemBase {
    
    public static SuperSystem mInstance;

	private TurretSubsystem turret;

	private ShooterSubsystem shooter;

	private ShotCalculator shotCalc;

	private HoodSubsystem hood;

	//private AprilTagFieldLayout kAprilTagMap = AprilTagFieldLayout.loadField(AprilTagFields);


    public static SuperSystem getInstance() {

        //Rethink this for how advantage kit does 
		if (mInstance == null) {
			mInstance = new SuperSystem();
		}
		return mInstance;
	}

	public SuperSystem(){

		shooter = new ShooterSubsystem();
		turret = new TurretSubsystem();
		hood = new HoodSubsystem();
		shotCalc = new ShotCalculator();
		shotCalc.getParameters(); // THIS IS IMPORTANT -  it will cause the field json to load, before teleop init

		hood.setShotCalculator(shotCalc);
		turret.setShotCalculator(shotCalc);
		shooter.setShotCalculator(shotCalc);
		
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
	}

	 public Command idleIntakes() {
		
		return IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.IDLE);
					
	}

	public Command Intake() {
		
		return IntakeSubsystem.mInstance.setpointCommand(IntakeSubsystem.INTAKE);
					
	}

	public Command Shoot(){
		return 
		Commands.parallel(
			ShufflaSubsystem.mInstance.setpointCommand(ShufflaSubsystem.SHOOT),
			ChuteSubsystem.mInstance.setpointCommand(ChuteSubsystem.SHOOT)
		
		);
	}

	public Command idleShooter(){

		return 
		Commands.parallel(
			ShufflaSubsystem.mInstance.setpointCommand(ShufflaSubsystem.IDLE),
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
			shooter.runPassingCommand(),
			hood.runPassingCommand()
		);
	}

	public Command passRight(){
		return Commands.parallel(
			turret.passRightCommand(),
			shooter.runPassingCommand(),
			hood.runPassingCommand()
		);
	}

	public Command passAuto(){
		return Commands.parallel(
			turret.passAutoCommand(),
			shooter.runPassingCommand(),
			hood.runPassingCommand()
		);
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

	public Command setManualShooterVelocity(){
		//use the tunable value;

		return
		Commands.parallel(
			shooter.setManualShooterVelocity(),
			hood.setManualHoodAngle()
		);
	}

	public Command resetAutoMap(){
		return 
		Commands.sequence(		
			shooter.resetAutoMap(),
		 	hood.resetAutoMap()
		);
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
