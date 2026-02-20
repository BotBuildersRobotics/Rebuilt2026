package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.AllianceFlipUtil;
import frc.robot.subsystems.chute.ChuteSubsystem;
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
		shotCalc = new ShotCalculator();
		shotCalc.getParameters(); // THIS IS IMPORTANT -  it will cause the field json to load, before teleop init

		turret.setShotCalculator(shotCalc);
		shooter.setShotCalculator(shotCalc);
		
		shooter.setDefaultCommand(shooter.runTrackTargetActiveShootingCommand());
		turret.setDefaultCommand(turret.runTrackTargetActiveShootingCommand());

		
		
	}

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		
        builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);

    }

    @Override
	public void periodic() {
		
		 // Clear shooting parameters
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

	public Command AimAtCenterHub(){
		return turret.pointAtFieldPosition(frc.robot.FieldConstants.Hub.innerCenterPoint.toTranslation2d());
	}

	public Command AimAtPassingZone(){
		return turret.pointAtFieldPosition((frc.robot.FieldConstants.Outpost.centerPoint));
	}





	public Command agitateCommand(){
		return //runOnce( () ->
					
					
						Commands.sequence( 
							Shoot(),
						//	Intake(),
							Commands.sequence(
								PivotSubsystem.mInstance.setpointCommand(PivotSubsystem.AGITATE)),
							//Commands.waitUntil(PivotSubsystem.mInstance.isPositionWithinTolerance()),
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
