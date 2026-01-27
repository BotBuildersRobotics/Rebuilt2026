package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.AllianceFlipUtil;
import frc.robot.subsystems.chute.ChuteSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shuffla.ShufflaSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;



public class SuperSystem extends SubsystemBase {
    
    public static SuperSystem mInstance;

	//private AprilTagFieldLayout kAprilTagMap = AprilTagFieldLayout.loadField(AprilTagFields);


    public static SuperSystem getInstance() {

        //Rethink this for how advantage kit does 
		if (mInstance == null) {
			mInstance = new SuperSystem();
		}
		return mInstance;
	}

	public SuperSystem(){
		
	}

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		
        builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);

    }

    @Override
	public void periodic() {
		
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
		return TurretSubsystem.mInstance.pointAtFieldPosition(frc.robot.FieldConstants.Hub.innerCenterPoint.toTranslation2d());
	}

	public Command AimAtPassingZone(){
		return TurretSubsystem.mInstance.pointAtFieldPosition((frc.robot.FieldConstants.Outpost.centerPoint));
	}

}
