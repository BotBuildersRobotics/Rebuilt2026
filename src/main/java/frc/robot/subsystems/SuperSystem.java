package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.AllianceFlipUtil;
import frc.robot.subsystems.intake.IntakeSubsystem;
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

	public Command AimAtCenterHub(){
		return TurretSubsystem.mInstance.pointAtFieldPosition(frc.robot.FieldConstants.hubCenter);
	}

	public Command AimAtPassingZone(){
		return TurretSubsystem.mInstance.pointAtFieldPosition((frc.robot.FieldConstants.fieldLayout.getTagPose(29).get().getTranslation().toTranslation2d()));
	}

}
