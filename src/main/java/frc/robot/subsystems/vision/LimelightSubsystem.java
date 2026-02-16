package frc.robot.subsystems.vision;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTracer;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
	protected final List<VisionIOLimelight> ios = new ArrayList<>();

	public LimelightSubsystem(VisionIOConfig... configs) {
		super("Limelight");

		for (VisionIOConfig config : configs) {
			VisionIOLimelight io = new VisionIOLimelight(config);

			if (Robot.isReal()) {
				LimelightHelpers.setCameraPose_RobotSpace(
						config.name,
						config.robotToCameraOffset.getX(),
						config.robotToCameraOffset.getY(),
						config.robotToCameraOffset.getZ(),
						Units.radiansToDegrees(
								config.robotToCameraOffset.getRotation().getX()),
						Units.radiansToDegrees(
								config.robotToCameraOffset.getRotation().getY()),
						Units.radiansToDegrees(
								config.robotToCameraOffset.getRotation().getZ()));
			}

			ios.add(io);
		}
	}

	@Override
	public void periodic() {
		for (VisionIOLimelight io : ios) {
			io.update();
		}
		outputTelemetry();
	}

	public void disable(boolean disable) {
		for (VisionIOLimelight io : ios) {
			io.disable(disable);
		}
	}

	public boolean getDisabled() {
		return !ios.isEmpty() && ios.get(0).getDisabled();
	}

	public void outputTelemetry() {
		LoggedTracer.record("Vision");
	}

	public static class LimelightConfigBackRight extends VisionIOConfig {
		public LimelightConfigBackRight() {
			name = "limelight-right";
			robotToCameraOffset = new Pose3d();
			aprilTagVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);
		}
	}
	public static class LimelightConfigBackLeft extends VisionIOConfig {
		public LimelightConfigBackLeft() {
			name = "limelight-left";
			robotToCameraOffset = new Pose3d();
			aprilTagVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);
		}
	}
	public static class LimelightConfigBackSide extends VisionIOConfig {
		public LimelightConfigBackSide() {
		 	name = "limelight-side";
			robotToCameraOffset = new Pose3d();
			aprilTagVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);
		}
	}
}
