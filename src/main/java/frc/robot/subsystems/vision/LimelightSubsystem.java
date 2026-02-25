package frc.robot.subsystems.vision;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTracer;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
	protected final List<VisionIOLimelight> ios = new ArrayList<>();

	// 0 = all cameras push updates, 1 = only best camera pushes
	private static final LoggedTunableNumber singleCameraMode =
			new LoggedTunableNumber("Vision/SingleCameraMode", 0);

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
		// Fetch estimates from all cameras
		for (VisionIOLimelight io : ios) {
			io.update();
		}

		// Decide which camera(s) push to drivetrain
		if (singleCameraMode.get() >= 1.0) {
			// Single camera mode: only push the best estimate
			VisionIOLimelight bestIO = null;
			double bestScore = Double.MAX_VALUE;

			for (VisionIOLimelight io : ios) {
				PoseEstimate est = io.getLatestPoseEstimate();
				if (est == null) continue;

				// Lower avgTagDist + more tags = better. Score: avgTagDist / tagCount
				double score = est.avgTagDist / est.tagCount;
				if (score < bestScore) {
					bestScore = score;
					bestIO = io;
				}
			}

			if (bestIO != null) {
				bestIO.pushToDrivetrain();
				SmartDashboard.putString("Vision/ActiveCamera", bestIO.getName());
				SmartDashboard.putNumber("Vision/ActiveCameraScore", bestScore);
			}
		} else {
			// All cameras push updates
			for (VisionIOLimelight io : ios) {
				io.pushToDrivetrain();
			}
			SmartDashboard.putString("Vision/ActiveCamera", "all");
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

		// AdvantageKit structured logging for replay
		Logger.recordOutput("Vision/SingleCameraMode", singleCameraMode.get() >= 1.0);
		Logger.recordOutput("Vision/CameraCount", ios.size());
		for (int i = 0; i < ios.size(); i++) {
			PoseEstimate est = ios.get(i).getLatestPoseEstimate();
			String prefix = "Vision/Camera" + i + "/";
			Logger.recordOutput(prefix + "Name", ios.get(i).getName());
			if (est != null) {
				Logger.recordOutput(prefix + "Pose", est.pose);
				Logger.recordOutput(prefix + "TagCount", est.tagCount);
				Logger.recordOutput(prefix + "AvgTagDist", est.avgTagDist);
			}
		}
	}

}
