package frc.robot.subsystems.vision;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTracer;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
	protected final List<VisionIOLimelight> ios = new ArrayList<>();

	// 0 = all cameras push updates, 1 = only best camera pushes
	private static final LoggedTunableNumber singleCameraMode =
			new LoggedTunableNumber("Vision/SingleCameraMode", 0);

	// Calibration mode: cameras still update but don't push to drivetrain
	private boolean calibrationMode = false;

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

		if (calibrationMode) {
			// In calibration mode: log per-camera poses but don't push to drivetrain
			outputCalibrationTelemetry();
			outputTelemetry();
			return;
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

	/**
	 * Toggles calibration mode. In calibration mode, cameras still update
	 * but pose estimates are NOT pushed to the drivetrain. Each camera's
	 * estimated pose is logged individually so you can compare them while
	 * rotating the robot in place.
	 *
	 * Look at VisionCal/{camera-name}/X, Y, HeadingDeg on SmartDashboard.
	 * At a known position, rotate the robot 180°. If a camera's X or Y
	 * shifts significantly, its offset is wrong.
	 */
	public Command toggleCalibrationMode() {
		return Commands.runOnce(() -> {
			calibrationMode = !calibrationMode;
			SmartDashboard.putBoolean("VisionCal/Active", calibrationMode);
		}).ignoringDisable(true);
	}

	public Command enableCalibrationMode() {
		return Commands.runOnce(() -> {
			calibrationMode = true;
			SmartDashboard.putBoolean("VisionCal/Active", calibrationMode);
		}).ignoringDisable(true);
	}

	public Command disableCalibrationMode() {
		return Commands.runOnce(() -> {
			calibrationMode = false;
			SmartDashboard.putBoolean("VisionCal/Active", calibrationMode);
		}).ignoringDisable(true);
	}

	private void outputCalibrationTelemetry() {
		Pose2d odomPose = frc.robot.subsystems.drive.DriveSubsystem.mInstance.getPose();
		SmartDashboard.putNumber("VisionCal/Odometry/X", odomPose.getX());
		SmartDashboard.putNumber("VisionCal/Odometry/Y", odomPose.getY());
		SmartDashboard.putNumber("VisionCal/Odometry/HeadingDeg", odomPose.getRotation().getDegrees());

		for (VisionIOLimelight io : ios) {
			PoseEstimate est = io.getLatestPoseEstimate();
			String prefix = "VisionCal/" + io.getName() + "/";

			if (est != null) {
				SmartDashboard.putNumber(prefix + "X", est.pose.getX());
				SmartDashboard.putNumber(prefix + "Y", est.pose.getY());
				SmartDashboard.putNumber(prefix + "HeadingDeg", est.pose.getRotation().getDegrees());
				SmartDashboard.putNumber(prefix + "TagCount", est.tagCount);
				SmartDashboard.putNumber(prefix + "AvgTagDist", est.avgTagDist);

				// Show error vs odometry
				SmartDashboard.putNumber(prefix + "ErrorX", est.pose.getX() - odomPose.getX());
				SmartDashboard.putNumber(prefix + "ErrorY", est.pose.getY() - odomPose.getY());

				Logger.recordOutput(prefix + "Pose", est.pose);
				Logger.recordOutput(prefix + "ErrorX", est.pose.getX() - odomPose.getX());
				Logger.recordOutput(prefix + "ErrorY", est.pose.getY() - odomPose.getY());
			} else {
				SmartDashboard.putNumber(prefix + "TagCount", 0);
			}
		}
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
