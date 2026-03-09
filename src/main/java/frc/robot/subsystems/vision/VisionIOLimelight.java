
package frc.robot.subsystems.vision;


import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight extends VisionIO {
	private Pose2d latestEstimate = new Pose2d();
	private Time latestEstimateTime = Units.Seconds.of(0.0);
	private PoseEstimate latestPoseEstimate = null;
	private final VisionIOConfig config;

	protected StructPublisher<Pose2d> visPose = NetworkTableInstance.getDefault()
			.getTable("SmartDashboard/Vision")
			.getStructTopic("", Pose2d.struct)
			.publish();

	public VisionIOLimelight(VisionIOConfig config) {
		this.config = config;
	}

	/**
	 * Stores the pose estimate locally without pushing to drivetrain.
	 * Call pushToDrivetrain() to actually send the update.
	 */
	@Override
	public void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum) {

		SmartDashboard.putNumber(config.name + "/FGPA Timestamp", Timer.getFPGATimestamp());

		if(poseEstimate == null){
			latestPoseEstimate = null;
			return;
		}
		SmartDashboard.putNumber(config.name + "/Tag Count", poseEstimate.tagCount);

		SmartDashboard.putNumber(
				config.name + "/Estimate to FGPA Timestamp", Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds));

		if (poseEstimate.tagCount >= minTagNum) {
			latestEstimate = poseEstimate.pose;
			latestEstimateTime = Units.Seconds.of(poseEstimate.timestampSeconds);
			latestPoseEstimate = poseEstimate;
			visPose.set(poseEstimate.pose);
		} else {
			latestPoseEstimate = null;
		}
	}

	/**
	 * Pushes the latest valid pose estimate to the drivetrain's vision filter.
	 * Rejects poses outside the field boundary.
	 */
	public void pushToDrivetrain() {
		if (latestPoseEstimate != null && isInsideField(latestPoseEstimate.pose)) {
			DriveSubsystem.mInstance.getGeneratedDrive();
			DriveSubsystem.mInstance.addVisionUpdate(
					latestPoseEstimate.pose,
					Units.Seconds.of(latestPoseEstimate.timestampSeconds),
					LimelightConstants.enabledVisionStdDevs.times(latestPoseEstimate.avgTagDist));
		}
	}

	private static boolean isInsideField(Pose2d pose) {
		double x = pose.getX();
		double y = pose.getY();
		return x >= 0 && x <= FieldConstants.fieldLength
			&& y >= 0 && y <= FieldConstants.fieldWidth;
	}

	public PoseEstimate getLatestPoseEstimate() {
		return latestPoseEstimate;
	}

	public Pose2d getLatestEstimate() {
		return latestEstimate;
	}

	public Time getLatestEstimateTime() {
		return latestEstimateTime;
	}

	@Override
	public void update() {
		updateGyro();
		setLatestEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name), 1);

		// When MegaTag1 sees 2+ tags, use its rotation to correct gyro drift.
		// High translation std devs so only the heading is trusted.
		if (!disabled) {
			PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);
			if (mt1 != null && mt1.tagCount >= 2 && isInsideField(mt1.pose)) {
				DriveSubsystem.mInstance.addVisionUpdate(
					mt1.pose,
					Units.Seconds.of(mt1.timestampSeconds),
					VecBuilder.fill(9999.0, 9999.0, 0.1)
				);
				SmartDashboard.putNumber(config.name + "/MT1HeadingDeg", mt1.pose.getRotation().getDegrees());
			}
		}

		SmartDashboard.putBoolean(config.name + "/Disabled", disabled);
	}

	@Override
	public void disable(boolean disable) {
		super.disable(disable);

		if (disabled) {
			LimelightHelpers.setPipelineIndex(config.name, LimelightConstants.kDisabledPipeline);
		} else {
			LimelightHelpers.setPipelineIndex(config.name, LimelightConstants.kEnabledPipeline);
		}
	}

	private void updateGyro() {

		Rotation2d theta = DriveSubsystem.mInstance.getPose().getRotation();
		LimelightHelpers.SetRobotOrientation(config.name, theta.getDegrees(), 0, 0, 0, 0, 0);
	}

	public String getName() {
		return config.name;
	}
}
