package frc.robot.subsystems.vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.drive.DriveSubsystem;

public class Limelight extends LimelightSubsystem {
	public static final Limelight mInstance = new Limelight();

	private Pose2d lastPose = new Pose2d();
	private long numPoseStableUpdates = 0;

	private Limelight() {
		super(
			LimelightConstants.getVisionIOConfigBackLeft(),
			LimelightConstants.getVisionIOConfigBackRight(),
			LimelightConstants.getVisionIOConfigSide()
		);
	}

	@Override
	public void periodic() {
		try {
			super.periodic();

			// Find the most recent estimate across all cameras
			Pose2d mostRecentPose = null;
			Time mostRecentTime = Units.Seconds.of(0.0);
			for (VisionIOLimelight io : ios) {
				Time ioTime = io.getLatestEstimateTime();
				if (ioTime.gt(mostRecentTime)) {
					mostRecentTime = ioTime;
					mostRecentPose = io.getLatestEstimate();
				}
			}

			if (mostRecentPose != null && mostRecentPose != lastPose) {
				if (DriveSubsystem.mInstance.getPose().getTranslation().getDistance(mostRecentPose.getTranslation())
						< LimelightConstants.agreedTranslationUpdateEpsilon.in(Units.Meters)) {
					numPoseStableUpdates++;
				} else {
					numPoseStableUpdates = 0;
				}
				lastPose = mostRecentPose;
			}

			SmartDashboard.putNumber("Vision/Num Agreed Stable Updates", numPoseStableUpdates);
		} catch (Exception e) {
			SmartDashboard.putNumber("Limelight/Crash", Timer.getFPGATimestamp());
			SmartDashboard.putString("Limelight/Crash Exception", e.getMessage());
		}
	}

	public Time getLastUpdateTime() {
		Time latest = Units.Seconds.of(0.0);
		for (VisionIOLimelight io : ios) {
			Time ioTime = io.getLatestEstimateTime();
			if (ioTime.gt(latest)) {
				latest = ioTime;
			}
		}
		return latest;
	}

	public Pose2d getLatestUpdate() {
		return lastPose;
	}

	public boolean getPoseStable() {
		return numPoseStableUpdates > LimelightConstants.agreedTranslationUpdatesThreshold;
	}
}
