package frc.robot.subsystems.vision;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import frc.robot.subsystems.vision.LimelightSubsystem.LimelightConfigBackLeft;
import frc.robot.subsystems.vision.LimelightSubsystem.LimelightConfigBackRight;
import frc.robot.subsystems.vision.LimelightSubsystem.LimelightConfigBackSide;



public class LimelightConstants {
	public static final String kLimelightName = "limelight";

	public static final int kEnabledPipeline = 0;
	public static final int kDisabledPipeline = 1;
	public static final Vector<N3> enabledVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);

	public static Pose3d kRobotToCameraOffsetLeft;
	public static Pose3d kRobotToCameraOffsetRight;
	public static Pose3d kRobotToCameraOffsetSide;

	static {
		/*5 up 10 side


L:
X = 253.24371985mm 
Y = 266.08534095mm 
Z = 302.38062549mm 

R:
X = 253.24371789mm 
Y = -266.08534095mm 
Z = 302.38062549mm 

Side:
X = 341.20250702
Y = -211.61549741
Z = 481.82229676
 */

			kRobotToCameraOffsetLeft = new Pose3d(
					new Translation3d(
							Units.Centimeters.of(525.32),
							Units.Centimeters.of(26.1),
							Units.Centimeters.of(30.24)
						),
					new Rotation3d(
							Units.Degree.of(0),
							Units.Degree.of(0),
							Units.Degree.of(180)
						)
						);

			kRobotToCameraOffsetRight = new Pose3d(
					new Translation3d(
							Units.Centimeters.of(25.32),
							Units.Centimeters.of(26.61),
							Units.Centimeters.of(30.24)
						),
					new Rotation3d(
							Units.Degree.of(0),
							Units.Degree.of(0),
							Units.Degree.of(180)
						)
						);
			
			kRobotToCameraOffsetSide = new Pose3d(
					new Translation3d(
							Units.Centimeters.of(50),
							Units.Centimeters.of(15),
							Units.Centimeters.of(0)
						),
						
					new Rotation3d(
							Units.Degree.of(0),
							Units.Degree.of(0),
							Units.Degree.of(90)
						)
						);

	}

	public static final LimelightConfigBackRight getVisionIOConfigBackRight() {
		LimelightConfigBackRight config = new LimelightConfigBackRight();
		config.name = "limelight-right";
		config.robotToCameraOffset = kRobotToCameraOffsetRight;
		return config;
	}

	public static final LimelightConfigBackLeft getVisionIOConfigBackLeft() {
		LimelightConfigBackLeft config = new LimelightConfigBackLeft();
		config.name = "limelight-left";
		config.robotToCameraOffset = kRobotToCameraOffsetLeft;
		return config;
	}

	public static final LimelightConfigBackSide getVisionIOConfigSide() {
		LimelightConfigBackSide config = new LimelightConfigBackSide();
		config.name = "limelight-side";
		config.robotToCameraOffset = kRobotToCameraOffsetSide;
		return config;
	}

	public static final VisionIOLimelight getVisionIO(VisionIOConfig config) {
		return new VisionIOLimelight(config);
	}

	public static final int agreedHeadingUpdatesThreshold = 100;
	public static final Angle agreedHeadingUpdateEpsilon = Units.Degrees.of(2.0);
	public static final int agreedTranslationUpdatesThreshold = 100;
	public static final Distance agreedTranslationUpdateEpsilon = Units.Centimeters.of(10.0);
}
