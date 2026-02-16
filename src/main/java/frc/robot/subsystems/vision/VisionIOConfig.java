package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;

public abstract class VisionIOConfig {
   
		public  String name;
		public  Pose3d robotToCameraOffset;
		public  Vector<N3> aprilTagVisionStdDevs;
	
}
