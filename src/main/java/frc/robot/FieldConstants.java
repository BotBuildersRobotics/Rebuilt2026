package frc.robot;

// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.



import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;


/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = 100.0;// AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = 50.00; //AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

 // public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
 // public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  public static final Translation2d hubCenter = new Translation2d(4.6256194, 4.0346376);

 
   public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
       
    public static final double startingLineX = 4.040481;

  
  public enum AprilTagLayoutType {

    
    
   // OFFICIAL("2026-official"),
    //NONE("2026-none");

    /*AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Constants.disableHAL
                    ? Path.of("src", "main", "deploy", "apriltags", name + ".json")
                    : Path.of(
                        Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException(
            "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
      }*/

    //}

   // private final AprilTagFieldLayout layout;
    //private final String layoutString;
  }
}