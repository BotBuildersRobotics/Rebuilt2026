package frc.robot.subsystems.vision.photon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class FuelDetectionSubsystem extends SubsystemBase {

    public static final FuelDetectionSubsystem mInstance = new FuelDetectionSubsystem();

    private final PhotonCamera camera;

    // The pipeline index configured in PhotonVision for yellow fuel detection
    private static final int FUEL_DETECTION_PIPELINE = 0;

    // Tuning: how much of the camera frame a single fuel fills (fraction 0.0-1.0)
    // Calibrate this by placing one fuel in front of the camera and reading the area
    private static final double SINGLE_FUEL_AREA = 0.05;

    // Maximum number of fuel the robot can hold
    private static final int MAX_FUEL_CAPACITY = 5;

    private double fuelPercentage = 0.0;
    private int detectedFuelCount = 0;
    private boolean cameraConnected = false;

    private FuelDetectionSubsystem() {
        camera = new PhotonCamera("photon-fuel");
        camera.setPipelineIndex(FUEL_DETECTION_PIPELINE);
    }

    @Override
    public void periodic() {
        cameraConnected = camera.isConnected();

        if (!cameraConnected) {
            SmartDashboard.putBoolean("Fuel/CameraConnected", false);
            return;
        }

        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            detectedFuelCount = targets.size();

            // Sum up the area of all detected yellow objects
            double totalArea = 0.0;
            for (PhotonTrackedTarget target : targets) {
                totalArea += target.getArea();
            }

            // Convert total area to a percentage of full capacity
            // area is percentage of image (0-100 from PhotonVision)
            double maxExpectedArea = SINGLE_FUEL_AREA * MAX_FUEL_CAPACITY * 100.0;
            fuelPercentage = Math.min(100.0, (totalArea / maxExpectedArea) * 100.0);
        } else {
            detectedFuelCount = 0;
            fuelPercentage = 0.0;
        }

        SmartDashboard.putBoolean("Fuel/CameraConnected", cameraConnected);
        SmartDashboard.putNumber("Fuel/DetectedCount", detectedFuelCount);
        SmartDashboard.putNumber("Fuel/Percentage", fuelPercentage);

        Logger.recordOutput("Fuel/CameraConnected", cameraConnected);
        Logger.recordOutput("Fuel/DetectedCount", detectedFuelCount);
        Logger.recordOutput("Fuel/Percentage", fuelPercentage);
    }

    /**
     * Returns how full the robot is with fuel, from 0.0 to 100.0.
     */
    public double getFuelPercentage() {
        return fuelPercentage;
    }

    /**
     * Returns the number of individual fuel blobs detected.
     */
    public int getDetectedFuelCount() {
        return detectedFuelCount;
    }

    public boolean isCameraConnected() {
        return cameraConnected;
    }

    /**
     * Returns true if the robot is considered "full" (above the threshold).
     */
    public boolean isFull() {
        return fuelPercentage >= 90.0;
    }

    /**
     * Returns true if the robot has no fuel detected.
     */
    public boolean isEmpty() {
        return fuelPercentage < 5.0;
    }
}
