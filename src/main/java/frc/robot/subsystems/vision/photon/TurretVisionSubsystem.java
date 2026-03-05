package frc.robot.subsystems.vision.photon;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Set;

public class TurretVisionSubsystem extends SubsystemBase {

    public static final TurretVisionSubsystem mInstance = new TurretVisionSubsystem();

    private final PhotonCamera camera;
    private final LinearFilter yawFilter = LinearFilter.movingAverage(5);

    // Hub AprilTag IDs (both alliance sides)
    private static final Set<Integer> HUB_TAG_IDS = Set.of(
        26, 20, 18, 21,  // Blue-side hub faces
        4, 10, 8, 11     // Red-side hub faces (alliance-flipped equivalents)
    );

    private static final LoggedTunableNumber maxCorrectionDeg =
        new LoggedTunableNumber("TurretVision/MaxCorrectionDeg", 15.0);
    private static final LoggedTunableNumber ambiguityThreshold =
        new LoggedTunableNumber("TurretVision/AmbiguityThreshold", 0.2);

    private double filteredCorrectionDeg = 0.0;
    private double rawYaw = 0.0;
    private int targetCount = 0;
    private boolean hasTarget = false;
    private boolean cameraConnected = false;

    private TurretVisionSubsystem() {
        camera = new PhotonCamera("photon-turret");
    }

    @Override
    public void periodic() {
        cameraConnected = camera.isConnected();

        if (!cameraConnected) {
            clearTarget();
            logValues();
            return;
        }

        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            clearTarget();
            logValues();
            return;
        }

        // Average the yaw of all visible hub tags that pass ambiguity.
        // When we see two tags (e.g. 26+21 or 26+18), the average yaw
        // points toward the hub center rather than one face, preventing oscillation.
        double yawSum = 0.0;
        int count = 0;

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (!HUB_TAG_IDS.contains(target.getFiducialId())) {
                continue;
            }
            if (target.getPoseAmbiguity() >= ambiguityThreshold.get()) {
                continue;
            }
            yawSum += target.getYaw();
            count++;
        }

        if (count == 0) {
            clearTarget();
            logValues();
            return;
        }

        hasTarget = true;
        targetCount = count;
        rawYaw = yawSum / count;

        double clamped = MathUtil.clamp(rawYaw, -maxCorrectionDeg.get(), maxCorrectionDeg.get());
        filteredCorrectionDeg = yawFilter.calculate(clamped);

        logValues();
    }

    private void clearTarget() {
        hasTarget = false;
        targetCount = 0;
        rawYaw = 0.0;
        filteredCorrectionDeg = yawFilter.calculate(0.0);
    }

    private void logValues() {
        SmartDashboard.putBoolean("TurretVision/CameraConnected", cameraConnected);
        SmartDashboard.putBoolean("TurretVision/HasTarget", hasTarget);
        SmartDashboard.putNumber("TurretVision/TargetCount", targetCount);
        SmartDashboard.putNumber("TurretVision/RawYaw", rawYaw);
        SmartDashboard.putNumber("TurretVision/CorrectionDeg", filteredCorrectionDeg);

        Logger.recordOutput("TurretVision/CameraConnected", cameraConnected);
        Logger.recordOutput("TurretVision/HasTarget", hasTarget);
        Logger.recordOutput("TurretVision/TargetCount", targetCount);
        Logger.recordOutput("TurretVision/RawYaw", rawYaw);
        Logger.recordOutput("TurretVision/CorrectionDeg", filteredCorrectionDeg);
    }

    /** Returns the filtered aim correction in degrees. 0 if no valid target. */
    public double getAimCorrectionDeg() {
        return filteredCorrectionDeg;
    }

    /** Returns true if a valid hub AprilTag is currently visible. */
    public boolean hasTarget() {
        return hasTarget;
    }
}
