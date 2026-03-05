package frc.robot.subsystems.vision.photon;

import edu.wpi.first.math.MathUtil;
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

    // Hub AprilTag IDs (both alliance sides)
    private static final Set<Integer> HUB_TAG_IDS = Set.of(
        26, 20, 18, 21,  // Blue-side hub faces
        4, 10, 8, 11     // Red-side hub faces (alliance-flipped equivalents)
    );

    private static final LoggedTunableNumber maxCorrectionDeg =
        new LoggedTunableNumber("TurretVision/MaxCorrectionDeg", 15.0);
    private static final LoggedTunableNumber ambiguityThreshold =
        new LoggedTunableNumber("TurretVision/AmbiguityThreshold", 0.2);
    // Max degrees the correction can change per tick (50Hz = 0.02s)
    private static final LoggedTunableNumber maxRateDegPerTick =
        new LoggedTunableNumber("TurretVision/MaxRateDegPerTick", 0.2);
    // EMA smoothing factor (0-1). Lower = smoother/slower. 0.1 = heavy smoothing.
    private static final LoggedTunableNumber emaAlpha =
        new LoggedTunableNumber("TurretVision/EmaAlpha", 0.08);
    // Deadband: ignore raw yaw below this (degrees). Prevents hunting near center.
    private static final LoggedTunableNumber deadbandDeg =
        new LoggedTunableNumber("TurretVision/DeadbandDeg", 1.0);

    private double emaYaw = 0.0;
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

        // Deadband: treat small errors as zero to prevent hunting
        double deadbanded = Math.abs(rawYaw) < deadbandDeg.get() ? 0.0 : rawYaw;

        double clamped = MathUtil.clamp(deadbanded, -maxCorrectionDeg.get(), maxCorrectionDeg.get());

        // Exponential moving average — much smoother than a windowed average
        double alpha = MathUtil.clamp(emaAlpha.get(), 0.01, 1.0);
        emaYaw = alpha * clamped + (1.0 - alpha) * emaYaw;

        // Rate-limit: clamp how fast the output can change per tick
        double maxDelta = maxRateDegPerTick.get();
        filteredCorrectionDeg += MathUtil.clamp(
            emaYaw - filteredCorrectionDeg, -maxDelta, maxDelta);

        logValues();
    }

    private void clearTarget() {
        hasTarget = false;
        targetCount = 0;
        rawYaw = 0.0;
        // Gently decay EMA toward 0 when no target visible
        double alpha = MathUtil.clamp(emaAlpha.get(), 0.01, 1.0);
        emaYaw = (1.0 - alpha) * emaYaw;
    }

    private void logValues() {
        SmartDashboard.putBoolean("TurretVision/CameraConnected", cameraConnected);
        SmartDashboard.putBoolean("TurretVision/HasTarget", hasTarget);
        SmartDashboard.putNumber("TurretVision/TargetCount", targetCount);
        SmartDashboard.putNumber("TurretVision/RawYaw", rawYaw);
        SmartDashboard.putNumber("TurretVision/EmaYaw", emaYaw);
        SmartDashboard.putNumber("TurretVision/CorrectionDeg", filteredCorrectionDeg);

        Logger.recordOutput("TurretVision/CameraConnected", cameraConnected);
        Logger.recordOutput("TurretVision/HasTarget", hasTarget);
        Logger.recordOutput("TurretVision/TargetCount", targetCount);
        Logger.recordOutput("TurretVision/RawYaw", rawYaw);
        Logger.recordOutput("TurretVision/EmaYaw", emaYaw);
        Logger.recordOutput("TurretVision/CorrectionDeg", filteredCorrectionDeg);
    }

    /** Resets the correction to zero and clears the filter history. */
    public void resetCorrection() {
        filteredCorrectionDeg = 0.0;
        emaYaw = 0.0;
        rawYaw = 0.0;
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
