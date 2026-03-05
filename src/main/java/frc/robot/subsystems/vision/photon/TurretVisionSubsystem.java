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
    // How many degrees to nudge the correction per tick toward the measured error.
    // At 0.05 deg/tick and 50Hz, full correction of 5° takes ~2 seconds.
    private static final LoggedTunableNumber integralStepDeg =
        new LoggedTunableNumber("TurretVision/IntegralStepDeg", 0.05);
    // Deadband: ignore raw yaw below this (degrees). Prevents hunting near center.
    private static final LoggedTunableNumber deadbandDeg =
        new LoggedTunableNumber("TurretVision/DeadbandDeg", 1.5);

    private double correctionDeg = 0.0;
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

        // Integral accumulator: nudge correction toward the error each tick.
        // When the tag is centered (yaw inside deadband), stop nudging — hold value.
        // This avoids the feedback loop where tracking yaw directly causes
        // the turret to overshoot, lose the tag, and snap back.
        if (Math.abs(rawYaw) > deadbandDeg.get()) {
            double step = integralStepDeg.get();
            // Nudge in the direction of the error
            if (rawYaw > 0) {
                correctionDeg += step;
            } else {
                correctionDeg -= step;
            }
            // Clamp to max range
            correctionDeg = MathUtil.clamp(correctionDeg,
                -maxCorrectionDeg.get(), maxCorrectionDeg.get());
        }
        // If inside deadband: do nothing, hold current correction

        logValues();
    }

    private void clearTarget() {
        hasTarget = false;
        targetCount = 0;
        rawYaw = 0.0;
        // Hold correction — don't decay when tags disappear briefly.
        // Only resetCorrection() (called on stow) zeroes it out.
    }

    private void logValues() {
        SmartDashboard.putBoolean("TurretVision/CameraConnected", cameraConnected);
        SmartDashboard.putBoolean("TurretVision/HasTarget", hasTarget);
        SmartDashboard.putNumber("TurretVision/TargetCount", targetCount);
        SmartDashboard.putNumber("TurretVision/RawYaw", rawYaw);
        SmartDashboard.putNumber("TurretVision/CorrectionDeg", correctionDeg);

        Logger.recordOutput("TurretVision/CameraConnected", cameraConnected);
        Logger.recordOutput("TurretVision/HasTarget", hasTarget);
        Logger.recordOutput("TurretVision/TargetCount", targetCount);
        Logger.recordOutput("TurretVision/RawYaw", rawYaw);
        Logger.recordOutput("TurretVision/CorrectionDeg", correctionDeg);
    }

    /** Resets the correction to zero. Called on stow. */
    public void resetCorrection() {
        correctionDeg = 0.0;
        rawYaw = 0.0;
    }

    /** Returns the accumulated aim correction in degrees. */
    public double getAimCorrectionDeg() {
        return correctionDeg;
    }

    /** Returns true if a valid hub AprilTag is currently visible. */
    public boolean hasTarget() {
        return hasTarget;
    }
}
