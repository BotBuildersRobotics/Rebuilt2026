
// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.lib.AllianceFlipUtil;
import frc.robot.subsystems.drive.DriveSubsystem;


import org.littletonrobotics.junction.Logger;


public class ShotCalculator {
  private static ShotCalculator instance;
 private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  public static Transform3d robotToTurret = new Transform3d(0.0, 0.0, 0.44, Rotation3d.kZero);//new Transform3d(-0.16073, -0.1103, 0.44, Rotation3d.kZero);
  

  static {

    minDistance = 1.34;
    maxDistance = 5.70;
    phaseDelay = 0.03;

   launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(0.0));
    launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(0.0));
    launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(15.0));
    launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(20.0)); // in front of climb
    launchHoodAngleMap.put(3.0, Rotation2d.fromDegrees(25.0));
    launchHoodAngleMap.put(3.35, Rotation2d.fromDegrees(30)); //AUTO
    launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(32));
    launchHoodAngleMap.put(4.09, Rotation2d.fromDegrees(42.0));
    launchHoodAngleMap.put(4.09, Rotation2d.fromDegrees(45));
    launchHoodAngleMap.put(4.40, Rotation2d.fromDegrees(45)); // Near Depot
    launchHoodAngleMap.put(4.77, Rotation2d.fromDegrees(50.0));
    launchHoodAngleMap.put(5.57, Rotation2d.fromDegrees(40.5));
    launchHoodAngleMap.put(5.60, Rotation2d.fromDegrees(42.0));
    launchHoodAngleMap.put(5.70, Rotation2d.fromDegrees(45.0));

    launchFlywheelSpeedMap.put(1.68, 160.0); //
    launchFlywheelSpeedMap.put(2.13, 170.0); //
    launchFlywheelSpeedMap.put(2.81, 175.0); //in front of climb
    launchFlywheelSpeedMap.put(3.08, 178.0); //AUTO 
    launchFlywheelSpeedMap.put(3.35, 183.0);  
    launchFlywheelSpeedMap.put(3.42, 184.0); //
    launchFlywheelSpeedMap.put(3.6, 195.0); //
    launchFlywheelSpeedMap.put(3.82, 200.0);
    launchFlywheelSpeedMap.put(4.09, 204.0);
    launchFlywheelSpeedMap.put(4.20, 205.0);
     launchFlywheelSpeedMap.put(4.30, 206.0); // near depot
     launchFlywheelSpeedMap.put(4.40, 208.0); // near depot
     launchFlywheelSpeedMap.put(4.60, 212.0); // near depot
    launchFlywheelSpeedMap.put(4.77, 215.0);
    launchFlywheelSpeedMap.put(5.17, 218.0);
    launchFlywheelSpeedMap.put(5.57, 220.0);
    launchFlywheelSpeedMap.put(5.60, 223.0);
    launchFlywheelSpeedMap.put(5.70, 225.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  private Double lastContinuousTurretAngleDeg = null;

  public LaunchingParameters getAltParameters(){
     if (latestParameters != null) {
      return latestParameters;
    }

        Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        Pose2d robotPose = DriveSubsystem.mInstance.getDrivetrain().getState().Pose;
        Translation2d turretOffsetRobot = robotToTurret.getTranslation().toTranslation2d();
        Translation2d turretOffsetField = turretOffsetRobot.rotateBy(robotPose.getRotation());
        Translation2d turretPosField = robotPose.getTranslation().plus(turretOffsetField);
        Pose2d turretPoseField = new Pose2d(turretPosField, robotPose.getRotation());

        double turretToTargetDistance = target.getDistance(turretPoseField.getTranslation());

        // Field-relative velocity of the robot
        ChassisSpeeds v = DriveSubsystem.mInstance.getDrivetrain().getFieldVelocity();

        // Compute turret's *field-relative* translational velocity:
        // v_turret = v_robot + omega x r
        // r is robotToTurret rotated into field frame.
        double omega = v.omegaRadiansPerSecond;

        // robotToTurret in *robot frame* is (x,y). In field, r_field = R(theta)*r_robot
        double rfx = turretOffsetField.getX();
        double rfy = turretOffsetField.getY();

        // omega x r = (-omega*rfy, omega*rfx)
        double turretVelX = v.vxMetersPerSecond + (-omega * rfy);
        double turretVelY = v.vyMetersPerSecond + ( omega * rfx);

        // Lookahead based on time of flight (distance -> tof)
        double tof = timeOfFlightMap.get(turretToTargetDistance);
        Translation2d offset = new Translation2d(turretVelX * tof, turretVelY * tof);

        // Clamp the lookahead so it never crosses past the hub.
        // Decompose offset into radial (toward target) and lateral components.
        // The lateral component is preserved for shoot-on-the-fly correctness.
        // The radial component is clamped so the lookahead stays at least
        // kMinLookaheadDistance from the target.
        final double kMinLookaheadDistance = 0.5; // meters
        Translation2d toTarget = target.minus(turretPoseField.getTranslation());
        double rx = toTarget.getX() / turretToTargetDistance;
        double ry = toTarget.getY() / turretToTargetDistance;

        double radialOffset  =  rx * offset.getX() + ry * offset.getY();
        double lateralOffset = -ry * offset.getX() + rx * offset.getY();

        double clampedRadial = Math.min(radialOffset, turretToTargetDistance - kMinLookaheadDistance);

        Translation2d clampedOffset = new Translation2d(
            clampedRadial * rx - lateralOffset * ry,
            clampedRadial * ry + lateralOffset * rx
        );

        Pose2d lookaheadPose = new Pose2d(turretPoseField.getTranslation().plus(clampedOffset), turretPoseField.getRotation());

        double lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());

        // Aim from lookahead position to target (FIELD angle, wrapped)
        double turretFieldAngleWrappedDeg =
            target.minus(lookaheadPose.getTranslation()).getAngle().getDegrees();

        // Unwrap to continuous degrees
        if (lastContinuousTurretAngleDeg == null) {
            lastContinuousTurretAngleDeg = turretFieldAngleWrappedDeg;
        }

        double turretFieldAngleDeg = unwrapToNearestDeg(lastContinuousTurretAngleDeg, turretFieldAngleWrappedDeg);

        lastContinuousTurretAngleDeg = turretFieldAngleDeg;

        // Hood and flywheel from lookahead distance
        Rotation2d hoodAngleRot = launchHoodAngleMap.get(lookaheadDistance);
        double flywheelSpeed = launchFlywheelSpeedMap.get(lookaheadDistance);

    Logger.recordOutput("LaunchCalculator/Alt/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/Alt/TurretToTargetDistance", lookaheadDistance);

    latestParameters =
        new LaunchingParameters(
            lookaheadDistance >= minDistance
                && lookaheadDistance <= maxDistance,
            Rotation2d.fromDegrees(turretFieldAngleDeg),
            0.0,
            hoodAngleRot.getRadians(),
            0.0,
            flywheelSpeed);

    return latestParameters;
  }

  private static double unwrapToNearestDeg(double referenceDeg, double candidateWrappedDeg) {
        double delta = MathUtil.inputModulus(
            candidateWrappedDeg - referenceDeg,
            -180.0,
            180.0
        );
        return referenceDeg + delta;
    }


  public LaunchingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate distance from turret to target
     // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = DriveSubsystem.mInstance.getDrivetrain().getState().Pose;
    // getState().Speeds is already robot-relative; exp() expects robot-relative
    ChassisSpeeds robotRelativeVelocity = DriveSubsystem.mInstance.getGeneratedDrive().getState().Speeds;

    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

     // Calculate distance from turret to target
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d turretPosition = estimatedPose.transformBy(toTransform2d(robotToTurret));
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = DriveSubsystem.mInstance.getDrivetrain().getFieldVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    + robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.loopPeriodSecs);
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    SmartDashboard.putNumber("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);


    return latestParameters;
  }

   public Transform2d toTransform2d(Transform3d transform) {
    return new Transform2d(
        transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}