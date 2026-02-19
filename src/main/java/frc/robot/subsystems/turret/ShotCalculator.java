
// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.lib.AllianceFlipUtil;
import frc.robot.subsystems.drive.DriveSubsystem;

import org.littletonrobotics.junction.Logger;


public class ShotCalculator {
  private static ShotCalculator instance;
  private static Transform2d robotToTurret = new Transform2d(-0.25,0.25,Rotation2d.kZero);

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

  public record ShootingParameters(
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private ShootingParameters latestParameters = null;

  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Min/max distances for interpolation maps to prevent extrapolation.
  // Update these if you add data points outside this range.
  private static final double MIN_SHOOTING_DISTANCE_METERS = 1.8;
  private static final double MAX_SHOOTING_DISTANCE_METERS = 5.0;

  static {
    shotHoodAngleMap.put(1.8122, Rotation2d.fromDegrees(20.0));
    shotHoodAngleMap.put(2.612079, Rotation2d.fromDegrees(25.0));
    shotHoodAngleMap.put(3.75661, Rotation2d.fromDegrees(30.0));
    shotHoodAngleMap.put(4.96786, Rotation2d.fromDegrees(35.0));

    shotFlywheelSpeedMap.put(1.8122, 200.0);
    shotFlywheelSpeedMap.put(2.612079, 210.0);
    shotFlywheelSpeedMap.put(3.75661, 230.0);
    shotFlywheelSpeedMap.put(4.96786, 260.0);

    timeOfFlightMap.put(1.64227, 0.93);
    timeOfFlightMap.put(2.859544, 1.0);
    timeOfFlightMap.put(4.27071, 1.05);
  }

  public ShootingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate distance from turret to target
    Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
    Pose2d turretPosition = DriveSubsystem.mInstance.getDrivetrain().getState().Pose.transformBy(robotToTurret);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = DriveSubsystem.mInstance.getDrivetrain().getFieldVelocity();
    double robotAngle = DriveSubsystem.mInstance.getDrivetrain().getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double clampedDistanceForTOF = MathUtil.clamp(
        turretToTargetDistance,
        MIN_SHOOTING_DISTANCE_METERS,
        MAX_SHOOTING_DISTANCE_METERS);
    double timeOfFlight = timeOfFlightMap.get(clampedDistanceForTOF);
    double offsetX = turretVelocityX * timeOfFlight;
    double offsetY = turretVelocityY * timeOfFlight;
    Pose2d lookaheadPose =
        new Pose2d(
            turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
            turretPosition.getRotation());
    double lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());

    // Clamp distance to valid interpolation range to prevent extrapolation
    double clampedDistance = MathUtil.clamp(
        lookaheadTurretToTargetDistance,
        MIN_SHOOTING_DISTANCE_METERS,
        MAX_SHOOTING_DISTANCE_METERS);

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = shotHoodAngleMap.get(clampedDistance).getRadians();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        (turretAngle.getRadians() - lastTurretAngle.getRadians()) / Constants.loopPeriodSecs;
    hoodVelocity = (hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs;
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new ShootingParameters(
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            shotFlywheelSpeedMap.get(clampedDistance));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}