package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.lib.AllianceFlipUtil;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.photon.TurretVisionSubsystem;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSubsystem extends MotorSubsystem<MotorIO> {
    
  private static final double minAngle = Units.degreesToRadians(-210.0);
  private static final double maxAngle = Units.degreesToRadians(210.0);
  private static final double trackOverlapMargin = Units.degreesToRadians(10);
  private static final double trackCenterRads = (minAngle + maxAngle) / 2;
  private static final double trackMinAngle = trackCenterRads - Math.PI - trackOverlapMargin;
  private static final double trackMaxAngle = trackCenterRads + Math.PI + trackOverlapMargin;

  private Rotation2d goalAngle = Rotation2d.kZero;
  private double goalVelocityRadPerSec = 0.0;
  private double lastGoalAngle = 0.0;

  private double turretOffset;
  private boolean turretZeroed = true;

  private TurretVisionSubsystem turretVision;
  private static final LoggedTunableNumber visionCorrectionEnabled =
      new LoggedTunableNumber("Turret/VisionCorrectionEnabled", 0.0);
  // 1.0 = use position+velocity feedforward, 0.0 = use Motion Magic
  private static final LoggedTunableNumber useVelocityFeedforward =
      new LoggedTunableNumber("Turret/UseVelocityFeedforward", 0.0);

  private ShootState shootState = ShootState.ACTIVE_SHOOTING;
  private boolean stowed = false;

  private Mechanism2d turretMech;
  private MechanismLigament2d turretLigament;
  private Field2d fieldViz;
  private Translation2d currentTarget = null;

  public enum ShootState {
    ACTIVE_SHOOTING,
    TRACKING
  }

  //public static final TurretSubsystem mInstance = new TurretSubsystem();

  private ShotCalculator shotCalc;
  
	public TurretSubsystem() {
		super(TurretConstants.getMotorIO(), "Turret Motor");
    initMechanism();


    // Alternative: Point at the hub/speaker
    //setDefaultCommand(pointAtFieldPosition(frc.robot.FieldConstants.hubCenter));
	}

  private void initMechanism(){
    turretMech = new Mechanism2d(3, 2);
    MechanismRoot2d turretRoot = turretMech.getRoot("root", 1.5, 0.1);
    turretLigament = turretRoot.append(new MechanismLigament2d("Turret", 1, 0));

    // Initialize field visualization for AdvantageScope
    fieldViz = new Field2d();
    SmartDashboard.putData("Turret Field Viz", fieldViz);
  }

  public void setShotCalculator(ShotCalculator shotCalc){
    this.shotCalc = shotCalc;
  }

  public void setTurretVision(TurretVisionSubsystem turretVision) {
    this.turretVision = turretVision;
  }

  private double getVisionCorrectionDeg() {
    if (turretVision == null || visionCorrectionEnabled.get() < 0.5) {
      return 0.0;
    }
    if (!turretVision.hasTarget()) {
      return 0.0;
    }
    return turretVision.getAimCorrectionDeg();
  }

   public void periodic() {
        super.periodic(); // Critical: Updates motor inputs from simulation or hardware

        /*SmartDashboard.putBoolean("Turret/ControlLoopActive", DriverStation.isEnabled() && turretZeroed);
        SmartDashboard.putNumber("Turret/CurrentPositionDeg", Units.radiansToDegrees(getTurretAngle()));
        SmartDashboard.putNumber("Turret/MaxVelocity", maxVelocity.get());
        SmartDashboard.putNumber("Turret/MaxAcceleration", maxAcceleration.get());*/

        if(DriverStation.isEnabled() && turretZeroed){
          Rotation2d robotAngle = DriveSubsystem.mInstance.getState().Pose.getRotation();

          Rotation2d robotRelativeGoalAngle = goalAngle.minus(robotAngle);

          // Debug telemetry
          SmartDashboard.putNumber("Turret/RobotAngleDeg", robotAngle.getDegrees());
          SmartDashboard.putNumber("Turret/GoalAngleDeg", goalAngle.getDegrees());

          // Find best wrapped angle within mechanical limits
          boolean hasBestAngle = false;
          double bestAngle = 0;

        double minLegalAngle =
          switch (shootState) {
            case ACTIVE_SHOOTING -> minAngle;
            case TRACKING -> trackMinAngle;
          };
      double maxLegalAngle =
          switch (shootState) {
            case ACTIVE_SHOOTING -> maxAngle;
            case TRACKING -> trackMaxAngle;
          };
      for (int i = -2; i < 3; i++) {
        double potentialSetpoint = robotRelativeGoalAngle.getRadians() + Math.PI * 2.0 * i;
        if (potentialSetpoint < minLegalAngle || potentialSetpoint > maxLegalAngle) {
          continue;
        } else {
          if (!hasBestAngle) {
            bestAngle = potentialSetpoint;
            hasBestAngle = true;
          }
          if (Math.abs(lastGoalAngle - potentialSetpoint) < Math.abs(lastGoalAngle - bestAngle)) {
            bestAngle = potentialSetpoint;
          }
        }
      }
      lastGoalAngle = bestAngle;

      double clampedAngle = MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle);

      // Choose control mode: position+velocity feedforward for shoot-on-the-move,
      // or Motion Magic for smooth profiled movement
      if (useVelocityFeedforward.get() >= 0.5 && goalVelocityRadPerSec != 0.0) {
        this.applySetpoint(Setpoint.withPositionVelocitySetpoint(
            Radians.of(clampedAngle),
            RadiansPerSecond.of(goalVelocityRadPerSec)));
      } else {
        this.applySetpoint(Setpoint.withMotionMagicSetpoint(Radians.of(clampedAngle)));
      }

      SmartDashboard.putNumber("Turret/BestAngleRad", bestAngle);
      SmartDashboard.putNumber("Turret/SetpointPositionRad", clampedAngle);
      SmartDashboard.putNumber("Turret/Offset", turretOffset);

    }

    // Always update the turret mechanism visual (even when disabled)
    double robotRelativeAngleDeg = Units.radiansToDegrees(getTurretAngle());
    turretLigament.setAngle(robotRelativeAngleDeg);
    SmartDashboard.putData("Turret Mech", turretMech);

    // Calculate and display field-relative angle
    Pose2d robotPose = DriveSubsystem.mInstance.getState().Pose;
    Rotation2d robotAngle = robotPose.getRotation();
    double fieldRelativeAngleDeg = robotAngle.getDegrees() + robotRelativeAngleDeg;
    SmartDashboard.putNumber("Turret/RobotRelativeAngleDeg", robotRelativeAngleDeg);
    SmartDashboard.putNumber("Turret/FieldRelativeAngleDeg", fieldRelativeAngleDeg);
    SmartDashboard.putBoolean("Turret/Stowed", stowed);

    // Update field visualization for AdvantageScope
    // Create a pose at the robot's position with the turret's field-relative angle
    Rotation2d turretFieldAngle = Rotation2d.fromDegrees(fieldRelativeAngleDeg);
    Pose2d turretAimPose = new Pose2d(robotPose.getTranslation(), turretFieldAngle);
    fieldViz.setRobotPose(turretAimPose);

    // Show the target position if using pointAtFieldPosition
    if (currentTarget != null) {
      fieldViz.getObject("Target").setPose(new Pose2d(currentTarget, new Rotation2d()));
    }

    SmartDashboard.putString("State", shootState.toString());

    // AdvantageKit structured logging for replay
    Logger.recordOutput("Turret/RobotRelativeAngleDeg", robotRelativeAngleDeg);
    Logger.recordOutput("Turret/FieldRelativeAngleDeg", fieldRelativeAngleDeg);
    Logger.recordOutput("Turret/AimPose", turretAimPose);
    Logger.recordOutput("Turret/ShootState", shootState.toString());
    Logger.recordOutput("Turret/Stowed", stowed);
    Logger.recordOutput("Turret/GoalAngleDeg", goalAngle.getDegrees());
    Logger.recordOutput("Turret/Offset", turretOffset);

   }

   private void setFieldRelativeTarget(Rotation2d angle) {
    this.goalAngle = angle;
    this.goalVelocityRadPerSec = 0.0;
  }

  private void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
    this.goalAngle = angle;
    this.goalVelocityRadPerSec = velocityRadPerSec;
  }

  private void zero() {
    turretZeroed = true;
    turretOffset = 0.0; 
    setCurrentPosition(Radians.of(0.0));
  }

  public void offsetLeft(){
    turretOffset += 5.0;
  }

  public void offsetRight(){
    turretOffset -= 5.0;
  }

  public double getTurretAngle() {
    return  Units.degreesToRadians(turretOffset) + getPosition().in(Radians);
  }

  public double getTurretVelocity() {
    return getVelocity().in(RadiansPerSecond); 
  }

  public void setShootState(ShootState state){
    this.shootState = state;
  }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = shotCalc.getParameters();
          double totalOffset = turretOffset - getVisionCorrectionDeg();
          setFieldRelativeTarget(
              params.turretAngle().plus(Rotation2d.fromDegrees(totalOffset)),
              params.turretVelocity());
          setShootState(ShootState.TRACKING);
        });
  }

  public Command runTrackTargetActiveShootingCommand() {
    return run(
        () -> {
          if (stowed) {
            // Hold 0° robot-relative (straight ahead)
            Rotation2d robotAngle = DriveSubsystem.mInstance.getState().Pose.getRotation();
            setFieldRelativeTarget(robotAngle);
            setShootState(ShootState.ACTIVE_SHOOTING);
          } else {
            var params = shotCalc.getParameters();
            double totalOffset = turretOffset - getVisionCorrectionDeg();
            setFieldRelativeTarget(
                params.turretAngle().plus(Rotation2d.fromDegrees(totalOffset)),
                params.turretVelocity());
            setShootState(ShootState.ACTIVE_SHOOTING);
          }
        });
  }

  public void setStowed(boolean stowed) {
    this.stowed = stowed;
    if (stowed) {
      turretOffset = 0.0;
      if (turretVision != null) {
        turretVision.resetCorrection();
      }
    }
  }

  public boolean isStowed() {
    return stowed;
  }

  public Command runFixedCommand(Supplier<Rotation2d> angle) {
    return run(
        () -> {
          setFieldRelativeTarget(angle.get().plus(Rotation2d.fromDegrees(turretOffset)));
          setShootState(ShootState.TRACKING);
        });
  }

  /**
   * Command to point the turret at a specific field position.
   *
   * @param targetPosition Field position to aim at (in meters)
   * @return Command that continuously tracks the target position
   */
  public Command pointAtFieldPosition(Translation2d targetPosition) {
    return run(() -> {
      // Store target for visualization
      currentTarget = targetPosition;

      // Get turret's actual field position (not robot center)
      Pose2d robotPose = DriveSubsystem.mInstance.getState().Pose;
      Pose2d turretPose = robotPose.transformBy(
          ShotCalculator.getInstance().toTransform2d(ShotCalculator.robotToTurret));
      Translation2d turretPosition = turretPose.getTranslation();

      // Calculate vector from turret to target
      Translation2d toTarget = targetPosition.minus(turretPosition);

      // Calculate angle to target (in field coordinates)
      Rotation2d angleToTarget = toTarget.getAngle();

      // Set the turret goal
      setFieldRelativeTarget(angleToTarget);
      setShootState(ShootState.ACTIVE_SHOOTING);

      // Debug telemetry
      SmartDashboard.putNumber("Turret/TargetX", targetPosition.getX());
      SmartDashboard.putNumber("Turret/TargetY", targetPosition.getY());
      SmartDashboard.putNumber("Turret/AngleToTargetDeg", angleToTarget.getDegrees());
      SmartDashboard.putNumber("Turret/DistanceToTarget", toTarget.getNorm());
      SmartDashboard.putNumber("Turret/Offset",turretOffset);
    });
  }

  public Command zeroCommand() {
    return runOnce(this::zero).ignoringDisable(true);
  }

  /**
   * Holds the turret at 0° robot-relative (straight ahead) for climbing.
   * Overrides the tracking default command until interrupted.
   */
  public Command stowCommand() {
    return run(() -> {
      Rotation2d robotAngle = DriveSubsystem.mInstance.getState().Pose.getRotation();
      // Set field-relative goal to match robot heading = 0° robot-relative
      setFieldRelativeTarget(robotAngle);
      setShootState(ShootState.ACTIVE_SHOOTING);
    });
  }

  /**
   * Aim to pass fuel over the left bump into the alliance zone.
   * Alliance-flipped so it works for both blue and red.
   */
  public Command passLeftCommand() {
    return pointAtFieldPosition(
        AllianceFlipUtil.apply(FieldConstants.PassingTargets.leftPassTarget));
  }

  /**
   * Aim to pass fuel over the right bump into the alliance zone.
   * Alliance-flipped so it works for both blue and red.
   */
  public Command passRightCommand() {
    return pointAtFieldPosition(
        AllianceFlipUtil.apply(FieldConstants.PassingTargets.rightPassTarget));
  }

  /**
   * Auto-selects left or right passing target based on which side of the
   * field the robot is currently on (relative to field center Y).
   */
  /**
   * Test commands for turret calibration.
   * Logs all intermediate values so you can verify each stage of the aiming pipeline.
   *
   * testAimStraightAhead: Points turret at 0° robot-relative.
   *   If turret doesn't face forward, the zero/gear ratio is wrong.
   *
   * testAimRobotRelative: Points turret at a tunable robot-relative angle.
   *   Use to verify gear ratio: set to 90° and measure if turret is actually at 90°.
   *
   * testAimAtHub: Points at the hub using field pose, bypassing ShotCalculator.
   *   Logs the expected field-relative angle so you can compare with what you see.
   */
  private static final LoggedTunableNumber testAngleDeg =
      new LoggedTunableNumber("Turret/TestAngleDeg", 90.0);

  public Command testAimStraightAhead() {
    return run(() -> {
      // Command the motor directly to 0 radians — no field math involved
      this.applySetpoint(Setpoint.withMotionMagicSetpoint(Radians.of(0.0)));

      SmartDashboard.putNumber("Turret/Test/CommandedRad", 0.0);
      SmartDashboard.putNumber("Turret/Test/ActualRad", getPosition().in(Radians));
      SmartDashboard.putNumber("Turret/Test/ErrorDeg",
          Units.radiansToDegrees(getPosition().in(Radians)));
    });
  }

  public Command testAimRobotRelative() {
    return run(() -> {
      double targetRad = Units.degreesToRadians(testAngleDeg.get());
      this.applySetpoint(Setpoint.withMotionMagicSetpoint(Radians.of(targetRad)));

      SmartDashboard.putNumber("Turret/Test/CommandedDeg", testAngleDeg.get());
      SmartDashboard.putNumber("Turret/Test/CommandedRad", targetRad);
      SmartDashboard.putNumber("Turret/Test/ActualRad", getPosition().in(Radians));
      SmartDashboard.putNumber("Turret/Test/ActualDeg",
          Units.radiansToDegrees(getPosition().in(Radians)));
      SmartDashboard.putNumber("Turret/Test/ErrorDeg",
          testAngleDeg.get() - Units.radiansToDegrees(getPosition().in(Radians)));
    });
  }

  public Command testAimAtHub() {
    return run(() -> {
      Pose2d robotPose = DriveSubsystem.mInstance.getState().Pose;
      Rotation2d robotAngle = robotPose.getRotation();

      // Compute turret field position
      Pose2d turretPose = robotPose.transformBy(
          ShotCalculator.getInstance().toTransform2d(ShotCalculator.robotToTurret));

      // Angle from turret to hub (field-relative)
      Translation2d hubTarget =
          AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
      Translation2d toHub = hubTarget.minus(turretPose.getTranslation());
      Rotation2d fieldAngleToHub = toHub.getAngle();

      // Convert to robot-relative
      double robotRelativeRad = fieldAngleToHub.minus(robotAngle).getRadians();

      // Send directly to motor (no trapezoid profile, no offset)
      this.applySetpoint(Setpoint.withMotionMagicSetpoint(Radians.of(robotRelativeRad)));

      // Log everything for debugging
      SmartDashboard.putNumber("Turret/Test/RobotX", robotPose.getX());
      SmartDashboard.putNumber("Turret/Test/RobotY", robotPose.getY());
      SmartDashboard.putNumber("Turret/Test/RobotHeadingDeg", robotAngle.getDegrees());
      SmartDashboard.putNumber("Turret/Test/TurretFieldX", turretPose.getX());
      SmartDashboard.putNumber("Turret/Test/TurretFieldY", turretPose.getY());
      SmartDashboard.putNumber("Turret/Test/HubX", hubTarget.getX());
      SmartDashboard.putNumber("Turret/Test/HubY", hubTarget.getY());
      SmartDashboard.putNumber("Turret/Test/FieldAngleToHubDeg", fieldAngleToHub.getDegrees());
      SmartDashboard.putNumber("Turret/Test/RobotRelativeToHubDeg",
          Units.radiansToDegrees(robotRelativeRad));
      SmartDashboard.putNumber("Turret/Test/ActualPositionDeg",
          Units.radiansToDegrees(getPosition().in(Radians)));
      SmartDashboard.putNumber("Turret/Test/DistToHub", toHub.getNorm());
    });
  }

  public Command passAutoCommand() {
    return run(() -> {
      Pose2d robotPose = DriveSubsystem.mInstance.getState().Pose;
      double robotY = robotPose.getY();
      double centerY = FieldConstants.fieldWidth / 2.0;

      // Pick the pass target on the same side of the field as the robot
      Translation2d target;
      if (AllianceFlipUtil.shouldFlip()) {
        // Red alliance: Y is flipped
        target = (robotY < centerY)
            ? AllianceFlipUtil.apply(FieldConstants.PassingTargets.leftPassTarget)
            : AllianceFlipUtil.apply(FieldConstants.PassingTargets.rightPassTarget);
      } else {
        // Blue alliance
        target = (robotY > centerY)
            ? FieldConstants.PassingTargets.leftPassTarget
            : FieldConstants.PassingTargets.rightPassTarget;
      }

      currentTarget = target;

      Pose2d turretPose = robotPose.transformBy(
          ShotCalculator.getInstance().toTransform2d(ShotCalculator.robotToTurret));
      Translation2d toTarget = target.minus(turretPose.getTranslation());

      setFieldRelativeTarget(toTarget.getAngle());
      setShootState(ShootState.ACTIVE_SHOOTING);

      SmartDashboard.putNumber("Turret/PassTargetX", target.getX());
      SmartDashboard.putNumber("Turret/PassTargetY", target.getY());
      SmartDashboard.putNumber("Turret/PassDistance", toTarget.getNorm());
    });
  }



}
