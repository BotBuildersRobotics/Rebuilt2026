package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.ShootState;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.fasterxml.jackson.core.format.MatchStrength;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.AngleUnit;
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

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Turret/MaxVelocity");

      
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Turret/MaxAcceleration");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA");

  static {
        maxVelocity.initDefault(12.0);
        maxAcceleration.initDefault(24.0);
        kP.initDefault(2000.0);
        kD.initDefault(50.0);
        kA.initDefault(0.0);
  }


  private Rotation2d goalAngle = Rotation2d.kZero;
  private double goalVelocity = 0.0;
  private double lastGoalAngle = 0.0;


  TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
  private double turretOffset;
  private boolean turretZeroed = true;

  private ShootState shootState = ShootState.ACTIVE_SHOOTING;

  private Mechanism2d turretMech;
  private MechanismLigament2d turretLigament;
  private Field2d fieldViz;
  private Translation2d currentTarget = null;

  private State setpoint = new State();

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

   public void periodic() {
        super.periodic(); // Critical: Updates motor inputs from simulation or hardware

        /*SmartDashboard.putBoolean("Turret/ControlLoopActive", DriverStation.isEnabled() && turretZeroed);
        SmartDashboard.putNumber("Turret/CurrentPositionDeg", Units.radiansToDegrees(getTurretAngle()));
        SmartDashboard.putNumber("Turret/MaxVelocity", maxVelocity.get());
        SmartDashboard.putNumber("Turret/MaxAcceleration", maxAcceleration.get());*/

        // Update profile constraints
        if (maxVelocity.hasChanged(hashCode()) || maxAcceleration.hasChanged(hashCode())) {
            profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
        }



        if(DriverStation.isEnabled() && turretZeroed){
          Rotation2d robotAngle = DriveSubsystem.mInstance.getState().Pose.getRotation();
          double robotAngularVelocity =
          DriveSubsystem.mInstance.getState().Speeds.omegaRadiansPerSecond;

          Rotation2d robotRelativeGoalAngle = goalAngle.minus(robotAngle);
          double robotRelativeGoalVelocity = goalVelocity - robotAngularVelocity;

          // Debug telemetry
          SmartDashboard.putNumber("Turret/RobotAngleDeg", robotAngle.getDegrees());
          SmartDashboard.putNumber("Turret/GoalAngleDeg", goalAngle.getDegrees());
          SmartDashboard.putNumber("Turret/RobotAngularVelRadPerSec", robotAngularVelocity);

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

      State goalState = new State(MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle), robotRelativeGoalVelocity);

      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);

      SmartDashboard.putNumber("Turret/BestAngleRad", bestAngle);
      SmartDashboard.putNumber("Turret/SetpointPositionRad", setpoint.position);
      SmartDashboard.putNumber("Turret/SetpointVelocityRadPerSec", setpoint.velocity);
     
      SmartDashboard.putNumber("Turret/Offset",turretOffset);

      this.applySetpoint(Setpoint.withMotionMagicSetpoint(Radians.of(setpoint.position)));
      //this.applySetpoint(Setpoint.withPositionVelocitySetpoint(Radians.of(setpoint.position), RadiansPerSecond.of(setpoint.velocity)));
      
      
      SmartDashboard.putNumber("Turret/GoalPositionRad", bestAngle);
      SmartDashboard.putNumber("Turret/SetpointPositionRad", setpoint.position);
     
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

   }

   private void setFieldRelativeTarget(Rotation2d angle, double velocity) {
    this.goalAngle = angle;
    this.goalVelocity = velocity;
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
    return  turretOffset + getPosition().in(Radians); 
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
          setFieldRelativeTarget(params.turretAngle().plus(Rotation2d.fromDegrees(turretOffset)), params.turretVelocity());
          setShootState(ShootState.TRACKING);
        });
  }

  public Command runTrackTargetActiveShootingCommand() {
    return run(
        () -> {
          var params = shotCalc.getParameters();
          setFieldRelativeTarget(params.turretAngle().plus(Rotation2d.fromDegrees(turretOffset)), params.turretVelocity());
          setShootState(ShootState.ACTIVE_SHOOTING);
        });
  }

  public Command runFixedCommand(Supplier<Rotation2d> angle, DoubleSupplier velocity) {
    return run(
        () -> {
          setFieldRelativeTarget(angle.get().plus(Rotation2d.fromDegrees(turretOffset)), velocity.getAsDouble());
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

      // Get robot's current position on the field
      Pose2d robotPose = DriveSubsystem.mInstance.getState().Pose;
      Translation2d robotPosition = robotPose.getTranslation();

      // Calculate vector from robot to target
      Translation2d toTarget = targetPosition.minus(robotPosition);

      // Calculate angle to target (in field coordinates)
      Rotation2d angleToTarget = new Rotation2d(toTarget.getX(), toTarget.getY());

      // Calculate angular velocity if target or robot is moving (0 for stationary target)
      double angularVelocity = 0.0;

      // Set the turret goal
      setFieldRelativeTarget(angleToTarget, angularVelocity);
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



}
