package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class TurretSubsystem extends SubsystemBase {
    
  private static final double minAngle = Units.degreesToRadians(-210.0);
  private static final double maxAngle = Units.degreesToRadians(210.0);
  private static final double trackOverlapMargin = Units.degreesToRadians(10);
  private static final double trackCenterRads = (minAngle + maxAngle) / 2;
  private static final double trackMinAngle = trackCenterRads - Math.PI - trackOverlapMargin;
  private static final double trackMaxAngle = trackCenterRads + Math.PI + trackOverlapMargin;

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Turret/MaxVelocity");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA");

  static {
        maxVelocity.initDefault(12.0);
        kP.initDefault(2000.0);
        kD.initDefault(50.0);
        kA.initDefault(0.0);
  }

 // private ShootState shootState = ShootState.ACTIVE_SHOOTING;
  private Rotation2d goalAngle = Rotation2d.kZero;
  private double goalVelocity = 0.0;
  private double lastGoalAngle = 0.0;


  SlewRateLimiter profile = new SlewRateLimiter(maxVelocity.get());
  private double turretOffset;

  private ShootState shootState = ShootState.ACTIVE_SHOOTING;

  public enum ShootState {
    ACTIVE_SHOOTING,
    TRACKING
  }

   public void periodic() {

        // Stop when disabled
        if (DriverStation.isDisabled()) {
        //outputs.mode = TurretIOOutputMode.BRAKE;

           // if (coastOverride.getAsBoolean()) {
                //outputs.mode = TurretIOOutputMode.COAST;
           // }
        }

        // Update profile constraints
        if (maxVelocity.hasChanged(hashCode())) {
            profile = new SlewRateLimiter(maxVelocity.get());
        }

        // Reset profile when disabled
        if (DriverStation.isDisabled()) {
       // profile.reset(getPosition());
        //lastGoalAngle = getPosition();
        }


        Rotation2d robotAngle = DriveSubsystem.mInstance.getState().Pose.getRotation();
        double robotAngularVelocity =
        DriveSubsystem.mInstance.getState().Speeds.omegaRadiansPerSecond;

        Rotation2d robotRelativeGoalAngle = goalAngle.minus(robotAngle);
        double robotRelativeGoalVelocity = goalVelocity - robotAngularVelocity;

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

      double setpoint = profile.calculate(MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle));
     /* Logger.recordOutput("Turret/GoalPositionRad", bestAngle);
      Logger.recordOutput("Turret/GoalVelocityRadPerSec", robotRelativeGoalVelocity);
      Logger.recordOutput("Turret/SetpointPositionRad", setpoint);
      Logger.recordOutput("Turret/SetpointVelocityRadPerSec", robotRelativeGoalVelocity);*/

     /* outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
      outputs.position = setpoint - turretOffset;
      outputs.velocity = robotRelativeGoalVelocity;
      outputs.kP = kP.get();
      outputs.kD = kD.get();*/



   }

   private void setFieldRelativeTarget(Rotation2d angle, double velocity) {
    this.goalAngle = angle;
    this.goalVelocity = velocity;
  }

  private void zero() {
    //turretOffset = -inputs.positionRads;
  }

  public double getPosition() {
    return 0.0; //return inputs.positionRads + turretOffset;
  }

  @AutoLogOutput(key = "Turret/MeasuredVelocityRadPerSec")
  public double getVelocity() {
    return 0.0; //return inputs.velocityRadsPerSec;
  }

  public void setShootState(ShootState state){
    this.shootState = state;
  }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters();
          setFieldRelativeTarget(params.turretAngle(), params.turretVelocity());
          setShootState(ShootState.TRACKING);
        });
  }

  public Command runTrackTargetActiveShootingCommand() {
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters();
          setFieldRelativeTarget(params.turretAngle(), params.turretVelocity());
          setShootState(ShootState.ACTIVE_SHOOTING);
        });
  }

  public Command runFixedCommand(Supplier<Rotation2d> angle, DoubleSupplier velocity) {
    return run(
        () -> {
          setFieldRelativeTarget(angle.get(), velocity.getAsDouble());
          setShootState(ShootState.TRACKING);
        });
  }

  public Command zeroCommand() {
    return runOnce(this::zero).ignoringDisable(true);
  }


}
