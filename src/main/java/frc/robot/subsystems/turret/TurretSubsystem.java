package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.ShootState;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.fasterxml.jackson.core.format.MatchStrength;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSubsystem extends MotorSubsystem<MotorIOTalonFX> {
    
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
        kP.initDefault(2000.0);
        kD.initDefault(50.0);
        kA.initDefault(0.0);
  }


  private Rotation2d goalAngle = Rotation2d.kZero;
  private double goalVelocity = 0.0;
  private double lastGoalAngle = 0.0;


  TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
  private double turretOffset;
  private boolean turretZeroed = false;

  private ShootState shootState = ShootState.ACTIVE_SHOOTING;

  private Mechanism2d turretMech;
  private MechanismLigament2d turretLigament;

  private State setpoint = new State();

  public enum ShootState {
    ACTIVE_SHOOTING,
    TRACKING
  }

  public static final TurretSubsystem mInstance = new TurretSubsystem();

  
	public TurretSubsystem() {
		super(TurretConstants.getMotorIO(), "Turret Motor");
    initMechanism();
	}

  private void initMechanism(){
    turretMech = new Mechanism2d(3, 2);
    MechanismRoot2d turretRoot = turretMech.getRoot("root", 1.5, 0.1);
    turretLigament = turretRoot.append(new MechanismLigament2d("Turret", 1, 0));
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
            profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
        }

        // Reset profile when disabled
        if (DriverStation.isDisabled()) {
       // profile.reset(getPosition());
        //lastGoalAngle = getPosition();
        }


        if(DriverStation.isEnabled() && turretZeroed){
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

      State goalState = new State(MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle), robotRelativeGoalVelocity);

      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);

      this.applySetpoint(Setpoint.withMotionMagicSetpoint(Radians.of(setpoint.position)));
      
      /* Logger.recordOutput("Turret/GoalPositionRad", bestAngle);
      Logger.recordOutput("Turret/GoalVelocityRadPerSec", robotRelativeGoalVelocity);
      Logger.recordOutput("Turret/SetpointPositionRad", setpoint);
      Logger.recordOutput("Turret/SetpointVelocityRadPerSec", robotRelativeGoalVelocity);*/
      SmartDashboard.putNumber("Turret/GoalPositionRad", bestAngle);
      SmartDashboard.putNumber("Turret/SetpointPositionRad", setpoint.position);
     /* outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
      outputs.position = setpoint - turretOffset;
      outputs.velocity = robotRelativeGoalVelocity;
      outputs.kP = kP.get();
      outputs.kD = kD.get();*/

     // turretLigament.setAngle(robotRelativeGoalAngle);
     turretLigament.setAngle(new Rotation2d(getTurretAngle()));
    }
    
    SmartDashboard.putData("Turret Mech", turretMech);

    SmartDashboard.putString("State", shootState.toString());

   }

   private void setFieldRelativeTarget(Rotation2d angle, double velocity) {
    this.goalAngle = angle;
    this.goalVelocity = velocity;
  }

  private void zero() {
    turretZeroed = true;
    turretOffset = -getPosition().in(Radians); //-inputs.positionRads;
  }

  public double getTurretAngle() {
    return  turretOffset + getPosition().in(Radians); //return inputs.positionRads + turretOffset;
  }

  @AutoLogOutput(key = "Turret/MeasuredVelocityRadPerSec")
  public double getTurretVelocity() {
    return getVelocity().in(RadiansPerSecond); //return inputs.velocityRadsPerSec;
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
