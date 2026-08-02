package frc.robot.subsystems.turret;

import frc.robot.Constants;
import frc.robot.lib.LoggedTracer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.lib.AllianceFlipUtil;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSubsystem extends MotorSubsystem<MotorIO> {
    
  private static final double minAngle = Units.degreesToRadians(-180.0);
  private static final double maxAngle = Units.degreesToRadians(180.0);
  private static final double trackOverlapMargin = Units.degreesToRadians(10);
  private static final double trackCenterRads = (minAngle + maxAngle) / 2;
  private static final double trackMinAngle = trackCenterRads - Math.PI - trackOverlapMargin;
  private static final double trackMaxAngle = trackCenterRads + Math.PI + trackOverlapMargin;

  private Rotation2d goalAngle = Rotation2d.kZero;
  private double goalVelocityRadPerSec = 0.0;
  private double lastGoalAngle = 0.0;
  private double lastClampedAngle = 0.0; // last commanded mechanism setpoint (rad), for isOnTarget()

  // Position tolerance for reporting the turret "on target" before feeding a ball.
  private static final LoggedTunableNumber onTargetToleranceDeg =
      new LoggedTunableNumber("Turret/OnTargetToleranceDeg", 10.5);

   //1st game red - needed 3 ticks to right - 15
   //2nd game - we have none
  private double turretOffset = 0; 
   
  private boolean turretZeroed = true;

  // --- Homing / drift correction ---------------------------------------------------------
  // Fixed active-high digital sensor (wired into an analog port) that detects the magnet on
  // the moving turret. When the turret is stowed and the magnet is over the sensor, we snap
  // the motor position back to true zero, cancelling accumulated drift.
  private final TurretHomingSensor homingSensor =
      new TurretHomingSensor(TurretConstants.HOMING_ANALOG_CHANNEL);
  private static final LoggedTunableNumber autoRezeroEnabled =
      new LoggedTunableNumber("Turret/Homing/AutoRezeroEnabled", 1.0);
  // Wait this long after the turret returns to stow before re-zeroing, so we snap once the
  // turret has settled at rest — not while the magnet is sweeping past the sensor on the way in.
  private static final LoggedTunableNumber stowSettleSeconds =
      new LoggedTunableNumber("Turret/Homing/StowSettleSeconds", 1.0);
  // Extra safety: only re-zero when the turret is essentially stationary.
  private static final LoggedTunableNumber rezeroMaxVelRadPerSec =
      new LoggedTunableNumber("Turret/Homing/MaxVelRadPerSec", 0.15);
  // Max position error from home to allow a settle-based auto re-zero (fallback when the turret
  // parks at home without sweeping fully through the sensor).
  private static final LoggedTunableNumber rezeroMaxErrorDeg =
      new LoggedTunableNumber("Turret/Homing/RezeroMaxErrorDeg", 3.0);
  // Edge-midpoint homing: when the turret sweeps fully across the digital sensor, the rising and
  // falling edges bracket the magnet; their midpoint is true home, independent of stiction/direction.
  private boolean prevHomingTriggered = false;
  private double risingEdgePos = Double.NaN;   // mechanism angle (rad) where the band was entered
  private double risingEdgeTime = 0.0;
  // Position at the previous loop's sensor sample. An edge is only observed one loop late — up to
  // velocity × loop-time past the true transition, always biased in the sweep direction (at 40+ ms
  // loops this alone produced alternating ±5–15° "drift corrections"). The true edge lies between
  // the previous and current samples, so their midpoint is the unbiased estimate.
  private double prevHomingPos = Double.NaN;
  private int edgeHomeCount = 0;
  // Reject spurious crossings: the band width must be plausible and the sweep must be reasonably quick
  // (a genuine pass through, not enter-sit-leave).
  private static final LoggedTunableNumber edgeBandMinDeg =
      new LoggedTunableNumber("Turret/Homing/EdgeBandMinDeg", 1.0);
  private static final LoggedTunableNumber edgeBandMaxDeg =
      new LoggedTunableNumber("Turret/Homing/EdgeBandMaxDeg", 30.0);
  private static final LoggedTunableNumber edgeMaxCrossSeconds =
      new LoggedTunableNumber("Turret/Homing/EdgeMaxCrossSeconds", 1.5);
  // Guard against a huge correction from a bad reading.
  private static final LoggedTunableNumber edgeMaxCorrectionDeg =
      new LoggedTunableNumber("Turret/Homing/EdgeMaxCorrectionDeg", 15.0);
  // Tracks the current stow episode: when stow began, and whether we've already re-zeroed for it.
  private boolean wasStowed = false;
  private double stowStartTimestamp = 0.0;
  private boolean rezeroedThisStow = false;
  private int rezeroCount = 0;
  // Encoder value the auto-home snaps to (i.e. the calibrated offset from the magnet's physical
  // position to true zero). Starts at the constant; a manual zero jog trims it so auto-homing
  // reproduces the operator's corrected zero instead of undoing it.
  private double homingIndexRad = TurretConstants.HOMING_INDEX_POSITION_RAD;
  // Step size (degrees) for a single manual zero jog. Positive = turret moves left (CCW).
  private static final LoggedTunableNumber zeroNudgeStepDeg =
      new LoggedTunableNumber("Turret/ZeroNudgeStepDeg", 1.0);

  // TEMP: cable issue — set false to re-enable tracking
  private static final boolean LOCKED_TO_ZERO = false;

  // 1.0 = use position+velocity feedforward (with robot yaw-rate compensation) once close to target;
  // 0.0 = always Motion Magic. Enabled by default so shoot-on-the-move tracks instead of lagging.
  private static final LoggedTunableNumber useVelocityFeedforward =
      new LoggedTunableNumber("Turret/UseVelocityFeedforward", 1.0);
  // Position error (deg) below which we switch from Motion Magic (acquiring) to velocity feedforward
  // (tracking). Raised from 20: at 20 the loop dropped OUT of feedforward into Motion Magic during
  // fast robot rotation (exactly when the yaw-rate FF is needed), so lag grew. Keep FF engaged
  // through normal moving-target error; only the initial big acquisition slew uses Motion Magic.
  private static final LoggedTunableNumber trackingFeedforwardErrorDeg =
      new LoggedTunableNumber("Turret/TrackingFeedforwardErrorDeg", 45.0);

  private ShootState shootState = ShootState.ACTIVE_SHOOTING;
  private boolean stowed = true;
  private Rotation2d turretAnglePreset = null; // null = use shot calculator

  /**
   * Show/outreach mode: hold a fixed <em>robot-relative</em> angle (null = not held). This is the
   * same idea as {@link #stowed}, which holds 0 deg robot-relative, generalised to any angle — the
   * turret follows the chassis instead of holding a field bearing, so it stays pointed at the same
   * spot relative to the robot no matter how the driver spins. That is what you want aiming into a
   * net at a demo, where there is no field pose to hold a bearing against.
   *
   * <p>{@link #stowed} still wins over this, because stow is what the homing/re-zero logic keys off.
   */
  private Rotation2d robotRelativeHold = null;

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
  private double lastPassDistanceM = 0.0;

  /** Returns the most recent turret-to-passing-target distance (metres). Updated by passAutoCommand(). */
  public double getPassDistance() { return lastPassDistanceM; }


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

    // Initialize field visualization for AdvantageScope. Only registered in tuning mode —
    // once registered, SmartDashboard.updateValues() re-serializes it every loop.
    fieldViz = new Field2d();
    if (Constants.tuningMode) {
      SmartDashboard.putData("Turret Field Viz", fieldViz);
    }

    // Gear-ratio calibration buttons (run from AdvantageScope/Shuffleboard):
    //  1. Point turret straight ahead, press "Turret/Zero".
    //  2. Set Turret/TestAngleDeg (start at 45), press "Turret/RunTest".
    //  3. Measure the ACTUAL physical sweep, type it into Turret/Test/MeasuredDeg.
    //  4. Read the corrected value off Turret/Test/SuggestedRatio.
    SmartDashboard.putData("Turret/Zero", zeroCommand());
    SmartDashboard.putData("Turret/RunTest", testAimRobotRelative());
    // Drive to stow and re-zero off the homing magnet (drift correction).
    SmartDashboard.putData("Turret/Home", homeCommand());
    // Manual zero jog (use while stowed to nudge onto true forward).
    SmartDashboard.putData("Turret/NudgeZeroLeft", nudgeZeroLeftCommand());
    SmartDashboard.putData("Turret/NudgeZeroRight", nudgeZeroRightCommand());
  }

  public void setShotCalculator(ShotCalculator shotCalc){
    this.shotCalc = shotCalc;
  }

   public void periodic() {
        super.periodic(); // Critical: Updates motor inputs from simulation or hardware

        // Sample the homing encoder every loop (advances its debouncer) and, when the turret
        // is parked at stow with the magnet over the sensor, snap out any accumulated drift.
        homingSensor.update();
        updateHomingEdges();
        updateHomingRezero();

        /*SmartDashboard.putBoolean("Turret/ControlLoopActive", DriverStation.isEnabled() && turretZeroed);
        SmartDashboard.putNumber("Turret/CurrentPositionDeg", Units.radiansToDegrees(getTurretAngle()));
        SmartDashboard.putNumber("Turret/MaxVelocity", maxVelocity.get());
        SmartDashboard.putNumber("Turret/MaxAcceleration", maxAcceleration.get());*/

        if(DriverStation.isEnabled() && turretZeroed){
          Rotation2d robotAngle = DriveSubsystem.mInstance.getState().Pose.getRotation();

          if (LOCKED_TO_ZERO || stowed) {
            goalAngle = robotAngle; // hold 0° robot-relative regardless of commands (stow overrides tracking)
            goalVelocityRadPerSec = 0.0;
          } else if (robotRelativeHold != null) {
            // Show mode: hold a fixed angle off the chassis (see robotRelativeHold).
            goalAngle = robotAngle.plus(robotRelativeHold);
            goalVelocityRadPerSec = 0.0;
          }

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
      lastClampedAngle = clampedAngle; // remember the commanded setpoint for isOnTarget()

      // Robot-relative turret slew rate needed to hold aim. The setpoint is (goalAngle - robotAngle),
      // so its derivative is (field bearing rate) - (robot yaw rate). Without the yaw-rate term the
      // turret lags badly whenever the robot rotates (logs showed 49-111deg error when slewing fast).
      // BUT while stowed goalAngle tracks robotAngle, so the robot-relative setpoint is CONSTANT and its
      // true derivative is 0 — feeding -robotYawRate here would command the turret to counter-rotate
      // against the chassis (hold field heading), drifting it off home whenever the robot turns. So the
      // feedforward is zeroed while stowed; the position loop holds home and rejects the disturbance.
      // A robotRelativeHold is the same case — the setpoint is a constant offset from the chassis, so
      // its true derivative is 0 too. Feeding -robotYawRate there would drag the turret off the net
      // whenever the driver turned.
      double robotYawRate = DriveSubsystem.mInstance.getState().Speeds.omegaRadiansPerSecond;
      double turretVelFF = (LOCKED_TO_ZERO || stowed || robotRelativeHold != null)
          ? 0.0
          : (goalVelocityRadPerSec - robotYawRate);

      // Hybrid control: while acquiring (far from target) use Motion Magic so big moves stay smoothly
      // profiled (preserves the overshoot tuning). Once close, switch to position + velocity
      // feedforward so the turret tracks a moving target instead of chasing it.
      double posErrRad = Math.abs(clampedAngle - getPosition().in(Radians));
      boolean tracking = posErrRad < Units.degreesToRadians(trackingFeedforwardErrorDeg.get());
      if (useVelocityFeedforward.get() >= 0.5 && tracking) {
        this.applySetpoint(Setpoint.withPositionVelocitySetpoint(
            Radians.of(clampedAngle),
            RadiansPerSecond.of(turretVelFF)));
      } else {
        this.applySetpoint(Setpoint.withMotionMagicSetpoint(Radians.of(clampedAngle)));
      }

      SmartDashboard.putNumber("Turret/RobotYawRate", robotYawRate);
      Logger.recordOutput("Turret/TurretVelFF", turretVelFF);
      Logger.recordOutput("Turret/Tracking", tracking);

      SmartDashboard.putNumber("Turret/BestAngleRad", bestAngle);
      SmartDashboard.putNumber("Turret/SetpointPositionRad", clampedAngle);
      SmartDashboard.putNumber("Turret/Offset", turretOffset);

    } else {
      // Disabled (or not yet zeroed): keep the goal pinned to "hold current position" so the
      // control loop doesn't lunge on the first enabled loop. Otherwise goalAngle sits at its
      // field-relative default (0 deg field) and — until the stow default command overwrites it a
      // loop later — the turret would try to point at absolute field zero, a large robot-relative
      // swing that reads as the turret "searching" before settling home.
      goalAngle = DriveSubsystem.mInstance.getState().Pose.getRotation(); // robot-relative 0 == home
      goalVelocityRadPerSec = 0.0;
      lastGoalAngle = getPosition().in(Radians);
    }

    double robotRelativeAngleDeg = Units.radiansToDegrees(getTurretAngle());
    Pose2d robotPose = DriveSubsystem.mInstance.getState().Pose;
    Rotation2d robotAngle = robotPose.getRotation();
    double fieldRelativeAngleDeg = robotAngle.getDegrees() + robotRelativeAngleDeg;
    

    Rotation2d turretFieldAngle = Rotation2d.fromDegrees(fieldRelativeAngleDeg);
    Pose2d turretAimPose = new Pose2d(robotPose.getTranslation(), turretFieldAngle);

    // Visualization widgets (Mechanism2d + turret Field2d) only in tuning mode — they are
    // re-serialized by SmartDashboard.updateValues() every loop. Turret/AimPose is still
    // logged below for AdvantageScope regardless.
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("Turret/RobotRelativeAngleDeg", robotRelativeAngleDeg);
      SmartDashboard.putNumber("Turret/FieldRelativeAngleDeg", fieldRelativeAngleDeg);
      SmartDashboard.putBoolean("Turret/Stowed", stowed);

      turretLigament.setAngle(robotRelativeAngleDeg);
      SmartDashboard.putData("Turret Mech", turretMech);
      fieldViz.setRobotPose(turretAimPose);
      if (currentTarget != null) {
        fieldViz.getObject("Target").setPose(new Pose2d(currentTarget, new Rotation2d()));
      }
      SmartDashboard.putString("State", shootState.toString());
    }

    

    // Live gear-ratio calibration: suggested = currentRatio * (commanded / measured).
    double measuredDeg = testMeasuredDeg.get();
    if (Math.abs(measuredDeg) > 1e-3) {
      double suggestedRatio =
          TurretConstants.SENSOR_TO_MECHANISM_RATIO * (testAngleDeg.get() / measuredDeg);
      SmartDashboard.putNumber("Turret/Test/SuggestedRatio", suggestedRatio);
      SmartDashboard.putNumber("Turret/Test/CurrentRatio", TurretConstants.SENSOR_TO_MECHANISM_RATIO);
    }

    // AdvantageKit structured logging for replay
    Logger.recordOutput("Turret/RobotRelativeAngleDeg", robotRelativeAngleDeg);
    Logger.recordOutput("Turret/FieldRelativeAngleDeg", fieldRelativeAngleDeg);
    Logger.recordOutput("Turret/AimPose", turretAimPose);
    Logger.recordOutput("Turret/ShootState", shootState.toString());
    Logger.recordOutput("Turret/Stowed", stowed);
    Logger.recordOutput("Turret/GoalAngleDeg", goalAngle.getDegrees());
    Logger.recordOutput("Turret/Offset", turretOffset);
    LoggedTracer.record("TurretPeriodic");

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

  /**
   * Edge-midpoint homing for the digital sensor. When the turret sweeps fully through the sensor,
   * the rising edge (enter band) and falling edge (exit band) bracket the magnet; their midpoint is
   * true home — independent of where stiction would let it settle and of the sweep direction. On a
   * valid crossing, shifts the encoder frame so that midpoint reads the calibrated home index.
   * Purely observational (reads position at the transitions) plus a small frame shift.
   */
  private void updateHomingEdges() {
    boolean trig = homingSensor.isTriggered();
    double pos = getPosition().in(Radians);
    double now = Timer.getFPGATimestamp();

    // Velocity compensation: the transition happened somewhere between the previous sample and
    // this one, so use the midpoint of the two positions instead of the (late, direction-biased)
    // current position.
    double edgePos = Double.isNaN(prevHomingPos) ? pos : (prevHomingPos + pos) / 2.0;

    if (trig && !prevHomingTriggered) {
      // Entered the band.
      risingEdgePos = edgePos;
      risingEdgeTime = now;
    } else if (!trig && prevHomingTriggered && !Double.isNaN(risingEdgePos)) {
      // Exited the band — a full crossing. Validate width and duration, then correct.
      double bandDeg = Math.abs(Units.radiansToDegrees(edgePos - risingEdgePos));
      boolean plausible =
          bandDeg >= edgeBandMinDeg.get()
          && bandDeg <= edgeBandMaxDeg.get()
          && (now - risingEdgeTime) <= edgeMaxCrossSeconds.get();
      if (plausible && autoRezeroEnabled.get() >= 0.5) {
        double midpoint = (risingEdgePos + edgePos) / 2.0;
        double drift = midpoint - homingIndexRad; // how far the band centre has walked from home
        if (Math.abs(Units.radiansToDegrees(drift)) <= edgeMaxCorrectionDeg.get()) {
          // Shift the whole frame so the band centre now reads the home index.
          setCurrentPosition(Radians.of(pos - drift));
          pos -= drift; // keep the local sample (stored as prevHomingPos below) in the new frame
          lastGoalAngle -= drift;
          lastClampedAngle -= drift;
          turretZeroed = true;
          edgeHomeCount++;
          Logger.recordOutput("Turret/Homing/EdgeDriftDeg", Units.radiansToDegrees(drift));
          Logger.recordOutput("Turret/Homing/EdgeBandDeg", bandDeg);
        }
      }
      risingEdgePos = Double.NaN;
    }
    prevHomingTriggered = trig;
    prevHomingPos = pos;
    Logger.recordOutput("Turret/Homing/EdgeHomeCount", edgeHomeCount);
  }

  /**
   * Auto drift-correction. When the turret is stowed, nearly stationary, and the homing magnet
   * is over the fixed sensor, resets the motor's position to the known index angle. Latched so
   * it fires once per pass through the magnet and re-arms only after leaving the window.
   */
  private void updateHomingRezero() {
    double now = Timer.getFPGATimestamp();

    // Start a fresh stow episode the moment the turret is commanded to stow.
    if (stowed && !wasStowed) {
      stowStartTimestamp = now;
      rezeroedThisStow = false;
    }
    wasStowed = stowed;

    double stowedFor = stowed ? now - stowStartTimestamp : 0.0;
    boolean settled = stowed && stowedFor >= stowSettleSeconds.get();

    // Only re-zero when the turret is already settled CLOSE to home. The magnet's detection band is
    // wider than the stiction settling band, so if we snap to zero while parked a few degrees off
    // (stiction), we inject that offset into the zero — that walks the aim over a match. Requiring a
    // small position error means we only correct genuine small encoder drift, never stiction offset.
    double homeErrorRad = Math.abs(getPosition().in(Radians) - homingIndexRad);
    boolean nearHome = homeErrorRad <= Units.degreesToRadians(rezeroMaxErrorDeg.get());

    boolean canRezero =
        autoRezeroEnabled.get() >= 0.5
        && settled
        && !rezeroedThisStow
        && homingSensor.isAtIndex()
        && nearHome
        && Math.abs(getTurretVelocity()) < rezeroMaxVelRadPerSec.get();

    Logger.recordOutput("Turret/Homing/HomeErrorDeg", Units.radiansToDegrees(homeErrorRad));

    if (canRezero) {
      rezeroFromHoming();
      rezeroedThisStow = true; // once per stow episode
    }

    Logger.recordOutput("Turret/Homing/StowedForSec", stowedFor);
    Logger.recordOutput("Turret/Homing/RezeroedThisStow", rezeroedThisStow);
    Logger.recordOutput("Turret/Homing/RezeroCount", rezeroCount);
  }

  /**
   * Snaps the turret's tracked position to the homing index angle, cancelling drift.
   *
   * <p>Only corrects the mechanical encoder position. The operator's aiming trim
   * ({@code turretOffset}) is a shooting-time correction and is deliberately left untouched,
   * so it survives every stow/home cycle and still applies only when shooting.
   */
  private void rezeroFromHoming() {
    turretZeroed = true;
    setCurrentPosition(Radians.of(homingIndexRad));
    lastGoalAngle = homingIndexRad;
    rezeroCount++;
  }

  /**
   * Manual homing command: hold the turret at stow, then re-zero the moment the magnet is
   * detected. Useful as a start-of-match routine or an operator button. Does not require the
   * turret to already be stowed — it drives to stow first.
   */
  public Command homeCommand() {
    return run(() -> {
          setStowed(true);
          Rotation2d robotAngle = DriveSubsystem.mInstance.getState().Pose.getRotation();
          setFieldRelativeTarget(robotAngle); // 0deg robot-relative == stow
          setShootState(ShootState.ACTIVE_SHOOTING);
        })
        .until(homingSensor::isAtIndex)
        .andThen(runOnce(this::rezeroFromHoming));
  }

  public void offsetLeft(){
    turretOffset += 2;
  }

  public void offsetRight(){
    turretOffset -= 2;
  }

  public void zeroOffset(){
    turretOffset = 0;
  }

  /**
   * Manually jog the turret's zero position. Intended for use while stowed: if the true zero has
   * drifted, jog the turret onto true forward and that shifted position becomes the new zero.
   *
   * <p>Works by shifting the encoder reference. While stowed the setpoint is a fixed 0, so shifting
   * the reference makes the turret physically rotate by {@code deltaDeg} and then hold there as the
   * new zero. This is independent of {@code turretOffset} (the shooting-only aiming trim). The
   * homing index is trimmed by the same amount so a later auto-home reproduces this corrected zero
   * rather than snapping back to the raw magnet position.
   *
   * @param deltaDeg Degrees to jog; positive moves the turret left (CCW).
   */
  public void nudgeZero(double deltaDeg){
    double deltaRad = Units.degreesToRadians(deltaDeg);
    setCurrentPosition(Radians.of(getPosition().in(Radians) - deltaRad));
    homingIndexRad -= deltaRad;
    Logger.recordOutput("Turret/ZeroNudgeTotalIndexRad", homingIndexRad);
  }

  public void nudgeZeroLeft(){
    nudgeZero(zeroNudgeStepDeg.get());
  }

  public void nudgeZeroRight(){
    nudgeZero(-zeroNudgeStepDeg.get());
  }

  /** Jog the zero one step left (CCW). Runs while disabled too, for bench calibration. */
  public Command nudgeZeroLeftCommand(){
    return runOnce(this::nudgeZeroLeft).ignoringDisable(true);
  }

  /** Jog the zero one step right (CW). Runs while disabled too, for bench calibration. */
  public Command nudgeZeroRightCommand(){
    return runOnce(this::nudgeZeroRight).ignoringDisable(true);
  }

  /**
   * True mechanical turret angle (robot-relative, radians). Does NOT include the operator
   * aiming trim ({@code turretOffset}): the trim is added to the aim target in the shoot
   * commands, so once the turret drives there {@code getPosition()} already reflects it.
   * Adding it here too would double-count it in the shot calculator and field-relative viz.
   */
  public double getTurretAngle() {
    return getPosition().in(Radians);
  }

  public double getTurretVelocity() {
    return getVelocity().in(RadiansPerSecond);
  }

  /**
   * True when the turret has arrived at its commanded angle (within tolerance) and is zeroed.
   * Gate the shooter feed on this so a ball isn't fed mid-slew during a large turn.
   */
  public boolean isOnTarget() {
    if (!turretZeroed) {
      return false;
    }
    double errorRad = Math.abs(getPosition().in(Radians) - lastClampedAngle);
    boolean onTarget = errorRad <= Units.degreesToRadians(onTargetToleranceDeg.get());
    Logger.recordOutput("Turret/OnTarget", onTarget);
    Logger.recordOutput("Turret/OnTargetErrorDeg", Units.radiansToDegrees(errorRad));
    return onTarget;
  }

  public void setShootState(ShootState state){
    this.shootState = state;
  }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = shotCalc.getParameters();
          double totalOffset = turretOffset;
          setFieldRelativeTarget(
              params.turretAngle().plus(Rotation2d.fromDegrees(totalOffset)),
              params.turretVelocity());
          setShootState(ShootState.TRACKING);
        });
  }

  public void setTurretAnglePreset(Rotation2d angle) {
    this.turretAnglePreset = angle;
  }

  public void clearTurretAnglePreset() {
    this.turretAnglePreset = null;
  }

  public Command runTrackTargetActiveShootingCommand() {
    return run(
        () -> {
          if (stowed) {
            // Hold 0° robot-relative (straight ahead)
            Rotation2d robotAngle = DriveSubsystem.mInstance.getState().Pose.getRotation();
            setFieldRelativeTarget(robotAngle);
            setShootState(ShootState.ACTIVE_SHOOTING);
          } else if (turretAnglePreset != null) {
            double totalOffset = turretOffset;
            setFieldRelativeTarget(turretAnglePreset.plus(Rotation2d.fromDegrees(totalOffset)));
            setShootState(ShootState.ACTIVE_SHOOTING);
          } else {
            var params = shotCalc.getParameters();
            double totalOffset = turretOffset;
            setFieldRelativeTarget(
                params.turretAngle().plus(Rotation2d.fromDegrees(totalOffset)),
                params.turretVelocity());
            setShootState(ShootState.ACTIVE_SHOOTING);
          }
        });
  }

  /**
   * Show/outreach mode: swing to and hold a fixed robot-relative angle for as long as this command
   * runs. Positive is left (CCW looking down on the robot / from behind it), matching
   * {@link #offsetLeft()}. 90 aims out the robot's left side.
   *
   * <p>Owns the turret while it runs, so it displaces the tracking default command entirely — no
   * shot-calculator or pose lookups happen at all, which is the point at a venue with no AprilTags.
   * Un-stows on start and re-stows on end, so the turret parks back over the homing sensor and the
   * auto re-zero logic keeps working between shots.
   *
   * <p>Gate the feed on {@link #isOnTarget()} with this — a 90 deg slew is not instant, and without
   * that gate the first ball leaves while the turret is still swinging.
   */
  public Command runRobotRelativeHoldCommand(java.util.function.DoubleSupplier degrees) {
    return run(() -> {
          robotRelativeHold = Rotation2d.fromDegrees(degrees.getAsDouble());
          setShootState(ShootState.ACTIVE_SHOOTING);
        })
        .beforeStarting(() -> stowed = false)
        .finallyDo(
            interrupted -> {
              robotRelativeHold = null;
              stowed = true;
            });
  }

  public void setStowed(boolean stowed) {
    this.stowed = stowed;
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

      lastPassDistanceM = toTarget.getNorm();

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
        AllianceFlipUtil.apply(FieldConstants.Depot.depotCenter.toTranslation2d()));
    //return pointAtFieldPosition(
    //    AllianceFlipUtil.apply(FieldConstants.PassingTargets.leftPassTarget));
  }

  /**
   * Aim to pass fuel over the right bump into the alliance zone.
   * Alliance-flipped so it works for both blue and red.
   */
  public Command passRightCommand() {
    return pointAtFieldPosition(
        AllianceFlipUtil.apply(FieldConstants.Outpost.centerPoint));
  //  return pointAtFieldPosition(
   //     AllianceFlipUtil.apply(FieldConstants.PassingTargets.rightPassTarget));
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

  // Type the physically-measured sweep here after running the test; SuggestedRatio updates live.
  private static final LoggedTunableNumber testMeasuredDeg =
      new LoggedTunableNumber("Turret/Test/MeasuredDeg", 0.0);

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

      lastPassDistanceM = toTarget.getNorm();

      setFieldRelativeTarget(toTarget.getAngle());
      setShootState(ShootState.ACTIVE_SHOOTING);

      SmartDashboard.putNumber("Turret/PassTargetX", target.getX());
      SmartDashboard.putNumber("Turret/PassTargetY", target.getY());
      SmartDashboard.putNumber("Turret/PassDistance", lastPassDistanceM);
    });
  }



}
