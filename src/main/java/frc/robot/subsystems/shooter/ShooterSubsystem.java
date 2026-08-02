package frc.robot.subsystems.shooter;

import frc.robot.lib.LoggedTracer;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO.Setpoint;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.turret.ShotCalculator;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem  extends MotorSubsystem<MotorIOTalonFX> {
	
    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();

    private static final LoggedTunableNumber passingSpeed = new LoggedTunableNumber("Shooter/PassingSpeed");
    private static final LoggedTunableNumber lobPassingSpeed = new LoggedTunableNumber("Shooter/LobPassingSpeed");
    private static final LoggedTunableNumber atSpeedToleranceRPS = new LoggedTunableNumber("Shooter/AtSpeedToleranceRPS");
    // 1.0 = spin down the flywheels while the robot is in the neutral zone (can't score the hub
    // from there, so save energy/wear). 0.0 = always spin per the shot calculator.
    private static final LoggedTunableNumber offInNeutralZone = new LoggedTunableNumber("Shooter/OffInNeutralZone", 1.0);
    // Debounce the neutral-zone boundary so pose jitter at the line doesn't strobe the flywheel
    // between idle and spun-up while parked on the edge.
    private static final LoggedTunableNumber neutralZoneDebounceSec = new LoggedTunableNumber("Shooter/NeutralZoneDebounceSec", 0.15);
    private Debouncer neutralZoneDebouncer = new Debouncer(0.15, DebounceType.kBoth);
    private double neutralZoneDebounceConfigured = 0.15;

    private double flywheelSpeedOffset = 15.0;
    private Double flywheelSpeedPreset = null; // null = use shot calculator
    // Set true while a shot is actively being commanded. Suppresses the neutral-zone spin-down so
    // the flywheel still runs when the driver shoots from the neutral zone; the gate only idles the
    // flywheel when we're NOT shooting.
    private boolean activelyShooting = false;
    
    //RPM = radians / second * 9.5493
    // 10 rads / sec = 95.493 RPM
   // public static final Setpoint COAST = Setpoint.withVelocitySetpoint(AngularVelocity.ofBaseUnits(10, RadiansPerSecond));

	//public static final ShooterSubsystem mInstance = new ShooterSubsystem();

	public static final Setpoint SHOOT = Setpoint.withVoltageSetpoint(ShooterConstants.kShootVoltage);
	
    private ShotCalculator shotCalc;

    private double setpointVal = 0.0;

	public ShooterSubsystem() {
		super(ShooterConstants.getMotorIO(), "Shooter Rollers");
        passingSpeed.initDefault(275);
        lobPassingSpeed.initDefault(360);
        atSpeedToleranceRPS.initDefault(10.0);
	}

    /** Returns true when the flywheel is within tolerance of its current target velocity. */
    public boolean isAtSpeed() {
        if (setpointVal <= 0) return false;
        double actualRPS = getVelocity().baseUnitMagnitude();
        Logger.recordOutput("Shooter/AtSpeedActualRPS", actualRPS);
        Logger.recordOutput("Shooter/AtSpeedSetpointRPS", setpointVal);
        
        return Math.abs(actualRPS - setpointVal) < atSpeedToleranceRPS.get();
    }


    public void setShotCalculator(ShotCalculator shotCalc){
        this.shotCalc = shotCalc;
    }

    /*
    
    FOC - adjust kV first, then kP for
    response. */

    private void runVelocity(double velocityRadPerSec) {
        this.applySetpoint(Setpoint.withVelocitySetpoint(
            AngularVelocity.ofBaseUnits(velocityRadPerSec, RotationsPerSecond)));

        // this.applySetpoint(Setpoint.withVelocityFOCSetpoint(
        //    RotationsPerSecond.of(rps)));
    }

     public void periodic() {
        super.periodic();
        double velocityRPS = this.getVelocity().in(RotationsPerSecond);
        SmartDashboard.putNumber("Shooter/SpeedRPS", velocityRPS);
        SmartDashboard.putNumber("Shooter/SetpointRPS", setpointVal);
        SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());

        Logger.recordOutput("Shooter/VelocityRPS", velocityRPS);
        Logger.recordOutput("Shooter/VelocitySetPoint", setpointVal);
        Logger.recordOutput("Shooter/SpeedOffset", flywheelSpeedOffset);
        Logger.recordOutput("Shooter/AtSpeed", isAtSpeed());
        LoggedTracer.record("ShooterPeriodic");
     }

    private void stop() {
        this.applySetpoint(IDLE);
    }

    public void incrementFlywheelOffset() {
        flywheelSpeedOffset += 5.0;
    }

    public void decrementFlywheelOffset() {
        flywheelSpeedOffset -= 5.0;
    }

    public void resetFlywheelOffset() {
        flywheelSpeedOffset = 0.0;
    }

    /** True when the robot is in the central neutral zone (used to gate hub shooting). */
    private boolean inNeutralZone() {
        return FieldConstants.inNeutralZone(DriveSubsystem.mInstance.getState().Pose.getX());
    }

    public Command runTrackTargetActiveShootingCommand() {
        return run(
            () -> {
                // Rebuild the debouncer if its tunable changed (rare, only while tuning).
                if (neutralZoneDebounceSec.get() != neutralZoneDebounceConfigured) {
                    neutralZoneDebounceConfigured = neutralZoneDebounceSec.get();
                    neutralZoneDebouncer = new Debouncer(neutralZoneDebounceConfigured, DebounceType.kBoth);
                }
                // Advance the debouncer every loop (regardless of activelyShooting) so its timing stays consistent.
                boolean inZoneDebounced = neutralZoneDebouncer.calculate(inNeutralZone());
                boolean neutralZoneOff = offInNeutralZone.get() >= 0.5 && !activelyShooting && inZoneDebounced;
                Logger.recordOutput("Shooter/OffInNeutralZone", neutralZoneOff);

                if (neutralZoneOff) {
                    // Can't score the hub from the neutral zone — keep the flywheels off.
                    setpointVal = 0.0;
                    stop();
                    return;
                }

                if (flywheelSpeedPreset != null) {
                    setpointVal = flywheelSpeedPreset + flywheelSpeedOffset;
                } else {
                    setpointVal = shotCalc.getParameters().flywheelSpeed() + flywheelSpeedOffset;
                }
                runVelocity(setpointVal);
            });
    }

    /**
     * Show/outreach mode fixed shot. Holds the flywheel at {@code rps} with no distance regression,
     * no flywheel preset/offset, and no neutral-zone gating — none of which mean anything away from a
     * real field.
     *
     * <p>Unlike {@link #runFixedCommand(DoubleSupplier)} this also publishes {@code setpointVal}, so
     * {@link #isAtSpeed()} actually works against the commanded speed. {@code runFixedCommand} goes
     * straight to {@code runVelocity()} and leaves {@code setpointVal} stale, which makes
     * {@code isAtSpeed()} return false forever — a feed gated on it would never open.
     */
    public Command runShowShotCommand(DoubleSupplier rps) {
        return run(
            () -> {
                setpointVal = rps.getAsDouble();
                runVelocity(setpointVal);
            });
    }

    public Command runFixedCommand(DoubleSupplier velocity) {
        return run(
            () -> {

                runVelocity(velocity.getAsDouble());

            });
    }

    public Command runAtVelocityCommand(double rps) {
        return run(() -> runVelocity(rps));
    }

    /** Suppress the neutral-zone spin-down while a shot is being commanded. */
    public void setActivelyShooting(boolean value) {
        activelyShooting = value;
    }

    public void setFlywheelPreset(double rps) {
        flywheelSpeedPreset = rps;
    }

    public Command setFlywheelPresetCommand(double rps) {
        return Commands.runOnce(() -> flywheelSpeedPreset = rps);
    }

    public Command clearFlywheelPresetCommand() {
        return Commands.runOnce(() -> flywheelSpeedPreset = null);
    }

    public void clearflywheelPreset(){
        flywheelSpeedPreset = null;
    }

    public Command runPassingCommand(DoubleSupplier distanceM) {
        return run(() -> {
            double speed = shotCalc.getFlywheelSpeedForDistance(distanceM.getAsDouble());
            runVelocity(Double.isNaN(speed) ? passingSpeed.get() : speed);
        });
    }

    public Command runLobPassingCommand(DoubleSupplier distanceM) {
        return run(() -> {
            double speed = shotCalc.getFlywheelSpeedForDistance(distanceM.getAsDouble());
            runVelocity(Double.isNaN(speed) ? lobPassingSpeed.get() : speed);
        });
    }

    public Command stopCommand()
    {
        return runOnce(this::stop);
    }

    private final VoltageOut m_voltReq = new VoltageOut(0.0);
    private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(8), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> ((MotorIOTalonFX)this.getMotorIO()).setMotorControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

}