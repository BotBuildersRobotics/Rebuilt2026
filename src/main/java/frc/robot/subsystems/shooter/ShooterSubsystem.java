package frc.robot.subsystems.shooter;

import frc.robot.lib.io.MotorIO.Setpoint;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.subsystems.turret.ShotCalculator;

public class ShooterSubsystem  extends MotorSubsystem<MotorIOTalonFX> {
	
    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    
    //RPM = radians / second * 9.5493
    // 10 rads / sec = 95.493 RPM
   // public static final Setpoint COAST = Setpoint.withVelocitySetpoint(AngularVelocity.ofBaseUnits(10, RadiansPerSecond));

	public static final ShooterSubsystem mInstance = new ShooterSubsystem();

	public static final Setpoint SHOOT = Setpoint.withVoltageSetpoint(ShooterConstants.kShootVoltage);
	

	public ShooterSubsystem() {
		super(ShooterConstants.getMotorIO(), "Shooter Rollers");
	}

    private void runVelocity(double velocityRadsPerSec) {
        this.applySetpoint(Setpoint.withVelocitySetpoint(
            AngularVelocity.ofBaseUnits(velocityRadsPerSec, RadiansPerSecond)));
    }

    private void stop() {
        this.applySetpoint(IDLE);
    }

    public Command runTrackTargetActiveShootingCommand() {
        return run(
            () -> {
            var params = ShotCalculator.getInstance().getParameters();
                runVelocity( params.flywheelSpeed());
            
            });
    }

    public Command runFixedCommand(DoubleSupplier velocity) {
        return run(
            () -> {
            
                runVelocity(velocity.getAsDouble());
            
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