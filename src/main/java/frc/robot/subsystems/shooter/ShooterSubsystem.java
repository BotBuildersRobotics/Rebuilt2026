package frc.robot.subsystems.shooter;

import frc.robot.lib.io.MotorIO.Setpoint;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.subsystems.turret.ShotCalculator;

public class ShooterSubsystem  extends MotorSubsystem<MotorIOTalonFX> {
	
    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    
    //RPM = radians / second * 9.5493
    // 10 rads / sec = 95.493 RPM
   // public static final Setpoint COAST = Setpoint.withVelocitySetpoint(AngularVelocity.ofBaseUnits(10, RadiansPerSecond));

	public static final ShooterSubsystem mInstance = new ShooterSubsystem();

	public ShooterSubsystem() {
		super(ShooterConstants.getMotorIO(), "Shooter Rollers");
	}

    private void runVelocity(double velocityRadsPerSec){

        this.applySetpoint(Setpoint.withVelocitySetpoint(AngularVelocity.ofBaseUnits(velocityRadsPerSec, RadiansPerSecond)));
    }

    private void stop(){
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

    public Command stopCommnad()
    {
        return runOnce(this::stop);
    }

}