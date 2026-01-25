package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.ServoMotorSubsystem;
import frc.robot.subsystems.turret.ShotCalculator;


public class HoodSubsystem extends ServoMotorSubsystem<MotorIOTalonFX> {

    private static final double minAngle = Units.degreesToRadians(0);
    private static final double maxAngle = Units.degreesToRadians(50);

    public static final HoodSubsystem mInstance = new HoodSubsystem();

    private double goalAngle = 0.0;
    private double goalVelocity = 0.0;

	private HoodSubsystem() {
		super(
				HoodConstants.getMotorIO(),
				"Hood",
				HoodConstants.converter.toAngle(HoodConstants.kEpsilonThreshold));
                
		setCurrentPosition(HoodConstants.converter.toAngle(HoodConstants.kStowPosition));
		
	}

    public void periodic() {
        super.periodic();

        //work out the distance the hood should be
        double positionInRadians = MathUtil.clamp(goalAngle, minAngle, maxAngle);
     
        //this.applySetpoint(Setpoint.withMotionMagicSetpoint(Radians.of(positionInRadians)));
        this.applySetpoint(Setpoint.withPositionVelocitySetpoint(Radians.of(positionInRadians), RadiansPerSecond.of(goalVelocity)));
      
    }

    private void setGoalParams(double angle, double velocity){

        goalAngle = angle;
        goalVelocity = velocity;

    }

    public Command runTrackTargetActiveShootingCommand() {
        return run(
            () -> {
            var params = ShotCalculator.getInstance().getParameters();
                setGoalParams(params.hoodAngle(), params.hoodVelocity());
            
            });
    }

    public Command runFixedCommand(DoubleSupplier angle, DoubleSupplier velocity) {
        return run(
            () -> {
            
                setGoalParams(angle.getAsDouble(), velocity.getAsDouble());
            
            });
    }
    
}
