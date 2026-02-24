package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.ServoMotorSubsystem;
import frc.robot.subsystems.turret.ShotCalculator;


public class HoodSubsystem extends ServoMotorSubsystem<MotorIOTalonFX> {

    private static final double minAngle = Units.degreesToRadians(0);
    private static final double maxAngle = Units.degreesToRadians(50);

    private double goalAngle = 0.0;
    private double goalVelocity = 0.0;

    private boolean manualTune = false;
    private static final LoggedTunableNumber manualHood = new LoggedTunableNumber("Hood/Manual");

    private ShotCalculator shotCalc;

	public HoodSubsystem() {
		super(
				HoodConstants.getMotorIO(),
				"Hood",
				HoodConstants.converter.toAngle(HoodConstants.kEpsilonThreshold));
                
		setCurrentPosition(HoodConstants.converter.toAngle(HoodConstants.kStowPosition));

        manualHood.initDefault(0);
		
	}

    public void setShotCalculator(ShotCalculator shotCalc){
        this.shotCalc = shotCalc;
    }

    public void periodic() {
        super.periodic();

        //work out the distance the hood should be
        double positionInRadians = MathUtil.clamp(goalAngle, minAngle, maxAngle);
     
        if(!this.manualTune){
            //this.applySetpoint(Setpoint.withMotionMagicSetpoint(Radians.of(positionInRadians)));
            this.applySetpoint(Setpoint.withPositionVelocitySetpoint(Radians.of(positionInRadians), RadiansPerSecond.of(goalVelocity)));
        }
       SmartDashboard.putNumber("Hood/Position",this.getPosition().baseUnitMagnitude());
    }

    private void setGoalParams(double angle, double velocity){

        goalAngle = angle;
        goalVelocity = velocity;

    }

    public Command setManualHoodAngle(){

        return run(() -> {
            this.manualTune = true;
           double positionInRadians = MathUtil.clamp(manualHood.get(), minAngle, maxAngle);
            this.applySetpoint(Setpoint.withPositionVelocitySetpoint(Radians.of(positionInRadians), RadiansPerSecond.of(1)));

        });

    }
    public Command resetAutoMap(){
        return Commands.runOnce(() -> this.manualTune = false);
    }

    public Command runTrackTargetActiveShootingCommand() {
        return run(
            () -> {
            var params = this.shotCalc.getParameters();
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
