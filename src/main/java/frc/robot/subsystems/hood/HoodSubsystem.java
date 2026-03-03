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
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.ServoMotorSubsystem;
import frc.robot.subsystems.turret.ShotCalculator;
import org.littletonrobotics.junction.Logger;


public class HoodSubsystem extends ServoMotorSubsystem<MotorIOTalonFX> {

    private static final double minAngleDeg = 0;
    private static final double maxAngleDeg = 50;

    private double goalAngleDeg = 0.0;
    private double goalVelocity = 0.0;

    private boolean manualTune = false;
    private boolean stowed = false;
    private static final LoggedTunableNumber manualHood = new LoggedTunableNumber("Hood/Manual");
    private static final LoggedTunableNumber passingAngle = new LoggedTunableNumber("Hood/PassingAngle");

    private ShotCalculator shotCalc;

	public HoodSubsystem() {
		super(
				HoodConstants.getMotorIO(),
				"Hood",
				HoodConstants.converter.toAngle(HoodConstants.kEpsilonThreshold));
                
		setCurrentPosition(HoodConstants.converter.toAngle(HoodConstants.kStowPosition));

        manualHood.initDefault(0);
        passingAngle.initDefault(15);

	}

    public void setShotCalculator(ShotCalculator shotCalc){
        this.shotCalc = shotCalc;
    }

    public void periodic() {
        super.periodic();

       
        if(!this.manualTune){
            this.applySetpoint(Setpoint.withMotionMagicSetpoint(Degrees.of(goalAngleDeg)));
        }
       SmartDashboard.putNumber("Hood/Position",this.getPosition().baseUnitMagnitude());
       SmartDashboard.putNumber("Hood/GoalAngleDeg", goalAngleDeg);
       SmartDashboard.putBoolean("Hood/ManualTune", manualTune);

       // AdvantageKit structured logging for replay
       Logger.recordOutput("Hood/PositionDeg", this.getPosition().in(Degrees));
       Logger.recordOutput("Hood/GoalAngleDeg", goalAngleDeg);
       Logger.recordOutput("Hood/ManualTune", manualTune);
    }

    private void setGoalParamsDeg(double angleDeg, double velocity){

        goalAngleDeg = angleDeg;
        goalVelocity = velocity;

    }

    public Command setManualHoodAngle(){

        return run(() -> {
            this.manualTune = true;
            this.applySetpoint(Setpoint.withMotionMagicSetpoint(Degrees.of(manualHood.get())));
        });

    }
    public Command resetAutoMap(){
        return Commands.runOnce(() -> this.manualTune = false);
    }

    public Command runTrackTargetActiveShootingCommand() {
        return run(
            () -> {
            if (stowed) {
                setGoalParamsDeg(0.0, 0.0);
            } else {
                var params = this.shotCalc.getParameters();
                setGoalParamsDeg(Units.radiansToDegrees(params.hoodAngle()), params.hoodVelocity());
            }
            });
    }

    public void setStowed(boolean stowed) {
        this.stowed = stowed;
    }

    public boolean isStowed() {
        return stowed;
    }

    public Command runPassingCommand() {
        return run(() -> setGoalParamsDeg(passingAngle.get(), 0.0));
    }

    public Command runFixedCommand(DoubleSupplier angleDeg, DoubleSupplier velocity) {
        return run(
            () -> {

                setGoalParamsDeg(angleDeg.getAsDouble(), velocity.getAsDouble());

            });
    }
    
}
