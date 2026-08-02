package frc.robot.subsystems.hood;

import frc.robot.lib.LoggedTracer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.ServoMotorSubsystem;
import frc.robot.subsystems.turret.ShotCalculator;
import org.littletonrobotics.junction.Logger;


public class HoodSubsystem extends ServoMotorSubsystem<MotorIOTalonFX> {

    private double goalAngleDeg = 0.0;

    private boolean stowed = true;
    private static final LoggedTunableNumber passingAngle = new LoggedTunableNumber("Hood/PassingAngle");
    private static final LoggedTunableNumber lobPassingAngle = new LoggedTunableNumber("Hood/LobPassingAngle");

    private ShotCalculator shotCalc;

	public HoodSubsystem() {
		super(
				HoodConstants.getMotorIO(),
				"Hood",
				HoodConstants.converter.toAngle(HoodConstants.kEpsilonThreshold));
                
		setCurrentPosition(HoodConstants.converter.toAngle(HoodConstants.kStowPosition));

        passingAngle.initDefault(35);
        lobPassingAngle.initDefault(45);

	}

    public void setShotCalculator(ShotCalculator shotCalc){
        this.shotCalc = shotCalc;
    }

    public void periodic() {
        super.periodic();

       
        this.applySetpoint(Setpoint.withMotionMagicSetpoint(Degrees.of(goalAngleDeg)));
       SmartDashboard.putNumber("Hood/Position",this.getPosition().in(Degrees));
       SmartDashboard.putNumber("Hood/GoalAngleDeg", goalAngleDeg);

       // AdvantageKit structured logging for replay
       Logger.recordOutput("Hood/PositionDeg", this.getPosition().in(Degrees));
       Logger.recordOutput("Hood/GoalAngleDeg", goalAngleDeg);
       LoggedTracer.record("HoodPeriodic");
    }

    private void setGoalAngleDeg(double angleDeg){
        goalAngleDeg = angleDeg;
    }

    public Command runTrackTargetActiveShootingCommand() {
        return run(
            () -> {
            if (stowed) {
                setGoalAngleDeg(0.0);
            } else {
                var params = this.shotCalc.getParameters();
                setGoalAngleDeg(Units.radiansToDegrees(params.hoodAngle() * HoodConstants.fudgeFactor));
            }
            });
    }

    /** Absolute error between the hood's actual angle and its commanded goal, in degrees. */
    public double getGoalErrorDeg() {
        return Math.abs(this.getPosition().in(Degrees) - goalAngleDeg);
    }

    public void setStowed(boolean stowed) {
        this.stowed = stowed;
    }

    public boolean isStowed() {
        return stowed;
    }

    public Command runPassingCommand() {
        return run(() -> setGoalAngleDeg(passingAngle.get() * HoodConstants.fudgeFactor));
    }

    public Command runLobPassingCommand() {
        return run(() -> setGoalAngleDeg(lobPassingAngle.get() * HoodConstants.fudgeFactor));
    }

    public Command runFixedCommand(DoubleSupplier angleDeg) {
        return run(() -> setGoalAngleDeg(angleDeg.getAsDouble()));
    }
    
}
