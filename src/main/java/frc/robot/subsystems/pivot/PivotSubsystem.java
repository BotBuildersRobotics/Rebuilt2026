package frc.robot.subsystems.pivot;


import frc.robot.lib.io.ServoMotorSubsystem;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.lib.io.MotorIOTalonFX;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

public class PivotSubsystem extends ServoMotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint STOW_DEFENCE = Setpoint.withMotionMagicSetpoint(PivotConstants.kDefenceStow);
	public static final Setpoint STOW_FULL = Setpoint.withMotionMagicSetpoint(PivotConstants.kFullStowPosition);
	public static final Setpoint DEPLOY = Setpoint.withMotionMagicSetpoint(PivotConstants.kDeployPosition);
	
    
	public final static Setpoint AGITATE = Setpoint.withMotionMagicSetpoint(PivotConstants.agitatePosition);


	public static final PivotSubsystem mInstance = new PivotSubsystem();

	public PivotSubsystem() {
		super(
                PivotConstants.getMotorIO(),
				"Intake Pivot",
				PivotConstants.kEpsilonThreshold);
		setCurrentPosition(PivotConstants.kFullStowPosition);
		applySetpoint(STOW_FULL);
	}


    @Override
    public void periodic() {
        super.periodic();

        // AdvantageKit structured logging for replay
        Logger.recordOutput("Pivot/PositionDeg", getPosition().in(Degrees));
        Logger.recordOutput("Pivot/StatorCurrentAmps", getStatorCurrent().in(Units.Amps));
    }

    public BooleanSupplier isPositionWithinTolerance(){
		final Angle currentPosition = getPosition();
		final Angle targetPosition = Degrees.of(getSetpoint().baseUnits); 
		

		return () -> currentPosition.isNear(targetPosition, Degrees.of(5));
	}

	public Command resetDeployPosition(){

		return Commands.runOnce(() -> this.setCurrentPosition(Degrees.of(45)));
	}

	/**
	 * Drives the pivot down at a slow voltage until stator current spikes
	 * (indicating contact with the bumper), then sets that position as the
	 * deploy angle and holds it.
	 */
	public Command findDeployLimitCommand() {
		// Slow downward voltage (negative = toward deploy/down)
		final double homingVolts = 4.5;
		// Current threshold indicating the mechanism has hit the hard stop
		final double currentThresholdAmps = 6.0;
		// Ignore current for this long to get past inrush spike
		final double ignoreCurrentSecs = 0.75;

		Timer timer = new Timer();

		return
			// Drive down, ignoring current for the first 250ms
			runOnce(() -> timer.restart())
			.andThen(run(() -> {
				applySetpoint(Setpoint.withVoltageSetpoint(Volts.of(homingVolts)));
				SmartDashboard.putNumber("Pivot/HomingCurrent",
					getStatorCurrent().in(Units.Amps));
			})
			.until(() -> timer.hasElapsed(ignoreCurrentSecs)
				&& getStatorCurrent().in(Units.Amps) > currentThresholdAmps))
			// Stop, mark this as the deploy position, hold it
			.andThen(runOnce(() -> {
				timer.stop();
				Angle foundPosition = getPosition();
				setCurrentPosition(PivotConstants.kDeployPosition);
				applySetpoint(DEPLOY);
				SmartDashboard.putNumber("Pivot/FoundDeployDeg",
					foundPosition.in(Degrees));
			}));
	}

}