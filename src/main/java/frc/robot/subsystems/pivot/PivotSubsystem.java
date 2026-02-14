package frc.robot.subsystems.pivot;


import frc.robot.lib.io.ServoMotorSubsystem;
import edu.wpi.first.units.measure.Angle;
import frc.robot.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.lib.io.MotorIOTalonFX;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

public class PivotSubsystem extends ServoMotorSubsystem<MotorIOTalonFX> {
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


    public BooleanSupplier isPositionWithinTolerance(){
		final Angle currentPosition = getPosition();
		final Angle targetPosition = Degrees.of(getSetpoint().baseUnits); //Degrees.of(25);
		

		return () -> currentPosition.isNear(targetPosition, Degrees.of(5));
	}


}