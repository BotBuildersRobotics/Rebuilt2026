package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.ShowConstants;
import frc.robot.generated.TunerConstants;

import java.util.function.UnaryOperator;

public class DriveConstants {
	public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;

	public static final LinearAcceleration kMaxAcceleration = Units.MetersPerSecondPerSecond.of(12.0);
	public static final AngularVelocity kMaxAngularRate = Units.RadiansPerSecond.of(2.75 * Math.PI);
	public static final AngularAcceleration kMaxAngularAcceleration =
			kMaxAngularRate.div(0.1).per(Units.Second);

	public static final SwerveRequest.FieldCentric teleopRequest =
			new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	public static final UnaryOperator<SwerveRequest.FieldCentric> teleopRequestUpdater =
			(SwerveRequest.FieldCentric request) -> {
				double xDesiredRaw = -ControlBoardConstants.mDriverController.getLeftY();
				double yDesiredRaw = -ControlBoardConstants.mDriverController.getLeftX();
				double rotDesiredRaw = -ControlBoardConstants.mDriverController.getRightX();
				double xFancy = getDeadbandedStick(xDesiredRaw);
				double yFancy = getDeadbandedStick(yDesiredRaw);
				double rotFancy = getDeadbandedStick(rotDesiredRaw);

				SmartDashboard.putNumber("Sticks/hypot/raw", Math.hypot(xDesiredRaw, yDesiredRaw));

				// SHOW MODE: one speed, always. The competition logic here had three branches — a
				// shooting cap, a left-stick turbo at 2x, and full speed otherwise — and there is now no
				// path back to any of them. Around a crowd the robot must never be able to reach full
				// speed, so the cap is unconditional rather than something a binding turns on and off.
				// Rotation is capped less hard than translation because the chassis is the only aiming
				// device in show mode.
				LinearVelocity speedLimit =
						DriveConstants.kMaxSpeed.times(ShowConstants.kDriveSpeedFraction.get());
				AngularVelocity rotLimit =
						DriveConstants.kMaxAngularRate.times(ShowConstants.kDriveRotationFraction.get());

				return request.withVelocityX(speedLimit.times(xFancy))
						.withVelocityY(speedLimit.times(yFancy))
						.withRotationalRate(rotLimit.times(rotFancy));
			};

	public static double getDeadbandedStick(double rawValue) {
		if (Math.abs(rawValue) < ControlBoardConstants.stickDeadband) {
			return 0.0;
		} else {
			double unsignedValue = (Math.abs(rawValue) - ControlBoardConstants.stickDeadband)
					/ (1.0 - ControlBoardConstants.stickDeadband);
			return (rawValue > 0 ? unsignedValue : -unsignedValue);
		}
	}


	public static final SwerveRequest.RobotCentric RobotCentricRequest =
			new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);


	
	

	
}