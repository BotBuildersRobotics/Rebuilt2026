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

import frc.robot.generated.TunerConstants;
import frc.robot.lib.LoggedTunableNumber;

import java.util.function.UnaryOperator;

public class DriveConstants {
	public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
	public static final LinearVelocity kMaxSpeedFAST = kMaxSpeed.times(2.0);

	public static final LinearAcceleration kMaxAcceleration = Units.MetersPerSecondPerSecond.of(12.0);
	public static final AngularVelocity kMaxAngularRate = Units.RadiansPerSecond.of(2.75 * Math.PI);
	public static final AngularVelocity kMaxAngularRateFAST = kMaxAngularRate.times(2.0);
	public static final AngularAcceleration kMaxAngularAcceleration =
			kMaxAngularRate.div(0.1).per(Units.Second);

	// Fraction of max speed allowed while shooting (0.0 to 1.0)
	private static final LoggedTunableNumber shootingSpeedFraction =
			new LoggedTunableNumber("Drive/ShootingSpeedFraction", 0.3);
	private static boolean shootingSpeedLimited = false;

	public static void setShootingSpeedLimited(boolean limited) {
		shootingSpeedLimited = limited;
	}

	public static boolean isShootingSpeedLimited() {
		return shootingSpeedLimited;
	}


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

				LinearVelocity speedLimit;
				AngularVelocity rotLimit;

				if (shootingSpeedLimited) {
					double fraction = shootingSpeedFraction.get();
					speedLimit = DriveConstants.kMaxSpeed.times(fraction);
					rotLimit = DriveConstants.kMaxAngularRate.times(fraction);
				} else if (ControlBoardConstants.mDriverController.leftStick().getAsBoolean()) {
					speedLimit = DriveConstants.kMaxSpeedFAST;
					rotLimit = DriveConstants.kMaxAngularRateFAST;
				} else {
					speedLimit = DriveConstants.kMaxSpeed;
					rotLimit = DriveConstants.kMaxAngularRate;
				}

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