package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.io.ServoMotorSubsystem.ServoHomingConfig;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.robot.Ports;

public class PivotConstants {
	public static final double kGearing = 46.0;

	public static final Angle kDefenceStow = Units.Degrees.of(0);
	public static final Angle kDeployPosition = Units.Degrees.of( 90);
	public static final Angle kFullStowPosition = Units.Degrees.of(45);
    public static final Angle agitatePosition = Units.Degrees.of(-2);

	public static final Angle kEpsilonThreshold = Units.Degrees.of(2.0);

	
	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Slot0.kP = 50;
		config.Slot0.kD = 0.0;
		config.Slot0.kS = 0.0;
		config.Slot0.kG = 0.0;

		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

		config.MotionMagic.MotionMagicCruiseVelocity = 7.0;
		config.MotionMagic.MotionMagicAcceleration = 15.0;

		config.Voltage.PeakForwardVoltage = 12.0;
		config.Voltage.PeakReverseVoltage = -12.0;

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 40.0;

		config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		//config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		//config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kFullStowPosition.in(Units.Rotations);

		//config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		//config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kDeployPosition.in(Units.Rotations);
		return config;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.mainID = Ports.PIVOT.getDeviceNumber();
		config.mainBus = Ports.PIVOT.getBus();
		config.time = Units.Seconds;
		config.unit = Units.Degrees;
		return config;
	}

	public static MotorIOTalonFX getMotorIO() {
		
		return new MotorIOTalonFX(getIOConfig());
		 
	}

	

	public static ServoHomingConfig getServoHomingConfig() {
		ServoHomingConfig config = new ServoHomingConfig();
		config.kHomePosition = kDeployPosition;
		config.kHomingTimeout = Units.Seconds.of(0.2);
		config.kHomingVoltage = Units.Volts.of(-1.0);
		config.kSetHomedVelocity = Units.DegreesPerSecond.of(1.0);

		return config;
	}
}