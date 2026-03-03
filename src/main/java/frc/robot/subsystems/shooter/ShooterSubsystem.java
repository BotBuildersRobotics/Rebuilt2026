package frc.robot.subsystems.shooter;

import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.io.MotorIO.Setpoint;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lib.io.MotorIOTalonFX;
import frc.robot.lib.io.MotorSubsystem;
import frc.robot.subsystems.turret.ShotCalculator;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem  extends MotorSubsystem<MotorIOTalonFX> {
	
    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();

    private static final LoggedTunableNumber manualShooter = new LoggedTunableNumber("Shooter/Manual");
    private static final LoggedTunableNumber passingSpeed = new LoggedTunableNumber("Shooter/PassingSpeed");

    private boolean manualTune = false;
    
    //RPM = radians / second * 9.5493
    // 10 rads / sec = 95.493 RPM
   // public static final Setpoint COAST = Setpoint.withVelocitySetpoint(AngularVelocity.ofBaseUnits(10, RadiansPerSecond));

	//public static final ShooterSubsystem mInstance = new ShooterSubsystem();

	public static final Setpoint SHOOT = Setpoint.withVoltageSetpoint(ShooterConstants.kShootVoltage);
	
    private ShotCalculator shotCalc;

    private double setpointVal = 0.0;

	public ShooterSubsystem() {
		super(ShooterConstants.getMotorIO(), "Shooter Rollers");
        manualShooter.initDefault(200);
        passingSpeed.initDefault(222);
	}


    public void setShotCalculator(ShotCalculator shotCalc){
        this.shotCalc = shotCalc;
    }

    /*
    
    FOC - adjust kV first, then kP for
    response. */

    private void runVelocity(double velocityRadsPerSec) {
       // this.applySetpoint(Setpoint.withVelocitySetpoint(
       //     AngularVelocity.ofBaseUnits(velocityRadsPerSec, RotationsPerSecond)));

         this.applySetpoint(Setpoint.withVelocityFOCSetpoint(
            AngularVelocity.ofBaseUnits(velocityRadsPerSec, RotationsPerSecond)));
    }

    public Command setManualShooterVelocity(){

        return run(() -> {
            this.manualTune = true;
            
           // this.applySetpoint(Setpoint.withVelocitySetpoint(
           // AngularVelocity.ofBaseUnits(manualShooter.get(), RotationsPerSecond)));

              this.applySetpoint(Setpoint.withVelocityFOCSetpoint(
            AngularVelocity.ofBaseUnits(manualShooter.get(), RotationsPerSecond)));

        });

    }

     public void periodic() {
        super.periodic();
         SmartDashboard.putNumber("Shooter/Speed",this.getVelocity().baseUnitMagnitude());
         SmartDashboard.putBoolean("Shooter/Manual", manualTune);
        SmartDashboard.putNumber("Shooter/VelocitySetPoint",setpointVal );

         // AdvantageKit structured logging for replay
         Logger.recordOutput("Shooter/VelocityRPS", this.getVelocity().baseUnitMagnitude());
          Logger.recordOutput("Shooter/VelocitySetPoint",setpointVal );
         Logger.recordOutput("Shooter/ManualTune", manualTune);
     }

    public Command resetAutoMap(){
        return Commands.runOnce(() -> this.manualTune = false);
    }

    private void stop() {
        this.applySetpoint(IDLE);
    }

    public Command runTrackTargetActiveShootingCommand() {
        return run(
            () -> {
                if(!this.manualTune){
                    var params = shotCalc.getParameters();
                    setpointVal = params.flywheelSpeed();
                    runVelocity( setpointVal);
                }
            
            });
    }

    public Command runFixedCommand(DoubleSupplier velocity) {
        return run(
            () -> {
            
                runVelocity(velocity.getAsDouble());
            
            });
    }

    public Command runPassingCommand() {
        return run(() -> runVelocity(passingSpeed.get()));
    }

    public Command stopCommand()
    {
        return runOnce(this::stop);
    }

    private final VoltageOut m_voltReq = new VoltageOut(0.0);
    private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(8), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> ((MotorIOTalonFX)this.getMotorIO()).setMotorControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

}