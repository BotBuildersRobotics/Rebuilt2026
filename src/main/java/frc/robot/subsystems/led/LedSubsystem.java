package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.subsystems.shuffla.ShufflaSubsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import static edu.wpi.first.units.Units.*;

public class LedSubsystem extends SubsystemBase {

    private static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
    private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();
    private static final RGBWColor kBlue = RGBWColor.fromHex("#00001600").orElseThrow();

    private final CANdle candle = new CANdle(Ports.CANDLE.getDeviceNumber(), Ports.CANDLE.getBus());


    public static final LedSubsystem mInstance = new LedSubsystem();

    public LedSubsystem(){
         var cfg = new CANdleConfiguration();
        /* set the LED strip type and brightness */
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        candle.getConfigurator().apply(cfg);

        /* clear all previous animations */
        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }
    }

    

    public Command setGreen(){
       return  Commands.runOnce( () -> candle.setControl(new SolidColor(0, 3).withColor(kGreen)));
    }

    public Command setRed(){
       return  Commands.runOnce( () -> candle.setControl(new SolidColor(0, 3).withColor(kRed)));
    }

    public Command setBlue(){
       return  Commands.runOnce( () -> candle.setControl(new SolidColor(0, 3).withColor(kRed)));
    }
}