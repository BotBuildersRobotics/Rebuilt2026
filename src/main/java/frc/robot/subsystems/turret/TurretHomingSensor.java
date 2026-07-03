package frc.robot.subsystems.turret;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Robot;
import frc.robot.lib.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

/**
 * Fixed digital homing sensor used as a homing index for the turret.
 *
 * <p>A magnet rides on the moving part of the turret; this sensor is bolted to the frame at the
 * stow position. It is a digital, <em>active-high</em> sensor whose output is wired into a
 * roboRIO <em>analog</em> input, so we read its voltage and threshold it: voltage above
 * {@code triggerVolts} means the line is high and the magnet is over the sensor. When that
 * reading holds long enough to debounce, the turret is physically at its true stow/zero, so the
 * motor's accumulated drift can be reset.
 *
 * <p>Only active on real hardware. In simulation the input is null and every reading reports
 * {@code false}, so nothing can false-trigger a re-zero.
 *
 * <p>If a future sensor is wired active-low, flip {@link #ACTIVE_HIGH}.
 */
public class TurretHomingSensor {

  // Sensor polarity. True: line is high (voltage above threshold) when the magnet is present.
  private static final boolean ACTIVE_HIGH = true;

  // Digital high/low split point (volts). A digital sensor drives ~0V low and ~3.3-5V high,
  // so the midpoint is a safe default; expose it in case the sensor's high level is unusual.
  private static final LoggedTunableNumber triggerVolts =
      new LoggedTunableNumber("Turret/Homing/TriggerVolts", 2.5);
  // How long the reading must stay triggered before we trust it (rejects fly-by noise).
  private static final LoggedTunableNumber debounceSeconds =
      new LoggedTunableNumber("Turret/Homing/DebounceSeconds", 0.10);

  private final AnalogInput input;
  private Debouncer debouncer;
  private double debounceConfigured;

  private boolean triggered = false;
  private boolean atIndex = false;

  public TurretHomingSensor(int analogChannel) {
    input = Robot.isReal() ? new AnalogInput(analogChannel) : null;
    debounceConfigured = debounceSeconds.get();
    debouncer = new Debouncer(debounceConfigured, DebounceType.kRising);
  }

  /**
   * Samples the sensor once. Call exactly once per robot loop (from the subsystem's periodic)
   * so the debouncer advances on a fixed cadence. Logs raw and processed values for tuning.
   */
  public void update() {
    // Rebuild the debouncer only if its tunable changed (rare, and only while tuning).
    if (debounceSeconds.get() != debounceConfigured) {
      debounceConfigured = debounceSeconds.get();
      debouncer = new Debouncer(debounceConfigured, DebounceType.kRising);
    }

    double voltage = getVoltage();
    boolean high = input != null && voltage >= triggerVolts.get();
    triggered = ACTIVE_HIGH ? high : !high;
    atIndex = debouncer.calculate(triggered);

    Logger.recordOutput("Turret/Homing/Present", input != null);
    Logger.recordOutput("Turret/Homing/Voltage", voltage);
    Logger.recordOutput("Turret/Homing/Triggered", triggered);
    Logger.recordOutput("Turret/Homing/AtIndex", atIndex);
  }

  /** Raw analog voltage from the digital sensor's output line; 0 in simulation. */
  public double getVoltage() {
    return input != null ? input.getVoltage() : 0.0;
  }

  /** True the instant the magnet is over the sensor (not debounced). Used to re-arm the latch. */
  public boolean isTriggered() {
    return triggered;
  }

  /** Debounced "turret is at its homing index" — gate a re-zero on this. */
  public boolean isAtIndex() {
    return atIndex;
  }
}
