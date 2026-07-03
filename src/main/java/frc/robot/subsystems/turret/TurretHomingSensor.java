package frc.robot.subsystems.turret;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Robot;
import frc.robot.lib.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

/**
 * Fixed analog absolute encoder used as a homing index for the turret.
 *
 * <p>A magnet rides on the moving part of the turret; this encoder is bolted to the frame at the
 * stow position. As the magnet sweeps past, the encoder's analog output rises to a repeatable
 * value. We treat that reading as a <em>digital</em> index: when the voltage sits inside a
 * calibrated window (and stays there long enough to debounce), the turret is physically at its
 * true stow/zero, so the motor's accumulated drift can be reset.
 *
 * <p>Only active on real hardware. In simulation the input is null and every reading reports
 * "not present", so nothing can false-trigger a re-zero.
 *
 * <p>Calibration: park the turret at stow, watch {@code Turret/Homing/Voltage} in AdvantageScope,
 * and set {@code WindowLowVolts}/{@code WindowHighVolts} to bracket the value you see (with a
 * little margin on each side).
 */
public class TurretHomingSensor {

  // Voltage window that corresponds to the magnet sitting over the sensor at stow.
  private static final LoggedTunableNumber windowLowVolts =
      new LoggedTunableNumber("Turret/Homing/WindowLowVolts", 2.3);
  private static final LoggedTunableNumber windowHighVolts =
      new LoggedTunableNumber("Turret/Homing/WindowHighVolts", 2.7);
  // How long the reading must stay in-window before we trust it (rejects fly-by noise).
  private static final LoggedTunableNumber debounceSeconds =
      new LoggedTunableNumber("Turret/Homing/DebounceSeconds", 0.10);

  private final AnalogInput input;
  private Debouncer debouncer;
  private double debounceConfigured;

  private boolean inWindow = false;
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
    inWindow = input != null
        && voltage >= windowLowVolts.get()
        && voltage <= windowHighVolts.get();
    atIndex = debouncer.calculate(inWindow);

    Logger.recordOutput("Turret/Homing/Present", input != null);
    Logger.recordOutput("Turret/Homing/Voltage", voltage);
    Logger.recordOutput("Turret/Homing/InWindow", inWindow);
    Logger.recordOutput("Turret/Homing/AtIndex", atIndex);
  }

  /** Raw analog voltage; 0 in simulation. */
  public double getVoltage() {
    return input != null ? input.getVoltage() : 0.0;
  }

  /** True the instant the magnet is over the sensor (not debounced). Used to re-arm the latch. */
  public boolean isInWindow() {
    return inWindow;
  }

  /** Debounced "turret is at its homing index" — gate a re-zero on this. */
  public boolean isAtIndex() {
    return atIndex;
  }
}
