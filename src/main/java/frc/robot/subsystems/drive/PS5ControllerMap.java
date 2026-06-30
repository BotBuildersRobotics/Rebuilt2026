package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * {@link ControllerMap} backed by a {@link CommandPS5Controller}.
 *
 * <p>Buttons are mapped to the equivalent physical position on the DualSense:
 * a=cross (bottom), b=circle (right), x=square (left), y=triangle (top),
 * back=create, start=options, bumpers=L1/R1, triggers=L2/R2, stick presses=L3/R3.
 */
public class PS5ControllerMap implements ControllerMap {

	private final CommandPS5Controller c;

	public PS5ControllerMap(int port) {
		c = new CommandPS5Controller(port);
	}

	@Override
	public Trigger a() {
		return c.cross();
	}

	@Override
	public Trigger b() {
		return c.circle();
	}

	@Override
	public Trigger x() {
		return c.square();
	}

	@Override
	public Trigger y() {
		return c.triangle();
	}

	@Override
	public Trigger back() {
		return c.create();
	}

	@Override
	public Trigger start() {
		return c.options();
	}

	@Override
	public Trigger leftBumper() {
		return c.L1();
	}

	@Override
	public Trigger rightBumper() {
		return c.R1();
	}

	@Override
	public Trigger leftTrigger() {
		return c.L2();
	}

	@Override
	public Trigger rightTrigger() {
		return c.R2();
	}

	@Override
	public Trigger leftStick() {
		return c.L3();
	}

	@Override
	public Trigger rightStick() {
		return c.R3();
	}

	@Override
	public Trigger povUp() {
		return c.povUp();
	}

	@Override
	public Trigger povDown() {
		return c.povDown();
	}

	@Override
	public double getLeftX() {
		return c.getLeftX();
	}

	@Override
	public double getLeftY() {
		return c.getLeftY();
	}

	@Override
	public double getRightX() {
		return c.getRightX();
	}

	@Override
	public GenericHID getHID() {
		return c.getHID();
	}
}