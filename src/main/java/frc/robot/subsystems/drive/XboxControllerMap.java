package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** {@link ControllerMap} backed by a {@link CommandXboxController}. */
public class XboxControllerMap implements ControllerMap {

	private final CommandXboxController c;

	public XboxControllerMap(int port) {
		c = new CommandXboxController(port);
	}

	@Override
	public Trigger a() {
		return c.a();
	}

	@Override
	public Trigger b() {
		return c.b();
	}

	@Override
	public Trigger x() {
		return c.x();
	}

	@Override
	public Trigger y() {
		return c.y();
	}

	@Override
	public Trigger back() {
		return c.back();
	}

	@Override
	public Trigger start() {
		return c.start();
	}

	@Override
	public Trigger leftBumper() {
		return c.leftBumper();
	}

	@Override
	public Trigger rightBumper() {
		return c.rightBumper();
	}

	@Override
	public Trigger leftTrigger() {
		return c.leftTrigger();
	}

	@Override
	public Trigger rightTrigger() {
		return c.rightTrigger();
	}

	@Override
	public Trigger leftStick() {
		return c.leftStick();
	}

	@Override
	public Trigger rightStick() {
		return c.rightStick();
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