package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Brand-agnostic view of a command controller.
 *
 * <p>Method names mirror the Xbox layout (the layout this codebase was originally written against),
 * but each method is mapped to the equivalent physical button/axis on whatever controller is plugged
 * in. This lets all binding code reference {@code ControllerMap} instead of a concrete
 * {@code CommandXboxController}/{@code CommandPS5Controller}, so the controller type can be swapped by
 * flipping a single flag in {@link ControlBoardConstants}.
 */
public interface ControllerMap {

	// Face buttons (by Xbox name / physical position):
	//   a = bottom, b = right, x = left, y = top
	Trigger a();

	Trigger b();

	Trigger x();

	Trigger y();

	// Menu buttons
	Trigger back();

	Trigger start();

	// Bumpers
	Trigger leftBumper();

	Trigger rightBumper();

	// Triggers (analog, exposed as a Trigger past the default threshold)
	Trigger leftTrigger();

	Trigger rightTrigger();

	// Stick presses
	Trigger leftStick();

	Trigger rightStick();

	// D-pad
	Trigger povUp();

	Trigger povDown();

	// Analog stick axes
	double getLeftX();

	double getLeftY();

	double getRightX();

	/** Underlying HID, used for rumble. */
	GenericHID getHID();
}