package frc.robot.OI;

import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControllerXbox {

	CommandXboxController m_controller;
	public Trigger rightBumper, x, y, rightTrigger, leftTrigger;

	public DriverControllerXbox(int xboxControllerPort) {
		m_controller = new CommandXboxController(xboxControllerPort);
		rightBumper = m_controller.rightBumper();
		leftTrigger = m_controller.leftTrigger();
		x = m_controller.x();
		y = m_controller.y();
		rightTrigger = m_controller.rightTrigger();
	}

	public double getDriveForward() {
		double val = addDeadzoneScaled(m_controller.getLeftY(), 0.1);
		return -Math.signum(val) * Math.pow(val, 2);
	}

	public double getDriveLeft() {
		double val = addDeadzoneScaled(m_controller.getLeftX(), 0.1);
		return -Math.signum(val) * Math.pow(val, 2);
	}

	public double getDriveRotation() {
		double val = addDeadzoneScaled(m_controller.getRightX(), 0.1);
		return -Math.signum(val) * Math.pow(val, 2);
	}

	public double addDeadzoneScaled(double input, double deadzone) {
		if (Math.abs(input) < deadzone) {
			return 0;
		} else {
			// return (input - Math.signum(input) * deadzone) / (1 - deadzone);
			return input;
		}
	}

}