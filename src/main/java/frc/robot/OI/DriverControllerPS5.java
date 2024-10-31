package frc.robot.OI;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControllerPS5 {

	CommandPS5Controller m_controller;
	public Trigger rightBumper, cross, circle, rightTrigger, leftTrigger, leftBumper, square;

	public DriverControllerPS5(int ps5ControllerPort) {
		m_controller = new CommandPS5Controller(ps5ControllerPort);
		rightBumper = m_controller.R1();
		leftBumper = m_controller.L1();
		leftTrigger = m_controller.L2();
		cross = m_controller.cross();
		circle = m_controller.circle();
		rightTrigger = m_controller.R2();
		square = m_controller.square();
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