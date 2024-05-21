package frc.robot.OI;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriverControllerXbox implements DriverController {

  CommandXboxController m_controller;

  public DriverControllerXbox(int xboxControllerPort) {
    m_controller = new CommandXboxController(xboxControllerPort);
  }

  @Override
  public double getDriveForward() {
    double val = addDeadzoneScaled(m_controller.getLeftY(), 0.03);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveLeft() {
    double val = addDeadzoneScaled(m_controller.getLeftX(), 0.03);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveRotation() {
    double val = addDeadzoneScaled(m_controller.getRightX(), 0.03);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  public double addDeadzoneScaled(double input, double deadzone) {
    if (Math.abs(input) < deadzone) {
        return 0;
    } else {
        return (input - Math.signum(input) * deadzone) / (1 - deadzone);
    }
}

}