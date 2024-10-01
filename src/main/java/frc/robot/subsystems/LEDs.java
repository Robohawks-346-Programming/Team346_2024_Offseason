package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision.Vision;

public class LEDs extends SubsystemBase {

	private AddressableLED led;
	private AddressableLEDBuffer ledBuffer;

	// Rainbow
	private int rainbowFirstPixelHue = 0;

	// Other
	private final Timer flashTimer = new Timer();

	private NotePath m_notePath;
	private Vision m_vision;

	public LEDs(NotePath notePath, Vision vision) {
		led = new AddressableLED(Constants.LEDConstants.LED_PORT);
		ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.NUMBER_OF_LEDS);
		led.setLength(ledBuffer.getLength());
		led.setData(ledBuffer);
		led.start();
		m_notePath = notePath;
		m_vision = vision;
	}

	@Override
	public void periodic() {
		if (DriverStation.isDisabled()) {
			rainbow(ledBuffer);
		} else {
			if (m_notePath.getLaserBreak()) {
				setColor(ledBuffer, Color.kGreen);
			} else if (m_vision.getNoteVisible()) {
				setColor(ledBuffer, Color.kOrangeRed);
			} else {
				setColor(ledBuffer, Color.kRed);
			}
		}
		led.setData(ledBuffer);
	}

	private void setRGB(AddressableLEDBuffer buffer, int r, int g, int b) {
		for (int i = 0; i < buffer.getLength(); i++) {
			buffer.setRGB(i, r, g, b);
		}
	}

	private void setHSV(AddressableLEDBuffer buffer, int h, int s, int v) {
		for (int i = 0; i < buffer.getLength(); i++) {
			buffer.setHSV(i, h, s, v);
		}
	}

	private void setLED(AddressableLEDBuffer buffer, Color color) {
		for (int i = 0; i < buffer.getLength(); i++) {
			buffer.setLED(i, color);
		}
	}

	private void clear(AddressableLEDBuffer buffer) {
		for (int i = 0; i < buffer.getLength(); i++) {
			setRGB(buffer, 0, 0, 0);
		}
	}

	private void rainbow(AddressableLEDBuffer buffer) {
		for (int i = 0; i < buffer.getLength(); i++) {
			int hue = (rainbowFirstPixelHue + 90 + (i * 180 / buffer.getLength())) % 180;
			setHSV(buffer, hue, 255, 127);
		}
		rainbowFirstPixelHue = (rainbowFirstPixelHue + 1) % 180;
	}

	private void flashColor(Color color, AddressableLEDBuffer buffer) {
		if ((flashTimer.get() * 10 % 10) % 5 < 3) {
			return;
		}
		for (int i = buffer.getLength() / 2; i < 3 * buffer.getLength() / 4; i++) {
			setLED(buffer, color);
		}
	}

	private void setColor(AddressableLEDBuffer buffer, Color color) {
		for (int i = 0; i < buffer.getLength(); i++) {
			setLED(buffer, color);
		}
	}
}