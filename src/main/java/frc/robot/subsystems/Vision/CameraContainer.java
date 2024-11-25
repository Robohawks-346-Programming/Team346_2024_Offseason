package frc.robot.subsystems.Vision;

import java.util.List;

public interface CameraContainer {
	public List<Camera> getCameras();

	public void update();
}
