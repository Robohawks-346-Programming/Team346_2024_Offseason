package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraParams;

public class CameraContainerReplay implements CameraContainer {
	private List<Camera> cameras = new ArrayList<>();

	public CameraContainerReplay(List<CameraParams> params) {
		for (CameraParams param : params) {
			cameras.add(new Camera(param, new CameraIO() {
			}));
		}
	}

	public List<Camera> getCameras() {
		return cameras;
	}

	public void update() {
		for (Camera camera : cameras) {
			camera.update();
		}
	}
}