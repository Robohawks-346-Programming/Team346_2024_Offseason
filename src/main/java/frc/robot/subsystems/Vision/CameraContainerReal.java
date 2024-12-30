package frc.robot.subsystems.Vision;

import frc.robot.subsystems.Vision.CameraIOArducam;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants.VisionConstants.CameraParams;

public class CameraContainerReal implements CameraContainer {
	private List<Camera> cameras = new ArrayList<>();

	public CameraContainerReal(List<CameraParams> params) {
		for (CameraParams param : params) {
			cameras.add(new Camera(param, CameraIOArducam.fromRealCameraParams(param)));
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
