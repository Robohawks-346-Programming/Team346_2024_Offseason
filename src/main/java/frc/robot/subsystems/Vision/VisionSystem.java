package frc.robot.subsystems.Vision;

import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSystem extends SubsystemBase {
	private CameraContainer container;

	private Consumer<CameraMeasurement> cameraConsumer = (c) -> {
	};

	public VisionSystem(CameraContainer container) {
		this.container = container;
	}

	@Override
	public void periodic() {
		container.update();
		for (Camera camera : container.getCameras()) {
			if (camera.hasNewMeasurement()) {
				cameraConsumer.accept(camera.getLatestMeasurement());
			}
		}
	}

	public static record CameraMeasurement(
			Pose2d pose, double timestamp, Matrix<N3, N1> variance) {
	}

}
