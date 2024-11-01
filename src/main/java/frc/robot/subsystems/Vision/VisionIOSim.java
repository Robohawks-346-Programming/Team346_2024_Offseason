package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {

	private VisionSystemSim visionSim = new VisionSystemSim("main");
	private final PhotonCamera camera;
	private String name;

	private Supplier<SwerveModuleState[]> getModuleStates;
	private Pose2d latestOdometryPose;
	private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition()
	};
	private Timer dtTimer = new Timer();

	public VisionIOSim(String name, Transform3d vehicleToCamera, Supplier<SwerveModuleState[]> getModuleStates) {
		this.getModuleStates = getModuleStates;
		this.name = name;

		visionSim.addAprilTags(VisionConstants.tagLayout);

		var pose = DriveConstants.initialPose;
		latestOdometryPose = new Pose2d(
				pose.getX(),
				pose.getY(),
				Rotation2d.fromRadians(pose.getRotation().getRadians()));

		camera = new PhotonCamera(this.name);
		SimCameraProperties props = new SimCameraProperties();
		props.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
		props.setFPS(25);
		props.setCalibError(0.25, 0.08);

		PhotonCameraSim cameraSim = new PhotonCameraSim(camera, props);
		visionSim.addCamera(cameraSim, vehicleToCamera);

		cameraSim.enableRawStream(true);
		cameraSim.enableProcessedStream(true);

	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		inputs.name = this.name;
		updateOdometry();
		visionSim.update(latestOdometryPose);
		SmartDashboard.putData("PhotonSimField", visionSim.getDebugField());
	}

	private void updateOdometry() {
		SwerveModulePosition[] deltas = new SwerveModulePosition[4];
		SwerveModuleState[] states = getModuleStates.get();

		double dt = dtTimer.get();
		dtTimer.reset();
		dtTimer.start();

		for (int i = 0; i < states.length; i++) {
			deltas[i] = new SwerveModulePosition(
					states[i].speedMetersPerSecond * dt
							- lastModulePositions[i].distanceMeters,
					Rotation2d.fromRadians(
							states[i].angle
									.minus(lastModulePositions[i].angle)
									.getRadians()));
		}

		Twist2d twist = DriveConstants.DRIVE_KINEMATICS.toTwist2d(deltas);
		latestOdometryPose = latestOdometryPose.exp(twist);

		Logger.recordOutput("Vision/GroundTruth", latestOdometryPose);
	}
}