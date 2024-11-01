package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SteadyStateKalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.TestDrive.Drive;
import frc.robot.subsystems.TestDrive.GyroIOInputsAutoLogged;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;

public class Vision extends SubsystemBase {

	private VisionSystemSim visionSim = new VisionSystemSim("main");
	private final VisionIO[] cameras = new VisionIO[2];
	private final VisionIOInputsAutoLogged[] cameraInputs = new VisionIOInputsAutoLogged[2];
	private double count;
	private CommandSwerveDrivetrain m_drive;
	private Matrix<N3, N1> stdevs;

	public Vision(CommandSwerveDrivetrain drive, VisionIO camera1, VisionIO camera2) {
		m_drive = drive;
		cameras[0] = camera1;
		cameras[1] = camera2;
		cameraInputs[0] = new VisionIOInputsAutoLogged();
		cameraInputs[1] = new VisionIOInputsAutoLogged();
		if (Robot.isSimulation()) {
			visionSim.addAprilTags(VisionConstants.tagLayout);
			setupSimCameras();
		}
	}

	@Override
	public void periodic() {

		m_drive.getPose();
		SmartDashboard.putNumber("Vision Count", count);
		if (Robot.isSimulation()) {
			visionSim.update(m_drive.getPose());
		}

		for (int i = 0; i < cameras.length; i++) {
			cameras[i].updateInputs(cameraInputs[i]);
		}
		for (int i = 0; i < cameras.length; i++) {
			Logger.processInputs("Vision/" + cameraInputs[i].name, cameraInputs[i]);
		}
		for (int i = 0; i < cameras.length; i++) {
			if (cameraInputs[i].wasAccepted) {
				m_drive.addVisionMeasurement(cameraInputs[i].latestFieldToRobot, cameraInputs[i].latestTimestampSeconds,
						cameraUncertainty(cameraInputs[i]));
				count++;
			}
		}

	}

	private Matrix<N3, N1> cameraUncertainty(VisionIOInputs inputs) {
		if (inputs.averageTagDistanceM < VisionConstants.skewCutoffDistance) {
			return DriverStation.isTeleop()
					? VisionConstants.teleopCameraUncertainty
					: VisionConstants.lowCameraUncertainty;
		} else if (inputs.averageTagDistanceM < VisionConstants.lowUncertaintyCutoffDistance) {
			return Math.abs(inputs.averageTagYaw.getDegrees()) < VisionConstants.skewCutoffRotation
					? VisionConstants.lowCameraUncertainty
					: VisionConstants.highCameraUncertainty;
		} else {
			return VisionConstants.highCameraUncertainty;
		}
	}

	private void setupSimCameras() {
		PhotonCamera camera1 = new PhotonCamera(cameraInputs[0].name);
		PhotonCamera camera2 = new PhotonCamera(cameraInputs[1].name);
		SimCameraProperties props = new SimCameraProperties();
		props.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
		props.setFPS(25);
		props.setCalibError(0.25, 0.08);

		PhotonCameraSim cameraSim1 = new PhotonCameraSim(camera1, props);
		PhotonCameraSim cameraSim2 = new PhotonCameraSim(camera2, props);
		visionSim.addCamera(cameraSim1, VisionConstants.vehicleToCameras[2]);
		visionSim.addCamera(cameraSim2, VisionConstants.vehicleToCameras[3]);
	}

}