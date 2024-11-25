package frc.robot.subsystems.Vision;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SteadyStateKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants.VisionConstants.CameraParams;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.TestDrive.Drive;
import frc.robot.subsystems.TestDrive.GyroIOInputsAutoLogged;
import frc.robot.subsystems.Vision.CameraIO.CameraIOInputs;
import frc.robot.subsystems.Vision.VisionSystem.CameraMeasurement;

public class Camera {

	private final CameraIO io;
	private final CameraIOInputsAutoLogged cameraInputs;
	private double count;
	private String name;

	public Camera(CameraParams params, CameraIO io) {

		this.io = io;
		cameraInputs = new CameraIOInputsAutoLogged();

	}

	public void update() {
		io.updateInputs(cameraInputs);
		Logger.processInputs("Vision/" + name, cameraInputs);

	}

	public boolean hasNewMeasurement() {
		return cameraInputs.wasAccepted;
	}

	private Matrix<N3, N1> cameraUncertainty() {
		if (cameraInputs.averageTagDistanceM < VisionConstants.skewCutoffDistance) {
			return DriverStation.isTeleop()
					? VisionConstants.teleopCameraUncertainty
					: VisionConstants.lowCameraUncertainty;
		} else if (cameraInputs.averageTagDistanceM < VisionConstants.lowUncertaintyCutoffDistance) {
			return Math.abs(cameraInputs.averageTagYaw.getDegrees()) < VisionConstants.skewCutoffRotation
					? VisionConstants.lowCameraUncertainty
					: VisionConstants.highCameraUncertainty;
		} else {
			return VisionConstants.highCameraUncertainty;
		}
	}

	public CameraMeasurement getLatestMeasurement() {
		return new CameraMeasurement(
				cameraInputs.latestFieldToRobot, cameraInputs.latestTimestampSeconds, cameraUncertainty());
	}

}
