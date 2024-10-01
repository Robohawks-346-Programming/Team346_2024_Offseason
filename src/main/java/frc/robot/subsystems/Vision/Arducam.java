package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Arducam {

	private static final AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

	private final PhotonCamera camera;
	private final PhotonPoseEstimator poseEstimator;

	private volatile boolean hasNewPose = false;
	private volatile Pose3d calculatedPose = new Pose3d();
	private volatile Pose3d intermediatePose = new Pose3d();
	private volatile double timestamp = 1;
	private String name;

	private CommandSwerveDrivetrain m_drive;

	public Arducam(String cameraName, Transform3d vehicleToCamera, CommandSwerveDrivetrain drive) {
		camera = new PhotonCamera(cameraName);
		poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
				vehicleToCamera);
		poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
		name = cameraName;
		m_drive = drive;
	}

	public void periodic() {
		SmartDashboard.putBoolean(name, camera.isConnected());
		// SmartDashboard.putNumber(name, count);
		if (!camera.isConnected())
			return;

		PhotonPipelineResult result = camera.getLatestResult();
		Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
		if (estimatedPose.isEmpty())
			return;
		EstimatedRobotPose estimation = estimatedPose.get();
		if (estimation.timestampSeconds == timestamp)
			return;
		if (estimation.targetsUsed.size() == 1)
			return;

		intermediatePose = estimation.estimatedPose;

		if (intermediatePose.getZ() > 1 || intermediatePose.getZ() < -0.1) {
			return;
		}

		for (PhotonTrackedTarget target : estimation.targetsUsed) {
			if (target.getPoseAmbiguity() > 0.18) {
				return;
			}
		}

		// if (Math.abs(Math.toDegrees(intermediatePose.getX()) -
		// RobotContainer.drivetrain.poseEstimator.getEstimatedPosition().getX()) <
		// 0.05){
		// return;
		// }

		// if (Math.abs(Math.toDegrees(intermediatePose.getY()) -
		// RobotContainer.drivetrain.poseEstimator.getEstimatedPosition().getY()) <
		// 0.05){
		// return;
		// }

		// if (Math.abs(intermediatePose.getRotation().getAngle() -
		// RobotContainer.drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees())
		// < 5){
		// return;
		// }

		calculatedPose = estimation.estimatedPose;
		timestamp = estimation.timestampSeconds;
		hasNewPose = true;
		// SmartDashboard.putBoolean(name+"Works", true);

	}

	public boolean hasNewObservation() {
		return hasNewPose;
	}

	public void recordVisionObservation() {
		m_drive.addVisionMeasurement(calculatedPose.toPose2d(), timestamp);
		hasNewPose = false;
	}

	public double getSpeakerFull() {
		if (name == "BR" || name == "BL") {
			if (DriverStation.getAlliance().isPresent()
					&& DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
				for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
					if (target.getFiducialId() == 4) {
						return target.getBestCameraToTarget().getX();
					}
				}
				return -1.0;
			} else if (DriverStation.getAlliance().isPresent()
					&& DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
				for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
					if (target.getFiducialId() == 7) {
						return target.getBestCameraToTarget().getX();
					}
				}
				return -1.0;
			} else {
				return -1;
			}
		} else {
			return -1.0;
		}
	}

}