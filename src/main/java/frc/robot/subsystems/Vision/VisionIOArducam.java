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
import edu.wpi.first.math.geometry.Rotation2d;
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

public class VisionIOArducam implements VisionIO {

	private static final AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

	private final PhotonCamera camera;
	private final PhotonPoseEstimator poseEstimator;
	private double latestTimestampSeconds = 0.0;
	private String name;

	public VisionIOArducam(String cameraName, Transform3d vehicleToCamera) {
		camera = new PhotonCamera(cameraName);
		poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
				vehicleToCamera);
		poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
		name = cameraName;
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		inputs.connected = camera.isConnected();
		inputs.name = this.name;

		PhotonPipelineResult result = camera.getLatestResult();
		if (result.getTimestampSeconds() == latestTimestampSeconds) {
			inputs.isNewMeasurement = false;
			inputs.wasAccepted = false;
			return;
		}
		inputs.isNewMeasurement = true;
		latestTimestampSeconds = result.getTimestampSeconds();
		Optional<EstimatedRobotPose> photonPose = poseEstimator.update(result);

		photonPose.filter(VisionIOArducam::filterPhotonPose);

		photonPose.ifPresentOrElse(
				(pose) -> {
					inputs.latestFieldToRobot = pose.estimatedPose.toPose2d();
					inputs.nTags = pose.targetsUsed.size();
					inputs.latestTimestampSeconds = this.latestTimestampSeconds;
					inputs.averageTagDistanceM = calculateAverageTagDistance(pose);
					inputs.averageTagYaw = calculateAverageTagYaw(pose);
					inputs.wasAccepted = true;
				},
				() -> {
					inputs.wasAccepted = false;
				});

	}

	private static boolean filterPhotonPose(EstimatedRobotPose photonPose) {
		if (photonPose.targetsUsed.size() < 2) {
			return false;
		}

		for (PhotonTrackedTarget target : photonPose.targetsUsed) {
			if (target.getPoseAmbiguity() > 0.15) {
				return false;
			}
		}

		Pose3d pose = photonPose.estimatedPose;
		// check that the pose isn't insane
		if (pose.getZ() > 1 || pose.getZ() < -0.1) {
			return false;
		}

		return true;
	}

	private static double calculateAverageTagDistance(EstimatedRobotPose pose) {
		double distance = 0.0;
		for (PhotonTrackedTarget target : pose.targetsUsed) {
			distance += target.getBestCameraToTarget()
					.getTranslation()
					.getDistance(new Translation3d());
		}
		distance /= pose.targetsUsed.size();

		return distance;
	}

	private static Rotation2d calculateAverageTagYaw(EstimatedRobotPose pose) {
		double yawRad = 0.0;
		for (PhotonTrackedTarget target : pose.targetsUsed) {
			yawRad += target.getBestCameraToTarget().getRotation().getZ();
		}
		yawRad /= pose.targetsUsed.size();
		yawRad -= Math.PI * Math.signum(yawRad);

		return Rotation2d.fromRadians(yawRad);
	}

}