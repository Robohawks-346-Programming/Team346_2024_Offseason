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
	private volatile Matrix<N3, N1> stdevs;
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

		if (estimatedPose.get().targetsUsed.size() == 1) {
			double ambiguity = estimatedPose.get().targetsUsed.get(0).getPoseAmbiguity();
			if (ambiguity > VisionConstants.singleTagAmbiguityCutoff || ambiguity == -1) {
				return;
			}
		}

		Pose3d pose = estimatedPose.get().estimatedPose;
		if (pose.getZ() > 1 || pose.getZ() < -0.1) {
			return;
		}

		calculatedPose = estimatedPose.get().estimatedPose;
		timestamp = estimatedPose.get().timestampSeconds;
		stdevs = cameraUncertainty(calculateAverageTagDistance(estimatedPose.get()),
				estimatedPose.get().targetsUsed.size());
		hasNewPose = true;
		// SmartDashboard.putBoolean(name+"Works", true);

	}

	public boolean hasNewObservation() {
		return hasNewPose;
	}

	public void recordVisionObservation() {
		m_drive.addVisionMeasurement(calculatedPose.toPose2d(), timestamp, stdevs);
		hasNewPose = false;
	}

	private Matrix<N3, N1> cameraUncertainty(double averageTagDistanceM, int nTags) {
		/*
		 * On this year's field, AprilTags are arranged into rough 'corridors' between
		 * the stage and
		 * speaker, and a central 'desert,' where few tags can be found. It follows that
		 * we should
		 * determine the variance of our camera measurements based on that.
		 */
		if (nTags < 2) {
			return VisionConstants.singleTagUncertainty;
		} else if (averageTagDistanceM < 6.0 && this.robotInMidField()) {
			return VisionConstants.lowCameraUncertainty;
		} else {
			return VisionConstants.highCameraUncertainty;
		}
	}

	private boolean robotInMidField() {
		return m_drive.getPose().getX() > VisionConstants.midfieldLowThresholdM
				&& m_drive.getPose().getX() < VisionConstants.midfieldHighThresholdM;
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

}