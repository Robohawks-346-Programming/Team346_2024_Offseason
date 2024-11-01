package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

	@AutoLog
	public static class VisionIOInputs {
		public Pose2d latestFieldToRobot = new Pose2d();
		public double averageTagDistanceM = 0.0;
		public Rotation2d averageTagYaw = new Rotation2d();
		public int nTags = 0;
		public double latestTimestampSeconds = 0.0;
		public boolean connected = false;
		public boolean isNewMeasurement = false;
		public String name = "";
		public boolean wasAccepted = false;
	}

	public default void updateInputs(VisionIOInputs inputs) {
	}
}