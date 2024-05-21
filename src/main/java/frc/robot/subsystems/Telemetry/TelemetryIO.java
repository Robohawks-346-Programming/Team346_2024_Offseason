package frc.robot.subsystems.Telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface TelemetryIO {
    @AutoLog
    public static class TelemetryIOInputs {
        public Pose3d pose3d = new Pose3d();

        public Pose2d pose2d = new Pose2d();

        public SwerveModuleState[] moduleStates =
                new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
                };
        public double accelerationX = 0.0;
        public double accelerationY = 0.0;

        public double robotRotation;
        public double robotRotationVelocity;
    }

    public default void updateInputs(TelemetryIOInputs inputs) {}

    public default void setRobotPose(Pose3d pose) {}

    public default void setRobotPose(Pose2d pose) {}

    public default void setSwerveModuleStates(SwerveModuleState[] moduleStates) {}

    public default void setRobotRotation(double rotation) {}

    public default void setRobotRotationVelocity(double rotationVelocity) {}
}