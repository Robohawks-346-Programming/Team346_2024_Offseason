package frc.robot.subsystems.NotePath;

import org.littletonrobotics.junction.AutoLog;

public interface NotePathIO {

	@AutoLog
	public static class NotePathIOInputs {
		public double intakeRollerSpeed = 0.0;
		public double centeringMotorSpeed = 0.0;
		public double feederRollerSpeed = 0.0;
		public double ampRollerSpeed = 0.0;
		public double topRollerSpeed = 0.0;
		public double bottomRollerSpeed = 0.0;

		public boolean noteSensed = false;
	}

	public default void updateInputs(NotePathIOInputs inputs) {
	}

	public default void setIntakeSpeed(double speed) {
	}

	public default void setAmpRollerSpeed(double speed) {
	}

	public default void setFeederRollerSpeed(double speed) {
	}

	public default void setShooterSpeed(double speed) {
	}
}