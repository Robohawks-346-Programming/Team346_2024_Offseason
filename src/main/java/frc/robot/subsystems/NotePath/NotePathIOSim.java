package frc.robot.subsystems.NotePath;

import org.opencv.features2d.Feature2D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NotePathIOSim implements NotePathIO {
	/*
	 * Simulating the wheels directly doesn't make much sense here, so we can
	 * instead use SmartDashboard to puppeteer a note through the indexer
	 */
	private boolean noteInIndexer = false;

	private double intakeSpeed = 0.0;
	private double centeringSpeed = 0.0;
	private double ampSpeed = 0.0;
	private double feederSpeed = 0.0;
	private double topSpeed = 0.0;
	private double bottomSpeed = 0.0;

	public NotePathIOSim() {
		SmartDashboard.putBoolean("noteInIndexer", false);
	}

	@Override
	public void updateInputs(NotePathIOInputs inputs) {
		noteInIndexer = SmartDashboard.getBoolean("noteInIndexer", false);

		inputs.intakeRollerSpeed = intakeSpeed;
		inputs.centeringMotorSpeed = centeringSpeed;
		inputs.ampRollerSpeed = ampSpeed;
		inputs.feederRollerSpeed = feederSpeed;
		inputs.topRollerSpeed = topSpeed;
		inputs.bottomRollerSpeed = bottomSpeed;

		inputs.noteSensed = noteInIndexer;
	}

	@Override
	public void setIntakeSpeed(double speed) {
		intakeSpeed = MathUtil.clamp(speed, -1.0, 1.0);
		centeringSpeed = MathUtil.clamp(speed, -1.0, 1.0);
	}

	@Override
	public void setAmpRollerSpeed(double speed) {
		ampSpeed = MathUtil.clamp(speed, -1.0, 1.0);
	}

	@Override
	public void setFeederRollerSpeed(double speed) {
		feederSpeed = MathUtil.clamp(speed, -1.0, 1.0);
	}

	@Override
	public void setShooterSpeed(double speed) {
		topSpeed = MathUtil.clamp(speed, -1.0, 1.0);
		bottomSpeed = MathUtil.clamp(speed, -1.0, 1.0);
	}
}