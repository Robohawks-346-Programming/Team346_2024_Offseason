package frc.robot.subsystems.NotePath;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NotePathConstants;

public class NotePath extends SubsystemBase {
	private NotePathIO io;
	private NotePathIOInputsAutoLogged inputs = new NotePathIOInputsAutoLogged();

	private State state = State.IDLE;

	public NotePath(NotePathIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("notePath", inputs);

		switch (state) {
			case IDLE:
				idle();
				break;
			case INTAKE:
				if (!hasNote()) {
					intake();
				} else {
					idle();
				}
				break;
			case OUTAKE:
				outake();
				break;
			case AMP:
				amp();
				break;
			case REV:
				rev();
				break;
			case SPEAKER:
				speaker();
				break;
		}

		Logger.recordOutput("notePath/intake", inputs.intakeRollerSpeed != 0.0);
		Logger.recordOutput("notePath/centering", inputs.centeringMotorSpeed != 0.0);
		Logger.recordOutput("notePath/ampRoller", inputs.ampRollerSpeed != 0.0);
		Logger.recordOutput("notePath/feederRoller", inputs.feederRollerSpeed != 0.0);
		Logger.recordOutput("notePath/topShooter", inputs.topRollerSpeed != 0.0);
		Logger.recordOutput("notePath/bottomShooter", inputs.bottomRollerSpeed != 0.0);

		Logger.recordOutput("notePath/state", state.toString());
	}

	public boolean hasNote() {
		return inputs.noteSensed;
	}

	public void setState(State state) {
		this.state = state;
	}

	private void idle() {
		io.setAmpRollerSpeed(0);
		io.setFeederRollerSpeed(0);
		io.setIntakeSpeed(0);
		io.setShooterSpeed(0);
	}

	private void intake() {
		io.setAmpRollerSpeed(NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_1);
		io.setFeederRollerSpeed(NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_1);
		io.setIntakeSpeed(NotePathConstants.INTAKE_MOTOR_SPEED);
		io.setShooterSpeed(0);
	}

	private void outake() {
		io.setAmpRollerSpeed(-NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_1);
		io.setFeederRollerSpeed(-NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_1);
		io.setIntakeSpeed(-NotePathConstants.INTAKE_MOTOR_SPEED);
		io.setShooterSpeed(0);
	}

	private void speaker() {
		io.setAmpRollerSpeed(NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_2);
		io.setFeederRollerSpeed(NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_2);
		io.setIntakeSpeed(0);
		io.setShooterSpeed(100);
	}

	private void amp() {
		io.setAmpRollerSpeed(NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_2);
		io.setFeederRollerSpeed(-NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_1);
		io.setIntakeSpeed(0);
		io.setShooterSpeed(-10);
	}

	private void rev() {
		io.setAmpRollerSpeed(0);
		io.setFeederRollerSpeed(0);
		io.setIntakeSpeed(0);
		io.setShooterSpeed(100);
	}

	public enum State {
		IDLE, // brake
		INTAKE, // intake
		OUTAKE, // outake
		AMP, // Amp eject
		REV, // Revs Shooter
		SPEAKER // Speaker eject
	}
}
