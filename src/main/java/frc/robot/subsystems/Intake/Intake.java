package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	CANSparkMax intakeMotor, centeringMotor;

	public Intake() {
		intakeMotor = new CANSparkMax(Constants.NotePathConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
		centeringMotor = new CANSparkMax(Constants.NotePathConstants.CENTERING_MOTOR_ID, MotorType.kBrushless);

		intakeMotor.setSmartCurrentLimit(40);
		centeringMotor.setSmartCurrentLimit(40);

		intakeMotor.setIdleMode(IdleMode.kBrake);
		centeringMotor.setIdleMode(IdleMode.kBrake);

		centeringMotor.setInverted(false);

		intakeMotor.burnFlash();
		centeringMotor.burnFlash();
	}

	public void startIndex() {
		intakeMotor.set(Constants.NotePathConstants.INTAKE_MOTOR_SPEED);
		centeringMotor.set(Constants.NotePathConstants.INTAKE_MOTOR_SPEED);

	}

	public void reverseIndex() {
		intakeMotor.set(-Constants.NotePathConstants.INTAKE_MOTOR_SPEED);
		centeringMotor.set(-Constants.NotePathConstants.INTAKE_MOTOR_SPEED);
	}

	public void stopIndex() {
		intakeMotor.set(0);
		centeringMotor.set(0);
	}

}
