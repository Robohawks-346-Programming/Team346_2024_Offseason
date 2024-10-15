package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class NotePath extends SubsystemBase {
	CANSparkMax feederRoller, ampRollers, intakeMotor, centeringMotor;

	TalonFX topRoller, bottomRoller;
	private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

	private final VelocityVoltage voltage;
	private final VoltageOut volts;
	private final CoastOut coast;

	private DigitalInput laserBreak;

	public NotePath() {
		feederRoller = new CANSparkMax(Constants.NotePathConstants.FEEDER_ROLLER_MOTOR_ID, MotorType.kBrushless);
		ampRollers = new CANSparkMax(Constants.NotePathConstants.AMP_ROLLER_MOTOR_ID, MotorType.kBrushless);
		intakeMotor = new CANSparkMax(Constants.NotePathConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
		centeringMotor = new CANSparkMax(Constants.NotePathConstants.CENTERING_MOTOR_ID, MotorType.kBrushless);

		feederRoller.setSmartCurrentLimit(40);
		ampRollers.setSmartCurrentLimit(40);
		intakeMotor.setSmartCurrentLimit(40);
		centeringMotor.setSmartCurrentLimit(40);

		feederRoller.setIdleMode(IdleMode.kBrake);
		ampRollers.setIdleMode(IdleMode.kBrake);
		intakeMotor.setIdleMode(IdleMode.kBrake);
		centeringMotor.setIdleMode(IdleMode.kBrake);

		feederRoller.setInverted(true);
		ampRollers.setInverted(false);
		centeringMotor.setInverted(false);

		feederRoller.burnFlash();
		ampRollers.burnFlash();
		intakeMotor.burnFlash();
		centeringMotor.burnFlash();

		topRoller = new TalonFX(Constants.NotePathConstants.TOP_SPEAKER_ROLLER_MOTOR_ID);
		bottomRoller = new TalonFX(Constants.NotePathConstants.BOTTOM_SPEAKER_ROLLER_MOTOR_ID);

		shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		shooterConfig.Slot0.kP = Constants.NotePathConstants.SPEAKER_SHOOTER_P;
		shooterConfig.Slot0.kI = Constants.NotePathConstants.SPEAKER_SHOOTER_I;
		shooterConfig.Slot0.kD = Constants.NotePathConstants.SPEAKER_SHOOTER_D;

		shooterConfig.Slot0.kV = Constants.NotePathConstants.SPEAKER_SHOOTER_kV;

		bottomRoller.getConfigurator().apply(shooterConfig);
		topRoller.getConfigurator().apply(shooterConfig);

		laserBreak = new DigitalInput(Constants.NotePathConstants.BEAM_BREAK_PORT);

		voltage = new VelocityVoltage(0);
		volts = new VoltageOut(0);
		coast = new CoastOut();
	}

	public void startIndex() {
		feederRoller.set(Constants.NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_1);
		ampRollers.set(Constants.NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_1);
		intakeMotor.set(Constants.NotePathConstants.INTAKE_MOTOR_SPEED);
		centeringMotor.set(Constants.NotePathConstants.INTAKE_MOTOR_SPEED);
	}

	public void reverseIndex() {
		feederRoller.set(-Constants.NotePathConstants.FEEDER_ROLLER_SPEED);
		ampRollers.set(-Constants.NotePathConstants.FEEDER_ROLLER_SPEED);
		intakeMotor.set(-Constants.NotePathConstants.INTAKE_MOTOR_SPEED);
		centeringMotor.set(-Constants.NotePathConstants.INTAKE_MOTOR_SPEED);
	}

	public void stopIndex() {
		feederRoller.set(0);
		ampRollers.set(0);
		intakeMotor.set(0);
		centeringMotor.set(0);
		topRoller.setControl(coast);
		bottomRoller.setControl(coast);
	}

	public boolean getLaserBreak() {
		return !laserBreak.get();
	}

	public void ejectAmp() {
		feederRoller.set(-Constants.NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_1);
		ampRollers.set(Constants.NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_2);
	}

	public void ejectSpeaker() {
		feederRoller.set(Constants.NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_2);
		ampRollers.set(Constants.NotePathConstants.AMP_ROLLERS_ROLLER_SPEED_2);
	}

	public void setVelocity(double velocity, double velocity2) {
		topRoller.setControl(voltage.withVelocity(velocity));
		bottomRoller.setControl(voltage.withVelocity(velocity2));
	}

	public void setVoltage(double volt) {
		topRoller.setControl(volts.withOutput(volt));
		bottomRoller.setControl(volts.withOutput(volt));
	}

	public boolean isAtVelocity(double rev) {
		return Math.abs(topRoller.getVelocity().getValueAsDouble() - rev) < 30;
	}

	public Command outtake() {
		return Commands.run(() -> reverseIndex()).finallyDo(() -> stopIndex());
	}

	public Command intake() {
		return Commands.run(() -> startIndex())
				.until(() -> {
					return getLaserBreak();
				}).finallyDo(() -> stopIndex());
	}

	public Command index() {
		return Commands.runEnd(() -> startIndex(), () -> stopIndex());
	}

	public Command ejectAmpCommand() {
		return Commands.runEnd(() -> ejectAmp(), () -> stopIndex());
	}

	public Command ejectSpeakerCommand() {
		return Commands.runEnd(() -> ejectSpeaker(), () -> stopIndex());
	}

	public Command rev() {
		return Commands.run(() -> setVelocity(120, 120)).finallyDo(() -> stopIndex());
	}

	public Command shoot() {
		return Commands.sequence(
				rev().withTimeout(0.4),
				ejectSpeakerCommand().withTimeout(0.4));
	}

	public Command distanceShoot() {
		return Commands.sequence(
				outtake().withTimeout(0.1),
				ejectSpeakerCommand().withTimeout(0.75));
	}

	public Command feed() {
		return Commands.run(() -> setVelocity(90, 90)).finallyDo(() -> stopIndex());
	}
}