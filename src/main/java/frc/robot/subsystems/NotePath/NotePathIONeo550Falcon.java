package frc.robot.subsystems.NotePath;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.NotePath.NotePathIO.NotePathIOInputs;

public class NotePathIONeo550Falcon implements NotePathIO {
	private CANSparkMax feederRoller, ampRollers, intakeMotor, centeringMotor;

	private TalonFX topRoller, bottomRoller;
	private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
	private MotionMagicVelocityTorqueCurrentFOC motionMagicVoltage = new MotionMagicVelocityTorqueCurrentFOC(0);
	private MotionMagicConfigs config;

	private final VelocityVoltage voltage;
	private final CoastOut coast;

	private DigitalInput laserBreak;

	public NotePathIONeo550Falcon() {
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

		config = shooterConfig.MotionMagic;
		config.MotionMagicAcceleration = 1000;
		config.MotionMagicJerk = 4000;

		bottomRoller.getConfigurator().apply(shooterConfig);
		topRoller.getConfigurator().apply(shooterConfig);

		laserBreak = new DigitalInput(Constants.NotePathConstants.BEAM_BREAK_PORT);

		voltage = new VelocityVoltage(0);
		coast = new CoastOut();
		motionMagicVoltage = new MotionMagicVelocityTorqueCurrentFOC(0);
	}

	@Override
	public void updateInputs(NotePathIOInputs inputs) {
		inputs.intakeRollerSpeed = intakeMotor.get();
		inputs.centeringMotorSpeed = centeringMotor.get();
		inputs.ampRollerSpeed = ampRollers.get();
		inputs.feederRollerSpeed = feederRoller.get();
		inputs.topRollerSpeed = topRoller.get();
		inputs.bottomRollerSpeed = bottomRoller.get();

		inputs.noteSensed = !laserBreak.get();
	}

	@Override
	public void setIntakeSpeed(double speed) {
		intakeMotor.set(speed);
		centeringMotor.set(speed);
	}

	@Override
	public void setAmpRollerSpeed(double speed) {
		ampRollers.set(speed);
	}

	@Override
	public void setFeederRollerSpeed(double speed) {
		feederRoller.set(speed);
	}

	@Override
	public void setShooterSpeed(double speed) {
		if (speed != 0) {
			topRoller.setControl(motionMagicVoltage.withVelocity(speed));
			bottomRoller.setControl(motionMagicVoltage.withVelocity(speed));
		} else {
			topRoller.setControl(coast);
			bottomRoller.setControl(coast);
		}
	}

}
