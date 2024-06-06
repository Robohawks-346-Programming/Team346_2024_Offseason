package frc.robot.subsystems.TestDrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public final class CTREConfigs {
	public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
	public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
	public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

	public CTREConfigs() {
		/** Swerve CANCoder Configuration */
		swerveCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

		/** Swerve Angle Motor Configurations */
		/* Motor Inverts and Neutral Mode */
		swerveAngleFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		/* Gear Ratio and Wrapping Config */
		swerveAngleFXConfig.Feedback.SensorToMechanismRatio = (12.8 / 1.0);
		swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

		/* Current Limiting */
		swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = 25;
		swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = 40;
		swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = 0.4;

		swerveAngleFXConfig.CurrentLimits.StatorCurrentLimitEnable = false;

		/* PID Config */
		swerveAngleFXConfig.Slot0.kP = Constants.DriveConstants.TURN_P;
		swerveAngleFXConfig.Slot0.kI = Constants.DriveConstants.TURN_I;
		swerveAngleFXConfig.Slot0.kD = Constants.DriveConstants.TURN_D;

		/** Swerve Drive Motor Configuration */
		/* Motor Inverts and Neutral Mode */
		swerveDriveFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		/* Gear Ratio Config */
		swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.DriveConstants.DRIVETRAIN_GEAR_RATIO;

		/* Current Limiting */
		swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

		swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = 60;

		/* PID Config */
		swerveDriveFXConfig.Slot0.kP = Constants.DriveConstants.DRIVE_P;
		swerveDriveFXConfig.Slot0.kI = Constants.DriveConstants.DRIVE_I;
		swerveDriveFXConfig.Slot0.kD = Constants.DriveConstants.DRIVE_D;
	}
}