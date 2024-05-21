package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Constants {
	public static final Mode currentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;

	// public static final Mode currentMode = Mode.REPLAY;

	public static enum Mode {
		REAL, SIM, REPLAY
	}

	public static final class DriveConstants {
		public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.5);
		public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.5);
		public static final double HALF_TRACKWIDTH_METERS = Units.inchesToMeters(10.25);
		public static final double HALF_WHEELBASE_METERS = Units.inchesToMeters(10.25);
		public static final double DRIVETRAIN_GEAR_RATIO = 5.12; // For L4 Gear Ratio
		public static final double WHEEL_RADIUS = Units.inchesToMeters(2);
		public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_RADIUS * 2;

		public static final double MAX_DRIVE_BASE_RADIUS = Math
				.sqrt(Math.pow((DRIVETRAIN_TRACKWIDTH_METERS / 2), 2) + Math.pow((DRIVETRAIN_WHEELBASE_METERS / 2), 2));

		public static final double DRIVE_CONVERSION = WHEEL_CIRCUMFERENCE / DRIVETRAIN_GEAR_RATIO;
		public static final double TURN_CONVERSION = 12.8;

		public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
				new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // front left
				new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // front
																											// right
				new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // back left
				new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0) // back right
		);

		public static final int MAX_VOLTAGE = 12;
		public static final int DRIVE_CURRENT_LIMIT = 60;
		public static final int TURN_CURRENT_LIMIT = 25;
		public static final double SLIP_CURRENT = 80;

		public static final boolean IS_FIELD_RELATIVE = true;

		public static final double MAX_MOVE_VELOCITY = 5;
		public static final double MAX_TURN_VELOCITY = 5;

		// Front left Swerve Module
		public static final int FRONT_LEFT_DRIVE_ID = 21;
		public static final int FRONT_LEFT_TURN_ID = 22;
		public static final int FRONT_LEFT_ENCODER_ID = 23;
		public static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERT = true;
		public static final double FRONT_LEFT_TURN_OFFSET = 0.68212890625;

		// Back left Swerve Module
		public static final int BACK_LEFT_DRIVE_ID = 31;
		public static final int BACK_LEFT_TURN_ID = 32;
		public static final int BACK_LEFT_ENCODER_ID = 33;
		public static final boolean BACK_LEFT_DRIVE_MOTOR_INVERT = true;
		public static final double BACK_LEFT_TURN_OFFSET = 0.31982421875;

		// Front Right Swerve Module
		public static final int FRONT_RIGHT_DRIVE_ID = 24;
		public static final int FRONT_RIGHT_TURN_ID = 25;
		public static final int FRONT_RIGHT_ENCODER_ID = 26;
		public static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERT = true;
		public static final double FRONT_RIGHT_TURN_OFFSET = 0.916259765625;

		// Back Right Swerve Module
		public static final int BACK_RIGHT_DRIVE_ID = 34;
		public static final int BACK_RIGHT_TURN_ID = 35;
		public static final int BACK_RIGHT_ENCODER_ID = 36;
		public static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERT = true;
		public static final double BACK_RIGHT_TURN_OFFSET = 0.071533203125;

		public static final double DRIVE_P = 1.5;
		public static final double DRIVE_I = 0;
		public static final double DRIVE_D = 0;
		public static final double DRIVE_kS = 0;
		public static final double DRIVE_kV = 1.85;
		public static final double DRIVE_kA = 0;
		public static final Slot0Configs driveGains = new Slot0Configs()
				.withKP(DRIVE_P)
				.withKI(DRIVE_I)
				.withKD(DRIVE_D)
				.withKS(DRIVE_kS)
				.withKV(DRIVE_kV)
				.withKA(DRIVE_kA);

		public static final double TURN_P = 10;
		public static final double TURN_I = 0;
		public static final double TURN_D = 0;
		public static final Slot0Configs steerGains = new Slot0Configs()
				.withKP(TURN_P)
				.withKI(TURN_I)
				.withKD(TURN_D);

		// Sim Variables
		private static final double kSteerInertia = 0.00001;
		private static final double kDriveInertia = 0.001;
		private static final double kSteerFrictionVoltage = 0.25;
		private static final double kDriveFrictionVoltage = 0.25;

		private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
				.withPigeon2Id(0)
				.withCANbusName("sim");

		private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
				.withDriveMotorGearRatio(DRIVETRAIN_GEAR_RATIO)
				.withSteerMotorGearRatio(TURN_CONVERSION)
				.withWheelRadius(WHEEL_RADIUS)
				.withSlipCurrent(SLIP_CURRENT)
				.withSteerMotorGains(steerGains)
				.withDriveMotorGains(driveGains)
				.withSteerMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
				.withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
				// .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
				.withSteerInertia(kSteerInertia)
				.withDriveInertia(kDriveInertia)
				.withSteerFrictionVoltage(kSteerFrictionVoltage)
				.withDriveFrictionVoltage(kDriveFrictionVoltage)
				.withFeedbackSource(SteerFeedbackType.FusedCANcoder);
		// .withCouplingGearRatio(kCoupleRatio)
		// .withSteerMotorInverted(kSteerMotorReversed);

		private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
				FRONT_LEFT_TURN_ID,
				FRONT_LEFT_DRIVE_ID,
				FRONT_LEFT_ENCODER_ID,
				FRONT_LEFT_TURN_OFFSET,
				Units.inchesToMeters(-HALF_TRACKWIDTH_METERS),
				Units.inchesToMeters(-HALF_WHEELBASE_METERS),
				FRONT_LEFT_DRIVE_MOTOR_INVERT);
		private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
				FRONT_RIGHT_TURN_ID,
				FRONT_RIGHT_DRIVE_ID,
				FRONT_RIGHT_ENCODER_ID,
				FRONT_RIGHT_TURN_OFFSET,
				Units.inchesToMeters(-HALF_TRACKWIDTH_METERS),
				Units.inchesToMeters(HALF_WHEELBASE_METERS),
				FRONT_RIGHT_DRIVE_MOTOR_INVERT);
		private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
				BACK_LEFT_TURN_ID,
				BACK_LEFT_DRIVE_ID,
				BACK_LEFT_ENCODER_ID,
				BACK_LEFT_TURN_OFFSET,
				Units.inchesToMeters(HALF_TRACKWIDTH_METERS),
				Units.inchesToMeters(-HALF_WHEELBASE_METERS),
				BACK_LEFT_DRIVE_MOTOR_INVERT);
		private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
				BACK_RIGHT_TURN_ID,
				BACK_RIGHT_DRIVE_ID,
				BACK_RIGHT_ENCODER_ID,
				BACK_RIGHT_TURN_OFFSET,
				Units.inchesToMeters(HALF_TRACKWIDTH_METERS),
				Units.inchesToMeters(HALF_WHEELBASE_METERS),
				BACK_RIGHT_DRIVE_MOTOR_INVERT);

		public static final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
				DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
	}

	public static final class AutoConstants {
		public static final double AUTO_DRIVE_P = 0.5;
		public static final double AUTO_DRIVE_I = 0;
		public static final double AUTO_DRIVE_D = 0;

		public static final double AUTO_TURN_P = 1.9;
		public static final double AUTO_TURN_I = 0;
		public static final double AUTO_TURN_D = 0;

		public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig,
																															// this
																															// should
																															// likely
																															// live
																															// in
																															// your
																															// Constants
																															// class
				new PIDConstants(AUTO_DRIVE_P, AUTO_DRIVE_I, AUTO_DRIVE_D), // Translation PID constants
				new PIDConstants(AUTO_TURN_P, AUTO_TURN_I, AUTO_TURN_D), // Rotation PID constants
				6, // Max module speed, in m/s
				DriveConstants.MAX_DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to
														// furthest module.
				new ReplanningConfig() // Default path replanning config. See the API for the options here
		);

	}

	public static final class OperatorConstants {
		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int OPERATOR_CONTROLLER_PORT = 1;

		public static final double DEADZONE = 0.05;
	}

}
