package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Constants {

	public static enum Mode {
		REAL, SIM, REPLAY
	}

	public static final double loopTime = 0.02;

	public static final Mode currentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;

	// public static final Mode currentMode = Mode.REPLAY;

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

		public static final Pose2d initialPose = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90));

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

		public static final double DRIVE_P = 1.5;
		public static final double DRIVE_I = 0;
		public static final double DRIVE_D = 0;
		public static final double DRIVE_kS = 0;
		public static final double DRIVE_kV = 1.85;
		public static final double DRIVE_kA = 0;

		public static final double TURN_P = 150;
		public static final double TURN_I = 50;
		public static final double TURN_D = 0.2;
		public static final double TURN_kS = 0.25;
		public static final double TURN_kV = 1.5;
		public static final double TURN_kA = 0;

		// Both sets of gains need to be tuned to your individual robot.

		// The steer motor uses any SwerveModule.SteerRequestType control request with
		// the
		// output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
		private static final Slot0Configs steerGains = new Slot0Configs()
				.withKP(TURN_P).withKI(TURN_I).withKD(TURN_D)
				.withKS(TURN_kS).withKV(TURN_kV).withKA(TURN_kA);
		// When using closed-loop control, the drive motor uses the control
		// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
		private static final Slot0Configs driveGains = new Slot0Configs()
				.withKP(DRIVE_P).withKI(DRIVE_I).withKD(DRIVE_D)
				.withKS(DRIVE_kS).withKV(DRIVE_kV).withKA(DRIVE_kA);

		// The closed-loop output type to use for the steer motors;
		// This affects the PID/FF gains for the steer motors
		private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
		// The closed-loop output type to use for the drive motors;
		// This affects the PID/FF gains for the drive motors
		private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

		// The stator current at which the wheels start to slip;
		// This needs to be tuned to your individual robot
		private static final double kSlipCurrentA = 60.0;

		// Initial configs for the drive and steer motors and the CANcoder; these cannot
		// be null.
		// Some configs will be overwritten; check the `with*InitialConfigs()` API
		// documentation.
		private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								// Swerve azimuth does not require much torque output, so we can set a
								// relatively low
								// stator current limit to help avoid brownouts without impacting performance.
								.withStatorCurrentLimit(60)
								.withStatorCurrentLimitEnable(true));
		private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
				.withCurrentLimits(
						new CurrentLimitsConfigs()
								// Swerve azimuth does not require much torque output, so we can set a
								// relatively low
								// stator current limit to help avoid brownouts without impacting performance.
								.withStatorCurrentLimit(40)
								.withStatorCurrentLimitEnable(true));
		private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
		// Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
		private static final Pigeon2Configuration pigeonConfigs = null;

		// Theoretical free speed (m/s) at 12v applied output;
		// This needs to be tuned to your individual robot
		public static final double kSpeedAt12VoltsMps = 6.21;

		// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
		// This may need to be tuned to your individual robot
		private static final double kCoupleRatio = 3;

		private static final double kDriveGearRatio = 5.142857142857142;
		private static final double kSteerGearRatio = 12.8;
		private static final double kWheelRadiusInches = 2;

		private static final boolean kInvertLeftSide = false;
		private static final boolean kInvertRightSide = true;

		private static final String kCANbusName = "";
		private static final int kPigeonId = 0;

		// These are only used for simulation
		private static final double kSteerInertia = 0.00001;
		private static final double kDriveInertia = 0.001;
		// Simulated voltage necessary to overcome friction
		private static final double kSteerFrictionVoltage = 0.25;
		private static final double kDriveFrictionVoltage = 0.25;

		private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
				.withCANbusName(kCANbusName)
				.withPigeon2Id(kPigeonId)
				.withPigeon2Configs(pigeonConfigs);

		private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
				.withDriveMotorGearRatio(kDriveGearRatio)
				.withSteerMotorGearRatio(kSteerGearRatio)
				.withWheelRadius(kWheelRadiusInches)
				.withSlipCurrent(kSlipCurrentA)
				.withSteerMotorGains(steerGains)
				.withDriveMotorGains(driveGains)
				.withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
				.withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
				.withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
				.withSteerInertia(kSteerInertia)
				.withDriveInertia(kDriveInertia)
				.withSteerFrictionVoltage(kSteerFrictionVoltage)
				.withDriveFrictionVoltage(kDriveFrictionVoltage)
				.withFeedbackSource(SteerFeedbackType.FusedCANcoder)
				// .withCouplingGearRatio(kCoupleRatio)
				.withDriveMotorInitialConfigs(driveInitialConfigs)
				.withSteerMotorInitialConfigs(steerInitialConfigs)
				.withCANcoderInitialConfigs(cancoderInitialConfigs);

		// Front Left
		private static final int kFrontLeftDriveMotorId = 21;
		private static final int kFrontLeftSteerMotorId = 22;
		private static final int kFrontLeftEncoderId = 23;
		private static final double kFrontLeftEncoderOffset = 0.335;
		private static final boolean kFrontLeftSteerInvert = false;

		private static final double kFrontLeftXPosInches = 10.25;
		private static final double kFrontLeftYPosInches = 10.25;

		// Front Right
		private static final int kFrontRightDriveMotorId = 24;
		private static final int kFrontRightSteerMotorId = 25;
		private static final int kFrontRightEncoderId = 26;
		private static final double kFrontRightEncoderOffset = -0.435;
		private static final boolean kFrontRightSteerInvert = false;

		private static final double kFrontRightXPosInches = 10.25;
		private static final double kFrontRightYPosInches = -10.25;

		// Back Left
		private static final int kBackLeftDriveMotorId = 31;
		private static final int kBackLeftSteerMotorId = 32;
		private static final int kBackLeftEncoderId = 33;
		private static final double kBackLeftEncoderOffset = -0.31103515625;
		private static final boolean kBackLeftSteerInvert = false;

		private static final double kBackLeftXPosInches = -10.25;
		private static final double kBackLeftYPosInches = 10.25;

		// Back Right
		private static final int kBackRightDriveMotorId = 34;
		private static final int kBackRightSteerMotorId = 35;
		private static final int kBackRightEncoderId = 36;
		private static final double kBackRightEncoderOffset = 0.407;
		private static final boolean kBackRightSteerInvert = false;

		private static final double kBackRightXPosInches = -10.25;
		private static final double kBackRightYPosInches = -10.25;

		private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
				kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
				Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide)
				.withSteerMotorInverted(kFrontLeftSteerInvert);
		private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
				kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
				Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches),
				kInvertRightSide)
				.withSteerMotorInverted(kFrontRightSteerInvert);
		private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
				kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
				Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide)
				.withSteerMotorInverted(kBackLeftSteerInvert);
		private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
				kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
				Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
				kInvertRightSide)
				.withSteerMotorInverted(kBackRightSteerInvert);

		public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants,
				FrontLeft,
				FrontRight, BackLeft, BackRight);
	}

	public static final class AutoConstants {
		public static final double AUTO_DRIVE_P = 0.5;
		public static final double AUTO_DRIVE_I = 0;
		public static final double AUTO_DRIVE_D = 0;

		public static final double AUTO_TURN_P = 1.9;
		public static final double AUTO_TURN_I = 0;
		public static final double AUTO_TURN_D = 0;

	}

	public static final class OperatorConstants {
		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int OPERATOR_CONTROLLER_PORT = 1;

		public static final double DEADZONE = 0.05;
	}

	public static final class NotePathConstants {
		public static final int FEEDER_ROLLER_MOTOR_ID = 6;
		public static final int AMP_ROLLER_MOTOR_ID = 5;
		public static final int INTAKE_MOTOR_ID = 15;
		public static final int CENTERING_MOTOR_ID = 16;

		public static final double FEEDER_ROLLER_SPEED = 0.45;
		public static final double AMP_ROLLERS_ROLLER_SPEED_1 = 0.5;
		public static final double AMP_ROLLERS_ROLLER_SPEED_2 = 0.9;
		public static final double INTAKE_MOTOR_SPEED = 0.95;

		public static final int TOP_SPEAKER_ROLLER_MOTOR_ID = 10;
		public static final int BOTTOM_SPEAKER_ROLLER_MOTOR_ID = 11;

		public static final double SPEAKER_SHOOTER_P = 0.5;
		public static final double SPEAKER_SHOOTER_I = 0;
		public static final double SPEAKER_SHOOTER_D = 0;

		public static final double SPEAKER_SHOOTER_kV = 0;

		public static final int BEAM_BREAK_PORT = 9;
	}

	public static final class PivotConstants {
		public static final int PIVOT_MOTOR_ID = 8;

		public static final double PIVOT_GEAR_RATIO = 55.6; // 55.6 motor rev : 1 arm rev

		public static final double HOME_PIVOT_ANGLE = -55;
		public static final double TRAP_PIVOT_ANGLE = 90; // off vertical
		public static final double SOURCE_PIVOT_ANGLE = 45; // off vertical
		public static final double PIVOT_ANGLE_THRESHOLD = 10; // in degrees

		public static final double PIVOT_P = 25;
		public static final double PIVOT_I = 0;
		public static final double PIVOT_D = 3;

		public static final double PIVOT_kS = 0.4;
		public static final double PIVOT_kG = 0.67;
		public static final double PIVOT_kV = 0.2;
		public static final double PIVOT_kA = 0;

		public static InterpolatingDoubleTreeMap getPivotMap() {
			InterpolatingDoubleTreeMap pivotMap = new InterpolatingDoubleTreeMap();
			pivotMap.put(Units.inchesToMeters(57), -55.0);
			pivotMap.put(Units.inchesToMeters(77), -43.5);
			pivotMap.put(Units.inchesToMeters(96.5), -38.0);
			pivotMap.put(Units.inchesToMeters(116.5), -30.5);
			pivotMap.put(Units.inchesToMeters(136.5), -27.25);
			pivotMap.put(Units.inchesToMeters(156.5), -23.25);
			pivotMap.put(Units.inchesToMeters(176.5), -21.0);
			pivotMap.put(Units.inchesToMeters(196.5), -20.5);
			pivotMap.put(Units.inchesToMeters(216.5), -20.25);
			return pivotMap;
		}
	}

	public static final class ClimberConstants {
		public static final int LEFT_HOOK_MOTOR_ID = 2;
		public static final int RIGHT_HOOK_MOTOR_ID = 3;

		public static final double HOOK_P = 0.05;
		public static final double HOOK_I = 0;
		public static final double HOOK_D = 0;

		public static final double CLIMBER_REV_THRESHOLD = 5;

		public static final double CLIMB_REV = 1000;
		public static final double CLIMB_HOME_REV = 860;

	}

	public static final class LEDConstants {
		public static final int LED_PORT = 0;
		public static final int NUMBER_OF_LEDS = 81;
	}

	public static final class VisionConstants {

		public static final String[] cameraNames = {
				"FL",
				"FR",
				"BL",
				"BR"
		};

		public static final Transform3d[] vehicleToCameras = { // 10 deg yaw, 5 deg pitch
				new Transform3d(new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(5.75),
						Units.inchesToMeters(25.5)), new Rotation3d(0, 0, 0)),
				new Transform3d(new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(5.75),
						Units.inchesToMeters(25.5)), new Rotation3d(0, 0, 0)),
				new Transform3d(
						new Translation3d(Units.inchesToMeters(-9.75), Units.inchesToMeters(-12.75),
								Units.inchesToMeters(20.6666666666)),
						new Rotation3d(0, 0, Units.degreesToRadians(180))),
				new Transform3d(new Translation3d(Units.inchesToMeters(9.75), Units.inchesToMeters(-12.75),
						Units.inchesToMeters(20.66666666)), new Rotation3d(0, 0, Units.degreesToRadians(180)))
		};

		public static final double lowUncertaintyCutoffDistance = 6.5;
		public static final double skewCutoffDistance = 5.8;
		public static final double skewCutoffRotation = Units.degreesToRadians(50);

		public static final Matrix<N3, N1> teleopCameraUncertainty = VecBuilder.fill(2, 2, 2);
		public static final Matrix<N3, N1> lowCameraUncertainty = VecBuilder.fill(8, 8, 8);
		public static final Matrix<N3, N1> highCameraUncertainty = VecBuilder.fill(16.0, 16.0, 16);
		public static final Matrix<N3, N1> driveUncertainty = VecBuilder.fill(0.1, 0.1, 0.1);

		public static final AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

	}
}