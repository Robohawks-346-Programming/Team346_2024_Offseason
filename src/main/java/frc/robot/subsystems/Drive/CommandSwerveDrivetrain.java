package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

	private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();
	private SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();

	private SendableChooser<Command> autoChooser;

	private static final double kSimLoopPeriod = 0.005; // Original: 5 ms
	private Notifier simNotifier = null;
	private double lastSimTime;

	/* Keep track if we've ever applied the operator perspective before or not */
	private boolean hasAppliedOperatorPerspective = false;
	/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
	private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
	/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
	private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

	private Supplier<Pose2d> getFieldToRobot = () -> new Pose2d();
	private Supplier<Translation2d> getRobotVelocity = () -> new Translation2d();

	private Pose2d redGoal = new Pose2d(new Translation2d(16.579342, 5.547868), new Rotation2d());
	private Pose2d blueGoal = new Pose2d(new Translation2d(0.0381, 5.547868), new Rotation2d());

	public CommandSwerveDrivetrain(
			SwerveDrivetrainConstants driveTrainConstants,
			double OdometryUpdateFrequency,
			SwerveModuleConstants... modules) {
		super(driveTrainConstants, OdometryUpdateFrequency, modules);
		if (Constants.currentMode == Constants.Mode.SIM) {
			startSimThread();
		}

		configurePathPlanner();
		setBrakeMode();
		CommandScheduler.getInstance().registerSubsystem(this);
	}

	public CommandSwerveDrivetrain(
			SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		if (Constants.currentMode == Constants.Mode.SIM) {
			startSimThread();
		}

		configurePathPlanner();
		setBrakeMode();
		CommandScheduler.getInstance().registerSubsystem(this);
	}

	public void configurePathPlanner() {
		double driveBaseRadius = 0;
		for (var moduleLocation : m_moduleLocations) {
			driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
		}

		Pathfinding.setPathfinder(new LocalADStar());

		AutoBuilder.configureHolonomic(
				() -> getPose(),
				this::seedFieldRelative,
				this::getCurrentRobotChassisSpeeds,
				(speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig,
													// this
													// should
													// likely
													// live
													// in
													// your
													// Constants
													// class
						new PIDConstants(Constants.AutoConstants.AUTO_DRIVE_P, Constants.AutoConstants.AUTO_DRIVE_I,
								Constants.AutoConstants.AUTO_DRIVE_D), // Translation PID constants
						new PIDConstants(Constants.AutoConstants.AUTO_TURN_P, Constants.AutoConstants.AUTO_TURN_I,
								Constants.AutoConstants.AUTO_TURN_D), // Rotation PID constants
						6, // Max module speed, in m/s
						driveBaseRadius, // Drive base radius in meters. Distance from robot center to
											// furthest module.
						new ReplanningConfig() // Default path replanning config. See the API for the options here
				),
				() -> {
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this);

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		PathPlannerPath traj1 = PathPlannerPath.fromChoreoTrajectory("StageTestBot");
		PathPlannerPath traj2 = PathPlannerPath.fromChoreoTrajectory("StageTestBot.1");
		PathPlannerPath traj3 = PathPlannerPath.fromChoreoTrajectory("StageTestBot.2");
	}

	public Command getAutoCommand() {
		return autoChooser.getSelected();
	}

	public ChassisSpeeds getCurrentRobotChassisSpeeds() {
		return m_kinematics.toChassisSpeeds(getState().ModuleStates);
	}

	private void startSimThread() {
		lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		simNotifier = new Notifier(
				() -> {
					final double currentTime = Utils.getCurrentTimeSeconds();
					double deltaTime = currentTime - lastSimTime;
					lastSimTime = currentTime;

					/* use the measured time delta, get battery voltage from WPILib */
					updateSimState(deltaTime, RobotController.getBatteryVoltage());
				});
		simNotifier.startPeriodic(kSimLoopPeriod);
	}

	public void setBrakeMode() {
		for (int i = 0; i < 3; i++) {
			this.getModule(i)
					.getDriveMotor()
					.setNeutralMode(NeutralModeValue.Brake);
			this.getModule(i)
					.getSteerMotor()
					.setNeutralMode(NeutralModeValue.Coast);
		}
	}

	public Pose2d getPose() {
		return this.getState().Pose;
	}

	public void drive(double vx, double vy, double omega) {
		setControl(
				driveFieldCentric
						.withVelocityX(vx)
						.withVelocityY(vy)
						.withRotationalRate(omega)
						.withDeadband(0.0)
						.withRotationalDeadband(0.0));
	}

	public void setPoseSupplier(Supplier<Pose2d> getFieldToRobot) {
		this.getFieldToRobot = getFieldToRobot;
	}

	public void setVelocitySupplier(Supplier<Translation2d> getRobotVelocity) {
		this.getRobotVelocity = getRobotVelocity;
	}

	private boolean isRedAlliance() {
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance != null) {
			return alliance.get() == DriverStation.Alliance.Red;
		}
		return false;
	}

	public double getDistanceFromSpeaker() {
		Pose2d target = isRedAlliance() ? redGoal : blueGoal;
		Pose2d robot = this.getPose();
		double distance = PhotonUtils.getDistanceToPose(target, robot);
		return distance;
	}

	public double getHeadingToSpeaker() {
		Pose2d target = isRedAlliance() ? redGoal : blueGoal;
		Pose2d robot = this.getPose();
		Rotation2d robotYaw = Rotation2d
				.fromRadians(Math.atan2(target.getY() - robot.getTranslation().getY(),
						target.getX() - robot.getTranslation().getX()))
				.plus(new Rotation2d(Units.degreesToRadians(180)));
		// SmartDashboard.putNumber("Heading To Target", robotYaw.getDegrees());
		return robotYaw.getDegrees();
	}
}