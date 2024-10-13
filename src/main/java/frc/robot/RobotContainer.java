// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.beans.Visibility;
import java.rmi.dgc.Lease;
import java.security.AllPermission;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.OI.DriverControllerXbox;
import frc.robot.commands.DistanceShoot;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.NotePath;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.Telemetry;
import frc.robot.subsystems.Drive.TelemetryIO;
import frc.robot.subsystems.Drive.TelemetryIOLive;
import frc.robot.subsystems.Drive.TelemetryIOSim;
import frc.robot.subsystems.Vision.Vision;

public class RobotContainer {

	CommandSwerveDrivetrain drivetrain = DriveConstants.DriveTrain;
	DriverControllerXbox m_driverControls;
	Telemetry telemetry;
	Pivot pivot = new Pivot(drivetrain);
	NotePath notePath = new NotePath();
	Climber climber = new Climber();
	Vision vision = new Vision(drivetrain);
	LEDs leds = new LEDs(notePath, vision);
	Joystick operatorControl = new Joystick(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

	// Drive drive;
	// public static Drive drive;
	// CommandXboxController controller;
	// private final LoggedDashboardChooser<Command> autoChooser;

	public RobotContainer() {
		NamedCommands.registerCommand("Intake 2", Commands.parallel(notePath.intake(),
				pivot.driveDown()));
		NamedCommands.registerCommand("Shoot 2", notePath.shoot());
		NamedCommands.registerCommand("Distance Shoot 2", new DistanceShoot(drivetrain, pivot, notePath));
		configureSubsystems();
		configureBindings();
		configureCommands();
		drivetrain.configurePathPlanner();
		// autoChooser = new LoggedDashboardChooser<>("Auto Choices",
		// AutoBuilder.buildAutoChooser());
		// autoChooser.addDefaultOption("StageTestBot", new
		// PathPlannerAuto("StageTestBot"));
	}

	private void configureBindings() {
		m_driverControls = new DriverControllerXbox(OperatorConstants.DRIVER_CONTROLLER_PORT);

		// controller = new CommandXboxController(0);

		JoystickButton BUTTON_1 = new JoystickButton(operatorControl, 1),
				BUTTON_2 = new JoystickButton(operatorControl, 2),
				BUTTON_3 = new JoystickButton(operatorControl, 3),
				BUTTON_4 = new JoystickButton(operatorControl, 4),
				BUTTON_5 = new JoystickButton(operatorControl, 5),
				BUTTON_6 = new JoystickButton(operatorControl, 6),
				BUTTON_7 = new JoystickButton(operatorControl, 7),
				BUTTON_8 = new JoystickButton(operatorControl, 8),
				BUTTON_9 = new JoystickButton(operatorControl, 9),
				BUTTON_10 = new JoystickButton(operatorControl, 10),
				BUTTON_11 = new JoystickButton(operatorControl, 11),
				BUTTON_12 = new JoystickButton(operatorControl, 12),
				BUTTON_13 = new JoystickButton(operatorControl, 13),
				BUTTON_14 = new JoystickButton(operatorControl, 14),
				BUTTON_15 = new JoystickButton(operatorControl, 15),
				BUTTON_16 = new JoystickButton(operatorControl, 16);

		BUTTON_1.whileTrue(Commands.parallel(notePath.intake(),
				pivot.driveDown()));
		BUTTON_10.whileTrue(notePath.ejectSpeakerCommand());
		BUTTON_3.whileTrue(notePath.rev());
		BUTTON_4.onTrue(pivot.moveArm(-32));
		BUTTON_8.onTrue(pivot.moveArm(-60));
		BUTTON_5.onTrue(pivot.moveArm(55));
		BUTTON_6.onTrue(pivot.moveArm(0));
		BUTTON_7.onTrue(pivot.moveArm(90));
		BUTTON_2.whileTrue(notePath.outtake());
		BUTTON_3.whileTrue(notePath.rev());
		BUTTON_4.onTrue(pivot.moveArm(-32));
		BUTTON_5.onTrue(pivot.moveArm(55));
		BUTTON_6.onTrue(pivot.moveArm(0));
		BUTTON_7.onTrue(pivot.moveArm(90));
		BUTTON_8.onTrue(pivot.moveArm(-60));
		BUTTON_9.whileTrue(notePath.index());
		BUTTON_10.whileTrue(notePath.ejectSpeakerCommand());
		BUTTON_11.whileTrue(climber.leftHookUp());
		BUTTON_12.whileTrue(climber.moveHooksUp());
		BUTTON_13.whileTrue(climber.moveHooksDown());
		BUTTON_14.whileTrue(climber.rightHookUp());
		BUTTON_15.whileTrue(pivot.moveArm(-21));
		BUTTON_16.whileTrue(notePath.ejectAmpCommand());

		m_driverControls.rightBumper.onTrue(Commands.runOnce(() -> drivetrain.seedFieldRelative(
				new Pose2d(new Translation2d(drivetrain.getPose().getX(), drivetrain.getPose().getY()),
						apply(drivetrain.getPose().getRotation())))));
		m_driverControls.rightTrigger.onTrue(new DistanceShoot(drivetrain, pivot, notePath));
		m_driverControls.leftTrigger.onTrue(
				new TeleopDrive(drivetrain, () -> m_driverControls.getDriveForward(),
						() -> m_driverControls.getDriveLeft(), () -> m_driverControls.getDriveRotation(),
						OperatorConstants.DEADZONE));

	}

	private void configureSubsystems() {
		switch (Constants.currentMode) {
			case REAL:
				telemetry = new Telemetry(DriveConstants.MAX_MOVE_VELOCITY, new TelemetryIOLive());
				break;
			case SIM:
				telemetry = new Telemetry(DriveConstants.MAX_MOVE_VELOCITY, new TelemetryIOSim());

				drivetrain.seedFieldRelative(new Pose2d());
				break;
			case REPLAY:
				telemetry = new Telemetry(DriveConstants.MAX_MOVE_VELOCITY, new TelemetryIO() {
				});
				drivetrain.seedFieldRelative(new Pose2d());
				break;
		}
		drivetrain.registerTelemetry(telemetry::telemeterize);
		// drivetrain.setPoseSupplier(telemetry::getFieldToRobot);
		// drivetrain.setVelocitySupplier(telemetry::getVelocity);

		// switch (Constants.currentMode) {
		// case REAL:
		// drive = new Drive(
		// new GyroIOPigeon(false),
		// new ModuleIOKraken(0),
		// new ModuleIOKraken(1),
		// new ModuleIOKraken(2),
		// new ModuleIOKraken(3));
		// break;
		// case SIM:
		// drive = new Drive(
		// new GyroIO() {
		// },
		// new ModuleIOSim(),
		// new ModuleIOSim(),
		// new ModuleIOSim(),
		// new ModuleIOSim());
		// break;
		// case REPLAY:
		// drive = new Drive(
		// new GyroIO() {
		// },
		// new ModuleIO() {
		// },
		// new ModuleIO() {
		// },
		// new ModuleIO() {
		// },
		// new ModuleIO() {
		// });
		// break;
		// }
	}

	private void configureCommands() {
		drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, () -> m_driverControls.getDriveForward(),
				() -> m_driverControls.getDriveLeft(), () -> m_driverControls.getDriveRotation(),
				OperatorConstants.DEADZONE));

		// drive.setDefaultCommand(
		// JoystickDrive.joystickDrive(
		// drive,
		// () -> -controller.getLeftY(),
		// () -> -controller.getLeftX(),
		// () -> -controller.getRightX()));
	}

	public Command getAutonomousCommand() {
		return drivetrain.getAutoCommand();
		// return autoChooser.get();
	}

	public static boolean shouldFlip() {
		return DriverStation.getAlliance().isPresent()
				&& DriverStation.getAlliance().get() == Alliance.Red;
	}

	public static Rotation2d apply(Rotation2d rotation) {
		if (shouldFlip()) {
			return new Rotation2d(-rotation.getCos(), rotation.getSin());
		} else {
			return rotation;
		}
	}

	public void setOperatorPerspective() {
		DriverStation.getAlliance().ifPresent((allianceColor) -> {
			drivetrain.setOperatorPerspectiveForward(
					allianceColor == Alliance.Red ? Rotation2d.fromDegrees(0)
							: Rotation2d.fromDegrees(180));
		});
	}
}
