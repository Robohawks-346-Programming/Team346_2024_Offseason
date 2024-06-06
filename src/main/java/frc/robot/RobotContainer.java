// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.OI.DriverControllerXbox;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.Telemetry;
import frc.robot.subsystems.Drive.TelemetryIO;
import frc.robot.subsystems.Drive.TelemetryIOLive;
import frc.robot.subsystems.Drive.TelemetryIOSim;
import frc.robot.subsystems.TestDrive.Drive;
import frc.robot.subsystems.TestDrive.GyroIO;
import frc.robot.subsystems.TestDrive.GyroIOPigeon;
import frc.robot.subsystems.TestDrive.ModuleIO;
import frc.robot.subsystems.TestDrive.ModuleIOKraken;
import frc.robot.subsystems.TestDrive.ModuleIOSim;

public class RobotContainer {

	// CommandSwerveDrivetrain drivetrain = DriveConstants.drivetrain;
	// DriverControllerXbox m_driverControls;
	// Telemetry telemetry;

	Drive drive;
	CommandXboxController controller;
	private final LoggedDashboardChooser<Command> autoChooser;

	public RobotContainer() {
		configureSubsystems();
		configureBindings();
		configureCommands();
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
		autoChooser.addDefaultOption("StageTestBot", new PathPlannerAuto("StageTestBot"));
	}

	private void configureBindings() {
		// m_driverControls = new
		// DriverControllerXbox(OperatorConstants.DRIVER_CONTROLLER_PORT);

		controller = new CommandXboxController(0);
	}

	private void configureSubsystems() {
		// switch (Constants.currentMode) {
		// case REAL:
		// telemetry = new Telemetry(DriveConstants.MAX_MOVE_VELOCITY, new
		// TelemetryIOLive());
		// break;
		// case SIM:
		// telemetry = new Telemetry(DriveConstants.MAX_MOVE_VELOCITY, new
		// TelemetryIOSim());

		// drivetrain.seedFieldRelative(new Pose2d());
		// break;
		// case REPLAY:
		// telemetry = new Telemetry(DriveConstants.MAX_MOVE_VELOCITY, new TelemetryIO()
		// {
		// });
		// drivetrain.seedFieldRelative(new Pose2d());
		// break;
		// }
		// drivetrain.registerTelemetry(telemetry::telemeterize);
		// drivetrain.setPoseSupplier(telemetry::getFieldToRobot);

		switch (Constants.currentMode) {
			case REAL:
				drive = new Drive(
						new GyroIOPigeon(false),
						new ModuleIOKraken(0),
						new ModuleIOKraken(1),
						new ModuleIOKraken(2),
						new ModuleIOKraken(3));
				break;
			case SIM:
				drive = new Drive(
						new GyroIO() {
						},
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim());
				break;
			case REPLAY:
				drive = new Drive(
						new GyroIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						});
				break;
		}
	}

	private void configureCommands() {
		// drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, () ->
		// m_driverControls.getDriveForward(),
		// () -> m_driverControls.getDriveLeft(), () ->
		// m_driverControls.getDriveRotation(),
		// OperatorConstants.DEADZONE));
		drive.setDefaultCommand(
				JoystickDrive.joystickDrive(
						drive,
						() -> -controller.getLeftY(),
						() -> -controller.getLeftX(),
						() -> -controller.getRightX()));
	}

	public Command getAutonomousCommand() {
		// return drivetrain.getAutoCommand();
		return autoChooser.get();
	}
}
