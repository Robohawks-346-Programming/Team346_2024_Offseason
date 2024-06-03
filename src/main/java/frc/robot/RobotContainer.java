// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.OI.DriverControllerXbox;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.Telemetry;
import frc.robot.subsystems.Drive.TelemetryIO;
import frc.robot.subsystems.Drive.TelemetryIOLive;
import frc.robot.subsystems.Drive.TelemetryIOSim;

public class RobotContainer {

	CommandSwerveDrivetrain drivetrain = DriveConstants.drivetrain;
	DriverControllerXbox m_driverControls;
	Telemetry telemetry;

	public RobotContainer() {
		configureSubsystems();
		configureBindings();
		configureCommands();
	}

	private void configureBindings() {
		m_driverControls = new DriverControllerXbox(OperatorConstants.DRIVER_CONTROLLER_PORT);
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
		drivetrain.setPoseSupplier(telemetry::getFieldToRobot);
	}

	private void configureCommands() {
		drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, () -> m_driverControls.getDriveForward(),
				() -> m_driverControls.getDriveLeft(), () -> m_driverControls.getDriveRotation(),
				OperatorConstants.DEADZONE));
	}

	public Command getAutonomousCommand() {
		return drivetrain.getAutoCommand();
	}
}
