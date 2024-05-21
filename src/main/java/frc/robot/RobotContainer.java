// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.OI.DriverControllerXbox;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

  CommandSwerveDrivetrain drivetrain;
  DriverControllerXbox m_driverControls;
  

  public RobotContainer() {
    configureSubsystems();
    configureBindings();
    configureCommands();
  }

  private void configureBindings() {
    m_driverControls = new DriverControllerXbox(OperatorConstants.DRIVER_CONTROLLER_PORT);
  }

  private void configureSubsystems(){
    drivetrain = DriveConstants.drivetrain;
  }

  private void configureCommands() {
    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, () -> m_driverControls.getDriveForward(),
        () -> m_driverControls.getDriveLeft(), () -> m_driverControls.getDriveRotation(),
        OperatorConstants.DEADZONE));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
