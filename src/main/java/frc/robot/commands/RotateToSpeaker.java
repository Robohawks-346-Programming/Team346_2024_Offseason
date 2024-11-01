package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.*;

public class RotateToSpeaker extends Command {
	PIDController m_turnController;
	double turnSpeed;
	CommandSwerveDrivetrain m_drive;

	public RotateToSpeaker(CommandSwerveDrivetrain drive) {
		m_drive = drive;
		addRequirements(m_drive);
		m_turnController = new PIDController(0.1, 0, 0);
		m_turnController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
	}

	@Override
	public void execute() {
		m_drive.setControl(
				new SwerveRequest.FieldCentric()
						.withVelocityX(0)
						.withVelocityY(0)
						.withRotationalRate(m_turnController.calculate(m_drive.getPose().getRotation().getRadians(),
								Math.toRadians(m_drive.getHeadingToSpeaker())))
						.withDriveRequestType(DriveRequestType.Velocity));
	}

	@Override
	public void end(boolean interrupted) {
		m_drive.setControl(
				new SwerveRequest.FieldCentric()
						.withVelocityX(0)
						.withVelocityY(0)
						.withRotationalRate(0)
						.withDriveRequestType(DriveRequestType.Velocity));
	}

	@Override
	public boolean isFinished() {
		// SmartDashboard.putNumber("Finished Rotation",
		// drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees());
		return Math.abs(m_drive.getPose().getRotation().getDegrees() - m_drive.getHeadingToSpeaker()) <= 3;
	}

}