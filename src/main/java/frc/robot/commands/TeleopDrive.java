package frc.robot.commands;

import java.util.Currency;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class TeleopDrive extends Command {
	CommandSwerveDrivetrain m_drive;
	Supplier<Double> xSpeed;
	Supplier<Double> ySpeed;
	Supplier<Double> zRotation;
	double curXSpeed;
	double curYSpeed;
	double curZRotation;

	double headingTarget;
	double deadzone;

	public TeleopDrive(CommandSwerveDrivetrain drive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
			Supplier<Double> zRotation, double deadzone) {
		m_drive = drive;
		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.zRotation = zRotation;
		this.deadzone = deadzone;
		headingTarget = m_drive.getPose().getRotation().getRadians();
		addRequirements(drive);
	}

	@Override
	public void execute() {
		curXSpeed = xSpeed.get();
		curYSpeed = ySpeed.get();
		curZRotation = zRotation.get();
		curXSpeed *= DriveConstants.MAX_MOVE_VELOCITY;
		curYSpeed *= DriveConstants.MAX_MOVE_VELOCITY;
		curZRotation *= DriveConstants.MAX_TURN_VELOCITY;
		// System.out.println("Running");
		if (DriverStation.getAlliance().isPresent()
				&& DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
			curXSpeed *= -1;
			curYSpeed *= -1;
		}

		m_drive.drive(curXSpeed, curYSpeed, curZRotation);
	}
}