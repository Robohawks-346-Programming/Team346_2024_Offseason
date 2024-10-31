package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.NotePath;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class AutoFeed extends SequentialCommandGroup {
	CommandSwerveDrivetrain m_drive;
	Pivot m_pivot;
	NotePath m_notePath;

	public AutoFeed(CommandSwerveDrivetrain drive, Pivot pivot, NotePath notePath) {
		m_drive = drive;
		m_pivot = pivot;
		m_notePath = notePath;
		addRequirements(m_drive, m_pivot, m_notePath);
		addCommands(
				Commands.sequence(
						Commands.parallel(
								new RotateToFeed(m_drive).withTimeout(1),
								m_notePath.feed().withTimeout(1)),
						Commands.race(m_notePath.distanceShoot(), m_notePath.feed())));
	}
}
