package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.NotePath;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DistanceShoot extends SequentialCommandGroup {
	CommandSwerveDrivetrain m_drive;
	Pivot m_pivot;
	NotePath m_notePath;

	public DistanceShoot(CommandSwerveDrivetrain drive, Pivot pivot, NotePath notePath) {
		m_drive = drive;
		m_pivot = pivot;
		m_notePath = notePath;
		addRequirements(m_drive, m_pivot, m_notePath);
		addCommands(
				Commands.sequence(
						Commands.parallel(
								new RotateToSpeaker(m_drive).withTimeout(1.25),
								m_pivot.distanceBasedArmPivot(),
								m_notePath.rev().withTimeout(1.25)),
						Commands.race(m_notePath.distanceShoot(), m_notePath.rev()),
						m_pivot.moveArm(-55)));
	}
}
