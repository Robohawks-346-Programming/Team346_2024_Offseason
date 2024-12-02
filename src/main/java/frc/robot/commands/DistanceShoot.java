package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.NotePath.NotePath;
import frc.robot.subsystems.NotePath.NotePath.State;
import frc.robot.subsystems.Pivot;

public class DistanceShoot extends SequentialCommandGroup {
	CommandSwerveDrivetrain m_drive;
	Pivot m_pivot;
	NotePath m_notePath;

	public DistanceShoot(CommandSwerveDrivetrain drive, Pivot pivot, NotePath notePath) {
		m_drive = drive;
		m_pivot = pivot;
		m_notePath = notePath;
		addRequirements(m_drive);
		addCommands(
				Commands.sequence(
						Commands.parallel(
								new RotateToSpeaker(m_drive).withTimeout(1.5),
								m_pivot.distanceBasedArmPivot(),
								Commands.runEnd(() -> notePath.setState(State.REV), () -> notePath.setState(State.IDLE))
										.withTimeout(1.5)),
						Commands.race(
								Commands.runEnd(() -> notePath.setState(State.SPEAKER),
										() -> notePath.setState(State.IDLE)).withTimeout(1.5),
								Commands.runEnd(() -> notePath.setState(State.REV), () -> notePath.setState(State.IDLE))
										.withTimeout(1.5)),
						m_pivot.moveArm(-55)));
	}
}
