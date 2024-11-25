// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.TestDrive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and
 * CANcoder
 *
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware
 * configurations (e.g. If using an analog encoder, copy from
 * "ModuleIOSparkMax")
 *
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward
 * motion on the drive motor will propel the robot forward) and copy the
 * reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOKraken implements ModuleIO {
	private final TalonFX driveTalon;
	private final TalonFX turnTalon;
	private final CANcoder cancoder;

	private final Queue<Double> timestampQueue;

	private final StatusSignal<Double> drivePosition;
	private final Queue<Double> drivePositionQueue;
	private final StatusSignal<Double> driveVelocity;
	private final StatusSignal<Double> driveAppliedVolts;
	private final StatusSignal<Double> driveCurrent;

	private final StatusSignal<Double> turnAbsolutePosition;
	private final StatusSignal<Double> turnPosition;
	private final Queue<Double> turnPositionQueue;
	private final StatusSignal<Double> turnVelocity;
	private final StatusSignal<Double> turnAppliedVolts;
	private final StatusSignal<Double> turnCurrent;

	private final CTREConfigs configs;

	// Gear ratios for SDS MK4i L2, adjust as necessary
	private final double DRIVE_GEAR_RATIO = DriveConstants.DRIVETRAIN_GEAR_RATIO;
	private final double TURN_GEAR_RATIO = DriveConstants.TURN_CONVERSION;

	private final boolean isTurnMotorInverted = true;
	private final Rotation2d absoluteEncoderOffset;

	public ModuleIOKraken(int index) {
		switch (index) {
			case 0:
				driveTalon = new TalonFX(21);
				turnTalon = new TalonFX(22);
				cancoder = new CANcoder(23);
				absoluteEncoderOffset = Rotation2d.fromRotations(0.68212890625);
				break;
			case 1:
				driveTalon = new TalonFX(24);
				turnTalon = new TalonFX(25);
				cancoder = new CANcoder(26);
				absoluteEncoderOffset = Rotation2d.fromRotations(0.916259765625);
				break;
			case 2:
				driveTalon = new TalonFX(31);
				turnTalon = new TalonFX(32);
				cancoder = new CANcoder(33);
				absoluteEncoderOffset = Rotation2d.fromRotations(0.31982421875);
				break;
			case 3:
				driveTalon = new TalonFX(34);
				turnTalon = new TalonFX(35);
				cancoder = new CANcoder(36);
				absoluteEncoderOffset = Rotation2d.fromRotations(0.071533203125);
				break;
			default:
				throw new RuntimeException("Invalid module index");
		}

		configs = new CTREConfigs();

		cancoder.getConfigurator().apply(configs.swerveCANcoderConfig);
		driveTalon.getConfigurator().apply(configs.swerveDriveFXConfig);
		turnTalon.getConfigurator().apply(configs.swerveAngleFXConfig);

		timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

		drivePosition = driveTalon.getPosition();
		drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
		driveVelocity = driveTalon.getVelocity();
		driveAppliedVolts = driveTalon.getMotorVoltage();
		driveCurrent = driveTalon.getSupplyCurrent();

		turnAbsolutePosition = cancoder.getAbsolutePosition();
		turnPosition = turnTalon.getPosition();
		turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
		turnVelocity = turnTalon.getVelocity();
		turnAppliedVolts = turnTalon.getMotorVoltage();
		turnCurrent = turnTalon.getSupplyCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(
				Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
		BaseStatusSignal.setUpdateFrequencyForAll(
				50.0,
				driveVelocity,
				driveAppliedVolts,
				driveCurrent,
				turnAbsolutePosition,
				turnVelocity,
				turnAppliedVolts,
				turnCurrent);
		driveTalon.optimizeBusUtilization();
		turnTalon.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		BaseStatusSignal.refreshAll(
				drivePosition,
				driveVelocity,
				driveAppliedVolts,
				driveCurrent,
				turnAbsolutePosition,
				turnPosition,
				turnVelocity,
				turnAppliedVolts,
				turnCurrent);

		inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
		inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
		inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
		inputs.driveCurrentAmps = new double[] { driveCurrent.getValueAsDouble() };

		inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
				.minus(absoluteEncoderOffset);
		inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
		inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
		inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
		inputs.turnCurrentAmps = new double[] { turnCurrent.getValueAsDouble() };

		inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
		inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
				.mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
				.toArray();
		inputs.odometryTurnPositions = turnPositionQueue.stream()
				.map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
				.toArray(Rotation2d[]::new);
		timestampQueue.clear();
		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveTalon.setControl(new VoltageOut(volts));
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnTalon.setControl(new VoltageOut(volts));
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		var config = new MotorOutputConfigs();
		config.Inverted = InvertedValue.CounterClockwise_Positive;
		config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		driveTalon.getConfigurator().apply(config);
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		var config = new MotorOutputConfigs();
		config.Inverted = isTurnMotorInverted
				? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		turnTalon.getConfigurator().apply(config);
	}
}