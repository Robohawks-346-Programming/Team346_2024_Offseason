package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
	CANSparkMax leftHook, rightHook;
	SparkPIDController leftHookPIDController, rightHookPIDController;
	RelativeEncoder leftHookEncoder, rightHookEncoder;
	boolean leftHookGood;
	boolean rightHookGood;

	public Climber() {
		leftHook = new CANSparkMax(Constants.ClimberConstants.LEFT_HOOK_MOTOR_ID, MotorType.kBrushless);
		rightHook = new CANSparkMax(Constants.ClimberConstants.RIGHT_HOOK_MOTOR_ID, MotorType.kBrushless);

		leftHookPIDController = leftHook.getPIDController();
		rightHookPIDController = rightHook.getPIDController();

		leftHookEncoder = leftHook.getEncoder();
		rightHookEncoder = rightHook.getEncoder();

		rightHook.setInverted(false);

		leftHookPIDController.setP(Constants.ClimberConstants.HOOK_P);
		leftHookPIDController.setI(Constants.ClimberConstants.HOOK_I);
		leftHookPIDController.setD(Constants.ClimberConstants.HOOK_D);
		leftHookPIDController.setFF(0.1);

		rightHookPIDController.setP(Constants.ClimberConstants.HOOK_P);
		rightHookPIDController.setI(Constants.ClimberConstants.HOOK_I);
		rightHookPIDController.setD(Constants.ClimberConstants.HOOK_D);
		leftHookPIDController.setFF(0.1);

		leftHook.setIdleMode(IdleMode.kBrake);
		rightHook.setIdleMode(IdleMode.kBrake);

		leftHook.burnFlash();
		rightHook.burnFlash();

		leftHookEncoder.setPosition(0);
		rightHookEncoder.setPosition(0);

	}

	@Override
	public void periodic() {
		// SmartDashboard.putBoolean("Left hook Good", leftHookGood);
		// SmartDashboard.putBoolean("Right hook Good", rightHookGood);

		// SmartDashboard.putNumber("Left hook rev", leftHookEncoder.getPosition());
		// SmartDashboard.putNumber("Right hook rev", rightHookEncoder.getPosition());
	}

	public boolean getHooksGood(double setpoint) {
		if (Math.abs(leftHookEncoder.getPosition() - setpoint) <= Constants.ClimberConstants.CLIMBER_REV_THRESHOLD) {
			leftHookGood = true;
		} else {
			leftHookGood = false;
		}

		if (Math.abs(rightHookEncoder.getPosition() - setpoint) <= Constants.ClimberConstants.CLIMBER_REV_THRESHOLD) {
			rightHookGood = true;
		} else {
			rightHookGood = false;
		}

		return leftHookGood && rightHookGood;
	}

	public void stopHooks() {
		leftHook.set(0);
		rightHook.set(0);
	}

	public Command rightHookUp() {
		return Commands.runEnd(() -> rightHook.set(0.8), () -> stopHooks());
	}

	public Command leftHookUp() {
		return Commands.runEnd(() -> leftHook.set(0.8), () -> stopHooks());
	}

	public Command moveHooksUp() {
		// leftHookPIDController.setReference(setpoint,
		// CANSparkMax.ControlType.kPosition);
		// rightHookPIDController.setReference(setpoint,
		// CANSparkMax.ControlType.kPosition);

		return Commands.runEnd(() -> Commands.parallel(
				this.rightHookUp(),
				this.leftHookUp()),
				() -> stopHooks());
	}

	public Command moveHooksDown() {
		return Commands.runEnd(() -> Commands.parallel(
				Commands.runEnd(() -> leftHook.set(-0.75), () -> stopHooks()),
				Commands.runEnd(() -> rightHook.set(-0.75), () -> stopHooks())),
				() -> stopHooks());
	}
}