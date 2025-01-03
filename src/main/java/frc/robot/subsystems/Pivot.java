package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Pivot extends SubsystemBase {
	private static TalonFX pivotMotor;
	private static TalonFXConfiguration pivotMotorConfig;

	private PositionVoltage position;
	private MotionMagicExpoVoltage volt;
	private MotionMagicConfigs configs;

	public final InterpolatingDoubleTreeMap pivotLookupTable = Constants.PivotConstants.getPivotMap();

	private CommandSwerveDrivetrain m_drive;

	private ArmFeedforward feedforward = new ArmFeedforward(
			Constants.PivotConstants.PIVOT_kS,
			Constants.PivotConstants.PIVOT_kG,
			Constants.PivotConstants.PIVOT_kV,
			Constants.PivotConstants.PIVOT_kA);

	public Pivot(CommandSwerveDrivetrain drive) {
		pivotMotor = new TalonFX(Constants.PivotConstants.PIVOT_MOTOR_ID);
		pivotMotorConfig = new TalonFXConfiguration();
		pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		pivotMotor.setInverted(true);

		pivotMotorConfig.Slot0.kP = Constants.PivotConstants.PIVOT_P;
		pivotMotorConfig.Slot0.kI = Constants.PivotConstants.PIVOT_I;
		pivotMotorConfig.Slot0.kD = Constants.PivotConstants.PIVOT_D;
		pivotMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		pivotMotorConfig.Slot0.kG = Constants.PivotConstants.PIVOT_kG;
		pivotMotorConfig.Slot0.kA = Constants.PivotConstants.PIVOT_kA;
		pivotMotorConfig.Slot0.kV = Constants.PivotConstants.PIVOT_kV;
		pivotMotorConfig.Slot0.kS = Constants.PivotConstants.PIVOT_kS;

		pivotMotorConfig.Feedback.SensorToMechanismRatio = Constants.PivotConstants.PIVOT_GEAR_RATIO;

		position = new PositionVoltage(0);

		volt = new MotionMagicExpoVoltage(0);
		volt.EnableFOC = true;
		configs = pivotMotorConfig.MotionMagic;
		configs.MotionMagicExpo_kA = 1;
		configs.MotionMagicExpo_kV = Constants.PivotConstants.PIVOT_kV;

		pivotMotor.getConfigurator().apply(pivotMotorConfig);

		// pivotMotor.setPosition(convertDegreesToRotations(Constants.PivotConstants.HOME_PIVOT_ANGLE));
		resetPivotAngle();

		m_drive = drive;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Arm Degrees", convertRotationsToDegrees(pivotMotor.getPosition().getValue()));
	}

	public boolean isAtPosition() {
		return (Math.abs(pivotLookupTable.get(m_drive.getDistanceFromSpeaker())
				- convertRotationsToDegrees(pivotMotor.getPosition().getValueAsDouble())) < 2);
	}

	public void moveArmToPosition(double wantedPosition) {
		// pivotMotor.setControl(position.withPosition(convertDegreesToRotations(wantedPosition)));
		pivotMotor.setControl(volt.withPosition(convertDegreesToRotations(wantedPosition)));
	}

	public double convertDegreesToRotations(double degrees) {
		return (degrees / 360.0);
	}

	public double convertRotationsToDegrees(double rotations) {
		return (rotations * 360.0);
	}

	public Command moveArm(double pos) {
		return Commands.runOnce(() -> pivotMotor.setControl(volt.withPosition(convertDegreesToRotations(pos))));
	}

	public Command distanceBasedArmPivot() {
		// SmartDashboard.putNumber("Distance from Speaker",
		// RobotContainer.drivetrain.getDistanceFromSpeaker());
		// SmartDashboard.putNumber("Wanted Arm Angle",
		// pivotLookupTable.get(RobotContainer.drivetrain.getDistanceFromSpeaker()));
		return Commands.runOnce(() -> pivotMotor.setControl(volt.withPosition(
				convertDegreesToRotations(pivotLookupTable.get(m_drive.getDistanceFromSpeaker())))));
	}

	public void resetPivotAngle() {
		pivotMotor.setPosition(convertDegreesToRotations(Constants.PivotConstants.HOME_PIVOT_ANGLE));
	}

	public void stopPivot() {
		pivotMotor.stopMotor();
	}

	public double getPosition() {
		return pivotMotor.getPosition().getValue();
	}

	public double getDegrees() {
		return convertRotationsToDegrees(pivotMotor.getPosition().getValue());
	}

	public void setPercent() {
		pivotMotor.set(-0.06);
	}

	public Command driveDown() {
		return Commands.runEnd(() -> pivotMotor.set(-0.02), () -> pivotMotor.set(0));
	}

	public Command feedPivot() {
		return Commands.runOnce(() -> pivotMotor.setControl(position.withPosition(
				convertDegreesToRotations(
						Math.toDegrees(
								Math.asin((m_drive.getDistanceFromSpeaker() * 9.81) / (Math.pow(28.728, 2)))
										* 0.5)))));
	}
}