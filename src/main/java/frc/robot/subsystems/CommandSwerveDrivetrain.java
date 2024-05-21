package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;


public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SendableChooser<Command> autoChooser;

    private static final double kSimLoopPeriod = 0.005; // Original: 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants,
            double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Constants.currentMode == Constants.Mode.SIM) {
            startSimThread();
        }

        CommandScheduler.getInstance().registerSubsystem(this);
        configurePathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        setBrakeMode();
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Constants.currentMode == Constants.Mode.SIM) {
            startSimThread();
        }

        CommandScheduler.getInstance().registerSubsystem(this);
        configurePathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        setBrakeMode();
    }

    public void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        Pathfinding.setPathfinder(new LocalADStar());

        AutoBuilder.configureHolonomic(
            () -> getPose(), 
            this::seedFieldRelative, 
            this::getCurrentRobotChassisSpeeds, 
            (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), 
            Constants.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG, 
                () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                this);
        
        SmartDashboard.putData("Auto Chooser", autoChooser);

        PathPlannerPath traj1 = PathPlannerPath.fromChoreoTrajectory("StageTestBot");
        PathPlannerPath traj2 = PathPlannerPath.fromChoreoTrajectory("StageTestBot.1");
        PathPlannerPath traj3 = PathPlannerPath.fromChoreoTrajectory("StageTestBot.2");
    }

    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - lastSimTime;
                            lastSimTime = currentTime;

                            /* use the measured time delta, get battery voltage from WPILib */
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void setBrakeMode() {
        for (int i = 0; i < 3; i++){
            this.getModule(i)
                    .getDriveMotor()
                    .setNeutralMode(NeutralModeValue.Brake);
            this.getModule(i)
                    .getSteerMotor()
                    .setNeutralMode(NeutralModeValue.Coast);
        }
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
}