package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Telemetry {
    private final double maxSpeed;

    private final TelemetryIO telemetryIo;
    private final TelemetryIOInputsAutoLogged telemetryInputs = new TelemetryIOInputsAutoLogged();

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed, TelemetryIO telemetryIo) {
        this.maxSpeed = maxSpeed;
        this.telemetryIo = telemetryIo;
    }

    /* What to publish over networktables for telemetry */
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    NetworkTable table = inst.getTable("Pose");
    DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    double robotRotation = 0;
    double robotRotationVelocity = 0;
    double robotRotationLast = 0;

    /* Robot speeds for general checking */
    NetworkTable driveStats = inst.getTable("Drive");
    DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
    DoublePublisher accelX = driveStats.getDoubleTopic("Acceleration X").publish();
    DoublePublisher accelY = driveStats.getDoubleTopic("Acceleration Y").publish();
    DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();

    double velocityXFiltered = 0.0;
    double velocityYFiltered = 0.0;
    LinearFilter velocityXFilter = LinearFilter.singlePoleIIR(0.1, 0.020);
    LinearFilter velocityYFilter = LinearFilter.singlePoleIIR(0.1, 0.020);
    LinearFilter accelXFilter = LinearFilter.singlePoleIIR(0.1, 0.020);
    LinearFilter accelYFilter = LinearFilter.singlePoleIIR(0.1, 0.020);

    double accelXFiltered = 0.0;
    double accelYFiltered = 0.0;
    /* Keep a reference of the last pose to calculate the speeds */
    Pose2d latestPose = new Pose2d();
    double lastTime = Utils.getCurrentTimeSeconds();

    SwerveModuleState[] latestModuleStates = new SwerveModuleState[4];

    /* Mechanisms to represent the swerve module states */
    Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
            m_moduleMechanisms[0]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[1]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[2]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[3]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
            m_moduleMechanisms[0]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new MechanismLigament2d(
                                    "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[1]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new MechanismLigament2d(
                                    "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[2]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new MechanismLigament2d(
                                    "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            m_moduleMechanisms[3]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new MechanismLigament2d(
                                    "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        /*
         * PSA: Do not call the Logger in this method. This method is called
         * by a separate thread, and the Logger does not support multi-threading.
         * Your robot program will crash.
         */
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });

        robotRotation = pose.getRotation().getRadians();

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(latestPose).getTranslation();

        Translation2d velocityFieldRelative = new Translation2d(pose.getX() - latestPose.getX(),
                pose.getY() - latestPose.getY())
                .div(diffTime);

        latestPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        double robotRotationDiff = robotRotation - robotRotationLast;
        robotRotationVelocity = robotRotationDiff / diffTime;

        robotRotationLast = robotRotation;

        speed.set(velocities.getNorm());
        velocityX.set(velocities.getX());
        velocityY.set(velocities.getY());
        accelX.set(telemetryInputs.accelerationX);
        accelY.set(telemetryInputs.accelerationY);

        velocityXFiltered = velocityXFilter.calculate(velocityFieldRelative.getX());
        velocityYFiltered = velocityYFilter.calculate(velocityFieldRelative.getY());
        accelXFiltered = accelXFilter.calculate(telemetryInputs.accelerationX);
        accelYFiltered = accelYFilter.calculate(telemetryInputs.accelerationY);
        odomPeriod.set(state.OdometryPeriod);

        latestModuleStates = state.ModuleStates;

        /* Telemeterize the module's states */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(
                    state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed));

            moduleStates[i] = state.ModuleStates[i];
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    /**
     * Calls the Logger to publish telemetry data. Call this method from the same
     * thread as
     * CommandScheduler.
     */
    public void logDataSynchronously() {
        Pose2d pose = new Pose2d(latestPose.getX(), latestPose.getY(), latestPose.getRotation());

        telemetryIo.setRobotPose(getFieldToRobot3d());
        telemetryIo.setRobotPose(pose);
        telemetryIo.setSwerveModuleStates(moduleStates);

        telemetryIo.setRobotRotation(robotRotation);
        telemetryIo.setRobotRotationVelocity(robotRotationVelocity);

        telemetryIo.updateInputs(telemetryInputs);
        Logger.processInputs("telemetry", telemetryInputs);
    }

    public double getRotationRadians() {
        return robotRotation;
    }

    public double getRobotRotationRadians() {
        return robotRotationVelocity;
    }

    public Pose2d getFieldToRobot() {
        return latestPose;
    }

    public Pose3d getFieldToRobot3d() {
        return new Pose3d(
                new Translation3d(latestPose.getX(), latestPose.getY(), 0),
                new Rotation3d(0, 0, getRotationRadians()));
    }

    public Translation2d getVelocity() {
        return new Translation2d(velocityXFiltered, velocityYFiltered);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            var state = latestModuleStates[i];
            states[i] = new SwerveModuleState(
                    state.speedMetersPerSecond,
                    Rotation2d.fromRadians(state.angle.getRadians()));
        }
        return states;
    }

    public double getVelocityX() {
        return velocityXFiltered;
    }

    public double getVelocityY() {
        return velocityYFiltered;
    }

    public double getAccelerationX() {
        return accelXFiltered;
    }

    public double getAccelerationY() {
        return accelYFiltered;
    }
}