package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.drive.requests.CloseDriveToPoseRequest;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
@Logged
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem
{
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default
                                                                                                          // ramp rate
                                                                                                          // (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default ramp
                                                                                                    // rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    /*
     * SysId routine for characterizing rotation. This is used to find PID gains for
     * the FieldCentricFacingAngle HeadingController. See the documentation of
     * SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI), null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(output ->
            {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, this));

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can access
     * them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, Angle[] swerveOffsets,
            SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, modules[0].withEncoderOffset(swerveOffsets[0]),
                modules[1].withEncoderOffset(swerveOffsets[1]), modules[2].withEncoderOffset(swerveOffsets[2]),
                modules[3].withEncoderOffset(swerveOffsets[3]));
        if (Utils.isSimulation())
        {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can access
     * them through getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation())
        {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can access
     * them through getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation in the form [x, y, theta]ᵀ, with
     *                                  units in meters and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation in the form [x, y, theta]ᵀ, with
     *                                  units in meters and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation())
        {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder()
    {
        try
        {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(() -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            DriveConstants.kPPDrivePID,
                            // PID constants for rotation
                            DriveConstants.kPPDriveRPID),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this // Subsystem for
                                                                                                  // requirements
            );
        } catch (Exception ex)
        {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier)
    {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysId()
    {
        return Commands.sequence(m_sysIdRoutineTranslation.dynamic(Direction.kForward),
                m_sysIdRoutineTranslation.dynamic(Direction.kReverse),
                m_sysIdRoutineTranslation.quasistatic(Direction.kForward),
                m_sysIdRoutineTranslation.quasistatic(Direction.kReverse),
                m_sysIdRoutineSteer.dynamic(Direction.kForward), m_sysIdRoutineSteer.dynamic(Direction.kReverse),
                m_sysIdRoutineSteer.quasistatic(Direction.kForward),
                m_sysIdRoutineSteer.quasistatic(Direction.kReverse), m_sysIdRoutineRotation.dynamic(Direction.kForward),
                m_sysIdRoutineRotation.dynamic(Direction.kReverse),
                m_sysIdRoutineRotation.quasistatic(Direction.kForward),
                m_sysIdRoutineRotation.quasistatic(Direction.kReverse));
    }

    @Override
    public void periodic()
    {
        /*
         * Periodically try to apply the operator perspective. If we haven't applied the
         * operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match. Otherwise, only check and apply the operator perspective if the DS
         * is disabled. This ensures driving behavior doesn't change until an explicit
         * disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled())
        {
            DriverStation.getAlliance().ifPresent(allianceColor ->
            {
                setOperatorPerspectiveForward(allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread()
    {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() ->
        {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds)
    {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement in the form [x, y, theta]ᵀ, with
     *                                 units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs)
    {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    public Command driveByJoystick(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation)
    {
        SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        return applyRequest(() -> fieldCentric
                .withVelocityX(forward.getAsDouble() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                .withVelocityY(strafe.getAsDouble() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                .withRotationalRate(rotation.getAsDouble()
                        * RobotConstants.DriveConstants.kMaxAngularVelocity.in(RadiansPerSecond)));
    }

    public Pose2d getPose()
    {
        return getState().Pose;
    }

    public Command resetOrientation(Rotation2d orientation)
    {
        return runOnce(() -> resetRotation(orientation));
    }

    public Command resetOrientation()
    {
        return Commands.defer(() -> resetOrientation(getOperatorForwardDirection()), Set.of(this));
    }

    public Command pathfindToPose(Pose2d pose)
    {
        return AutoBuilder.pathfindToPose(pose, DriveConstants.kPPConstraints);
    }

    public Command closeDriveToPose(Pose2d pose)
    {
        CloseDriveToPoseRequest request = new CloseDriveToPoseRequest(pose, DriveConstants.kCloseDriveTP,
                DriveConstants.kCloseDriveTI, DriveConstants.kCloseDriveTD, DriveConstants.kCloseDriveRP,
                DriveConstants.kCloseDriveRI, DriveConstants.kCloseDriveRD, DriveConstants.kPPMaxVelocity);
        return applyRequest(() -> request).until(request::isFinished);
    }

    public Command driveToPose(Pose2d pose)
    {
        return Commands.sequence(pathfindToPose(pose), closeDriveToPose(pose));
    }

    public Command driveAtRobotRelativeSpeeds(ChassisSpeeds speeds)
    {
        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond);
        return applyRequest(() -> request);
    }

    public Command goForward(double speed)
    {
        return driveAtRobotRelativeSpeeds(new ChassisSpeeds(speed, 0, 0));
    }

    public Command goBackward(double speed)
    {
        return driveAtRobotRelativeSpeeds(new ChassisSpeeds(-speed, 0, 0));
    }

    public Command strafeLeft(double speed)
    {
        return driveAtRobotRelativeSpeeds(new ChassisSpeeds(0, speed, 0));
    }

    public Command strafeRight(double speed)
    {
        return driveAtRobotRelativeSpeeds(new ChassisSpeeds(0, -speed, 0));
    }

    private Angle resetEncoderAngle(int moduleIdx, Angle targetAngle)
    {
        final var module = getModule(moduleIdx);
        final var currentAngle = Rotations.of(module.getCurrentState().angle.getRotations());
        final var delta = targetAngle.minus(currentAngle);
        final var cancoder = module.getEncoder();
        final var config = new CANcoderConfiguration();
        cancoder.getConfigurator().refresh(config);
        final var currentOffest = Rotations.of(config.MagnetSensor.MagnetOffset);
        var newOffset = currentOffest.plus(delta);
        newOffset = Radians.of(MathUtil.angleModulus(newOffset.in(Radians)));
        config.MagnetSensor.MagnetOffset = newOffset.in(Rotations);
        cancoder.getConfigurator().apply(config);
        return newOffset;
    }

    public Angle[] resetEncoderAngles(Angle[] targetAngles)
    {
        final var newOffsets = new Angle[targetAngles.length];
        for (int i = 0; i < targetAngles.length; i++)
        {
            newOffsets[i] = resetEncoderAngle(i, targetAngles[i]);
        }
        return newOffsets;
    }

    public void resetDriveEncoders()
    {
        final var offsets = resetEncoderAngles(new Angle[]
        { Degrees.of(0), Degrees.of(0), Degrees.of(0), Degrees.of(0) });
        Preferences.setDouble("kSwerveOffsetFrontLeft", offsets[0].in(Rotations));
        Preferences.setDouble("kSwerveOffsetFrontRight", offsets[1].in(Rotations));
        Preferences.setDouble("kSwerveOffsetBackLeft", offsets[2].in(Rotations));
        Preferences.setDouble("kSwerveOffsetBackRight", offsets[3].in(Rotations));
    }

    public Command resetEncoders()
    {
        return Commands.runOnce(this::resetDriveEncoders);
    }
}