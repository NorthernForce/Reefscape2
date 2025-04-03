// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.RobotConstants.DriveConstants;
import frc.robot.RobotConstants.SuperstructureGoal;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algae_extractor.AlgaeExtractor;
import frc.robot.subsystems.apriltags.Localizer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.leds.LEDS;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;

@Logged
public class RobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final SendableChooser<Pair<String, Command>> autonomousChooser;
    private final Localizer localizer = new Localizer();
    private final Manipulator manipulator = new Manipulator();
    private final Climber climber;
    private final Superstructure superstructure;
    private final AlgaeExtractor algaeExtractor;
    private final Vision vision;
    public boolean isFirstTime = false;

    private final LEDS leds;

    private boolean isRightSideAuto;

    public RobotContainer()
    {
        climber = new Climber(RobotConstants.ClimberConstants.kId, RobotConstants.ClimberConstants.kClimbSpeed,
                RobotConstants.ClimberConstants.kInverted);
        drive = TunerConstants.createDrivetrain();
        superstructure = new Superstructure();
        algaeExtractor = new AlgaeExtractor(RobotConstants.AlgaeRemoverConstants.kMotorId,
                RobotConstants.AlgaeRemoverConstants.kSensorId, RobotConstants.AlgaeRemoverConstants.kInverted,
                RobotConstants.AlgaeRemoverConstants.kGearRatio, RobotConstants.AlgaeRemoverConstants.kRemovingSpeed,
                RobotConstants.AlgaeRemoverConstants.kReturningSpeed);
        vision = new Vision("skynet", "Viewer");
        leds = new LEDS(RobotConstants.LEDConstants.kCANId, RobotConstants.LEDConstants.kLEDCount);
        configureBindings();
        autonomousChooser = getAutonomousChooser();
        SmartDashboard.putData("AutonomousChooser", autonomousChooser);
        SmartDashboard.putData("Test Left Reef", driveToReefLeft());
        SmartDashboard.putData("Reset Encoders", drive.resetEncoders());
        PortForwarder.add(1181, "10.1.72.11", 5800);
        PortForwarder.add(1182, "10.1.72.11", 1181);
        PortForwarder.add(1183, "10.1.72.13", 5800);
        PortForwarder.add(1184, "10.1.72.13", 1181);
        for (int i = 0; i <= 5; i++)
        {
            PortForwarder.add(5800 + i, "10.1.72.12", 5800 + i);
        }
        for (int i = 0; i <= 4; i++)
        {
            PortForwarder.add(5805 + i, "10.1.72.15", 5800 + i);
        }
        PortForwarder.add(1185, "10.1.72.15", 5805);
        PortForwarder.add(1186, "10.1.72.36", 1181);
        PortForwarder.add(1187, "10.1.72.36", 1182);
    }

    private static DoubleSupplier processJoystick(DoubleSupplier joystick)
    {
        return () ->
        {
            var x = MathUtil.applyDeadband(joystick.getAsDouble(), 0.1);
            return -x * Math.abs(x);
        };
    }

    private void configureBindings()
    {
        CommandXboxController driverController = new CommandXboxController(0);
        CommandXboxController manipulatorController = new CommandXboxController(1);

        drive.setDefaultCommand(drive.driveByJoystick(processJoystick(driverController::getLeftY),
                processJoystick(driverController::getLeftX), processJoystick(driverController::getRightX)));

        driverController.back().onTrue(drive.resetOrientation());

        driverController.leftBumper().whileTrue(driveToReefLeft());

        driverController.rightBumper().whileTrue(driveToReefRight());

        driverController.x().whileTrue(driveToNearestCoralStation());

        manipulatorController.leftBumper().whileTrue(drive.strafeLeft(0.4));

        manipulatorController.rightBumper().whileTrue(drive.strafeRight(0.4));

        // SUPER CONTROLLER MAPPINGS

        driverController.rightTrigger()
                .whileTrue(Commands.either(manipulator.slowOuttake().andThen(drive.strafeRight(0.8).withTimeout(0.5)),
                        manipulator.outtake(), () -> superstructure.isAtHeight(SuperstructureGoal.L1.getState())));

        driverController.leftTrigger().whileTrue(algaeExtractor.getExtractCommand());

        driverController.povDown()
                .onTrue(superstructure.moveToL4().withTimeout(1.75).onlyIf(() -> manipulator.hasCoral()));
        driverController.povRight()
                .onTrue(superstructure.moveToL3().withTimeout(1.75).onlyIf(() -> manipulator.hasCoral()));
        driverController.povUp()
                .onTrue(superstructure.moveToL2().withTimeout(1.75).onlyIf(() -> manipulator.hasCoral()));
        driverController.povLeft()
                .onTrue(superstructure.moveToL1().withTimeout(1.75).onlyIf(() -> manipulator.hasCoral()));

        driverController.y().onTrue(superstructure.moveToIntake().withTimeout(1.75));

        // END OF SUPER CONTROLLER MAPPINGS

        manipulatorController.rightTrigger()
                .whileTrue(Commands.either(manipulator.slowOuttake().andThen(drive.strafeRight(0.8).withTimeout(0.5)),
                        manipulator.outtake(), () -> superstructure.isAtHeight(SuperstructureGoal.L1.getState()))
                        .onlyIf(() -> superstructure.isAtTargetState()));

        manipulatorController.leftTrigger().whileTrue(algaeExtractor.getExtractCommand());

        algaeExtractor.setDefaultCommand(algaeExtractor.getReturnCommand());
        driverController.a().whileTrue(climber.getExtendCommand());
        driverController.b().whileTrue(climber.getRetractCommand());

        NamedCommands.registerCommand("DriveToCloseLeft",
                Commands.either(driveToReefRight(), driveToReefLeft(), () -> isRightSideAuto));
        NamedCommands.registerCommand("DriveToCloseRight",
                Commands.either(driveToReefLeft(), driveToReefRight(), () -> isRightSideAuto));

        superstructure.setDefaultCommand(superstructure.moveByJoystick(processJoystick(manipulatorController::getLeftY),
                processJoystick(manipulatorController::getRightY)));

        manipulatorController.a().onTrue(superstructure.moveToIntake().withTimeout(1.75));
        manipulatorController.povDown()
                .onTrue(superstructure.moveToL4().withTimeout(1.75).onlyIf(() -> manipulator.hasCoral()));
        manipulatorController.povRight()
                .onTrue(superstructure.moveToL3().withTimeout(1.75).onlyIf(() -> manipulator.hasCoral()));
        manipulatorController.povUp()
                .onTrue(superstructure.moveToL2().withTimeout(1.75).onlyIf(() -> manipulator.hasCoral()));
        manipulatorController.povLeft()
                .onTrue(superstructure.moveToL1().withTimeout(1.75).onlyIf(() -> manipulator.hasCoral()));

        manipulatorController.start().whileTrue(superstructure.getHomingCommand());

        NamedCommands.registerCommand("GoToL4Goal", superstructure.holdAtL4());
        NamedCommands.registerCommand("GoToL3Goal", superstructure.holdAtL3());
        NamedCommands.registerCommand("GoToL2Goal", superstructure.holdAtL2());
        NamedCommands.registerCommand("GoToL1Goal", superstructure.holdAtL1());
        NamedCommands.registerCommand("GoToIntake", superstructure.moveToIntake());
        NamedCommands.registerCommand("Intake", manipulator.intake());
        NamedCommands.registerCommand("Outtake",
                Commands.deadline(manipulator.outtake().andThen(Commands.waitSeconds(0.1)), superstructure.holdAtL4()));
        NamedCommands.registerCommand("RemoveAlgae",
                drive.strafeLeft(0.1).withTimeout(0.1)
                        .andThen(algaeExtractor.getExtractCommand().alongWith(drive.goBackward(0.2).withTimeout(2)))
                        .andThen(drive.stop()));

        leds.setDefaultCommand(leds.noAlliance());

        new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                && DriverStation.isDisabled() && isFirstTime).whileTrue(leds.redAlliance());

        new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                && DriverStation.isDisabled() && isFirstTime).whileTrue(leds.blueAlliance());

        new Trigger(() -> DriverStation.isAutonomousEnabled()).whileTrue(leds.auto());

        new Trigger(() -> DriverStation.getMatchTime() <= 20 && DriverStation.isTeleopEnabled()
                && manipulator.hasCoral() && !vision.isAligned()).whileTrue(leds.endgame());

        new Trigger(() -> DriverStation.getMatchTime() > 20 && DriverStation.isTeleopEnabled() && manipulator.hasCoral()
                && !vision.isAligned()).whileTrue(leds.happy());

        new Trigger(() -> DriverStation.isTeleopEnabled() && !manipulator.hasCoral()).whileTrue(leds.hungry());

        new Trigger(() -> DriverStation.isTeleopEnabled() && vision.isAligned()).whileTrue(leds.readyToPlace());
    }

    @NotLogged
    public Command getAutonomousCommand()
    {
        if (autonomousChooser.getSelected().getFirst() == "RIGHT.E.D.C")
        {
            isRightSideAuto = true;
        } else
        {
            isRightSideAuto = false;
        }
        return autonomousChooser.getSelected().getSecond();
    }

    public Command bruteOuttake()
    {
        return Commands.runOnce(() -> manipulator.setState(ManipulatorState.BRUTE_OUTTAKING))
                .until(() -> !manipulator.hasCoralInSensor());
    }

    public Distance getDistanceToPose(Pose2d pose)
    {
        return Meters.of(drive.getPose().getTranslation().getDistance(pose.getTranslation()));
    }

    public ReefSide getNearestReefSide()
    {
        ReefSide closest = FieldConstants.convertReefSideByAlliance(FieldConstants.ReefPositions.REEF_SIDES[0]);
        for (ReefSide side : FieldConstants.ReefPositions.REEF_SIDES)
        {
            side = FieldConstants.convertReefSideByAlliance(side);
            if (getDistanceToPose(side.center()).in(Meters) < getDistanceToPose(closest.center()).in(Meters))
            {
                closest = side;
            }
        }

        return closest;
    }

    public Pose2d getLeftPose()
    {
        return applyPlacingOffset(getNearestReefSide().left());
    }

    public Pose2d getRightPose()
    {
        return applyPlacingOffset(getNearestReefSide().right());
    }

    public Pose2d applyPlacingOffset(Pose2d pose)
    {
        return FieldConstants.applyOffset(pose, DriveConstants.kPlacingOffset);
    }

    @NotLogged
    public Command driveToReefLeft()
    {
        return Commands.defer(() -> drive.closeDriveToPose(applyPlacingOffset(getNearestReefSide().left()),
                () -> vision.getXOffset()), Set.of(drive, vision));
    }

    @NotLogged
    public Command driveToReefRight()
    {
        return Commands.defer(() -> drive.closeDriveToPose(applyPlacingOffset(getNearestReefSide().right()),
                () -> vision.getXOffset()), Set.of(drive, vision));
    }

    public Pose2d getNearestCoralStationPose()
    {
        var left = FieldConstants.convertPoseByAlliance(FieldConstants.CoralStations.LEFT);
        var right = FieldConstants.convertPoseByAlliance(FieldConstants.CoralStations.RIGHT);
        return getDistanceToPose(left).lte(getDistanceToPose(right)) ? left : right;
    }

    public Command driveToNearestCoralStation()
    {
        return Commands.defer(() -> drive.closeDriveToPose(getNearestCoralStationPose(), () -> vision.getXOffset()),
                Set.of(drive, vision));
    }

    public Command simpleLeave()
    {
        return basicAutoResetForStart().andThen(drive.goForward(1.0).withTimeout(2));
    }

    public Command basicAutoResetForStart()
    {
        return Commands.runOnce(
                () -> drive.resetPose(FieldConstants.convertPoseByAlliance(new Pose2d(0, 0, Rotation2d.k180deg))));
    }

    @NotLogged
    public SendableChooser<Pair<String, Command>> getAutonomousChooser()
    {
        SendableChooser<Pair<String, Command>> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Leave", Pair.of("Leave", simpleLeave()));
        chooser.addOption("Do nothing", Pair.of("Do nothing", Commands.print("Do nothing")));
        chooser.addOption("LEFT.J.K.L", Pair.of("LEFT.J.K.L", new PathPlannerAuto("LEFT.J.K.L")));
        chooser.addOption("RIGHT.E.D.C", Pair.of("RIGHT.E.D.C", new PathPlannerAuto("LEFT.J.K.L", true)));
        chooser.addOption("CENTER.PLACE.G", Pair.of("CENTER.PLACE.G", new PathPlannerAuto("CENTER.PLACE.G")));
        return chooser;
    }

    public void autonomousInit()
    {
        if (manipulator.hasCoralInSensor())
        {
            manipulator.setState(ManipulatorState.HAPPY);
        } else
        {
            manipulator.setState(ManipulatorState.HUNGRY);
        }
    }

    public void periodic()
    {
        localizer.updateWithReferencePose(drive.getPose());
        for (var pose : localizer.getEstimatedPoses())
        {
            drive.addVisionMeasurement(pose.pose(), pose.timestamp(), VecBuilder.fill(0.9, 0.9, 999999));
        }
        manipulator.setCanIntake(superstructure.isAtHeight(SuperstructureGoal.CORAL_STATION.getState()));
    }

    public Pose3d[] getComponentPoses()
    {
        return new Pose3d[]
        { new Pose3d(Inches.zero(), Inches.zero(), superstructure.getOuterElevator().getHeight(), Rotation3d.kZero),
                new Pose3d(Inches.zero(), Inches.zero(),
                        superstructure.getInnerElevator().getHeight()
                                .plus(superstructure.getOuterElevator().getHeight()),
                        Rotation3d.kZero),
                Pose3d.kZero, Pose3d.kZero };
    }

    public Pose3d getRobotPose3d()
    {
        return new Pose3d(drive.getPose());
    }

    public void resetPose()
    {
        drive.resetPose(new Pose2d());
    }

    public String[] getCommandsRunning()
    {
        ArrayList<String> commandsRunning = new ArrayList<>();
        if (drive.getCurrentCommand() != null)
        {
            commandsRunning.add(drive.getCurrentCommand().getName());
        }
        if (manipulator.getCurrentCommand() != null)
        {
            commandsRunning.add(manipulator.getCurrentCommand().getName());
        }
        if (superstructure.getCurrentCommand() != null)
        {
            commandsRunning.add(superstructure.getCurrentCommand().getName());
        }
        return commandsRunning.toArray(new String[0]);
    }
}
