// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.RobotConstants.DriveConstants;
import frc.robot.RobotConstants.SuperstructureGoal;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.apriltags.Localizer;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.commands.Outtake;
import frc.robot.subsystems.superstructure.Superstructure;

@Logged
public class RobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final SendableChooser<Command> autonomousChooser;
    private final Localizer localizer = new Localizer();
    private final Manipulator manipulator = new Manipulator();
    private final Superstructure superstructure;

    public RobotContainer()
    {
        drive = TunerConstants.createDrivetrain();
        superstructure = new Superstructure();
        configureBindings();
        autonomousChooser = getAutonomousChooser();
        SmartDashboard.putData("AutonomousChooser", autonomousChooser);
        SmartDashboard.putData("Test Left Reef", driveToReefLeft());
        SmartDashboard.putData("Reset Encoders", drive.resetEncoders());
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

        // manipulator.setDefaultCommand(
        // new IntakeWhileWaiting(manipulator).andThen(new
        // Purge(manipulator).withTimeout(Seconds.of(0.2)))
        // .andThen(new IntakeWhileWaiting(manipulator, 0.2)));

        driverController.back().onTrue(drive.resetOrientation());

        driverController.leftBumper().whileTrue(driveToReefLeft());

        driverController.rightBumper().whileTrue(driveToReefRight());

        driverController.x().whileTrue(driveToNearestCoralStation());

        manipulatorController.leftBumper().whileTrue(drive.strafeLeft(0.4));

        manipulatorController.rightBumper().whileTrue(drive.strafeRight(0.4));

        manipulatorController.rightTrigger()
                .whileTrue(Commands.either(
                        new Outtake(manipulator).withTimeout(Seconds.of(0.2))
                                .andThen(drive.strafeRight(0.3).withTimeout(0.5)),
                        new Outtake(manipulator), () -> superstructure.isAtHeight(SuperstructureGoal.L1.getState())));

        NamedCommands.registerCommand("DriveToCloseLeft", driveToReefLeft());
        NamedCommands.registerCommand("DriveToCloseRight", driveToReefRight());

        superstructure.setDefaultCommand(superstructure.moveByJoystick(processJoystick(manipulatorController::getLeftY),
                processJoystick(manipulatorController::getRightY)));

        manipulatorController.a().onTrue(superstructure.moveToIntake().withTimeout(1.75));
        manipulatorController.povDown().onTrue(superstructure.moveToL4().withTimeout(1.75));
        manipulatorController.povRight().onTrue(superstructure.moveToL3().withTimeout(1.75));
        manipulatorController.povUp().onTrue(superstructure.moveToL2().withTimeout(1.75));
        manipulatorController.povLeft().onTrue(superstructure.moveToL1().withTimeout(1.75));

        manipulatorController.start().whileTrue(superstructure.getHomingCommand());

        NamedCommands.registerCommand("GoToL4Goal", superstructure.moveToL4());
        NamedCommands.registerCommand("GoToL3Goal", superstructure.moveToL3());
        NamedCommands.registerCommand("GoToL2Goal", superstructure.moveToL2());
        NamedCommands.registerCommand("GoToL1Goal", superstructure.moveToL1());
        NamedCommands.registerCommand("GoToIntake", superstructure.moveToIntake());
    }

    @NotLogged
    public Command getAutonomousCommand()
    {
        return autonomousChooser.getSelected();
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

    public Pose2d applyPlacingOffset(Pose2d pose)
    {
        return FieldConstants.applyOffset(pose, DriveConstants.kPlacingOffset);
    }

    @NotLogged
    public Command driveToReefLeft()
    {
        return Commands.defer(() -> drive.driveToPose(applyPlacingOffset(getNearestReefSide().left())), Set.of(drive));
    }

    @NotLogged
    public Command driveToReefRight()
    {
        return Commands.defer(() -> drive.driveToPose(applyPlacingOffset(getNearestReefSide().right())), Set.of(drive));
    }

    public Pose2d getNearestCoralStationPose()
    {
        var left = FieldConstants.convertReefSideByAlliance(getNearestReefSide()).left();
        var right = FieldConstants.convertReefSideByAlliance(getNearestReefSide()).right();
        return getDistanceToPose(left).lte(getDistanceToPose(right)) ? left : right;
    }

    public Command driveToNearestCoralStation()
    {
        return Commands.defer(() -> drive.driveToPose(getNearestCoralStationPose()), Set.of(drive));
    }

    public Command simpleLeave()
    {
        return basicAutoResetForStart().andThen(drive.goForward(0.5).withTimeout(2));
    }

    public Command basicAutoResetForStart()
    {
        return Commands.runOnce(
                () -> drive.resetPose(FieldConstants.convertPoseByAlliance(new Pose2d(0, 0, Rotation2d.k180deg))));
    }

    @NotLogged
    public SendableChooser<Command> getAutonomousChooser()
    {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Leave", simpleLeave());
        chooser.addOption("Do nothing", Commands.print("Do nothing"));
        chooser.addOption("LEFT.J.K.L", new PathPlannerAuto("LEFT.J.K.L"));
        chooser.addOption("RIGHT.E.D.C", new PathPlannerAuto("RIGHT.E.D.C"));
        chooser.addOption("CENTER.PLACE.G", new PathPlannerAuto("CENTER.PLACE.G"));
        return chooser;
    }

    public void periodic()
    {
        localizer.updateWithReferencePose(drive.getPose());
        for (var pose : localizer.getEstimatedPoses())
        {
            drive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds,
                    VecBuilder.fill(0.9, 0.9, 999999));
        }
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
}
