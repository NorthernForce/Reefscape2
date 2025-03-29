// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.RobotConstants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.apriltags.Localizer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

@Logged
public class RobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final SendableChooser<Command> autonomousChooser;
    private final Localizer localizer = new Localizer();
    private final Climber climber;
    private final CommandXboxController driverController;

    public RobotContainer()
    {
        driverController = new CommandXboxController(0);
        climber = new Climber(RobotConstants.ClimberConstants.kId, RobotConstants.ClimberConstants.kEncoderId,
                RobotConstants.ClimberConstants.kClimbSpeed);
        drive = TunerConstants.createDrivetrain();
        configureBindings();
        autonomousChooser = getAutonomousChooser();
        SmartDashboard.putData("AutonomousChooser", autonomousChooser);
        SmartDashboard.putData("Test Left Reef", driveToReefLeft());

        driverController.x().whileTrue(climber.getExtendCommand());
        driverController.y().whileTrue(climber.getRetractCommand());
    }

    private static DoubleSupplier processJoystick(DoubleSupplier joystick)
    {
        return () ->
        {
            var x = MathUtil.applyDeadband(joystick.getAsDouble(), 0.1);
            return x * Math.abs(x);
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

        NamedCommands.registerCommand("DriveToCloseLeft", driveToReefLeft());
        NamedCommands.registerCommand("DriveToCloseRight", driveToReefRight());
    }

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

    public Command driveToReefLeft()
    {
        return Commands.defer(() -> drive.driveToPose(applyPlacingOffset(getNearestReefSide().left())), Set.of(drive));
    }

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
}
