// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

@Logged
public class RobotContainer
{
    private final CommandSwerveDrivetrain m_swerveDrivetrain;

    public RobotContainer()
    {
        m_swerveDrivetrain = TunerConstants.createDrivetrain();
        configureBindings();
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
        m_swerveDrivetrain
                .setDefaultCommand(m_swerveDrivetrain.driveByJoystick(processJoystick(driverController::getLeftY),
                        processJoystick(driverController::getLeftX), processJoystick(driverController::getRightX)));

        driverController.back().onTrue(m_swerveDrivetrain.resetOrientation());

    }

    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
