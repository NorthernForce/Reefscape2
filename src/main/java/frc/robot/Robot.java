// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged
public class Robot extends TimedRobot
{
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot()
    {
        m_robotContainer = new RobotContainer();
        DataLogManager.start();
        Epilogue.configure(config ->
        {
            config.root = "Robot";
            config.errorHandler = ErrorHandler.printErrorMessages();
            config.loggingPeriod = Seconds.of(0.1);
        });
        Epilogue.bind(this);
    }

    @Override
    public void robotPeriodic()
    {
        if (DriverStation.isFMSAttached())
        {
            try
            {
                m_robotContainer.periodic();
                CommandScheduler.getInstance().run();
            } catch (Exception e)
            {
                DriverStation.reportError(e.getMessage(), e.getStackTrace());
            }
        } else
        {
            m_robotContainer.periodic();
            CommandScheduler.getInstance().run();
        }
    }

    @Override
    public void disabledInit()
    {
    }

    @Override
    public void disabledPeriodic()
    {
    }

    @Override
    public void disabledExit()
    {
    }

    @Override
    public void autonomousInit()
    {
        m_robotContainer.autonomousInit();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        m_robotContainer.resetPose();
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void autonomousExit()
    {
    }

    @Override
    public void teleopInit()
    {
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void teleopExit()
    {
    }

    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic()
    {
    }

    @Override
    public void testExit()
    {
    }
}
