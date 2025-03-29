package frc.robot.subsystems.manipulator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.manipulator.Manipulator;

public class Outtake extends Command {
    private final double m_speed;
    private final Manipulator m_manipulator;

    public Outtake(Manipulator manipulator, double speed)
    {
        addRequirements(manipulator);
        m_manipulator = manipulator;
        m_speed = speed;
    }

    public Outtake(Manipulator manipulator)
    {
        this(manipulator, RobotConstants.ManipulatorConstants.kOuttakeSpeed);
    }

    @Override
    public void initialize()
    {
        m_manipulator.set(m_speed);
    }

    @Override
    public boolean isFinished()
    {
        return !m_manipulator.hasCoral();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_manipulator.stop();
    }
}
